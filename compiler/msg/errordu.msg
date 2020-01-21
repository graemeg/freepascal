#
#   German (UTF-8) Language File for Free Pascal
#   Latest updates contributed by Karl-Michael Schindler aka mischi
#   <karl-michael.schindler at web.de>
#
#   Based on errore.msg of SVN revision 42047
#
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1998-2019 by the Free Pascal Development team
#
#   See the file COPYING.v2, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#

# CodePage 65001

#
# The constants are build in the following order:
# <part>_<type>_<txtidentifier>
#
# <part> is the part of the compiler the message is used
#   asmr_     assembler parsing
#   asmw_     assembler writing/binary writers
#   unit_     unit handling
#   option_   command line parameter parsing
#   scan_     scanner
#   parser_   parser
#   type_     type checking
#   general_  general info
#   exec_     calls to assembler, external linker, binder
#   link_     internal linker
#   package_  package handling
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
#   o_   normal (e.g., "press enter to continue")
#
# <type> can contain a minus sign at the beginning to mark that
# the message is off by default. Look at type_w_explicit_string_cast
# for example.

#
# General
#
# 01027 is the last used one
#
# BeginOfTeX
%
% \section{General compiler messages}
% This section gives the compiler messages which are not fatal, but which
% display useful information. The number of such messages can be
% controlled with the various verbosity level \var{-v} switches.
% \begin{description}
general_t_compilername=01000_T_Compiler: $1
% When the \var{-vt} switch is used, this line tells you what compiler
% is used.
general_d_sourceos=01001_D_Quellbetriebssystem: $1
% When the \var{-vd} switch is used, this line tells you what the source
% operating system is.
general_i_targetos=01002_I_Zielbetriebssystem: $1
% When the \var{-vd} switch is used, this line tells you what the target
% operating system is.
general_t_exepath=01003_T_Der Pfad der ausführbaren Datei ist: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its binaries.
general_t_unitpath=01004_T_Der Unitpfad ist: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for compiled units. You can set this path with the \var{-Fu} option.
general_t_includepath=01005_T_Der Includepfad ist: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its include files (files used in \var{\{\$I xxx\}} statements).
% You can set this path with the \var{-Fi} option.
general_t_librarypath=01006_T_Der Bibliothekspfad ist: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for the libraries. You can set this path with the \var{-Fl} option.
general_t_objectpath=01007_T_Der Objektdateienpfad ist: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for object files you link in (files used in \var{\{\$L xxx\}} statements).
% You can set this path with the \var{-Fo} option.
general_i_abslines_compiled=01008_I_$1 Zeilen übersetzt, $2 Sekunden $3
% When the \var{-vi} switch is used, the compiler reports the number
% of lines compiled, and the time it took to compile them (real time,
% not program time).
general_f_no_memory_left=01009_F_Speicher voll
% The compiler doesn't have enough memory to compile your program. There are
% several remedies for this:
% \begin{itemize}
% \item If you're using the build option of the compiler, try compiling the
% different units manually.
% \item If you're compiling a huge program, split it up into units, and compile
% these separately.
% \item If the previous two don't work, recompile the compiler with a bigger
% heap. (You can use the \var{-Ch} option for this, \seeo{Ch}.)
% \end{itemize}
general_i_writingresourcefile=01010_I_Resourcestringtabellendatei $1 wird geschrieben
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_Fehler beim Schreiben der Resourcestringtabellendatei: $1
% This message is shown when the compiler encounters an error when writing
% the Resource String Table file.
general_i_fatal=01012_I_Fataler Fehler:
% Prefix for Fatal Errors.
general_i_error=01013_I_Fehler:
% Prefix for Errors.
general_i_warning=01014_I_Warnung:
% Prefix for Warnings.
general_i_note=01015_I_Anmerkung:
% Prefix for Notes.
general_i_hint=01016_I_Hinweis:
% Prefix for Hints.
general_e_path_does_not_exist=01017_E_Der Pfad "$1" existiert nicht
% The specified path does not exist.
general_f_compilation_aborted=01018_F_Übersetzen abgebrochen
% Compilation was aborted.
general_text_bytes_code=01019_Byte Code
% The size of the generated executable code, in bytes.
general_text_bytes_data=01020_Byte Daten
% The size of the generated program data, in bytes.
general_i_number_of_warnings=01021_I_$1 Warnung(en) ausgegeben
% Total number of warnings issued during compilation.
general_i_number_of_hints=01022_I_$1 Hinweis(e) ausgegeben
% Total number of hints issued during compilation.
general_i_number_of_notes=01023_I_$1 Anmerkung(en) ausgegeben
% Total number of notes issued during compilation.
general_f_ioerror=01024_F_I/O Fehler: $1
% During compilation an I/O error happened which allows no further compilation.
general_f_oserror=01025_F_Betriebsystemfehler: $1
% During compilation an operating system error happened which allows no further compilation.
general_e_exception_raised=01026_E_Kompilieren verursachte interne Ausnahme (exception)
% Compilation was aborted, due to an exception generation.
general_t_unitscope=01027_T_Benutzter Unit-Bereich: $1
% When the \var{-vt} switch is used, this line tells you what unit scopes (namespaces)
% the compiler is using when looking up units. You can add a unit scope with the
% \var{-FN} option.
%
% \end{description}
# EndOfTeX

#
# Scanner
#
# 02105 is the last used one
#
# BeginOfTeX
%
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compilation handling.
% \begin{description}
scan_f_end_of_file=02000_F_Unerwartetes Dateiende
% this typically happens in one of the following cases:
% \begin{itemize}
% \item The source file ends before the final \var{end.} statement. This
% happens mostly when the \var{begin} and \var{end} statements are not
% balanced;
% \item An include file ends in the middle of a statement.
% \item A comment was not closed.
% \end{itemize}
scan_f_string_exceeds_line=02001_F_Zeichenkette geht über das Zeilenende hinaus
% You forgot probably to include the closing ' in a string, so it occupies
% multiple lines.
scan_f_illegal_char=02002_F_Unzulässiges Zeichen "$1" ($2)
% An illegal character was encountered in the input file.
scan_f_syn_expected=02003_F_Syntaxfehler, "$1" erwartet, aber "$2" vorgefunden
% This indicates that the compiler expected a different token than
% the one you typed. It can occur almost anywhere it is possible to make an error
% against the Pascal language.
scan_t_start_include_file=02004_TL_Die Include-Datei $1 wird jetzt gelesen
% When you provide the \var{-vt} switch, the compiler tells you
% when it starts reading an included file.
scan_w_comment_level=02005_W_Kommentarschachtelungstiefe $1 gefunden
% When the \var{-vw} switch is used, then the compiler warns you if
% it finds nested comments. Nested comments are not allowed in Turbo Pascal
% and Delphi, and can be a possible source of errors.
scan_n_ignored_switch=02008_N_Compilerschalter $1 wurde ignoriert
% With \var{-vn} on, the compiler warns if it ignores a switch.
scan_w_illegal_switch=02009_W_Compilerschalter $1 ist ungültig
% You included a compiler switch (i.e. \var{\{\$... \}}) which the compiler
% does not recognise.
scan_w_switch_is_global=02010_W_Dieser globale Compilerschalter ist an der falschen Stelle und wird deshalb ignoriert
% The compiler switch is misplaced. It must be located at
% the start of the compilation unit, before the uses clause or any declaration.
scan_e_illegal_char_const=02011_E_Ungültige Char-Konstante
% This happens when you specify a character with its ASCII code, as in
% \var{\#96}, but the number is either illegal, or out of range.
scan_f_cannot_open_input=02012_F_Datei $1 kann nicht geöffnet werden
% \fpc cannot find the program or unit source file you specified on the
% command line.
scan_f_cannot_open_includefile=02013_F_Includedatei $1 kann nicht geöffnet werden
% \fpc cannot find the source file you specified in a \var{\{\$include ..\}}
% statement.
scan_e_illegal_pack_records=02015_E_Ungültige Record Ausrichtung "$1"
% You are specifying \var{\{\$PACKRECORDS n\} } or \var{\{\$ALIGN n\} }
% with an illegal value for \var{n}. For \$PACKRECORDS valid alignments are 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, and for \$ALIGN valid alignments are 1, 2, 4, 8, 16, 32, ON,
% OFF. Under mode MacPas \$ALIGN also supports MAC68K, POWER and RESET.
scan_e_illegal_pack_enum=02016_E_Ungültige minimale Größe der Aufzählung "$1"
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2,4, NORMAL or DEFAULT is valid here.
scan_e_endif_expected=02017_E_$ENDIF erwartet für $1 $2 definiert in $3 Zeile $4
% Your conditional compilation statements are unbalanced.
scan_e_preproc_syntax_error=02018_E_Syntaxfehler im Argument einer $if Direktive
% There is an error in the expression following the \var{\{\$if ..\}}, \var{\{\$ifc \}}
% or \var{\{\$setc \}} compiler directives.
scan_e_error_in_preproc_expr=02019_E_Syntaxfehler im Kontext einer $if Direktive
% There is an error in the expression following the \var{\{\$if ..\}} compiler
% directive.
scan_w_macro_cut_after_255_chars=02020_W_Inhalt des Makros wurde nach der Auswertung bei 255 Zeichen abgeschnitten
% The contents of macros cannot be longer than 255 characters.
scan_e_endif_without_if=02021_E_ENDIF ohne IF(N)DEF
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements are not balanced.
scan_f_user_defined=02022_F_Benutzerdefiniert: $1
% A user defined fatal error occurred. See also the \progref
scan_e_user_defined=02023_E_Benutzerdefiniert: $1
% A user defined error occurred. See also the \progref
scan_w_user_defined=02024_W_Benutzerdefiniert: $1
% A user defined warning occurred. See also the \progref
scan_n_user_defined=02025_N_Benutzerdefiniert: $1
% A user defined note was encountered. See also the \progref
scan_h_user_defined=02026_H_Benutzerdefiniert: $1
% A user defined hint was encountered. See also the \progref
scan_i_user_defined=02027_I_Benutzerdefiniert: $1
% User defined information was encountered. See also the \progref
scan_e_keyword_cant_be_a_macro=02028_E_Ein Makro, das den selben Namen wie ein Schlüsselwort hat, wird ignoriert
% You cannot redefine keywords with macros.
scan_f_macro_buffer_overflow=02029_F_Makropufferüberlauf während des Lesens oder Expandierens eines Makros
% Your macro or it's result was too long for the compiler.
scan_w_macro_too_deep=02030_W_Expansion des Makros überschreitet eine Makroschachtelungstiefe von 16
% When expanding a macro macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_w_wrong_styled_switch=02031_W_Compilerschalter innerhalb von Kommentaren der Form // werden nicht untersützt
% Compiler switches should be in normal Pascal style comments.
scan_d_handling_switch=02032_DL_Bearbeite Schalter "$1"
% When you set debugging info on (\var{-vd}) the compiler tells you when it
% is evaluating conditional compile statements.
scan_c_endif_found=02033_CL_ENDIF $1 bearbeitet
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifdef_found=02034_CL_IFDEF $1 bearbeitet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifopt_found=02035_CL_IFOPT $1 bearbeitet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_if_found=02036_CL_IF $1 bearbeitet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifndef_found=02037_CL_IFNDEF $1 bearbeitet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_else_found=02038_CL_ELSE $1 bearbeitet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_skipping_until=02039_CL_Quelltext wird ignoriert bis...
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements, and whether it is skipping or
% compiling parts.
scan_i_press_enter=02040_I_Drücken Sie <Return> um fortzusetzen
% When the \var{-vi} switch is used, the compiler stops compilation
% and waits for the \var{Enter} key to be pressed when it encounters
% a \var{\{\$STOP\}} directive.
scan_w_unsupported_switch=02041_W_Nicht unterstützter Schalter $1
% When warings are turned on (\var{-vw}), the compiler warns you about
% unsupported switches. This means that the switch is used in Delphi or
% Turbo Pascal, but not in \fpc.
scan_w_illegal_directive=02042_W_Ungültige Compilerdirektive $1
% When warings are turned on (\var{-vw}), the compiler warns you about
% unrecognised switches. For a list of recognised switches, see the \progref
scan_t_back_in=02043_TL_Wieder zurück in $1
% When you use the \var{-vt} switch, the compiler tells you when it has finished
% reading an include file.
scan_w_unsupported_app_type=02044_W_Nicht unterstützter Anwendungstyp: $1
% You get this warning if you specify an unknown application type
% with the directive \var{\{\$APPTYPE\}}.
scan_w_app_type_not_support=02045_W_APPTYPE wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$APPTYPE\}} directive is supported by win32 applications only.
scan_w_description_not_support=02046_W_Der Compilerschalter DESCRIPTION wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$DESCRIPTION\}} directive is not supported on this target OS.
scan_n_version_not_support=02047_N_VERSION wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$VERSION\}} directive is not supported on this target OS.
scan_n_only_exe_version=02048_N_VERSION kann in Units nicht verwendet werden
% The \var{\{\$VERSION\}} directive is only used for executable or DLL sources.
scan_w_wrong_version_ignored=02049_W_Falsches Format für VERSION-Direktive $1
% The \var{\{\$VERSION\}} directive format is major_version.minor_version
% where major_version and minor_version are words.
scan_e_illegal_asmmode_specifier=02050_E_Unbekannter Assemblermodusname: "$1"
% When you specify an assembler mode with the \var{\{\$ASMMODE xxx\}} directive,
% the compiler didn't recognize the mode you specified.
scan_w_no_asm_reader_switch_inside_asm=02051_W_ASM-Leser-Schalter ist innerhalb einer Asm-Sequenz nicht möglich, $1 wird erst in der nächsten Sequenz wirksam
% It is not possible to switch from one assembler reader to another
% inside an assembler block. The new reader will be used for next
% assembler statement only.
scan_e_wrong_switch_toggle=02052_E_Parameter für Schalter falsch, verwenden Sie ON/OFF oder +/-
% You need to use ON or OFF or a + or - to toggle the switch.
scan_e_resourcefiles_not_supported=02053_E_Resourcedateien werden vom aktuellen Zielbetriebssystem nicht unterstützt
% The target you are compiling for doesn't support resource files.
scan_w_include_env_not_found=02054_W_$1 ist keine Umgebungsvariable
% The included environment variable cannot be found in the environment; it will
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_Nicht erlaubter Wert für MAXFPUREGISTER-Direktive
% Valid values for this directive are 0..8 and NORMAL/DEFAULT.
scan_w_only_one_resourcefile_supported=02056_W_Vom aktuellen Zielbetriebssystem wird nur eine Resourcedatei unterstützt
% Only one resource file can be supported for this target - this is the case of
% OS/2 (EMX) currently. The first one found is used, the others are discarded.
scan_w_macro_support_turned_off=02057_W_Makrounterstützung ist ausgeschaltet
% A macro declaration has been found, but macro support is currently off,
% so the declaration will be ignored. To turn macro support on compile with
% -Sm on the command line or add \{\$MACRO ON\} in the source.
scan_e_invalid_interface_type=02058_E_Unbekannter Interfacetyp. Unterstützt wird COM, CORBA und DEFAULT
% The interface type that was specified is not supported.
scan_w_appid_not_support=02059_W_APPID wird nur von PalmOS unterstützt
% The \var{\{\$APPID\}} directive is only supported for the PalmOS target.
scan_w_appname_not_support=02060_W_APPNAME wird nur von PalmOS unterstützt
% The \var{\{\$APPNAME\}} directive is only supported for the PalmOS target.
scan_e_string_exceeds_255_chars=02061_E_Zeichenkettenkonstanten können maximal 255 Zeichen lang sein
% A single string constant can contain at most 255 chars. Try splitting up the
% string into multiple smaller parts and concatenate them with a + operator.
scan_f_include_deep_ten=02062_F_Include-Dateien können nur bis zu einer Tiefe von 16 geschachtelt werden
% When including include files the files have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_too_many_push=02063_F_Zu viele Ebenen von PUSH
% A maximum of 20 levels is allowed. This error occurs only in mode MacPas.
scan_e_too_many_pop=02064_E_POP ohne ein vorhergehendes PUSH
% This error occurs only in mode MacPas.
scan_e_error_macro_lacks_value=02065_E_Das Makro "$1" besitzt keinen Wert
% Thus the conditional compiling expression cannot be evaluated.
scan_e_wrong_switch_toggle_default=02066_E_Zum Modusumschalten kann nur ON/OFF/DEFAULT oder +/-/* verwendet werden
% You need to use ON or OFF or DEFAULT or a + or - or * to toggle the switch.
scan_e_mode_switch_not_allowed=02067_E_Der Modus Umschalter "$1" ist hier nicht erlaubt
% A mode switch has already been encountered, or, in the case of option -Mmacpas,
% a mode switch occurs after UNIT.
scan_e_error_macro_undefined=02068_E_Die Compile time Variable oder das Makro "$1" ist nicht definiert.
% Thus the conditional compile time expression cannot be evaluated. Only in mode MacPas.
scan_e_utf8_bigger_than_65535=02069_E_Der UTF-8 Code ist größer als 65535
% \fpc handles UTF-8 strings internally as widestrings, i.e. the char codes are limited to 65535.
scan_e_utf8_malformed=02070_E_Ungültige UTF-8 Zeichenkette
% The given string isn't a valid UTF-8 string.
scan_c_switching_to_utf8=02071_C_UTF-8 Signatur gefunden, verwende UTF-8 Code
% The compiler found a UTF-8 encoding signature (\$ef, \$bb, \$bf) at the beginning of a file,
% so it interprets it as a UTF-8 file.
scan_e_compile_time_typeerror=02072_E_Compile time Ausdruck: Erwartete $1 aber erhielt $2 bei $3
% The type-check of a compile time expression failed.
scan_n_app_type_not_support=02073_N_APPTYPE wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_e_illegal_optimization_specifier=02074_E_"$1" ist eine ungültige Optimierung 
% You specified an optimization with the \var{\{\$OPTIMIZATION xxx\}} directive,
% and the compiler didn't recognize the optimization you specified.
scan_w_setpeflags_not_support=02075_W_SETPEFLAGS wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$SETPEFLAGS\}} directive is not supported by the target OS.
scan_w_imagebase_not_support=02076_W_IMAGEBASE wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$IMAGEBASE\}} directive is not supported by the target OS.
scan_w_minstacksize_not_support=02077_W_MINSTACKSIZE wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$IMAGEBASE\}} directive is not supported by the target OS.
scan_w_maxstacksize_not_support=02078_W_MAXSTACKSIZE wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$MAXSTACKSIZE\}} directive is not supported by the target OS.
scanner_e_illegal_warn_state=02079_E_Ungültiger Wert "$1" für die $WARN Direktive
% Only ON and OFF can be used as state with a \var{\{\$WARN\}} compiler directive.
scan_e_only_packset=02080_E_Ungültiger Wert für das set packing
% Only 0, 1, 2, 4, 8, DEFAULT and NORMAL are allowed as packset parameters.
scan_w_pic_ignored=02081_W_PIC Direktive oder Schalter wird ignoriert
% Several targets, such as \windows, do not support nor need PIC,
% so the PIC directive and switch are ignored.
scan_w_unsupported_switch_by_target=02082_W_Der Schalter "$1" wird vom derzeit ausgewählten Zielbetriebssystem nicht unterstützt
% Some compiler switches like \$E are not supported by all targets.
scan_w_frameworks_darwin_only=02084_W_Framework-bezogene Optionen werden nur für Darwin/Mac OS X unterstützt
% Frameworks are not a known concept, or at least not supported by FPC,
% on operating systems other than Darwin/Mac OS X.
scan_e_illegal_minfpconstprec=02085_E_"$1" ist eine ungültige minimale Präzision von Fliesskommakonstanten
% Valid minimal precisions for floating point constants are default, 32 and 64,
% which mean respectively minimal (usually 32 bit), 32 bit and 64 bit precision.
scan_w_multiple_main_name_overrides=02086_W_Der Name der "main" Prozedur wird mehrfach überschrieben. Es war bisher auf "$1" gesetzt
% The name for the main entry procedure is specified more than once. Only the last
% name will be used.
scanner_w_illegal_warn_identifier=02087_W_Ungültige Bezeichner "$1" für die $WARN Direktive
% Identifier is not known by a \var{\{\$WARN\}} compiler directive.
scanner_e_illegal_alignment_directive=02088_E_Ungültige "alignment" Direktive
% The alignment directive is not valid. Either the alignment type is not known or the alignment
% value is not a power of two.
scanner_f_illegal_utf8_bom=02089_F_Eine Datei, die mit einem UTF-8 BOM anfängt, kann nicht in einem Modul mit einer anderen Codepage verwendet werden
% All source code that is part of a single compilation entity (program, library, unit) must be encoded
% in the same code page
scanner_w_directive_ignored_on_target=02090_W_Die Direktive "$1" wird für die aktuelle Zielplattform ignoriert
% Some directives are ignored for certain targets, such as changing the
% packrecords and packenum settings on managed platforms.
scan_w_unavailable_system_codepage=02091_W_Die aktuelle System-Codepage "$1" steht für den Compiler nicht zur Verfügung. Stelle die voreingestellte Codepage um auf "$2".
% The current system codepage is not known by the compiler.
% The compiler is compiled with support for several codepages built-in.
% The codepage of the operation system is not in that list. You will need to recompile
% the compiler with support for this codepage.
scan_w_setpeoptflags_not_support=02092_W_SETPEOPTFLAGS wird für das Ziel-OS nicht unterstützt
% The \var{\{\$SETPEOPTFLAGS\}} directive is not supported by the target OS.
scan_e_illegal_peflag=02093_E_Ungültiges Argument für SETPEFLAGS
% The given argument for SETPEFLAGS is neither a correct named value nor an
% ordinal value
scan_e_illegal_peoptflag=02094_E_Ungültiges Argument für SETPEOPTFLAGS
% The given argument for SETPEOPTFLAGS is neither a correct named value nor an
% ordinal value
scan_e_unsupported_switch=02095_E_Direktive $1 wird auf diesem Zielbetriebssystem nicht unterstützt
% Not all compiler directives are supported on all targets.
scan_w_invalid_stacksize=02096_W_Die spezifizierte Größe des Stack ist außerhalb des gültigen Bereichs der Plattform. Setzen der Größe des Stack ignoriert.
% The valid range for the stack size is 1024 - 67107839 on 32-bit and 64-bit
% platforms and 1024 - 65520 on 16-bit platforms. Additionally, for Turbo Pascal 7
% compatibility reasons, specifying a stack size of 65521 on 16-bit platforms
% actually sets the stack size to 65520.
scan_w_heapmax_lessthan_heapmin=02097_W_Die spezifizierte Größe des HeapMax ist kleiner als der des HeapMin. Setzen der Größe des HeapMax ignoriert.
% The HeapMax value (if specified) must be greater than or equal to the HeapMin
% value. Otherwise, the HeapMax value is ignored.
scan_e_illegal_hugepointernormalization=02098_E_Illegales Argument für HUGEPOINTERNORMALIZATION
% The only allowed values for HUGEPOINTERNORMALIZATION are BORLANDC, MICROSOFTC
% and WATCOMC.%
scan_e_illegal_asmcpu_specifier=02099_E_Ungültiger Assembler-CPU-Befehlssatz "$1" angegeben
% When you specify an assembler CPU with the \var{\{\$ASMCPU xxx\}} directive,
% the compiler didn't recognize the CPU you specified.
scan_w_syscall_convention_not_useable_on_target=02100_W_Die angegebene SysCall-Konvention kann auf diesem Ziel nicht verwendet werden
% The specified syscall convention using the \var{\{\$SYSCALL xxx\}} directive,
% is not useable on the current target system.
scan_w_syscall_convention_invalid=02101_W_Ungültige SysCall-Konvention angegeben
% The compiler did not recognize the syscall convention specified by the \var{\{\$SYSCALL xxx\}} directive.
scan_w_setpeuserversion_not_support=02102_W_SETPEUSERVERSION wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$SETPEUSERVERSION\}} directive is not supported by the target OS.
scan_w_setpeosversion_not_support=02103_W_SETPEOSVERSION wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$SETPEOSVERSION\}} directive is not supported by the target OS.
scan_w_setpesubsysversion_not_support=02104_W_SETPESUBSYSVERSION wird vom Zielbetriebssystem nicht unterstützt
% The \var{\{\$SETPESUBSYSVERSION\}} directive is not supported by the target OS.
scan_n_changecputype=02105_N_Geänderter CPU-Typ muss zum angegebenen Controller passen
%
% \end{description}
# EndOfTeX

#
# Parser
#
# 03350 is the last used one
#
# BeginOfTeX
%
% \section{Parser messages}
% This section lists all parser messages. The parser takes care of the
% semantics of you language, i.e. it determines if your Pascal constructs
% are correct.
% \begin{description}
parser_e_syntax_error=03000_E_Parser - Syntaxfehler
% An error against the Turbo Pascal language was encountered. This typically
% happens when an illegal character is found in the source file.
parser_e_dont_nest_interrupt=03004_E_INTERRUPT-Prozeduren dürfen nicht verschachtelt sein
% An \VAR{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_Prozedurtyp $1 wird ignoriert
% The specified procedure directive is ignored by FPC programs.
parser_e_no_overload_for_all_procs=03006_E_Nicht alle Deklarationen von "$1" sind mit OVERLOAD deklariert
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_export_name_double=03008_E_Doppelter Name für exportierte Funktion $1
% Exported function names inside a specific DLL must all be different.
parser_e_export_ordinal_double=03009_E_Doppelter Index für exportierte Funktion $1
% Exported function names inside a specific DLL must all be different.
parser_e_export_invalid_index=03010_E_Ungültiger Index for exportierte Funktion
% DLL function index must be in the range \var{1..\$FFFF}.
parser_w_parser_reloc_no_debug=03011_W_Für relozierbare DLL oder ausführbare Datei $1 funktionieren keine Debug-Information, deaktiviert.
% It is currently not possible to include debug information in a relocatable DLL.
parser_w_parser_win32_debug_needs_WN=03012_W_Um Win32-Code debuggen zu können, müssen die Relozierungen mit -WN option abgeschaltet werden.
% Stabs debug info is wrong for relocatable DLL or EXES. Use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_Konstruktor-Name muss INIT sein
% You are declaring a constructor with a name which isn't \var{init}, and the
% \var{-Ss} switch is in effect. See the switch \seeo{Ss}.
parser_e_destructorname_must_be_done=03014_E_Destruktor-Name muss DONE sein
% You are declaring a constructor with a name which isn't \var{done}, and the
% \var{-Ss} switch is in effect. See the switch \seeo{Ss}.
parser_e_proc_inline_not_supported=03016_E_Schlüsselwort INLINE nicht unterstützt
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler doesn't support C++
% styled inlining by default.
parser_w_constructor_should_be_public=03018_W_Konstruktor muss PUBLIC sein
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_Destruktor muss PUBLIC sein
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_Klasse darf nur einen Destruktor besitzen
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_Lokale Klassendefinitionen sind nicht zulässig
% Classes must be defined globally. They cannot be defined inside a
% procedure or function.
parser_f_no_anonym_objects=03022_F_Anonyme Klassendefinitionen sind nicht zulässig
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_n_object_has_no_vmt=03023_N_Das Objekt "$1" besitzt keine VMT
% This is a note indicating that the declared object has no
% virtual method table.
parser_e_illegal_parameter_list=03024_E_Ungültige Parameterliste
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_size=03026_E_Falsche Anzahl an Parameter im Aufruf von "$1" angegeben
% There is an error in the parameter list of the function or procedure --
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_Bezeichner "$1" ist keine Funktion, overload nicht möglich
% The compiler encountered a symbol with the same name as an overloaded
% function, but it isn't a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_Overload-Funktion darf keine identische Parameterliste aufweisen
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_Funktionskopf unterscheidet sich von voriger Deklaration "$1"
% You declared a function with the same parameters but
% different result type or function specifiers.
parser_e_header_different_var_names=03030_E_Funktionskopf von "$1" passt nicht zur Forward-Deklaration, Variablename ändert sich: $2 => $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but defined it with a different parameter list.
parser_n_duplicate_enum=03031_N_Werte in Aufzählungen müssen aufsteigend sein
% \fpc allows enumeration constructions as in C. Examine the following
% two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_e_no_with_for_variable_in_other_segments=03033_E_WITH kann nicht auf Variablen in anderen Segmenten angewendet werden
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_Funktionsverschachtelung größer als 31
% You can nest function definitions only 31 levels deep.
parser_e_range_check_error=03035_E_Bereichsprüfungsfehler bei Konstantenbestimmung
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_Bereichsprüfungsfehler bei Konstantenbestimmung
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_doppelter CASE-Wert
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_Obere Grenze der CASE-Bereichsangabe ist kleiner als untere Grenze
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless.
parser_e_type_const_not_possible=03039_E_Klassen oder Interfaces sind als typisierte Konstanten unzulässig
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_Funktionsvariablen von Overload-Funktionen sind unzulässig
% You are trying to assign an overloaded function to a procedural variable.
% This is not allowed.
parser_e_invalid_string_size=03041_E_Stringlänge muss ein Wert zwischen 1 und 255 sein
% The length of a string in Pascal is limited to 255 characters. You are
% trying to declare a string with length less than 1 or greater than 255.
parser_w_use_extended_syntax_for_objects=03042_W_Benutzen Sie die erweiterte Syntax von NEW und DISPOSE für Objekt-Instanzen
% If you have a pointer \var{a} to an object type, then the statement
% \var{new(a)} will not initialize the object (i.e. the constructor isn't
% called), although space will be allocated. You should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the object.
parser_w_no_new_dispose_on_void_pointers=03043_W_Verwendung von NEW oder DISPOSE mit untypisierten Zeigern ist ohne Aussage
parser_e_no_new_dispose_on_void_pointers=03044_E_Verwendung von NEW oder DISPOSE mit untypisierten Zeigern ist nicht möglich
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% It is accepted for compatibility in \var{TP} and \var{DELPHI} modes, but the
% compiler will still warn you if it finds such a construct.
parser_e_class_id_expected=03045_E_Klassenbezeichner erwartet
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., an object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_Typbezeichner ist hier nicht zulässig
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_Methodenbezeichner erwartet
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., an object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_Funktionskopf passt zu keiner Methode der Klasse $1
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., an object or class method, but the procedure name is not a
% procedure of this type.
parser_d_procedure_start=03049_DL_Procedure/Function $1
% When using the \var{-vd} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_Ungültige Fliesskommakonstante
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_FAIL darf nur in Konstruktoren verwendet werden
% You are using the \var{FAIL} instruction outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_Destruktoren können keine Parameter haben
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_members_via_class_ref=03053_E_Nur Klassenmethoden, Klasseneigenschaften und Klassenvariablen können über den Klassennamen angesprochen werden
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
parser_e_only_class_members=03054_E_Nur Klassenmethoden, Klasseneigenschaften und Klassenvariablen können in einer Klassenmethode angesprochen werden
% This is related to the previous error. You cannot call a method of an object
% from inside a class method. The following code would produce this error:
% \begin{verbatim}
% class procedure tobject.x;
%
% begin
%   free
% \end{verbatim}
% Because free is a normal method of a class it cannot be called from a class
% method.
parser_e_case_mismatch=03055_E_Konstante und CASE-Typ passen nicht zueinander
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_Das Symbol kann nicht aus einer Bibliothek exportiert werden
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_Eine Inherit-Methode wird durch $1 verdeckt
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendant class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_Es gibt keine Vorgängerklasse mit einer Methode, die damit überschrieben werden könnte: $1
% You try to \var{override} a virtual method of a parent class that doesn't
% exist.
parser_e_no_procedure_to_access_property=03059_E_Es gibt keine Member-Funktion um auf diese Property zuzugreifen
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_Die Stored Property - Direktive ist noch nicht implementiert
% This message is no longer used, as the \var{stored} directive has been implemented.
parser_e_ill_property_access_sym=03061_E_Ungültiges Symbol für den Zugriff auf die Property
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code would cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Kann an dieser Stelle nicht auf das geschützte Feld des Objekts zugreifen
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module where the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_Kann an dieser Stelle nicht auf das private Feld des Objekts zugreifen
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_e_overridden_methods_not_same_ret=03066_E_Überschriebene, virtuelle Methoden müssen den gleichen Ergebnistyp haben: "$2" wird überschrieben von "$1"
% If you declare overridden methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_Mit EXPORT deklarierte Funktionen dürfen nicht verschachtelt sein
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_Methoden dürfen nicht EXPORTiert werden
% You cannot declare a procedure that is a method for an object as
% \var{export}ed. That is, your methods cannot be called from a C program.
parser_e_call_by_ref_without_typeconv=03069_E_Aufrufe mit VAR-Parametern für Argument-Nr. $1 müssen exakt stimmen: "$2" gefunden, "$3" erwartet
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_Klasse ist keine Elternklasse zur aktuellen Klasse
% When calling inherited methods, you are trying to call a method of a strange
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_SELF ist nur in Methoden erlaubt
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_Methoden dürfen nur in anderen Methoden direkt mit dem Klassen-Typbezeichner aufgerufen werden
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_Unzulässige Verwendung von ':'
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_Bereichsprüfungsfehler im SET-Konstruktor oder doppeltes Set-Element
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, or two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_Zeiger auf Objekt erwartet
% You specified an illegal type in a \var{New} statement.
% The extended synax of \var{New} needs an object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_Ausdruck muss ein Konstruktor-Aufruf sein
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_Ausdruck muss ein Destruktor-Aufruf sein
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_Unzulässige Reihenfolge der Record-Elemente
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_Typ des Ausdrucks muss eine Klasse oder ein Recordtyp sein, $1 erhalten
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_Prozeduren können keinen Wert zurückliefern
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures cannot return a value.
parser_e_only_methods_allowed=03081_E_Konstruktoren, Destruktoren und Klassenoperatoren müssen Methoden sein
% You're declaring a procedure as destructor, constructor or class operator, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_Operator besitzt kein Overload
% You're trying to use an overloaded operator when it isn't overloaded for
% this type.
parser_e_no_such_assignment=03083_E_Es ist nicht möglich, die Zuweisung für gleiche Typen zu überladen
% You cannot overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_Unmögliche Operator-.berladung
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_Auslösen einer Ausnahme (exception) an dieser Stelle nicht möglich
% You are trying to re-raise an exception where it is not allowed. You can only
% re-raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_Die erweiterte Syntax von New oder Dispose ist für Klassen unzulässig
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{Dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_procedure_overloading_is_off=03088_E_Das Überladen von Prozeduren ist ausgeschaltet
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_Es ist nicht möglich, dieesen Operator zu überladen. Verwandte überladbare Operatoren sind "$1"
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_Vergleichsoperator muss einen booleschen Wert zurückgeben
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_Nur virtuelle Methoden können abstrakt sein
% You are declaring a method as abstract, when it isn't declared to be
% virtual.
parser_f_unsupported_feature=03092_F_Benutzung der nicht unterstützten Erweiterung: "$1"
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_Das Mischen von Klassen und Objekten ist unzulässig
% You cannot derive \var{objects}, \var{classes}, \var{cppclasses} and \var{interfaces} intertwined. E.g.
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_Unbekannte Prozedur, Direktive $1 wurde ignoriert
% The procedure direcive you secified is unknown.
parser_e_directive_only_one_var=03095_E_$1 kann nur auf eine alleinstehende Variable angewendet werden
% You cannot specify more than one variable before the \var{absolute}, \var{export}, \var{external},
% \var{weakexternal}, \var{public} and \var{cvar} directives.
% As a result, for example the following construct will provide this error:
% \begin{verbatim}
% Var Z : Longint;
%     X,Y : Longint absolute Z;
% \end{verbatim}
parser_e_absolute_only_to_var_or_const=03096_E_ABSOLUTE kann nur auf Variablen und Konstanten angewendet werden
% The address of an \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_Es kann nur EINE Variable initialisiert werden
% You cannot specify more than one variable with a initial value in Delphi mode.
parser_e_abstract_no_definition=03098_E_Abstrakte Methoden dürfen keine Definition (mit Rumpf) haben
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_Diese überladene Funktion darf nicht lokal sein (muss Exportiert werden)
% You are defining an overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_Virtuelle Methoden wurden ohne Konstruktor verwendet in $1
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_c_macro_defined=03101_CL_Makro definiert: $1
% When \var{-vc} is used, the compiler tells you when it defines macros.
parser_c_macro_undefined=03102_CL_Makrodefinition gelöscht: $1
% When \var{-vc} is used, the compiler tells you when it undefines macros.
parser_c_macro_set_to=03103_CL_Wert des Makros $1 auf $2 gesetzt
% When \var{-vc} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_Übersetze $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_UL_Parse Interface von Unit $1
% This tells you that the reading of the interface
% of the current unit has started
parser_u_parsing_implementation=03106_UL_Parse Implementation von $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_DL_Übersetze $1 zum zweiten Mal
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_property_found_to_override=03109_E_Es gibt keine Property diesen Namens, die überschrieben werden könnte
% You want to overrride a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_Es ist nur eine einzelne Default-Property erlaubt
% You specified a property as \var{Default}, but a parent class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_Die Default-Property muss eine Array-Property sein
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_Virtuelle Konstruktoren werden nur im Klassen-Objektmodell unterstützt
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_Keine Default-Property verfügbar
% You try to access a default property of a class, but this class (or one of
% its ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_Die Klasse kann keinen PUBLISHED Bereich haben, benutzen Sie den {$M+} Schalter
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, which turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_Forward-Deklaration der Klasse $1 muss hier aufgelöst werden, wenn sie als Elternklasse benutzt werden soll
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_Lokale Operatoren werden nicht unterstützt
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_Prozedur-Direktive $1 unzulässig im Interface-Bereich
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_Prozedur Direktive $1 unzulässig im Implementation-Bereich
% This procedure directive is not allowed in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_Prozedur-Direktive $1 unzulässig in einer Procvar-Deklaration
% This procedure directive cannot be part of a procedural of function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_Funktion ist bereits als Public oder Forward deklariert: $1
% You will get this error if a function is defined as \var{forward} twice.
% Or if it occurs in the \var{interface} section, and again as a \var{forward}
% declaration in the \var{implementation} section.
parser_e_not_external_and_export=03121_E_Kann nicht EXPORT und EXTERNAL gleichzeitig benutzen
% These two procedure directives are mutually exclusive.
parser_n_not_supported_for_inline=03123_N_$1 noch nicht innerhalb von inline Prozeduren/Funktionen unterstützt
% Inline procedures don't support this declaration.
parser_h_inlining_disabled=03124_H_Inlining deaktiviert
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_Schreibe Browser-Log $1
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+\}} switch).
parser_h_maybe_deref_caret_missing=03126_H_Möglicherweise fehlt eine Zeiger-Dereferenzierung
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_Gewählter Assembler-Leser nicht unterstützt
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_Prozedur-Direktive "$1" kann nicht mit $2 benutzt werden
% You specified a procedure directive that conflicts with other directives.
% For instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_Aufrufkonvention passt nicht zur Forward-Deklaration
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_property_cant_have_a_default_value=03131_E_Property kann keinen Vorgabewert haben
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_Der Vorgabewert einer Property muss eine Konstante sein
% The value of a \var{default} declared property must be known at compile
% time. The value you specified is only known at run time. This happens
% e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_Symbol darf nicht PUBLISHED sein, dies kann nur eine Klasse sein
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_Diese Property-Art kann nicht PUBLISHED sein
% Properties in a \var{published} section cannot be array properties.
% They must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_e_empty_import_name=03136_E_Leerer Importname angegeben
% Some targets need a name for the imported procedure or a \var{cdecl} specifier.
parser_e_division_by_zero=03138_E_Division durch Null
% A division by zero was encountered.
parser_e_invalid_float_operation=03139_E_Ungültige Fliesskomma-Operation
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_Obere Grenze des Bereichs ist kleiner als die untere Grenze
% The upper bound of an array declaration is less than the lower bound and this
% is not possible.
parser_w_string_too_long=03141_W_String "$1" ist länger als $2
% The size of the constant string is larger than the size you specified in
% string type definition.
parser_e_string_larger_array=03142_E_Stringlänge ist größer als die Länge des "array of char"
% The size of the constant string is larger than the size you specified in
% the \var{Array[x..y] of char} definition.
parser_e_ill_msg_expr=03143_E_Ungültiger Ausdruck nach der 'Message'-Direktive
% \fpc supports only integer or string values as message constants.
parser_e_ill_msg_param=03144_E_Message-Handler akzeptieren nur einen "Call by Reference" Parameter
% A method declared with the \var{message} directive as message handler
% can take only one parameter which must be declared as call by reference.
% Parameters are declared as call by reference using the \var{var}-directive.
parser_e_duplicate_message_label=03145_E_Doppeltes Message-Label: $1
% A label for a message is used twice in one object/class.
parser_e_self_in_non_message_handler=03146_E_SELF darf nur in Message-Handlern ein expliziter Parameter sein
% The \var{Self} parameter can only be passed explicitly to a method which
% is declared as message method handler
parser_e_threadvars_only_sg=03147_E_Threadvariablen können nur statisch oder global sein
% Threadvars must be static or global; you cannot declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has its own stack and local variables
% are stored on the stack.
parser_f_direct_assembler_not_allowed=03148_F_Direkter Assembler wird f+r binäres Ausgabeformat nicht unterstützt
% You cannot use direct assembler when using a binary writer. Choose an
% other output format or use another assembler reader.
parser_w_no_objpas_use_mode=03149_W_Laden Sie die OBJPAS Unit nicht manuell, benutzen Sie statdessen {$mode objfpc} oder {$mode delphi}
% You are trying to load the \file{ObjPas} unit manually from a \var{uses} clause.
% This is not a good idea. Use the \var{\{\$MODE OBJFPC\}} or
% \var{\{\$mode delphi\}} directives which load the unit automatically.
parser_e_no_object_override=03150_E_OVERRIDE darf in Objekten nicht verwendet werden
% \var{Override} is not supported for objects, use \var{virtual} instead to override
% a method of a parent object.
parser_e_cant_use_inittable_here=03151_E_Datentypen, die ein Initialiserung oder Finalisierung benötigen, können in varianten Records nicht verwendet werden
% Some data types (e.g. \var{ansistring}) need initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% cannot be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_Resourcestrings dürfen nur statisch oder global sein
% Resourcestring cannot be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_EXIT mit Argument darf hier nicht verwendet werden
% An exit statement with an argument for the return value cannot be used here. This
% can happen for example in \var{try..except} or \var{try..finally} blocks.
parser_e_stored_property_must_be_boolean=03154_E_Der Typ des STORED-Symbols muss boolesch sein
% If you specify a storage symbol in a property declaration, it must be a
% boolean type.
parser_e_ill_property_storage_sym=03155_E_Dieses Symbol ist als Speichersymbol unzulässig
% You cannot use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants.
parser_e_only_publishable_classes_can_be_published=03156_E_Nur Klassen, die im "$M+"-Modus übersetzt wurden, dürfen published sein
% A class-typed field in the published section of a class can only be a class which was
% compiled in \var{\{\$M+\}} or which is derived from such a class. Normally
% such a class should be derived from \var{TPersistent}.
parser_e_proc_directive_expected=03157_E_Prozedurdirektive erwartet
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
parser_e_invalid_property_index_value=03158_E_Der Wert für einen Property-Index muss ordinalen Typs sein
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_Prozedurname zu kurz um exportiert zu werden
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correctly with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_Es kann kein DEFFILE-Eintrag für unit-globale Variablen erzeugt werden
parser_e_dlltool_unit_var_problem2=03161_E_Übersetze ohne "-WD"-Option
% You need to compile this file without the -WD switch on the
% command line.
parser_f_need_objfpc_or_delphi_mode=03162_F_Es muss der Modus ObjFPC (-S2) oder Delphi (-Sd) aktiv sein, um dieses Modul zu übersetzen
% You need to use \var{\{\$MODE OBJFPC\}} or \var{\{\$MODE DELPHI\}} to compile this file.
% Or use the corresponding command line switch, either \var{-Mobjfpc} or \var{-MDelphi.}
parser_e_no_export_with_index_for_target=03163_E_Der Index darf nicht kleiner als $1 sein
% Exporting of functions or procedures with a specified index is not
% supported on this target.
parser_e_no_export_of_variables_for_target=03164_E_Das Ziel $1 unterstützt das Exportieren von Variablen nicht
% Exporting of variables is not supported on this target.
parser_e_improper_guid_syntax=03165_E_Falscher GUID-Syntax
% The GUID indication does not have the proper syntax. It should be of the form
% \begin{verbatim}
% {XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}
% \end{verbatim}
% Where each \var{X} represents a hexadecimal digit.
parser_w_interface_mapping_notfound=03168_W_Eine Prozedur mit dem Namen "$1", die $2.$3 implementieren könnte, kann nicht gefunden werden
% The compiler cannot find a suitable procedure which implements the given method of an interface.
% A procedure with the same name is found, but the arguments do not match.
parser_e_interface_id_expected=03169_E_Interface-Bezeichner erwartet
% This happens when the compiler scans a \var{class} declaration that contains
% \var{interface} function name mapping code like this:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% and the \var{interface} before the dot is not listed in the inheritance list.
parser_e_type_cant_be_used_in_array_index=03170_E_Der Typ "$1" kann nicht als Array-Index verwendet werden
% Types like \var{qword} or \var{int64} are not allowed as array index type.
parser_e_no_con_des_in_interfaces=03171_E_Kon- und Destruktoren sind in Interfaces nicht erlaubt
% Constructor and destructor declarations are not allowed in interfaces.
% In the most cases method \var{QueryInterface} of \var{IUnknown} can
% be used to create a new interface.
parser_e_no_access_specifier_in_interfaces=03172_E_Zugriffsbezeichner können in INTERFACEs und OBJCPROTOCOLs nicht benutzt werden
% The access specifiers \var{public}, \var{private}, \var{protected} and
% \var{published} cannot be used in interfaces, Objective-C protocols and categories because all methods
% of an interface/protocol/category must be public.
parser_e_no_vars_in_interfaces=03173_E_Ein Interface, ein Helfer, ein Objective-C Protokoll oder eine Kategorie darf keine Felder enthalten
% Declarations of fields are not allowed in interfaces, helpers and Objective-C protocols and categories.
% An interface/helper/protocol/category can contain only methods and properties with method read/write specifiers.
parser_e_no_local_proc_external=03174_E_Eine lokale Prozedur kann nicht als EXTERNAL deklariert werden
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance of errors very high.
parser_w_skipped_fields_before=03175_W_Einige Felder vor dem Feld "$1" sind nicht initialisiert
% In Delphi mode, not all fields of a typed constant record have to be
% initialized, but the compiler warns you when it detects such situations.
parser_e_skipped_fields_before=03176_E_Einige Felder vor dem Feld "$1" sind nicht initialisiert
% In all syntax modes but Delphi mode, you cannot leave some fields uninitialized
% in the middle of a typed constant record.
parser_w_skipped_fields_after=03177_W_Einige Felder nach dem Feld "$1" sind nicht initialisiert
% You can leave some fields at the end of a type constant record uninitialized
% (The compiler will initialize them to zero automatically). This may be the cause
% of subtle problems.
parser_e_varargs_need_cdecl_and_external=03178_E_Verwendung von VarArgs (oder '...' in MacPas) ohne CDecl/CPPDecl/MWPascal/StdCall oder External nicht möglich
% The varargs directive (or the ``...'' varargs parameter in MacPas mode) can only be
% used with procedures or functions that are declared with \var{external} and one of
% \var{cdecl}, \var{cppdecl}, \var{stdcall} and \var{mwpascal}. This functionality
% is only supported to provide a compatible interface to C functions like printf.
parser_e_self_call_by_value=03179_E_Self muss ein Call-By-Value-Parameter sein
% You cannot declare \var{Self} as a const or var parameter, it must always be
% a call-by-value parameter.
parser_e_interface_has_no_guid=03180_E_Interface "$1" hat keine Interface-Identifikation
% When you want to assign an interface to a constant, then the interface
% must have a GUID value set.
parser_e_illegal_field_or_method=03181_E_Klassenfeld oder Methoden-Bezeichner "$1" unbekannt
% Properties must refer to a field or method in the same class.
parser_w_proc_overriding_calling=03182_W_Überschreibe die Calling Convention "$1" mit "$2"
% There are two directives in the procedure declaration that specify a calling
% convention. Only the last directive will be used.
parser_e_no_procvarobj_const=03183_E_Die Konstante des Typs "procedure of object" kann nur mit NIL initialisiert werden
% You cannot assign the address of a method to a typed constant which has a
% 'procedure of object' type, because such a constant requires two addresses:
% that of the method (which is known at compile time) and that of the object or
% class instance it operates on (which cannot be known at compile time).
parser_e_default_value_only_one_para=03184_E_Der Default Value kann nur einem Parameter zugewiesen werden
% It is not possible to specify a default value for several parameters at once.
% The following is invalid:
% \begin{verbatim}
% Procedure MyProcedure (A,B : Integer = 0);
% \end{verbatim}
% Instead, this should be declared as
% \begin{verbatim}
% Procedure MyProcedure (A : Integer = 0; B : Integer = 0);
% \end{verbatim}
parser_e_default_value_expected_for_para=03185_E_Standard Parameter für "$1" benötigt
% The specified parameter requires a default value.
parser_w_unsupported_feature=03186_W_Verwendung eines nicht unterstützten Features!
% You're trying to force the compiler into doing something it cannot do yet.
parser_h_c_arrays_are_references=03187_H_C Arrays werden "by reference" übergeben
% Any array passed to a C function is passed
% by a pointer (i.e. by reference).
parser_e_C_array_of_const_must_be_last=03188_E_C array of const muss letztes Argument sein
% You cannot add any other argument after an \var{array of const} for
% \var{cdecl} functions, as the size pushed on stack for this argument is
% not known.
parser_h_type_redef=03189_H_Erneute Definition des Typs "$1"
% This is an indicator that a previously declared type is
% being redefined as something else. This may, or may not
% be, a potential source of errors.
parser_w_cdecl_has_no_high=03190_W_cdecl'ared Functionen haben keinen high Parameter
% Functions declared with the \var{cdecl} modifier do not pass an extra implicit parameter.
parser_w_cdecl_no_openstring=03191_W_cdecl'ared Functionen unterstützen keine open strings
% Openstring is not supported for functions that have the \var{cdecl} modifier.
parser_e_initialized_not_for_threadvar=03192_E_Als threadvar deklarierte Variable kann nicht initialisiert werden
% Variables declared as threadvar cannot be initialized with a default value.
% The variables will always be filled with zero at the start of a new thread.
parser_e_msg_only_for_classes=03193_E_Message directive ist nur in Klassen erlaubt
% The message directive is only supported for Class types.
parser_e_procedure_or_function_expected=03194_E_Procedure oder Function erwartet
% A class method can only be specified for procedures and functions.
parser_e_illegal_calling_convention=03195_W_Calling convention directive ignoriert: "$1"
% Some calling conventions are supported only by certain CPUs. I.e. most non-i386 ports support
% only the standard ABI calling convention of the CPU.
parser_e_no_object_reintroduce=03196_E_REINTRODUCE kann in Objekten nicht benutzt werden
% \var{reintroduce} is not supported for objects, Objective-C classes and Objective-C protocols.
parser_e_paraloc_only_one_para=03197_E_Jedes Argument muss seine eigene "location" haben
% If locations for arguments are specified explicitly as it is required by
% some syscall conventions, each argument must have its own location. Things
% like
% \begin{verbatim}
% procedure p(i,j : longint 'r1');
% \end{verbatim}
% are not allowed.
parser_e_paraloc_all_paras=03198_E_Jedes Argument muss seine explizite "location" haben
% If one argument has an explicit argument location, all arguments of a procedure
% must have one.
parser_e_illegal_explicit_paraloc=03199_E_"location" des Arguments unbekannt
% The location specified for an argument isn't recognized by the compiler.
parser_e_32bitint_or_pointer_variable_expected=03200_E_32 Bit-Integer oder Zeiger-Variable erwartet
% The libbase for MorphOS/AmigaOS can be given only as \var{longint}, \var{dword} or any pointer variable.
parser_e_goto_outside_proc=03201_E_Goto Statements zwischen verschiedenen Prozeduren sind nicht erlaubt
% It isn't allowed to use \var{goto} statements referencing labels outside the
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
parser_f_too_complex_proc=03202_F_Prozedur zu komplex, sie erfordert zu viele Register
% Your procedure body is too long for the compiler. You should split the
% procedure into multiple smaller procedures.
parser_e_illegal_expression=03203_E_Ungültiger Ausdruck
% This can occur under many circumstances. Usually when trying to evaluate
% constant expressions.
parser_e_invalid_integer=03204_E_Ungültiger integer Ausdruck
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
parser_e_invalid_qualifier=03205_E_Ungültiger Qualifier
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
parser_e_upper_lower_than_lower=03206_E_High Range Limit < low Range Limit
% You are declaring a subrange, and the high limit is less than the low limit of
% the range.
parser_e_macpas_exit_wrong_param=03207_E_Exits Parameter muss der Name der Prozedur sein, in der es benutzt wird oder einer übergeordneten Prozedur
% The parameter of a exit call in macpas mode must be either the name of the current subroutine or of a surrounding one
parser_e_illegal_assignment_to_count_var=03208_E_Ungültige Zuweisung zur for-loop Variable "$1"
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings. You also cannot assign values to
% loop variables inside the loop (Except in Delphi and TP modes). Use a while or
% repeat loop instead if you need to do something like that, since those
% constructs were built for that.
parser_e_no_local_var_external=03209_E_Lokale Variable kann nicht als EXTERNAL deklariert werden
% Declaring local variables as external is not allowed. Only global variables can reference
% external variables.
parser_e_proc_already_external=03210_E_Prozedur ist bereits als EXTERNAL deklariert
% The procedure is already declared with the EXTERNAL directive in an interface or
% forward declaration.
parser_w_implicit_uses_of_variants_unit=03211_W_Implizite Verwendung einer Variant Unit
% The Variant type is used in the unit without any used unit using the Variants unit. The
% compiler has implicitly added the Variants unit to the uses list. To remove this warning
% the Variants unit needs to be added to the uses statement.
parser_e_no_static_method_in_interfaces=03212_E_Class und statische Methoden können nicht in INTERFACES verwendet werden
% The specifier \var{class} and directive \var{static} cannot be used in interfaces
% because all methods of an interface must be public.
parser_e_arithmetic_operation_overflow=03213_E_Überlauf in arithmetischer Operation
% An operation on two integer values produced an overflow.
parser_e_protected_or_private_expected=03214_E_Protected oder private erwartet
% \var{strict} can be only used together with \var{protected} or \var{private}.
parser_e_illegal_slice=03215_E_SLICE kann nicht außerhalb der Parameterliste benutzt werden
% \var{slice} can be used only for arguments accepting an open array parameter.
parser_e_dispinterface_cant_have_parent=03216_E_Ein DISPINTERFACE kann keine Elternklasse haben.
% A DISPINTERFACE is a special type of interface which cannot have a parent class. Dispinterface always derive from IDispatch type.
parser_e_dispinterface_needs_a_guid=03217_E_Ein DISPINTERFACE benötigt einen GUID
% A DISPINTERFACE always needs an interface identification (a GUID).
parser_w_overridden_methods_not_same_ret=03218_W_Überschriebene Methoden müssen einen entsprechenden Rückgabetyp haben. Dieser Code kann abstürzen, weil er von einem Delphi Parser Bug abhängt (Methode "$2" wird durch "$1" überschrieben, die einen anderen Rückgabetyp hat).
% If you declare overridden methods in a class definition, they must
% have the same return type. Some versions of Delphi allow you to change the
% return type of interface methods, and even to change procedures into
% functions, but the resulting code may crash depending on the types used
% and the way the methods are called.
parser_e_dispid_must_be_ord_const=03219_E_Dispatch IDs müssen ganzzahlige Konstanten sein
% The \var{dispid} keyword must be followed by an ordinal constant (the dispid index).
parser_e_array_range_out_of_bounds=03220_E_Der Bereich des Array ist zu groß
% Regardless of the size taken up by its elements, an array cannot have more
% than high(ptrint) elements. Additionally, the range type must be a subrange
% of ptrint.
parser_e_packed_element_no_var_addr=03221_E_Bit packed Array-Elemente und Record-Felder haben keine Adressen
% If you declare an array or record as \var{packed} in Mac Pascal mode
% (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. This means it becomes impossible to take addresses
% of individual array elements or record fields. The only exception to this rule
% is in the case of packed arrays elements whose packed size is a multiple of 8 bits.
parser_e_packed_dynamic_open_array=03222_E_Dynamische Arrays können nicht packed sein
% Only regular (and possibly in the future also open) arrays can be packed.
parser_e_packed_element_no_loop=03223_E_Bit packed Array-Elemente und Record-Felder können nicht als Schleifenvariable verwerndet werden
% If you declare an array or record as \var{packed} in Mac Pascal mode
% (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. For performance reasons, they cannot be
% used as loop variables.
parser_e_type_var_const_only_in_records_and_classes=03224_E_VAR, TYPE und CONST sind nur innerhalb Records, Objekten und Klassen erlaubt
% The usage of VAR, TYPE and CONST to declare new types inside an object is allowed only inside
% records, objects and classes.
parser_e_cant_create_generics_of_this_type=03225_E_Dieser Typ kann nicht "generic" sein
% Only Classes, Objects, Interfaces and Records are allowed to be used as generic.
parser_w_no_lineinfo_use_switch=03226_W_Die LINEINFO Unit nicht manuell laden. Verwende statt dessen den Compilerschalter -gl
% Do not use the \file{lineinfo} unit directly, Use the \var{-gl} switch which
% automatically adds the correct unit for reading the selected type of debugging
% information. The unit that needs to be used depends on the type of
% debug information used when compiling the binary.
parser_e_no_funcret_specified=03227_E_Kein Funktionsergebnistyp für Funktion "$1" angegeben
% The first time you declare a function you have to declare it completely,
% including all parameters and the result type.
parser_e_special_onlygenerics=03228_E_Spezialisierung wird nur für generische Typen unterstützt
% Types which are not generics cannot be specialized.
parser_e_no_generics_as_params=03229_E_Generische Typen können bei der Spezialisierung generischer Typen nicht als Parameter benutzt werden
% When specializing a generic, only non-generic types can be used as parameters.
parser_e_type_object_constants=03230_E_Konstanten eines Objekts, das ein VMT enthält, sind unzulässig
% If an object requires a VMT either because it contains a constructor or virtual methods,
% it's not allowed to create constants of it. In TP and Delphi mode this is allowed
% for compatibility reasons.
parser_e_label_outside_proc=03231_E_Die Address von Labels, die außerhalb des aktuellen Scopes definiert wurden, können nicht verwendet werden
% It isn't allowed to take the address of labels outside the
% current procedure.
parser_e_initialized_not_for_external=03233_E_Extern deklarierte Variablen können nicht intialisiert werden 
% Variables declared as external cannot be initialized with a default value.
parser_e_illegal_function_result=03234_E_Ungültiger Funktionsergebnistyp
% Some types like file types cannot be used as function result.
parser_e_no_common_type=03235_E_"$1" und "$2" haben keinen gemeinsamen Typ
% To perform an operation on integers, the compiler converts both operands
% to their common type, which appears to be an invalid type. To determine the
% common type of the operands, the compiler takes the minimum of the minimal values
% of both types, and the maximum of the maximal values of both types. The common
% type is then minimum..maximum.
parser_e_no_generics_as_types=03236_E_Generische Typen können nicht ohne Spezialisierung als Typ für eine Variable verwendet werden
% Generics must be always specialized before being used as variable type.
parser_w_register_list_ignored=03237_W_Registerliste wird in reinen Assemblerroutinen ignoriert
% When using pure assembler routines, the list with modified registers is ignored.
parser_e_implements_must_be_class_or_interface=03238_E_Die Implements-Eigenschaft muss einen Klassen- oder Interface-Typ haben
% A property which implements an interface must be of type class or interface.
parser_e_implements_must_have_correct_type=03239_E_Die Implements-Eigenschaft muss ein Interface mit dem richtigen Typ implementieren; es wurde Typ "$1" gefunden, aber Typ "$2" erwartet.
% A property which implements an interface actually implements a different interface.
parser_e_implements_must_read_specifier=03240_E_Die Implements-Eigenschaft muss einen Lesen-Bezeichner haben
% A property  which implements an interface must have at least a read specifier.
parser_e_implements_must_not_have_write_specifier=03241_E_Die Implements-Eigenschaft darf keinen Schreib-Bezeichner haben
% A property  which implements an interface may not have a write specifier.
parser_e_implements_must_not_have_stored_specifier=03242_E_Die Implements-Eigenschaft darf keinen Gespeichert-Bezeichner haben
% A property  which implements an interface may not have a stored specifier.
parser_e_implements_uses_non_implemented_interface=03243_E_Die Implements-Eigenschaft benutzt das nicht implementierte Interface "$1"
% The interface which is implemented by a property is not an interface implemented by the class.
parser_e_unsupported_real=03244_E_Fliesskommavariablen werden für dieses Ziel nicht unterstützt
% The compiler parsed a floating point expression, but it is not supported.
parser_e_class_doesnt_implement_interface=03245_E_Klasse "$1" implementiert das Interface "$2" nicht
% The delegated interface is not implemented by the class given in the implements clause.
parser_e_class_implements_must_be_interface=03246_E_Der von Implements benutzte Typ muss ein Interface sein
% The \var{implements} keyword must be followed by an interface type.
parser_e_cant_export_var_different_name=03247_E_Variablen können für dieses Target nicht mit einem anderen Namen exportiert werden; füge der Deklaration den Namen mit einer "export" Direktive hinzu (Variablenname: $1, deklarierter Name für den Export: $2)
% On most targets it is not possible to change the name under which a variable
% is exported inside the \var{exports} statement of a library.
% In that case, you have to specify the export name at the point where the
% variable is declared, using the \var{export} and \var{alias} directives.
parser_e_weak_external_not_supported=03248_E_Schwache externe Symbole werden für dieses Target nicht unterstützt
% A "weak external" symbol is a symbol which may or may not exist at (either static
% or dynamic) link time. This concept may not be available (or implemented yet)
% on the current cpu/OS target.
parser_e_forward_mismatch=03249_E_Forward Typdefinition passt nicht
% Classes and interfaces being defined forward must have the same type
% when being implemented. A forward interface cannot be changed into a class.
parser_n_ignore_lower_visibility=03250_N_Die virtuelle Methode "$1" hat eine niedrigere Sichtbarkeit ($2) als die Elternklasse $3 ($4)
% The virtual method overrides an method that is declared with a higher visibility. This might give
% unexpected results. E.g., in case the new visibility is private then a call to ``inherited'' in a
% new child class will call the higher-visible method in a parent class and ignores the private method.
parser_e_field_not_allowed_here=03251_E_Felder sind nach der Definition einer Methode oder Eigenschaft nicht erlaubt. Beginne vorher eine neue Sichtbarkeitssektion
% Once a method or property has been defined in a class or object, you cannot define any fields afterwards
% without starting a new visibility section (such as \var{public}, \var{private}, etc.). The reason is
% that otherwise the source code can appear ambiguous to the compiler, since it is possible to use modifiers
% such as \var{default} and \var{register} also as field names.
parser_e_no_local_para_def=03252_E_Parameter oder Ergebnistypen können keine lokalen Typdeklarationen enthalten. Verwende eine getrennte Typdefinition in einem "type"-Block
% In Pascal, types are not considered to be identical simply because they are semantically equivalent.
% Two variables or parameters are only considered to be of the same type if they refer to the
% same type definition.
% As a result, it is not allowed to define new types inside parameter lists, because then it is impossible to
% refer to the same type definition in the procedure headers of the interface and implementation of a unit
% (both procedure headers would define a separate type). Keep in mind that expressions such as
% ``file of byte'' or ``string[50]'' also define a new type.
parser_e_abstract_and_sealed_conflict=03253_E_Konflikt zwischen ABSTRACT und SEALED 
% ABSTRACT and SEALED cannot be used together in one declaration
parser_e_sealed_descendant=03254_E_Kann keinen Nachfahren der SEALED Klasse "$1" erzeugen
% Sealed means that class cannot be derived by another class.
parser_e_sealed_class_cannot_have_abstract_methods=03255_E_Eine SEALED Klasse kann keine ABSTRACT Methode haben
% Sealed means that class cannot be derived. Therefore no one class is able to override an abstract method in a sealed class.
parser_e_only_virtual_methods_final=03256_E_Nur virtuelle Methoden können final sein.
% You are declaring a method as final, when it is not declared to be
% virtual.
parser_e_final_can_no_be_overridden=03257_E_Die finale Methode kann nicht überschrieben werden: "$1"
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_multiple_messages=03258_E_Pro Methode kann nur eine Nachricht verwendet werden
% It is not possible to associate multiple messages with a single method.
parser_e_invalid_enumerator_identifier=03259_E_Ungültiger Aufzählungsbezeichner: "$1"
% Only "MoveNext" and "Current" enumerator identifiers are supported.
parser_e_enumerator_identifier_required=03260_E_Aufzählungsbezeichner notwendig
% "MoveNext" or "Current" identifier must follow the \var{enumerator} modifier.
parser_e_enumerator_movenext_is_not_valid=03261_E_Die Aufzählungs-pattern-Methode "MoveNext" ist ungültig. Die Methode muss eine Funktion mit Rückgabetyp Boolean und ohne notwendige Argumente sein
% "MoveNext" enumerator pattern method must be a function with Boolean return type and no required arguments
parser_e_enumerator_current_is_not_valid=03262_E_Die Aufzählungs-pattern-Eigenschaft "Current" ist ungültig. Die Eigenschaft benötigt einen "Getter"
% "Current" enumerator pattern property must have a getter
parser_e_only_one_enumerator_movenext=03263_E_Pro Klasse/Objekt ist nur eine Aufzählungsmethode "MoveNext" erlaubt
% Class or Object can have only one enumerator MoveNext declaration.
parser_e_only_one_enumerator_current=03264_E_Pro Klasse/Objekt ist nur eine Aufzählungseigenschaft "Current" erlaubt
% Class or Object can have only one enumerator Current declaration.
parser_e_for_in_loop_cannot_be_used_for_the_type=03265_E_For in Schleife kann nicht für den Typ "$1" verwendet werden
% For in loop can be used not for all types. For example it cannot be used for the enumerations with jumps.
parser_e_objc_requires_msgstr=03266_E_Objective-C Nachrichten erfordern, dass ihr Objective-C selector-Name mit der Direktive "message" angegeben wird
% Objective-C messages require their Objective-C name (selector name) to be specified using the \var{message `someName:'} procedure directive.
% While bindings to other languages automatically generate such names based on the identifier you use (by replacing
% all underscores with colons), this is unsafe since nothing prevents an Objective-C method name to contain actual
% colons.
parser_e_objc_no_constructor_destructor=03267_E_Objective-C hat keine formalen Konstruktoren oder Destruktoren. Verwende die Nachrichten alloc, initXXX und dealloc
% The Objective-C language does not have any constructors or destructors. While there are some messages with a similar
% purpose (such as \var{init} and \var{dealloc}), these cannot be identified using automatic parsers and do not
% guarantee anything like Pascal constructors/destructors (e.g., you have to take care of only calling ``designated''
% inherited ``constructors''). For these reasons, we have opted to follow the standard Objective-C patterns for
% instance creation/destruction.
parser_e_message_string_too_long=03268_E_Der Name der Nachricht ist zu lang (max. 255 Zeichen)
% Due to compiler implementation reasons, message names are currently limited to 255 characters.
parser_e_objc_message_name_too_long=03269_E_Der Symbolname der Objective-C-Nachricht "$1" ist zu lang
% Due to compiler implementation reasons, mangled message names (i.e., the symbol names used in the assembler
% code) are currently limited to 255 characters.
parser_h_no_objc_parent=03270_H_Definieren einer neuen Objective-C root-Klasse. Um sie von einer anderen root-Klasse abzuleiten (z.B. NSObject), gib diese als Elternklasse an
% If no parent class is specified for an Object Pascal class, then it automatically derives from TObject.
% Objective-C classes however do not automatically derive from NSObject, because one can have multiple
% root classes in Objective-C. For example, in the Cocoa framework both NSObject and NSProxy are root classes.
% Therefore, you have to explicitly define a parent class (such as NSObject) if you want to derive your
% Objective-C class from it.
parser_e_no_objc_published=03271_E_Objective-C Klassen können keinen Abschnitt published haben
% In Object Pascal, ``published'' determines whether or not RTTI is generated. Since the Objective-C runtime always needs
% RTTI for everything, this specified does not make sense for Objective-C classes.
parser_f_need_objc=03272_F_Dieses Modul erfordert, dass der Objective-C Mode-Schalter übersetzt wird
% This error indicates the use of Objective-C language features without an Objective-C mode switch
% active. Enable one via the -M command line switch, or the {\$modeswitch x} directive.
parser_e_must_use_override=03273_E_Vererbte Methoden können nur in Objective-C und Java überschrieben werden, füge "override" hinzu (Vererbte Methode ist in $1 definiert)
parser_h_should_use_override=03274_H_Vererbte Methoden können nur in Objective-C und Java überschrieben werden, füge "override" hinzu (Vererbte Methode ist in $1 definiert)
% It is not possible to \var{reintroduce} methods in Objective-C or Java like in Object Pascal. Methods with the same
% name always map to the same virtual method entry. In order to make this clear in the source code,
% the compiler always requires the \var{override} directive to be specified when implementing overriding
% Objective-C or Java methods in Pascal. If the implementation is external, this rule is relaxed because Objective-C and Java
% do not have any \var{override}-style keyword (since it's the default and only behaviour in these languages),
% which makes it hard for automated header conversion tools to include it everywhere.
% The type in which the inherited method is defined is explicitly mentioned, because this may either
% be an objcclass or an objccategory in case of Objective-C.
parser_e_objc_message_name_changed=03275_E_Der Nachrichtenname "$1" in der vererbten Klasse unterscheidet sich vom Nachrichtennamen "$2" in der aktuellen Klasse
% An overriding Objective-C method cannot have a different message name than an inherited method. The reason
% is that these message names uniquely define the message to the Objective-C runtime, which means that
% giving them a different message name breaks the ``override'' semantics.
parser_e_unique_unsupported=03276_E_Noch können eindeutige Kopien von Objective-C oder Java Typen nicht erstellt werden
% Duplicating an Objective-C or Java type using \var{type x = type y;} is not yet supported. You may be able to
% obtain the desired effect using \var{type x = objcclass(y) end;} resp.{} \var{type x = class(y) end;} instead.
parser_e_no_category_as_types=03277_E_Objective-C Kategorien und Object-Pascal Klassenhelfer können nicht als Typen benutzt werden
% It is not possible to declare a variable as an instance of an Objective-C
% category or an Object Pascal class helper. A category/class helper adds
% methods to the scope of an existing class, but does not define a type by
% itself. An exception of this rule is when inheriting an Object Pascal class
% helper from another class helper.
parser_e_no_category_override=03278_E_Kategorien überschreiben Methoden nicht, sondern ersetzen sie. "reintroduce" benutzen
parser_e_must_use_reintroduce_objc=03279_E_Ersetzte Methoden können in Objective-C nur wieder eingeführt werden, füge "reintroduce" hinzu (Ersetzte Methode ist in $1 definiert)
parser_h_should_use_reintroduce_objc=03280_H_Ersetzte Methoden können in Objective-C nur wieder eingeführt werden, füge "reintroduce" hinzu (Ersetzte Methode ist in $1 definiert)
% A category replaces an existing method in an Objective-C class, rather than that it overrides it.
% Calling an inherited method from an category method will call that method in
% the extended class' parent, not in the extended class itself. The
% replaced method in the original class is basically lost, and can no longer be
% called or referred to. This behaviour corresponds somewhat more closely to
% \var{reintroduce} than to \var{override} (although in case of \var{reintroduce}
% in Object Pascal, hidden methods are still reachable via inherited).
% The type in which the inherited method is defined is explicitly mentioned, because this may either
% be an objcclass or an objccategory.
parser_e_implements_getter_not_default_cc=03281_E_Getter für das Interface implements müssen die voreingestellte calling convention des Ziels benutzen
% Interface getters are called via a helper in the run time library, and hence
% have to use the default calling convention for the target (\var{register} on
% i386 and x86\_64, \var{stdcall} on other architectures).
parser_e_no_refcounted_typed_file=03282_E_Typisierte Dateien können keine reference-counted Typen enthalten
% The data in a typed file cannot be of a reference counted type (such as
% \var{ansistring} or a record containing a field that is reference counted).
parser_e_operator_not_overloaded_2=03283_E_Überladenener Operator nicht vorhanden: $2 "$1"
% You are trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_operator_not_overloaded_3=03284_E_Überladenener Operator nicht vorhanden: "$1" $2 "$3"
% You are trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_more_array_elements_expected=03285_E_Erwarte ein weiteres Element für Array $1 
% When declaring a typed constant array, you provided to few elements to initialize the array
parser_e_string_const_too_long=03286_E_Stringkonstante zu lang, so lange ansistrings ausgeschaltet sind
% Only when a piece of code is compiled with ansistrings enabled (\var{\{\$H+\}}), string constants
% longer than 255 characters are allowed.
parser_e_invalid_univ_para=03287_E_Typ kann nicht als "univ" Parameter verwendet werden, weil seine Größe zur Zeit der Übersetzung unbekannt ist: "$1"
% \var{univ} parameters are compatible with all values of the same size, but this
% cannot be checked in case a parameter's size is unknown at compile time.
parser_e_only_one_class_constructor_allowed=03288_E_In der Klasse "$1" darf nur ein Klassenkonstruktor deklariert werden
% You are trying to declare more than one class constructor but only one class constructor can be declared.
parser_e_only_one_class_destructor_allowed=03289_E_In der Klasse "$1" darf nur ein Klassendestruktor deklariert werden
% You are trying to declare more than one class destructor but only one class destructor can be declared.
parser_e_no_paras_for_class_constructor=03290_E_Ein Klassenkonstruktur darf keine Parameter haben
% You are declaring a class constructor with a parameter list. Class constructor methods
% cannot have parameters.
parser_e_no_paras_for_class_destructor=03291_E_Ein Klassendestruktur darf keine Parameter haben
% You are declaring a class destructor with a parameter list. Class destructor methods
% cannot have parameters.
parser_f_modeswitch_objc_required=03292_F_Dieses Konstrukt erfordert, dass der Modenschalter \{\$modeswitch objectivec1\} aktiv ist
% Objective-Pascal constructs are not supported when \{\$modeswitch ObjectiveC1\}
% is not active.
parser_e_widestring_to_ansi_compile_time=03293_E_Eine Unicodechar/string Konstante zur Zeit der Übersetzung nicht in eine ansi/shortstring Konstante konvertiert werden
% It is not possible to use unicodechar and unicodestring constants in
% constant expressions that have to be converted into an ansistring or shortstring
% at compile time, for example inside typed constants. The reason is that the
% compiler cannot know what the actual ansi encoding will be at run time.
parser_e_objc_enumerator_2_0=03294_E_Objective-Pascal For-in Schleifen erfordern, dass der Modenschalter \{\$modeswitch ObjectiveC2\} aktiv ist
% Objective-C ``fast enumeration'' support was added in Objective-C 2.0, and
% hence the appropriate modeswitch has to be activated to expose this feature.
% Note that Objective-C 2.0 programs require Mac OS X 10.5 or later.
parser_e_objc_missing_enumeration_defs=03295_E_Der Kompiler findet die Typen NSFastEnumerationProtocol oder NSFastEnumerationState nicht in der Unit CocoaAll
% Objective-C for-in loops (fast enumeration) require that the compiler can
% find a unit called CocoaAll that contains definitions for the
% NSFastEnumerationProtocol and NSFastEnumerationState types. If you get this
% error, most likely the compiler is finding and loading an alternate CocoaAll
% unit.
parser_e_no_procvarnested_const=03296_E_Typisierte Konstanten des Typs 'procedure is nested' können nur mit NIL und globalen Prozeduren/Funktionen initialisiert werden
% A nested procedural variable consists of two components: the address of the
% procedure/function to call (which is always known at compile time), and also
% a parent frame pointer (which is never known at compile time) in case the
% procedural variable contains a reference to a nested procedure/function.
% Therefore such typed constants can only be initialized with global
% functions/procedures since these do not require a parent frame pointer.
parser_f_no_generic_inside_generic=03297_F_Die Deklaration von etwas generischem  innerhalb etwas anderem generischen ist nicht erlaubt
% At the moment, scanner supports recording of only one token buffer at the time
% (guarded by internal error 200511173 in tscannerfile.startrecordtokens).
% Since generics are implemented by recording tokens, it is not possible to
% have declaration of a generic (type or method) inside another generic.
parser_e_forward_intf_declaration_must_be_resolved=03298_E_Die Vorwärts-Deklaration "$1" muss aufgelöst sein, bevor eine Klasse ihr folgen oder sie implementieren kann
% An Objective-C protocol or Java Interface must be fully defined before classes can conform to it.
% This error occurs in the following situation (example for Objective-C, but the same goes for Java interfaces):
% \begin{verbatim}
%  Type MyProtocol = objcprotoocl;
%       ChildClass = Class(NSObject,MyProtocol)
%         ...
%       end;
% \end{verbatim}
% where \var{MyProtocol} is declared but not defined.
parser_e_no_record_published=03299_E_Record -Typen können keine öffentlichen Abschnitte (published sections) haben
% Published sections can be used only inside classes.
parser_e_no_destructor_in_records=03300_E_Destruktoren sind in Records und Helfern nicht erlaubt
% Destructor declarations are not allowed in records or helpers.
parser_e_class_methods_only_static_in_records=03301_E_Klassenmethoden müssen in Records statisch sein
% Class methods declarations are not allowed in records without static modifier.
% Records have no inheritance and therefore non static class methods have no sense for them.
parser_e_no_parameterless_constructor_in_records=03302_E_Konstruktoren ohne Parameter sind in Records und Record/Typhelfern nicht erlaubt
% Constructor declarations with no arguments aren't allowed in records or record/type helpers.
parser_e_at_least_one_argument_must_be_of_type=03303_E_Entweder das Ergebnis oder mindestens ein Parameter müssen vom Typ "$1" sein
% It is required that either the result of the routine or at least one of its parameters be of the specified type.
% For example class operators either take an instance of the structured type in which they are defined, or they return one.
parser_e_cant_use_type_parameters_here=03304_E_Typ-Parameter können initialization/finalization erfordern - Sie können deshalb nicht in varianten Records verwendet werden
% Type parameters may be specialized with types which (e.g. \var{ansistring}) need initialization/finalization
% code which is implicitly generated by the compiler. 
parser_e_externals_no_section=03305_E_"external" deklarierte Variablen dürfen nicht in einer "custom section" sein
% A section directive is not valid for variables being declared as external.
parser_e_section_no_locals=03306_E_Nicht-statische und nicht-globale Variablen dürfen keine Direktive "section" haben
% A variable placed in a custom section is always statically allocated so it must be either a static or global variable.
parser_e_not_allowed_in_helper=03307_E_"$1" ist in Helfertypen nicht erlaubt
% Some directives and specifiers like "virtual", "dynamic", "override" are not
% allowed inside helper types in mode ObjFPC (they are ignored in mode Delphi),
% because they have no meaning within helpers. Also "abstract" isn't allowed in
% either mode.
parser_e_no_class_constructor_in_helpers=03308_E_Klassenkonstruktoren sind in Helfern nicht erlaubt
% Class constructor declarations are not allowed in helpers.
parser_e_inherited_not_in_record=03309_E_"inherited" ist in einem Record nicht erlaubt
% As records don't support inheritance the use of "inherited" is prohibited for
% these as well as for record helpers (in mode "Delphi" only).
parser_e_no_types_in_local_anonymous_records=03310_E_Typ-Deklarationen sind in lokalen oder anonymen Records nicht erlaubt
% Records with types must be defined globally. Types cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_duplicate_implements_clause=03311_E_Zweifacher "implements"-Term für das Interface "$1"
% A class may delegate an interface using the "implements" clause only to a single property. Delegating it multiple times
% is a error.
parser_e_mapping_no_implements=03312_E_Das Interface "$1" kann nicht durch "$2" delegiert werden. Die Methode ist bereits aufgelöst
% Method resolution clause maps a method of an interface to a method of the current class. Therefore the current class
% has to implement the interface directly. Delegation is not possible.
parser_e_implements_no_mapping=03313_E_Das Interface "$1" kann keine Methoden-Auflösung haben, "$2" delegiert es bereits
% Method resolution is only possible for interfaces that are implemented directly, not by delegation.
parser_e_invalid_codepage=03314_E_Ungültige Codepage
% When declaring a string with a given codepage, the range of valid codepages values is limited
% to 0 to 65535.
parser_e_final_only_const_var=03315_E_Nur Felder (var-Sektionen) und Konstanten können in Objekt-Typen final sein
% A final (class) field must be assigned a single value in the (class) constructor, and cannot
% be overwritten afterwards. A final (typed) constant is read-only.
parser_e_final_only_external=03316_E_Finale Felder werden derzeit nur in externen Klassen unterstützt
% Support for final fields in non-external classes requires a full data flow
% analysis implementation in FPC, which it currently still lacks.
parser_e_no_typed_const=03317_E_Typisierte Konstanten sind hier nicht erlaubt, nur formale Konstanten
% Java interfaces define a namespace in which formal constant can be defined,
% but since they define no storage it is not possible to define typed constants
% in them (those are more or less the same as initialised class fields).
parser_e_java_no_inherited_constructor=03318_E_Konstruktoren werden in der JVM nicht automatisch geerbt; füge einen Konstruktur explizit hinzu, der, falls nötig, den geerbten aufruft
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% For compatibility with external Java code, FPC does the same. If you require access to the same
% constructors in a child class, define them in the child class and call the inherited one from
% there.
parser_d_internal_parser_string=03319_D_Parsen intern erzeugten Codes: $1
% The compiler sometimes internally constructs Pascal code that is subsequently
% injected into the program. These messages display such code, in order to help
% with debugging errors in them.
parser_e_feature_unsupported_for_vm=03320_E_Dieses Feature wird auf managed VM-Zielen nicht unterstützt
% Certain language features are not supported on targets that are managed virtual machines.
parser_e_jvm_invalid_virtual_constructor_call=03321_E_Einen virtuellen Konstruktor für die aktuelle Instanz innerhalb eines anderen Konstruktors aufzurufen, ist für das JVM-Ziel nicht möglich
% The JVM does not natively support virtual constructor. Unforunately, we are not aware of a way to
% emulate them in a way that makes it possible to support calling virtual constructors
% for the current instance inside another constructor.
parser_e_method_lower_visibility=03322_E_Die überschreibende Method "$1" kann keine niedrigere Sichtbarkeit ($2) haben als in der Elternklasse $3 ($4)
% The JVM does not allow lowering the visibility of an overriding method.
parser_e_nostackframe_without_assembler=03323_E_Prozedur/Funktion  mit der Aufrufoption NOSTACKFRAME aber ohne ASSEMBLER deklariert 
% nostackframe call modifier is supposed to be used in conjunction with assembler.
parser_e_nostackframe_with_locals=03324_E_Prozedur/Funktion  mit der Aufrufoption NOSTACKFRAME deklariert, aber die Größe des lokalen Stack ist $1
% nostackframe call modifier used without assembler modifier
% might still generate local stack needs.
parser_e_cannot_generate_property_getter_setter=03325_E_Kann die Eigenschaft getter/setter $1 nicht erzeugen weil ihr Name mit dem existierenden Bezeichner $2 kollidiert
% Automatically generated getters/setters cannot have the same name as existing
% identifiers, because this may change the behaviour of existing code.
parser_w_overriding_property_getter_setter=03326_W_Die automatische erzeugte Eigenschaft getter/setter $1 überschreibt den getter/setter mit dem selben Name in der Klasse $2
% Automatically generated property getters/setters on the JVM platform are virtual methods, because
% the JVM does not support non-virtual methods that can be changed in child classes. This means
% that if a child class changes an inherited property definition, the behaviour of that property
% can change compared to native targets since even if a variable is declared as the parent type,
% by calling the virtual method the getter from the child will be used. This is different from
% the behaviour on native targets or when not activating automatically generated setters/getters,
% because in that case only the declared type of a variable influences the property behaviour.
parser_w_case_difference_auto_property_getter_setter_prefix=03327_W_Diskrepanz in Groß/Kleinschreibung zwischen der deklarierten Eigenschaft getter/setter $1 und dem automatisch erzeugten Namen $2, behalte deklarierten Namen bei
% If a property's specified getter/setter already corresponded to the naming convention specified
% by the automatic getter/setter generation setting except in terms of upper/lowercase, the
% compiler will print a warning because it cannot necessarily change that other declaration itself
% not can it add one using the correct case (it could conflict with the original declaration).
% Manually correct the case of the getter/setter to conform to the desired coding rules.
% \var{TChild} overrides
parser_e_no_consts_in_local_anonymous_records=03328_E_Deklarationen von Konstanten sind in lokalen oder anonymen Records nicht erlaubt
% Records with constants must be defined globally. Constants cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_methods_in_local_anonymous_records=03329_E_Deklarationen von Methoden sind in lokalen oder anonymen Records nicht erlaubt
% Records with methods must be defined globally. Methods cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_properties_in_local_anonymous_records=03330_E_Deklarationen von Eigenschaften sind in lokalen oder anonymen Records nicht erlaubt
% Records with properties must be defined globally. Properties cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_class_in_local_anonymous_records=03331_E_Deklarationen von Klassenmitgliedern sind in lokalen oder anonymen Records nicht erlaubt
% Records with class members must be defined globally. Class members cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_not_allowed_in_record=03332_E_Sichtbarkeits-Abschnitt "$1" ist in Records nicht erlaubt
% The visibility sections \var(protected) and \var(strict protected) are only
% useful together with inheritance. Since records do not support that they are
% forbidden.
parser_e_dir_not_allowed=03333_E_Direktive "$1" ist hier nicht erlaubt
% This directive is not allowed in the given context. E.g. "static"
% is not allowed for instance methods or class operators.
parser_e_no_assembler_in_generic=03334_E_Assemblerblöcke sind innerhalb von "generics" nicht erlaubt
% The use of assembler blocks/routines is not allowed inside generics.
parser_e_property_only_sgr=03335_E_Eigenschaften sind nur statisch, global oder in strukturierten Typen erlaubt
% Properties cannot be declared local, only global, using the static
% directive or inside structured types.
parser_e_overloaded_have_same_mangled_name=03336_E_Überladene Routinen haben den gleichen "mangled" Namen
% Some platforms, such as the JVM platform, encode the parameters in the routine name in
% a prescribed way, and this encoding may map different Pascal types to the same encoded
% (a.k.a.\ ``mangled'') name. This error can only be solved by removing or changing the
% conflicting definitions' parameter declarations or routine names.
parser_e_default_value_val_const=03337_E_Voreinstellungswerte können nur für value, const und constref Parameter spezifiziert werden
% A default parameter value allows you to not specify a value for this parameter
% when calling the routine, and the compiler will instead pass the specified
% default (constant) value. As a result, default values can only be specified
% for parameters that can accept constant values.
parser_w_ptr_type_ignored=03338_W_Zeigertyp "$1" ignoriert
% The specified pointer type modifier is ignored, because it is not supported on
% the current platform. This happens, for example, when a far pointer is
% declared on a non-x86 platform.
parser_e_global_generic_references_static=03339_E_Ein globales, generisches Templat referenziert eine statische Symboltabelle
% A generic declared in the interface section of a unit must not reference symbols that belong
% solely to the implementation section of that unit.
parser_u_already_compiled=03340_UL_Die Unit $1 wurde inzwischen bereits kompiliert.
% This tells you that the recursive reading of the uses clauses triggered already
% a compilation of the current unit, so the current compilation can be aborted.
parser_e_explicit_method_implementation_for_specializations_not_allowed=03341_E_Die explizite Implementierung von Methoden ist bei der Spezialisierung von "Generics" nicht erlaubt
% Methods introduced in a generic must be implemented for the generic. It is not possible to implement them only for specializations.
parser_e_no_genfuncs_in_interfaces=03342_E_Generische Methoden sind in Interfaces nicht erlaubt
% Generic methods are not allowed in interfaces, because there is no way to specialize a suitable
% implementation.
parser_e_genfuncs_cannot_be_virtual=03343_E_Generische Methoden können nicht virtuell sein
% Generic methods can not be declared as virtual as there'd need to be a VMT entry for each
% specialization. This is however not possible with a static VMT.
parser_e_packages_not_supported=03344_E_Dynamische Pakete werden für das Ziel-OS nicht unterstützt
% Support for dynamic packages is not implemented for the specified target OS
% or it is at least not tested and thus disabled.
parser_e_cannot_use_hardfloat_in_a_softfloat_environment=03345_E_Die Direktive "HardFloat" kann nicht verwendet werden, wenn  "soft float code" erzeugt wird oder die fpu-Emulation eingeschaltet ist
% The \var{HardFloat} directive can only be used if an instruction set is used which supports floating point operations.
parser_e_invalid_internal_function_index=03346_E_Index $1 ist kein gültiger interner Funktionenindex
% The index specified for the \var{compilerproc} directive is not an index that's recognized
% by the compiler.
parser_w_operator_overloaded_hidden_3=03347_W_Überladen eines Operators durch internen Operator überdeckt: "$1" $2 "$3"
% An operator overload is defined for the specified overload, but the internal overload by the compiler
% takes precedence. This only happens for operators that had been overloadable before (e.g. dynamic array + dynamic array),
% but aren't anymore due to an internal operator being defined while this behavior is controllable by a modeswitch
% (in case of dynamic arrays that is the modeswitch \var{ArrayOperators}).
parser_e_threadvar_must_be_class=03348_E_Threadvariablen in Klassen oder Records müssen Klassenvariablen sein
% A \var{threadvar} section inside a class or record was started without it being prefixed by \var{class}.
parser_e_only_static_members_via_object_type=03349_E_Nur statische Methoden und Variablen können mit einem Objekttyp referenziert werden
% This error occurs in a situation like the following:
% \begin{verbatim}
% Type
%    TObj = object
%      procedure test;
%    end;
%
% begin
%   TObj.test;
% \end{verbatim}
% \var{test} is not a static method and hence cannot be called through a type, but only using an instance.
parse_e_callthrough_varargs=03350_E_Kann die variadische Funktion "$1" im C-Stil auf dieser Platform nicht als external redeklarieren; schon die erste Deklaration muss external sein
% If a function is declared normally in the interface or as a forward declaration, and then later as external, the compiler
% must generate a stub that calls the external function. Due to code generation limitations, this cannot be done on some
% platforms. Even on platforms where it is supported, this is quite inefficient.
%
% \end{description}
# EndOfTeX

#
# Type Checking
#
# 04125 is the last used one
#
# BeginOfTeX
%
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_Typen passen nicht zusammen
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_Inkompatible Typen: Habe "$1" erhalten, aber "$2" erwartet
% There is no conversion possible between the two types.
% Another possibility is that they are declared in different
% declarations:
% \begin{verbatim}
% Var
%    A1 : Array[1..10] Of Integer;
%    A2 : Array[1..10] Of Integer;
%
% Begin
%    A1:=A2; { This statement also gives this error. It
%              is due to the strict type checking of Pascal }
% End.
% \end{verbatim}
type_e_not_equal_types=04002_E_Typen sind verschieden bei $1 und $2
% The types are not equal.
type_e_type_id_expected=04003_E_Typbezeichner erwartet
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_Variablenbezeichner erwartet
% This happens when you pass a constant to a \var{Inc} var or \var{Dec}
% procedure. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_Integer-Ausdruck erwartet, aber "$1" erhalten
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_Booleschen Ausdruck erwartet, aber "$1" erhalten
% The expression must be a boolean type. It should be return \var{True} or
% \var{False}.
type_e_ordinal_expr_expected=04007_E_Ganzzahliger Ausdruck erwartet
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_Zeigertyp erwartet, aber "$1" erhalten
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_Klassentyp erwartet, aber "$1" erhalten
% The variable or expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_cant_eval_constant_expr=04011_E_Kann konstanten Ausdruck nicht auswerten
% This error can occur when the bounds of an array you declared do
% not evaluate to ordinal constants.
type_e_set_element_are_not_comp=04012_E_Set-Elemente sind nicht kompatibel
% You are trying to perform an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union.
type_e_set_operation_unknown=04013_E_Operation für Sets nicht implementiert
% several binary operations are not defined for sets.
% These include: \var{div}, \var{mod}, \var{**}, \var{>=} and \var{<=}.
% The last two may be  defined for sets in the future.
type_w_convert_real_2_comp=04014_W_Automatische Typumwandlung von Fliesskommatyp nach COMP (=integer mit 64 bit)
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{Comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_Verwenden sie DIV um ein Integer-Ergebnis zu erhalten
% When hints are on, then an integer division with the '/' operator will
% produce this message, because the result will then be of type real.
type_e_strict_var_string_violation=04016_E_Stringtypen müssen im "$V+"-Modus exakt übereinstimmen
% When compiling in \var{\{\$V+\}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_Succ oder Pred kann nicht auf Aufzählungen mit Zuweisungen angewendet werden
% If you declare an enumeration type which has C-like assignments
% in it, such as in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% then you cannot use the \var{Succ} or \var{Pred} functions with this enumeration.
type_e_cant_read_write_type=04018_E_Kann Variablen dieses Typs nicht lesen oder schreiben
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that variable's type.
% Only integer types, reals, pchars and strings can be read from or
% written to a text file. Booleans can only be written to text files.
type_e_no_readln_writeln_for_typed_file=04019_E_Kann ReadLn und WriteLn nicht bei typisierten Dateien verwenden
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_Kann Read und Write nicht bei untypisierten Dateien verwenden
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_Typkonflikt zwischen den Elementen des Sets
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(dword/qword) gibt oberes/unteres Word/DWord zurück
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword/int64/qword}
% which returns the lower/upper word/dword of the argument. \tp always uses
% a 16 bit \var{lo/hi} which always returns bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the \tp behavior you have
% to type cast the argument to a \var{word} or \var{integer}.
type_e_integer_or_real_expr_expected=04023_E_Integer- oder Real-Ausdruck erwartet
% The first argument to \var{str} must be a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_Falscher Typ "$1" im Array-Konstruktor
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_Inkompatible Typen für Argument Nr. #$1: habe $2 erhalten, aber $3 erwartet
% You are trying to pass an invalid type for the specified parameter.
type_e_no_method_and_procedure_not_compatible=04026_E_Methode (Variable) und Prozedur (Variable) sind nicht kompatibel
% You cannot assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_Unzulässige Konstante an interne Algebrafunktion übergeben
% The constant argument passed to a \var{ln} or \var{sqrt} function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_Von Konstanten kann keine Adresse bestimmt werden
% It is not possible to get the address of a constant, because they
% are not stored in memory, you can try making it a typed constant. This error
% can also be displayed if you try to pass a property to a var parameter.
type_e_argument_cant_be_assigned=04029_E_An das Argument kann nichts zugewiesen werden
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference arguments.
%
% Remark: Properties can be used on the left side of an assignment,
% nevertheless they cannot be used as arguments.
type_e_cannot_local_proc_to_procvar=04030_E_Kann lokale Prozedur/Funktion nicht an Prozedurvariable zuweisen
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling convention of a local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_Kann einer Adresse keine Werte zuweisen
% It's not allowed to assign a value to an address of a variable,constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_Kann einer Konstanten keine Werte zuweisen
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const. To allow
% changing the value, pass the parameter by value, or a parameter by reference
% (using var).
type_e_array_required=04033_E_Array Typ notwendig
% If you are accessing a variable using an index '[<x>]' then
% the type must be an array. In FPC mode a pointer is also allowed.
type_e_interface_type_expected=04034_E_Interface Typ erwartet, aber "$1" erhalten
% The compiler expected to encounter an interface type name, but got something else.
% The following code would produce this error:
% \begin{verbatim}
% Type
%   TMyStream = Class(TStream,Integer)
% \end{verbatim}
type_h_mixed_signed_unsigned=04035_H_Mischen von signed Ausdrücken und Longwords ergibt ein 64bit Ergebnis
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetic
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64-bit arithmetic which is slower than normal
% 32-bit arithmetic. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_Mischen von signed Ausdrücken und kardinalen Typen hier kann eine Bereichsüberschreitung verursachen
% If you use a binary operator (and, or, xor) and one of
% the operands is a longword while the other one is a signed expression, then,
% if range checking is turned on, you may get a range check error because in
% such a case both operands are converted to longword before the operation is
% carried out. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_e_typecast_wrong_size_for_assignment=04037_E_Typecast hat verschiedene Größen ($1 -> $2) in der Zuweisung
% Type casting to a type with a different size is not allowed when the variable is
% used in an assignment.
type_e_array_index_enums_with_assign_not_possible=04038_E_Enums mit Zuweisungen können nicht als Array-Index verwendet werden
% When you declared an enumeration type which has C-like
% assignments, such as in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use it as the index of an array.
type_e_classes_not_related=04039_E_Klassen- oder Objekttypen "$1" and "$2" sind nicht verwandt
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors.
type_w_classes_not_related=04040_W_Klassentypen "$1" und "$2" sind nicht verwandt
% There is a typecast from one class to another while the classes
% are not related. This will probably lead to errors.
type_e_class_or_interface_type_expected=04041_E_Klassen- oder Interface-Typ erwartet, aber "$1" erhalten
% The compiler expected a class or interface name, but got another type or identifier.
type_e_type_is_not_completly_defined=04042_E_Typ "$1" ist nicht vollständig definiert
% This error occurs when a type is not complete: i.e. a pointer type which points to
% an undefined type.
type_w_string_too_long=04043_W_String literal hat mehr Zeichen als short string length
% The size of the constant string, which is assigned to a shortstring,
% is longer than the maximum size of the shortstring (255 characters).
type_w_comparison_always_false=04044_W_Wegen der Bereiche der Konstanten und des Ausdrucks könnte das Vergleichsergebnis immer falsch sein
% There is a comparison between a constant and an expression where the constant is out of the
% valid range of values of the expression. Because of type promotion, the statement will always evaluate to
% false. Explicitly typecast the constant or the expression to the correct range to avoid this warning
% if you think the code is correct.
type_w_comparison_always_true=04045_W_Wegen der Bereiche der Konstanten und des Ausdrucks könnte das Vergleichsergebnis immer richtig sein
% There is a comparison between a constant and an expression where the constant is out of the
% valid range of values of the expression. Because of type promotion, the statement will always evaluate to
% true. Explicitly typecast the constant or the expression to the correct range to avoid this warning
% if you think the code is correct.
type_w_instance_with_abstract=04046_W_Konstruktion der Klasse "$1" mit der abstrakten Methode "$2"
% An instance of a class is created which contains non-implemented abstract
% methods. This will probably lead to a runtime error 211 in the code if that
% routine is ever called. All abstract methods should be overridden.
type_h_in_range_check=04047_H_Der linke Operand des IN Operators sollte byte Größe haben
% The left operand of the \var{in} operator is not an ordinal or enumeration which fits
% within 8 bits. This may lead to range check errors. The \var{in} operator
% currently only supports a left operand which fits within a byte. In the case of
% enumerations, the size of an element of an enumeration can be controlled with
% the \var{\{\$PACKENUM\}} or \var{\{\$Zn\}} switches.
type_w_smaller_possible_range_check=04048_W_Unpassende Typgrößen, Gefahr des Datenverlusts oder der Bereichsüberschreitung
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_h_smaller_possible_range_check=04049_H_Unpassende Typgrößen, Gefahr des Datenverlusts oder der Bereichsüberschreitung
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_e_cant_take_address_of_abstract_method=04050_E_Die Adresse einer abstrakten Methode kann nicht verwendet werden
% An abstract method has no body, so the address of an abstract method cannot be taken.
type_e_assignment_not_allowed=04051_E_Zuweisungen auf formale Parameter und offene Arrays sind nicht möglich
% You are trying to assign a value to a formal (untyped var, const or out)
% parameter, or to an open array.
type_e_constant_expr_expected=04052_E_Konstanter Ausdruck erwartet
% The compiler expects an constant expression, but gets a variable expression.
type_e_operator_not_supported_for_types=04053_E_Operation "$1" wird für die Typen "$2" und "$3" nicht unterstützt
% The operation is not allowed for the supplied types.
type_e_illegal_type_conversion=04054_E_Illegale Typ-Konversion: "$1" nach "$2"
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
type_h_pointer_to_longint_conv_not_portable=04055_H_Konversion zwischen ordinalen Typen und Zeigern ist nicht portierbar
% If you typecast a pointer to a longint (or vice-versa), this code will not compile
% on a machine using 64 bits addressing.
type_w_pointer_to_longint_conv_not_portable=04056_W_Konversion zwischen ordinalen Typen und Zeigern ist nicht portierbar
% If you typecast a pointer to an ordinal type of a different size (or vice-versa), this can
% cause problems. This is a warning to help in finding the 32-bit specific code where cardinal/longint is used
% to typecast pointers to ordinals. A solution is to use the ptrint/ptruint types instead.
type_e_cant_choose_overload_function=04057_E_Kann nicht bestimmen, welche der überladenen Funktionen aufgerufen werden soll
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
type_e_illegal_count_var=04058_E_Die Schleifenvariable muss einen ordinalen Typ haben
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
type_w_double_c_varargs=04059_W_Der konstante Wert vom Typ real wird für ein C Variablen-Argument zu double konvertiert. Ergänze eine explizite Typ-Konversion (typecast), um das zu verhindern
% In C, constant real values are double by default. For this reason, if you
% pass a constant real value to a variable argument part of a C function, FPC
% by default converts this constant to double as well. If you want to prevent
% this from happening, add an explicit typecast around the constant.
type_e_class_or_cominterface_type_expected=04060_E_Class oder COM interface Typ erwartet, statt dessen "$1" erhalten
% Some operators, such as the AS operator, are only applicable to classes or COM interfaces.
type_e_no_const_packed_array=04061_E_Konstante packed Arrays werden noch nicht unterstützt
% You cannot declare a (bit)packed array as a typed constant.
type_e_got_expected_packed_array=04062_E_Inkompatibler Typ für Argument $1: Erhielt "$2" erwartete "(Bit)Packed Array"
% The compiler expects a (bit)packed array as the specified parameter.
type_e_got_expected_unpacked_array=04063_E_Inkompatibler Typ für Argument $1: Erhielt "$2" erwartete "(not packed) Array"
% The compiler expects a regular (i.e., not packed) array as the specified parameter.
type_e_no_packed_inittable=04064_E_Elemente von packed Arrays können nicht von einem Typ sein, der initialisiert werden muss
% Support for packed arrays of types that need initialization
% (such as ansistrings, or records which contain ansistrings) is not yet implemented.
type_e_no_const_packed_record=04065_E_Konstante packed Records und Objekte werden noch nicht unterstützt
% You cannot declare a (bit)packed array as a typed constant at this time.
type_w_untyped_arithmetic_unportable=04066_W_Arithmetik "$1" mit typenlosem Zeiger ist nicht portierbar nach {$T+}, schlage typecast vor
% Addition/subtraction from an untyped pointer may work differently in \var{\{\$T+\}}.
% Use a typecast to a typed pointer.
type_e_cant_take_address_of_local_subroutine=04076_E_Die Address einer Subroutine, die als local markiert ist, kann nicht verwendet werden
% The address of a subroutine marked as local cannot be taken.
type_e_cant_export_local=04077_E_Eine Subroutine, die aus einer Unit als local markiert ist, kann nicht exportiert werden
% A subroutine marked as local cannot be exported from a unit.
type_e_not_automatable=04078_E_Typ "$1" ist nicht "automatable"
% Only byte, integer, longint, smallint, currency, single, double, ansistring,
% widestring, tdatetime, variant, olevariant, wordbool and all interfaces are automatable.
type_h_convert_add_operands_to_prevent_overflow=04079_H_Konvertierung des Operanden "$1" vor der Addition könnte Überlauf Fehler verhindern
% Adding two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the addition.
type_h_convert_sub_operands_to_prevent_overflow=04080_H_Konvertierung des Operanden "$1" vor der Subtraktion könnte Überlauf Fehler verhindern
% Subtracting two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the subtraction.
type_h_convert_mul_operands_to_prevent_overflow=04081_H_Konvertierung des Operanden "$1" vor der Multiplikation könnte Überlauf Fehler verhindern
% Multiplying two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the multiplication.
type_w_pointer_to_signed=04082_W_Die Konvertierung von Zeigern in einen Integertyp mit Vorzeichen kann zu falschen Ergebnissen bei Vergleichen und zu Bereichsüberschreitungen führen; verwenden sie statt dessen besser einen Typ ohne Vorzeichen
% The virtual address space on 32-bit machines runs from \$00000000 to \$ffffffff.
% Many operating systems allow you to allocate memory above \$80000000.
% For example both \windows and \linux allow pointers in the range \$0000000 to \$bfffffff.
% If you convert pointers to signed types, this can cause overflow and range check errors,
% but also \$80000000 < \$7fffffff. This can cause random errors in code like "if p>q".
type_e_interface_has_no_guid=04083_E_Interface Typ $1 hat keine gültige GUID
% When applying the as-operator to an interface or class, the desired interface (i.e. the right operand of the
% as-operator) must have a valid GUID.
type_e_invalid_objc_selector_name=04084_E_Ungültiger Objective-C-Selector-Name "$1"
% An Objective-C selector cannot be empty, must be a valid identifier or a single colon,
% and if it contains at least one colon it must also end in one.
type_e_expected_objc_method_but_got=04085_E_Erwartete eine Objective-C-Methode, erhielt aber $1
% A selector can only be created for Objective-C methods, not for any other kind
% of procedure/function/method.
type_e_expected_objc_method=04086_E_Erwartete eine Objective-C-Methode, oder den Namen einer konstanten Methode
% A selector can only be created for Objective-C methods, either by specifying
% the name using a string constant, or by using an Objective-C method identifier
% that is visible in the current scope.
type_e_no_type_info=04087_E_Für diesen Typ steht keine Typ-Information zu Verfügung
% Type information is not generated for some types, such as enumerations with gaps
% in their value range (this includes enumerations whose lower bound is different
% from zero).
type_e_ordinal_or_string_expr_expected=04088_E_Ausdruck mit ordinalem Typ oder Zeichenkette erwartet
% The expression must be an ordinal or string type.
type_e_string_expr_expected=04089_E_Ausdruck mit Zeichenkette erwartet
% The expression must be a string type.
type_w_zero_to_nil=04090_W_Konvertiere 0 zu NIL
% Use NIL rather than 0 when initialising a pointer. 
type_e_protocol_type_expected=04091_E_Objective-C Protokolltyp erwartet, erhielt aber "$1"
% The compiler expected a protocol type name, but found something else.
type_e_objc_type_unsupported=04092_E_Der Typ "$1" wird für die Verwendung mit den Laufzeitumgebungen Objective-C- oder Blocks nicht unterstützt.
% Objective-C and Blocks makes extensive use of run time type information (RTTI). This format
% is defined by the maintainers of the run time and can therefore not be adapted
% to all possible Object Pascal types. In particular, types that depend on
% reference counting by the compiler (such as ansistrings and certain kinds of
% interfaces) cannot be used as fields of Objective-C classes, cannot be
% directly passed to Objective-C methods or Blocks, and cannot be encoded using \var{objc\_encode}.
type_e_class_or_objcclass_type_expected=04093_E_Klasse oder Objective-C Klasse als Typ erwartet, erhielt aber "$1"
% It is only possible to create class reference types of \var{class} and \var{objcclass}
type_e_objcclass_type_expected=04094_E_Objective-C Klasse als Typ erwartet
% The compiler expected an \var{objcclass} type
type_w_procvar_univ_conflicting_para=04095_W_Erzwungener univ Parameter Typ in einer prozeduralen Variablen kann einen Absturz oder SpeicherKorruption verursachen: $1 auf $2
% \var{univ} parameters are implicitly compatible with all types of the same size,
% also in procedural variable definitions. That means that the following code is
% legal, because \var{single} and \var{longint} have the same size:
% \begin{verbatim}
% {$mode macpas}
% Type
%   TIntProc = procedure (l: univ longint);
%
%   procedure test(s: single);
%     begin
%       writeln(s);
%     end;
%
%   var
%     p: TIntProc;
%   begin
%     p:=test;
%     p(4);
%   end.
% \end{verbatim}
% This code may however crash on platforms that pass integers in registers and
% floating point values on the stack, because then the stack will be unbalanced.
% Note that this warning will not flag all potentially dangerous situations.
% when \var{test} returns.
type_e_generics_cannot_reference_itself=04096_E_Typ-Parameter bei der Spezialisation von Generics können den aktuel spezialisierten Typ nicht referenzieren
% Recursive specializations of generics like \var{Type MyType = specialize MyGeneric<MyType>;} are not possible.
type_e_type_parameters_are_not_allowed_here=04097_E_Typ-Parameter sind für nicht-generische Klassen/Record/Objekte Prozeduren und Funktionen nicht erlaubt
% Type parameters are only allowed for methods of generic classes, records or objects
type_e_generic_declaration_does_not_match=04098_E_Die generische Deklaration von "$1" unterscheidet sich vom der vorherigen Deklaration
% Generic declaration does not match the previous declaration
type_e_helper_type_expected=04099_E_Helfertyp erwartet
% The compiler expected a \var{class helper} type.
type_e_record_type_expected=04100_E_Recordtyp erwartet
% The compiler expected a \var{record} type.
type_e_class_helper_must_extend_subclass=04101_E_Abgeleitete Klassenhelfer müssen eine Unterklasse von "$1" oder die Klasse selbst erweitern
% If a class helper inherits from another class helper the extended class must
% extend either the same class as the parent class helper or a subclass of it
type_e_record_helper_must_extend_same_record=04102_E_Abgeleitete Record- und Typhelfer müssen "$1" erweitern
% If a record helper inherits from another record helper it must extend the same
% record that the parent record helper extended.
type_e_procedures_return_no_value=04103_E_Ungültige Zuweisung, eine Prozedur gibt keinen Wert zurück
% This error occurs when one tries to assign the result of a procedure or destructor call.
% A procedure or destructor returns no value so this is not
% possible.
type_w_implicit_string_cast=04104_W_Implizite Umwandlung einer Zeichenkette von "$1" nach "$2"
% An implicit type conversion from an ansi string type to an unicode string type is
% encountered. To avoid this warning perform an explicit type conversion.
type_w_implicit_string_cast_loss=04105_W_Implizite Umwandlung einer Zeichenkette von "$1" nach "$2", bei der möglicherweise Informationen verloren gehen
% An implicit type conversion from an unicode string type to an ansi string type is
% encountered. This conversion can lose data since not all unicode characters may be represented in the codepage of
% destination string type.
type_w_explicit_string_cast=04106_-W_Explizites Typecast einer Zeichenkette von "$1" nach "$2"
% An explicit typecast from an ansi string type to an unicode string type is
% encountered. This warning is off by default. You can turn it on to see all suspicious string conversions.
type_w_explicit_string_cast_loss=04107_-W_W_Explizites Typecast einer Zeichenkette von "$1" nach "$2", bei der möglicherweise Informationen verloren gehen
% An explicit typecast from an unicode string type to an ansi string type is
% encountered. This conversion can lose data since not all unicode characters may be represented in the codepage of
% destination string type. This warning is off by default. You can turn it on to see all the places with lossy string
% conversions.
type_w_unicode_data_loss=04108_W_Cast einer Unicode Konstanten, bei der möglicherweise Informationen verloren gehen
% Conversion from a WideChar to AnsiChar can lose data since now all unicode characters may be represented in the current
% system codepage
% You can nest function definitions only 31 levels deep.
type_e_range_check_error_bounds=04109_E_Bereichsüberschreitung bei der Berechnung einer Konstanten ($1 muss zwischen $2 und $3 liegen)
type_w_range_check_error_bounds=04110_W_Bereichsüberschreitung bei der Berechnung einer Konstanten ($1 muss zwischen $2 und $3 liegen)
% The constants are outside their allowed range.
type_e_type_not_allowed_for_default=04111_E_Dieser Typ wird für die intrinsische Funktion Default() nicht unterstützt
% Some types like for example Text and File Of X are not supported by the Default intrinsic.
type_e_java_class_method_not_static_virtual=04112_E_JVM virtuelle Klassenmethoden können nicht statisch sein
% Virtual class methods cannot be static when targeting the JVM platform, because
% the self pointer is required for correct dispatching.
type_e_invalid_final_assignment=04113_E_Finale (Klassen-)Felder können nur im (Klassen-)Konstruktur ihrer Klasse zugewiesen werden
% It is only possible to assign a value to a final (class) field inside a (class) constructor of its owning class.
type_e_no_managed_formal_assign_typecast=04114_E_Typecast für nicht typisierte Parameter wird auf managed Plattformen nicht unterstützt, weise stattdessen einfach einen Wert zu
% On managed platforms, untyped parameters are translated by the compiler into
% the equivalent of \var{var x: BaseClassType}. Non-class-based types passed to
% such parameters are automatically wrapped (or boxed) in a class, and after the
% call the potentially modified value is assigned back to the original variable.
% On the caller side, changing untyped var/out parameters happens by simply assigning
% values to them (either class-based or primitive ones). On the caller side,
% they will be extracted and if their type does not match the original variable's,
% an exception will be raised.
type_e_no_managed_assign_generic_typecast=04115_E_Auf managed Plattformen kann für die Zuweisungsseite eines Ausdrucks kein Typecast auf einen Supertyp erfolgen
% Managed platforms guarantee type safety at the bytecode level. This means that the virtual machine must be able
% to statically determine that no type-unsafe assignments or operations occur. By assigning a parent class type to a
% variable of a child type by typecasting the assignment side to the parent class type, the type safety would no
% longer be guaranteed and the generated code would fail verification at run time time.
type_w_interface_lower_visibility=04116_-W_Die Interfacemethode "$1" erhöht die Sichtbarkeit von "$2" auf öffentlich, wenn sie über eine Interfaceinstanz benutzt wird
type_e_interface_lower_visibility=04117_E_Die Interfacemethode "$1" hat eine größere Sichtbarkeit (öffentlich) als "$2"
% All methods in an interface have always public visibility. That means that if
% an interface method is implemented using a (strict) protected or private method,
% this method is actually publicly accessible via the interface. On the JVM
% target this situation results in an error because the JVM rejects such
% attempts to circumvent the visibility rules. On other targets this is a
% warning that is disabled by default because such situations are common
% practice, but it can be enabled in case you are concerned with keeping your
% code compilable for the JVM target.
type_e_typeof_requires_vmt=04118_E_TYPEOF kann nur auf Objekttypen mit VMT angewendet werden
% Typeof() intrinsic returns pointer to VMT of its argument. It cannot be used on object types that do not have VMT.
type_e_invalid_default_value=04119_E_Der Defaultwert für einen Parameter des Typs "$1" kann nicht definiert werden
% Parameters declared as structured types, such as files, variants, non-dynamic
% arrays and TP-style objects, cannot have a default value.
type_e_type_not_allowed_for_type_helper=04120_E_Typ "$1" kann durch einen Typhelfer nicht erweitert werden
% Types like procedural variables cannot be extended by type helpers
type_e_procedure_must_be_far=04121_E_Prozedur oder Funktion muss weit ("far") sein, damit seine Adresse verwendet werden kann: "$1"
% In certain i8086 memory models (medium, large and huge), procedures and functions
% have to be declared 'far' in order to allow their address to be taken.
type_w_instance_abstract_class=04122_W_Erzeugung einer Instanz der abstrakten Klasse "$1"
% The specified class is declared as \var{abstract} and thus no instance of this class
% should be created. This is merely a warning for Delphi compatibility.
type_e_function_reference_kind=04123_E_Unterprogrammreferenzen können nicht als "of object" oder "is nested" deklariert werden; sie können sich immer auf alle mögliche Arten von Unterprogrammen beziehen
% Subroutine references can refer to any kind of subroutine and hence do not
% require specialisation for methods or nested subroutines.
type_e_seg_procvardef_wrong_memory_model=04124_E_Prozedurvariablen speichern in diesem Memory-Modell keine Segment-Informationen
type_w_empty_constant_range_set=04125_W_Der erste Wert des Bereichs für den Mengen-Konstruktor is größer als der zweite. Der Bereich beschreibt deshalb eine leere Menge.
% If a set is constructed like this: \var{s:=[9..7];]}, then an empty set is generated. As this is something normally not desired, the compiler warns about it.
%
% \end{description}
# EndOfTeX

#
# Symtable
#
# 05098 is the last used one
#
# BeginOfTeX
%
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Bezeichner "$1" nicht gefunden
% The compiler doesn't know this symbol. Usually happens when you misspell
% the name of a variable or procedure, or when you forget to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_Interner Fehler in SymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_Doppelter Bezeichner "$1"
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_Bezeichner ist bereits definiert in $1 in Zeile $2
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_Unbekannter Bezeichner "$1"
% The identifier encountered hasn't been declared, or is used outside the
% scope where it's defined.
sym_e_forward_not_resolved=05005_E_Forward-Deklaration "$1" nicht gefunden
% This can happen in two cases:
% \begin{itemize}
% \item You declare a function in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_e_error_in_type_def=05007_E_Fehler in Typdefinition
% There is an error in your definition of a new array type.
% One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_forward_type_not_resolved=05009_E_Forward-Typ "$1" nicht gefunden
% The compiler encountered an unknown type.
sym_e_only_static_in_static=05010_E_Nur statische Variablen können in statischen oder äusseren Methoden verwendet werden
% A static method of an object can only access static variables.
sym_e_type_must_be_rec_or_object_or_class=05012_E_Record-, Objekt- oder Klassen-Typ erwartet
% The variable or expression isn't of the type \var{record} or \var{object} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Instanzen von Klassen oder Objekten mit abstrakten Methoden sind unzulässig
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_Label "$1" nicht definiert
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_Label "$1" benutzt aber nicht definiert
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_Ungültige Label-Deklaration
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO und LABEL werden nicht unterstützt (verwenden Sie den Schalter -Sg)
% You must use the -Sg switch to compile a program which has \var{label}s
% and \var{goto} statements. By default, \var{label} and \var{goto} are not
% supported.
sym_e_label_not_found=05018_E_Label nicht gefunden
% A \var{goto label} was encountered, but the label wasn't declared.
sym_e_id_is_no_label_id=05019_E_Bezeichner ist kein Label
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_Label ist bereits definiert
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_Ungültige Typdeklaration von Set-Elementen
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_Forward-Klassendefinition nicht gefunden: $1
% You declared a class, but you didn't implement it.
sym_n_unit_not_used=05023_H_Unit "$1" wird von "$2" nicht verwendet
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_Parameter nicht verwendet: $1
% This is a warning. The identifier was declared (locally or globally) but
% wasn't used (locally or globally).
sym_n_local_identifier_not_used=05025_N_Lokale Variable nicht verwendet: $1
% You have declared, but not used, a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_Werte-Parameter "$1" wurde zugewiesen, aber nie verwendet
% The identifier was declared (locally or globally) and
% assigned to, but is not used (locally or globally) after the assignment.
sym_n_local_identifier_only_set=05027_N_Lokale Variable "$1" wurde zugewiesen, aber nicht verwendet
% The variable in a procedure or function implementation is declared and
% assigned to, but is not used after the assignment.
sym_h_local_symbol_not_used=05028_H_Lokales $1-Element "$2" wird nicht verwendet
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_Privates Feld $1.$2 wird nie verwendet
% The indicated private field is defined, but is never used in the code.
sym_n_private_identifier_only_set=05030_N_Privates Feld $1.$2 wurde zugewiesen, aber nie verwendet
% The indicated private field is declared and assigned to, but never read.
sym_n_private_method_not_used=05031_N_Private Methode $1.$2 wird nie verwendet
% The indicated private method is declared but is never used in the code.
sym_e_set_expected=05032_E_Mengentyp erwartet
% The variable or expression isn't of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_Funktionsergebnis scheint nicht gesetzt zu sein
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_Typ "$1" ist im aktuellen Record mit C-Packing nicht korrekt ausgerichtet
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_Unbekannter Record-Feldbezeichner $1
% The field doesn't exist in the record definition.
sym_w_uninitialized_local_variable=05036_W_Lokale Variable "$1" wird verwendet, bevor ihr ein Wert zugewiesen wurde
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment).
sym_w_uninitialized_variable=05037_W_Variable "$1" scheint nicht initialisiert worden zu sein
% These messages are displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% wasn't initialized first (i.e. appeared in the left-hand side of an
% assignment).
sym_e_id_no_member=05038_E_Bezeichener verweist nicht auf ein Element: $1
% This error is generated when an identifier of a record,
% field or method is accessed while it is not defined.
sym_h_param_list=05039_H_Deklaration gefunden: $1
% You get this when you use the \var{-vh} switch.In the case of an overloaded procedure
% not being found. Then all candidate overloaded procedures are
% listed, with their parameter lists.
sym_e_segment_too_large=05040_E_Datensegment zu groß (max. 2GB)
% You get this when you declare a data element whose size exceeds the
% prescribed limit (2 Gb on 80386+/68020+ processors).
sym_e_no_matching_implementation_found=05042_E_Keine passende Implementation der Interface Methode "$1" gefunden
% There was no matching method found which could implement the interface
% method. Check argument types and result type of the methods.
sym_w_deprecated_symbol=05043_W_Symbol "$1" ist veraltet
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Use of this symbol
% should be avoided as much as possible.
sym_w_non_portable_symbol=05044_W_Symbol "$1" ist nicht portierbar
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{platform} is used. This symbol's value, use
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_non_implemented_symbol=05055_W_Symbol "$1" ist nicht implementiert
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{unimplemented} is used. This symbol is defined,
% but is not yet implemented on this specific platform.
sym_e_cant_create_unique_type=05056_E_Von diesem Typ kann kein eindeutiger Typ deklariert werden
% Only simple types like ordinal, float and string types are supported when
% redefining a type with \var{type newtype = type oldtype;}.
sym_h_uninitialized_local_variable=05057_H_Die Variable "$1" scheint nicht initialisiert zu sein
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. it did not appear in the left-hand side of an
% assignment).
sym_h_uninitialized_variable=05058_H_Die Variable "$1" scheint nicht initialisiert zu sein
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. t did not appear in the left-hand side of an
% assignment).
sym_w_function_result_uninitialized=05059_W_Die Ergebnisvariable der Funktion scheint nicht initialisiert zu sein
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. before it appeared in the left-hand side of an
% assignment).
sym_h_function_result_uninitialized=05060_H_Die Ergebnisvariable der Funktion scheint nicht initialisiert zu sein
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. it appears in the left-hand side of an
% assignment)
sym_w_identifier_only_read=05061_W_Die Variable "$1" wird gelesen, obwohl ihr aber noch kein Wert zugewiesen wurde
% You have read the value of a variable, but nowhere assigned a value to
% it.
sym_h_abstract_method_list=05062_H_Abstrakte Method "$1" gefunden 
% When getting a warning about constructing a class/object with abstract methods
% you get this hint to assist you in finding the affected method.
sym_w_experimental_symbol=05063_W_Symbol "$1" ist experimentell
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{experimental} is used. Experimental symbols
% might disappear or change semantics in future versions. Usage of this symbol
% should be avoided as much as possible.
sym_w_forward_not_resolved=05064_W_Forward Deklaration "$1" wird nicht aufgelöst und deshalb als extern angenommen
% This happens if you declare a function in the \var{interface} of a unit in macpas mode,
% but do not implement it.
sym_w_library_symbol=05065_W_Symbol "$1" gehört zu einer Bibliothek
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{library} is used. Library symbols may not be
% available in other libraries.
sym_w_deprecated_symbol_with_msg=05066_W_Symbol "$1" ist veraltet: "$2"
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Use of this symbol
% should be avoided as much as possible.
sym_e_no_enumerator=05067_E_Kann keinen Zähler für den Typ "$1" finden
% This means that compiler cannot find an appropriate enumerator to use in the for-in loop.
% To create an enumerator you need to define an operator enumerator or add a public or published
% GetEnumerator method to the class or object definition.
sym_e_no_enumerator_move=05068_E_Kann keine Methode "MoveNext" in der Aufzählung "$1" finden
% This means that compiler cannot find a public MoveNext method with the Boolean return type in
% the enumerator class or object definition.
sym_e_no_enumerator_current=05069_E_Kann keine Eigenschaft "Current" in der Aufzählung "$1" finden
% This means that compiler cannot find a public Current property in  the enumerator class or object
% definition.
sym_e_objc_para_mismatch=05070_E_Die Anzahl der deklarierten Parameter und die Anzahl der Doppelpunkte in der Nachrichtenzeichenkette stimmen nicht überein
% In Objective-C, a message name automatically contains as many colons as parameters.
% In order to prevent mistakes when specifying the message name in FPC, the compiler
% checks whether this is also the case here. Note that in case of messages taking a
% variable number of arguments translated to FPC via an \var{array of const} parameter,
% this final \var{array of const} parameter is not counted. Neither are the hidden
% \var{self} and \var{\_cmd} parameters.
sym_n_private_type_not_used=05071_N_Privater Typ "$1.$2" wird nie benutzt
% The indicated private type is declared but is never used in the code.
sym_n_private_const_not_used=05072_N_Private Konstante "$1.$2" wird nie benutzt
% The indicated private const is declared but is never used in the code.
sym_n_private_property_not_used=05073_N_Private Eigenschaft "$1.$2" wird nie benutzt
% The indicated private property is declared but is never used in the code.
sym_w_deprecated_unit=05074_W_Unit "$1" ist veraltet
% This means that a unit which is
% declared as \var{deprecated} is used. Deprecated units may no longer
% be available in newer versions of the library. Use of this unit
% should be avoided as much as possible.
sym_w_deprecated_unit_with_msg=05075_W_Unit "$1" ist veraltet: "$2"
% This means that a unit which is
% declared as \var{deprecated} is used. Deprecated units may no longer
% be available in newer versions of the library. Use of this unit
% should be avoided as much as possible.
sym_w_non_portable_unit=05076_W_Unit "$1" ist plattformabhängig
% This means that a unit which is
% declared as \var{platform} is used. This unit use
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_library_unit=05077_W_Unit "$1" gehört zu einer Bibliothek
% This means that a unit which is
% declared as \var{library} is used. Library units may not be
% available in other libraries.
sym_w_non_implemented_unit=05078_W_Unit "$1" ist nicht implementiert
% This means that a unit which is
% declared as \var{unimplemented} is used. This unit is defined,
% but is not yet implemented on this specific platform.
sym_w_experimental_unit=05079_W_Unit "$1" ist experimentell
% This means that a unit which is
% declared as \var{experimental} is used. Experimental units
% might disappear or change semantics in future versions. Usage of this unit
% should be avoided as much as possible.
sym_e_formal_class_not_resolved=05080_E_Die vollständige Definition der formal deklarierten ObjC-Klasse "$1" fehlt in diesem Geltungsbereich. Ergänzen sie die uses-Anweisung um die Unit mit der vollständigen Definition
% Objecive-C and Java classes can be imported formally, without using the unit in which it is fully declared.
% This enables making forward references to such classes and breaking circular dependencies amongst units.
% However, as soon as you wish to actually do something with an entity of this class type (such as
% access one of its fields, send a message to it, or use it to inherit from), the compiler requires the full definition
% of the class to be in scope.
sym_e_interprocgoto_into_init_final_code_not_allowed=05081_E_Gotos in die 'initialization'- oder 'finalization'-Blöcke einer Unit sind nicht erlaubt
% Gotos into initialization or finalization blockse of units are not allowed.
sym_e_external_class_name_mismatch1=05082_E_Ungültiger externer Name "$1" für die formale Klasse "$2"
sym_e_external_class_name_mismatch2=05083_E_Hierhin muss die vollständige Klassendefinition mit externem Namen "$1"
% When a class is declared using a formal external definition, the actual external
% definition (if any) must specify the same external name as the formal definition
% (since both definitions refer to the same actual class type).
sym_w_library_overload=05084_W_Möglicher Bibliothekenkonflikt: Das Symbol "$1" aus Bibliothek "$2" wurde auch in Bibliothek "$3" gefunden
% Some OS do not have library specific namespaces, for those
% OS, the function declared as "external 'libname' name 'funcname'",
% the 'libname' part is only a hint, funcname might also be loaded
% by another library. This warning appears if 'funcname' is used twice
% with two different library names.
sym_e_duplicate_id_create_java_constructor=05085_E_Kann den impliziten Konstruktor 'Create' nicht hinzufügen, weil der Bezeichner schon von "$1" verwendet wird
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% However, if a class does not explicitly declare at least one constructor, the compiler is
% required to add a public, parameterless constructor. In Java, constructors are nameless,
% but in FPC they are all called ``Create''. Therefore, if you do not add a constructor to
% a Java class and furthermore use the ``Create'' identifier for another entity (e.g., a field,
% or a parameterless method), the compiler cannot satisfy this requirement.
sym_e_no_matching_inherited_parameterless_constructor=05086_E_Kann den voreingestellten Konstruktor für die Klasse nicht erzeugen, weil die Elternklasse keinen parameterlosen Konstruktor hat
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% However, if a class does not explicitly declare at least one constructor, the compiler is
% required to add a public, parameterless constructor. This compiler must then call
% the parameterless constructor from the parent class inside this added constructor.
% This is however impossible if the parent class does not declare such a constructor.
% In this case you must add a valid constructor yourself.
sym_d_adding_helper_for=05087_D_Helfer für $1 hinzugefügt
% A helper for the mentioned type is added to the current scope
sym_e_param_list=05088_E_Deklaration gefunden: $1
% This message shows all overloaded declarations in case of an error.
sym_w_uninitialized_managed_local_variable=05089_W_Die lokale Variable "$1" eines managed Typs ist anscheinend nicht initialisiert
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_uninitialized_managed_variable=05090_W_Die Variable "$1" eines managed Typs ist anscheinend nicht initialisiert
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_uninitialized_managed_local_variable=05091_H_Die lokale Variable "$1" eines managed Typs ist anscheinend nicht initialisiert
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. it did not appear in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_uninitialized_managed_variable=05092_H_Die Variable "$1" eines managed Typs ist anscheinend nicht initialisiert
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. t did not appear in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_managed_function_result_uninitialized=05093_W_Die Ergebnisvariable einer Funktion eines managed Typs ist anscheinend nicht initialisiert
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. before it appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_managed_function_result_uninitialized=05094_H_Die Ergebnisvariable einer Funktion eines managed Typs ist anscheinend nicht initialisiert
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. it appears in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_duplicate_id=05095_W_Duplikat des Bezeichners "$1"
% The identifier was already declared in an Objective-C category that's in the
% same scope as the current identifier. This is a warning instead of an error,
% because while this hides the identifier from the category, there are often
% many unused categories in scope.
sym_e_generic_type_param_mismatch=05096_E_Der generische Typparameter "$1" passt nicht zu dem in der Deklaration
% The specified generic type parameter for the generic class, record or routine does
% not match with the one declared in the declaration of the generic class, record
% or routine.
sym_e_generic_type_param_decl=05097_E_Generischer Typparameter deklariert als "$1"
% Shows what the generic type parameter was originally declared as if a mismatch
% is found between a declaration and the definition.
sym_e_type_must_be_rec_or_object=05098_E_Record- oder Objecttyp erwartet
% The variable or expression isn't of the type \var{record} or \var{object}.
%
% \end{description}
# EndOfTeX

#
# Codegenerator
#
# 06060 is the last used one
#
# BeginOfTeX
%
% \section{Code generator messages}
% This section lists all messages that can be displayed if the code
% generator encounters an error condition.
% \begin{description}
cg_e_parasize_too_big=06009_E_Größe der Parameterliste übersteigt 65535 Bytes
% The I386 processor limits the parameter list to 65535 bytes. (The \var{RET}
% instruction causes this).
cg_e_file_must_call_by_reference=06012_E_Dateitypen müssen VAR Parameter sein
% You cannot specify files as value parameters, i.e., they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_Die Verwendung eines FAR-Zeigers ist in dieser Art nicht erlaubt
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_dont_call_exported_direct=06015_E_Mit EXPORT deklarierte Funktionen können nicht aufgerufen werden
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_Möglicherweise unzulässiger Aufruf eines Konstruktors oder Destruktors (passt nicht in diesen Kontext)
% The compiler detected that a constructor or destructor is called within a
% a method. This will probably lead to problems, since constructors / destructors
% require parameters on entry.
cg_n_inefficient_code=06017_N_Uneffiziente Programmierung
% You construction seems dubious to the compiler.
cg_w_unreachable_code=06018_W_Code wird niemals ausgeführt
% You specified a loop which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_cant_call_abstract_method=06020_E_Abstrakte Methoden können nicht direkt aufgerufen werden
% You cannot call an abstract method directly. Instead, you must call an
% overriding child method, because an abstract method isn't implemented.
cg_d_register_weight=06027_DL_Register $1 Gewichtung $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_d_stackframe_omited=06029_DL_Stackframe wird ausgelassen
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_e_unable_inline_object_methods=06031_E_Objektmethoden können nicht Inline sein
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_Procvar-Aufrufe können nicht Inline sein
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_Kein Code für Inline-Prozedur gespeichert
% The compiler couldn't store code for the inline procedure.
cg_e_can_access_element_zero=06035_E_Auf Element Null von Ansi/Wide- oder Longstring kann nicht zugegriffen werden, benutzen Sie stattdessen (Set)Length
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such string type.
cg_e_cannot_call_cons_dest_inside_with=06037_E_Konstruktoren und Destruktoren können in diesem Kontext nicht aufgerufen werden
% Inside a \var{With} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_Kann Messagehandler-Methode nicht direkt aufrufen
% A message method handler method cannot be called directly if it contains an
% explicit \var{Self} argument.
cg_e_goto_inout_of_exception_block=06039_E_Sprung in- oder aus dem Exceptionblock heraus
% It is not allowed to jump in or outside of an exception block like \var{try..finally..end;}.
% For example, the following code will produce this error:

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
cg_e_control_flow_outside_finally=06040_E_Kontrollfluss-Anweisungen sind in einem Finally-Block nicht erlaubt
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
% exit the procedure or search for another exception handler.
cg_w_parasize_too_big=06041_W_Größe der Parameter überschreitet die Grenze für bestimmte CPUs
% This indicates that you are declaring more than 64K of parameters, which
% might not be supported on other processor targets.
cg_w_localsize_too_big=06042_W_Größe der lokalen Variablen überschreitet die Grenze für bestimmte CPUs
% This indicates that you are declaring more than 32K of local variables, which
% might not be supported on other processor targets.
cg_e_localsize_too_big=06043_E_Größe der lokalen Variablen überschreitet die unterstützte Grenze
% This indicates that you are declaring more than 32K of local variables, which
% is not supported by this processor.
cg_e_break_not_allowed=06044_E_BREAK nicht zulässig
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06045_E_CONTINUE nicht zulässig
% You're trying to use \var{continue} outside a loop construction.
cg_f_unknown_compilerproc=06046_F_Unbekannte Compiler-Prozedur "$1". Überprüfe, ob die korrekte Laufzeit-Bibliothek verwendet wird
% The compiler expects that the runtime library contains certain subroutines. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a subroutine which the compiler needs for internal use.
cg_f_unknown_system_type=06047_F_Systemtyp "$1" konnte nicht gefunden werden. Überprüfe, ob die korrekte Laufzeit-Bibliothek verwendet wird
% The compiler expects that the runtime library contains certain type definitions. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_h_inherited_ignored=06048_H_Geerbter Aufruf einer abstrakten Methode ignoriert
% This message appears only in Delphi mode when you call an abstract method
% of a parent class via \var{inherited;}. The call is then ignored.
cg_e_goto_label_not_found=06049_E_Goto Label "$1": Das Label ist nicht definiert oder wurde bei der Optimierung entfernt
% The label used in the goto definition is not defined or optimized away by the
% unreachable code elemination.
cg_f_unknown_type_in_unit=06050_F_Kann den Typ "$1" nicht in der Unit "$2" finden. Überprüfe, ob die korrekte Laufzeit-Bibliothek verwendet wird
% The compiler expects that the runtime library contains certain type definitions. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_e_interprocedural_goto_only_to_outer_scope_allowed=06051_E_Interprozedurale gotos sind nur in äußere Subroutines erlaubt
% Gotos between subroutines are only allowed if the goto jumps from an inner to an outer subroutine or
% from a subroutine to the main program
cg_e_labels_cannot_defined_outside_declaration_scope=06052_E_ Label müssen im selben Bereich definiert werden, in dem sie deklariert werden
% In ISO mode, labels must be defined in the same scope as they are declared.
cg_e_goto_across_procedures_with_exceptions_not_allowed=06053_E_Eine Prozedur, die explizite oder implizite Exception-Frames enthält, darf nicht mit einem goto verlassen werden
% Non-local gotos might not be used to leave procedures using exceptions either implicitly or explicitly. Procedures
% which use automated types like ansistrings or class constructurs are affected by this too.
cg_e_mod_only_defined_for_pos_quotient=06054_E_Im ISO-Modus ist der Operator mod nur für positive Quotienten definiert
% In ISO pascal, only positive values are allowed for the quotient: \var{n mod m} is only valid if \var{m>0}.
cg_d_autoinlining=06055_DL_Automatisches inlining: $1
% Due to auto inlining turned on, the compiler auto inlines this subroutine.
cg_e_function_not_support_by_selected_instruction_set=06056_E_Die verwendete Funktion wird vom ausgewählten Befehlssatz nicht unterstützt: $1
% Some functions cannot be implemented efficiently for certain instruction sets, one example is fused multiply/add.
% To avoid very inefficient code, the compiler complains in this case, so either select another instruction set
% or replace the function call by alternative code
cg_f_max_units_reached=06057_F_Maximale Anzahl an Units ($1) für das aktuelle Zielsystem erreicht
% Depending of target architecture, the number of units is limited. This limit
% has been reached. A unit counts only if it contains initialization or finalization count.
cg_n_no_inline=06058_N_Der Aufruf der Routine "$1" ist als "inline" markiert, wird aber nicht so benhandelt
% The directive inline is only a hint to the compiler. Sometimes the compiler ignores this hint, a subroutine
% marked as inline is not inlined. In this case, this hint is given. Compiling with \var{-vd} might result in more information why
% the directive inline is ignored.
cg_e_case_incomplete=06059_E_Case-Anweisung deckt nicht alle möglichen Fälle ab
cg_w_case_incomplete=06060_W_Case-Anweisung deckt nicht alle möglichen Fälle ab
% The case statement does not contain labels for all possible values of the operand, and no else statement is present.
%
% \end{description}
# EndOfTeX

#
# Assembler reader
#
# 07141 is the last used one
#
asmr_d_start_reading=07000_DL_Starte $1 Stil Assembler Parsen
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_DL_Beende $1 Stil Assembler Parsen
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_Nicht-Label Bezeichner enthält @
% A identifier which isn't a label cannot contain a @.
asmr_e_building_record_offset=07004_E_Fehler beim Ermitteln des Recordoffsets
% There has an error occurred while building the offset of a record/object
% structure, this can happen when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_OFFSET ohne Bezeichner verwendet
% You can only use OFFSET with an identifier. Other syntaxes are not
% supported
asmr_e_type_without_identifier=07006_E_TYPE verwendet ohne Bezeichner
% You can only use TYPE with an identifier. Other syntaxes are not
% supported
asmr_e_no_local_or_para_allowed=07007_E_Lokale Variable oder Parameter können hier nicht verwendet werden
% You cannot use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the %ebp register so the
% address cannot be get directly.
asmr_e_need_offset=07008_E_Hier muss OFFSET verwendet werden
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_Hier muss "$" verwendet werden
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Verwendung von mehreren verschiebbaren Symbolen nicht möglich
% You cannot have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_Verschiebbares Symbol kann nur addiert werden
% Relocatable symbols (variable/typed constant) cannot be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_Ungültiger Konstantenausdruck
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_Verschiebbares Symbol ist nicht zulässig
% You cannot use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_Ungültige Verweis-Syntax
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_Sie können "$1" von diesem Code aus nicht erreichen
% You cannot read directly the value of a local variable or parameter
% of a higher level procedure in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_Lokale Symbole/Labels sind nicht als Referenz zulässig
% You cannot use local symbols/labels as references
asmr_e_wrong_base_index=07017_E_Ungültige Verwendung von Basis- und Index-Registern
% There is an error with the base and index register, they are
% probably incorrect
asmr_w_possible_object_field_bug=07018_W_Möglicher Fehler bei Objektfeld-Behandlung
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_Falscher Skalierungsfaktor angegeben
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_Mehrfache Verwendung fon Index-Registern
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_Ungültiger Operandentyp
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_Ungülitge Zeichenkette als Opcode-Operand: $1
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE und @DATA werden nicht unterstützt
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_Null-Label-Bezug nicht möglich
asmr_e_expr_zero_divide=07025_E_Division durch Null in Assembler-Ausdruck
% There is a division by zero in a constant expression
asmr_e_expr_illegal=07026_E_Ungültiger Ausdruck
% There is an illegal expression in a constant expression
asmr_e_escape_seq_ignored=07027_E_Escape-Sequenz ignoriert: $1
% There is a C-styled string, but the escape sequence in the string
% is unknown, and is therefore ignored
asmr_e_invalid_symbol_ref=07028_E_Ungültige Symbolverwendung
asmr_w_fwait_emu_prob=07029_W_FWAIT kann Emulationsprobleme mit emu387 verursachen
asmr_w_fadd_to_faddp=07030_W_$1 ohne Operand wurde in $1P übersetzt
asmr_w_enter_not_supported_by_linux=07031_W_Der ENTER-Befehl wird vom Linux-Kernel nicht unterstützt
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_Aufruf einer überladenen Funktion in Assembler
% There is a call to an overloaded method in the assembler block,
% this might be the sign there is a problem
asmr_e_unsupported_symbol_type=07033_E_Nicht unterstützter Symboltyp für Operand
asmr_e_constant_out_of_bounds=07034_E_Wert der Konstante außerhalb des zulässigen Bereichs
asmr_e_error_converting_decimal=07035_E_Fehler beim Umwandeln in Dezimal $1
% A constant decimal value does not have the correct syntax
asmr_e_error_converting_octal=07036_E_Fehler beim Umwandeln in Oktal $1
% A constant octal value does not have the correct syntax
asmr_e_error_converting_binary=07037_E_Fehler beim Umwandeln in Binär $1
% A constant binary value does not have the correct syntax
asmr_e_error_converting_hexadecimal=07038_E_Fehler beim Umwandeln in Hexadezimal $1
% A constant hexadecimal value does not have the correct syntax
asmr_h_direct_global_to_mangled=07039_H_$1 übersetzt nach $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 ist einer überladenen Funktion zugeordnet
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Kann SELF nicht außerhalb einer Methode verwenden
% There is a reference to the \var{self} symbol while it is not
% allowed. \var{self} can only be referenced inside methods
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Kann OLDEBP außerhalb einer verschachtelten Prozedur nicht verwenden
% There is a reference to the \var{oldebp} symbol while it is not
% allowed. \var{oldebp} can only be referenced inside nested routines
asmr_e_void_function=07043_W_Prozeduren können keinen Wert im Assembler-Code zurückliefern
% Trying to return a value while in a procedure. A procedure
% does not have any return value
asmr_e_SEG_not_supported=07044_E_SEG nicht unterstützt
asmr_e_size_suffix_and_dest_dont_match=07045_E_Größensuffix und Ziel- oder Quellgröße passen nicht zusammen
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_w_size_suffix_and_dest_dont_match=07046_W_Größensuffix und Ziel- oder Quellgröße passen nicht zusammen
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_e_syntax_error=07047_E_Assembler Syntaxfehler
% There is an assembler syntax error
asmr_e_invalid_opcode_and_operand=07048_E_Ungültige Kombination von Opcode und Operanden
% The opcode cannot be used with this type of operand
asmr_e_syn_operand=07049_E_Assembler Syntaxfehler im Operanden
asmr_e_syn_constant=07050_E_Assembler Syntaxfehler in Konstanten
asmr_e_invalid_string_expression=07051_E_Ungültiger Stringausdruck
asmr_w_const32bit_for_address=07052_W_Konstante mit Symbol $1 für Adresse erzeugt, die kein Zeiger ist
% A constant expression represents an address which does not fit
% into a pointer. The address is probably incorrect
asmr_e_unknown_opcode=07053_E_Unbekannter Opcode $1
% This opcode is not known
asmr_e_invalid_or_missing_opcode=07054_E_Ungültiger oder fehlender Opcode
asmr_e_invalid_prefix_and_opcode=07055_E_Ungültige Kombination von Prefix und Opcode: $1
asmr_e_invalid_override_and_opcode=07056_E_Ungültige Kombination von Override und Opcode: $1
asmr_e_too_many_operands=07057_E_Zu viele Operanden in der Zeile
% There are too many operands for this opcode. Check your
% assembler syntax
asmr_w_near_ignored=07058_W_NEAR ignoriert
asmr_w_far_ignored=07059_W_FAR ignoriert
asmr_e_dup_local_sym=07060_E_Doppelters lokales Symbol $1
asmr_e_unknown_local_sym=07061_E_Undefiniertes lokales Symbol $1
asmr_e_unknown_label_identifier=07062_E_Unbekannter Label-Bezeichner $1
asmr_e_invalid_register=07063_E_Ungültiger Registername
% There is an unknown register name used as operand.
asmr_e_invalid_fpu_register=07064_E_Ungültiger Name für Fliesskommaregister
% There is an unknown register name used as operand.
asmr_w_modulo_not_supported=07066_W_Modulo nicht unterstützt
asmr_e_invalid_float_const=07067_E_Ungültige Fliesskommakonstante $1
% The floating point constant declared in an assembler block is
% invalid.
asmr_e_invalid_float_expr=07068_E_Ungültiger Fliesskommaausdruck
% The floating point expression declared in an assembler block is
% invalid.
asmr_e_wrong_sym_type=07069_E_Falscher Symboltyp
asmr_e_cannot_index_relative_var=07070_E_Kann lokale Variable oder Parameter nicht mit Register indizieren
% Trying to index using a base register a symbol which is already relative
% to a register. This is not possible, and will probably lead to crashes.
asmr_e_invalid_seg_override=07071_E_Ungültiger Segmentoverride-Ausdruck
asmr_w_id_supposed_external=07072_W_Bezeichner $1 ist vermutlich External
% There is a reference to an undefined symbol. This will not result
% in an error, since the symbol might be external, but may cause
% problems at link time if the symbol is not defined anywhere.
asmr_e_string_not_allowed_as_const=07073_E_Strings sind als Konstanten unzulässig
% Character strings are not allowed as constants.
asmr_e_no_var_type_specified=07074_E_Typ der Variablen nicht angegeben
% The syntax expects a type identifier after the dot, but
% none was found.
asmr_w_assembler_code_not_returned_to_text=07075_E_Assemblercode kehrt nicht zum Text zurück
% There was a directive in the assembler block to change sections,
% but there is a missing return to the text section at the end
% of the assembler block. This might cause errors during link time.
asmr_e_not_directive_or_local_symbol=07076_E_Keine Direktive oder lokales Symbol $1
% This symbol is unknown.
asmr_w_using_defined_as_local=07077_E_Verwendung eines definierten Namens als lokales Label
asmr_e_dollar_without_identifier=07078_E_Dollarzeichen wird ohne Bezeichner verwendet
% A constant expression has an identifier which does not start with
% the $ symbol.
asmr_w_32bit_const_for_address=07079_W_32-Bit-Konstante für Adresse erzeugt
% A constant was used as an address. This is probably an error,
% since using absolute addresses will probably not work.
asmr_n_align_is_target_specific=07080_N_.align ist abhängig von Zielplattform, verwende .balign oder .p2align
% Using the .align directive is platform specific, and its meaning will vary
% from one platform to another.
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Kann bei Parametern, die auf Zeigern basieren, nicht direkt auf Felder zugreifen
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_Kann nicht direkt auf Felder von Objekten/Klassen zugreifen
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
asmr_e_unable_to_determine_reference_size=07083_E_Größe nicht spezifiziert und auch keine Möglichkeit die Größe der Operanden zu bestimmen
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference.
asmr_e_cannot_use_RESULT_here=07084_E_RESULT kann in dieser Funktion nicht verwendet werden
% Some functions which return complex types cannot use the \var{result}
% keyword.
asmr_w_adding_explicit_args_fXX=07086_W_"$1" ohne Operand übersetzt in "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" übersetzt in "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" übersetzt in "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_Das Zeichen < ist hier nicht erlaubt
% The shift operator requires the << characters. Only one
% of those characters was found.
asmr_e_invalid_char_greater=07090_E_Das Zeichen > ist hier nicht erlaubt
% The shift operator requires the >> characters. Only one
% of those characters was found.
asmr_w_align_not_supported=07093_W_ALIGN wird nicht unterstützt
asmr_e_no_inc_and_dec_together=07094_E_Inc und Dec können nicht gemeinsam vorkommen
% Trying to use an increment and a decrement within the same
% opcode on the 680x0. This is impossible.
asmr_e_invalid_reg_list_in_movem_or_fmovem=07095_E_Ungültige Registerliste für MOVEM oder FMOVEM
% Trying to use the \var{movem} or \var{fmovem} opcode with invalid registers
% to save or restore.
asmr_e_invalid_reg_list_for_opcode=07096_E_Ungültige Registerliste für diesen opcode
asmr_e_higher_cpu_mode_required=07097_E_Höherer cpu Modus notwendig ($1)
% Trying to use an instruction which is not supported in the current
% cpu mode. Use a higher cpu generation to be able to use this
% opcode in your assembler block
asmr_w_unable_to_determine_reference_size_using_dword=07098_W_Größe nicht spezifiziert und auch keine Möglichkeit die Größe der Operanden zu bestimmen. Es wird DWORD als Voreinstellung verwendet
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use DWORD as default.
asmr_e_illegal_shifterop_syntax=07099_E_Syntax Fehler beim Analysieren eines shifter Operanden
% ARM only; ARM assembler supports a so called shifter operand. The used syntax isn't
% a valid shifter operand. Example for an operation with shifter operand:
% \begin{verbatim}
% asm
%   orr     r2,r2,r2,lsl #8
% end;
% \end{verbatim}
asmr_e_packed_element=07100_E_Die Adresse der packed Komponente ist nicht an einer Byte Grenze
% Packed components (record fields and array elements) may start at an arbitrary
% bit inside a byte. On CPU which do not support bit-addressable memory (which
% includes all currently supported CPUs by FPC) you will therefore get an error
% message when trying to index arrays with elements whose size is not a multiple
% of 8 bits. The same goes for accessing record fields with such an address.
% multiple of 8 bits.
asmr_w_unable_to_determine_reference_size_using_byte=07101_W_Größe nicht angegeben und kann auch nicht aus der Größe der Operanden bestimmt werden. Verwende BYTE als Voreinstellung
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use BYTE as default.
asmr_w_no_direct_ebp_for_parameter=07102_W_Die Verwendung von +offset(%ebp) für Parameter ist hier ungültig
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_for_parameter_regcall=07103_W_Die Verwendung von +offset(%ebp) ist nicht mit der regcall Konvention kompatibel
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_neg_offset=07104_W_Die Verwendung von -offset(%ebp) wird für den Zugriff auf lokale Variablen nicht empfohlen
% Using -8(%ebp) to access a local variable is not recommended
asmr_w_direct_esp_neg_offset=07105_W_Verwendung von -offset(%esp); Zugriff kann einen Crash oder Datenverlust auslösen
% Using -8(%esp) to access a local stack is not recommended, as
% this stack portion can be overwritten by any function calls or interrupts.
asmr_e_no_vmtoffset_possible=07106_E_VMTOffset muss in Kombination mit einer virtuellen Methode verwendet werden; "$1" ist aber nicht virtuell
% Only virtual methods have VMT offsets
asmr_e_need_pic_ref=07107_E_Erzeuge eigentlich PIC, aber die Referenz ist nicht PIC-sicher
% The compiler has been configured to generate position-independent code
% (PIC), but there are position-dependent references in the current
% handwritten assembler instruction.
asmr_e_mixing_regtypes=07108_E_Alle Register in einem Registerset müssen in T und Breite übereinstimmen
% Instructions on the ARM architecture that take a register set as argument require that all registers
% in this set are of the same kind (e.g., integer, vfp) and width (e.g., single precision, double precision).
asmr_e_empty_regset=07109_E_Ein Registerset kann nicht leer sein
% Instructions on the ARM architecture that take a register set as argument require that such a set
% contains at least one register.
asmr_w_useless_got_for_local=07110_W_@GOTPCREL ist nutzlos und bei lokalen Symbole möglicherweise gefährlich
% The use of @GOTPCREL supposes an extra indirection that is
% not present if the symbol is local, which might lead to wrong assembler code
asmr_w_general_segment_with_constant=07111_W_Konstante in einem "general purpose segment" Register
% General purpose register should not have constant offsets
% as OS memory allocation might not be compatible with that.
asmr_e_bad_seh_directive_offset=07112_E_Ungültiger Offsetwert für $1
% Win64 SEH directives have certain restrictions on possible offset values, e.g. they should
% be positive and have 3 or 4 low bits clear.
asmr_e_bad_seh_directive_register=07113_E_Ungültiges Register für $1
% Win64 SEH directives accept only 64-bit integer registers or XMM registers.
asmr_e_seh_in_pure_asm_only=07114_E_SEH-Direktiven sind nur in reinen Assemblerroutinen erlaubt
% Win64 SEH directives are allowed only in pure assembler procedures, not in assembler
% blocks of regular procedures.
asmr_e_unsupported_directive=07115_E_Die Direktive "$1" wird für das aktuelle Ziel nicht unterstützt
asmr_e_complex_function_result_location=07116_E_Der Platz des Ergebnisses dieser Funktion nicht direkt in einem einzigen Operanden kodiert werden, wenn "nostackframe" verwendet wird
% Functions declared with the \var{nostackframe} modifier do not have a stack frame, and hence
% do not have a local variable to hold the temporary function result.  Accesses to the function
% result in such routines directly go to the registers or memory location that will be used
% to return their result to the caller.  In some cases this result may be spread over multiple
% registers, in which case it is not possible to access the location via its symbolic name.
% You have to directly use the appropriate register names in this case.
asmr_e_wrong_gotpcrel_intel_syntax=07117_E_GOTPCREL Referenzen in der Intel Assembler-Syntax können kein Basis- oder Index-Register enthalten und ihre Offsets müssen 0 sein.
% The syntax for a gotpcrel PIC memory expression in Intel assembler syntax is
% \verb*[global_symbol wrt ..gotpcrel]*
asmr_e_no_gotpcrel_support=07118_E_GOTPCREL Relocations werden vom aktuellen Ziel nicht unterstützt
% Not all targets support position-independent code using a global offset table.
% Use a different way to access symbols in a position-indepent way in these cases.
asmr_w_global_access_without_got=07119_W_Auf exportierte/globale Symbole sollte via GOT zugegriffen werden
% Global symbols (symbols from the unit interface, or defined in a program
% or library) should be accessed via the GOT when generating position-indepent code.
asmr_w_check_mem_operand_size=07120_W_Überprüfe die Größe des Speicheroperanden "$1"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_size3=07121_W_Überprüfe die Größe des Speicheroperanden "$1: Sie ist $2 bits, aber [$3 bits] werden erwartet"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_size_offset=07122_W_Überprüfe die Größe des Speicheroperanden "$1: Sie ist $2 bits, aber[$3 bits + $4 byte offset] werden erwartet"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_negative_offset=07123_W_Überprüfe "$1: Der Offset des Speicheroperanden ist negativ "$2 byte"
% The offset of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_automap_multiple_size=07124_W_Überprüfe "$1: Die Größe des Speicheroperanden ist leer, aber es existieren andere Definitionen der Speichergröße =>> abbilden auf $2 (kleinste Option)"
% The size of memory operand is empty and we have different definitions of possible memory sizes. Check it!
asmr_e_invalid_ref_register=07125_E_Ungültiges Register in Speicherreferenzausdruck: "$1"
% FPU, vector and sometimes integer registers cannot be used in memory reference
% expressions, due to limitations of the cpu architecture or simple because
% it is not meaningful.
asmr_e_seg_without_identifier=07126_E_SEG ohne Bezeichner benutzt
% You can only use SEG with an identifier. Other syntaxes are not
% supported
asmr_e_CODE_or_DATA_without_SEG=07127_E_@CODE und @DATA können nur mit dem SEG-Operator benutzt werden
% You can only use @CODE and @DATA symbols together with the SEG operator
asmr_e_const16bit_for_segment=07128_E_Nicht ausreichend Platz (16 Bit benötigt) für die Segment-Konstante des Symbols $1
% Specifying a segment constant for a symbol via the SEG operator requires at
% least 16 bits of space for the segment. This error occurs, if you specify
% less, for example, if you use 'DB SEG symbol' instead of 'DW SEG symbol'.
asmr_e_invalid_code_value=07129_E_Ungültiger Wert des Arguments für die .code-Direktive
% The ARM assembler only allows the values 16 and 32 to be used as arguments to the .code directive
asmr_w_unable_to_determine_constant_size_using_byte=07130_W_Keine Größe für die Konstante angegeben und es konnte auch keine bestimmt werden. Es wird BYTE als Voreinstellung verwendet.
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte, word, dword, etc.) it
% should use for the constant. Based on its value, BYTE is used.
asmr_w_unable_to_determine_constant_size_using_word=07131_W_Keine Größe für die Konstante angegeben und es konnte auch keine bestimmt werden. Es wird WORD als Voreinstellung verwendet.
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte, word, dword, etc.) it
% should use for the constant. Based on its value, WORD is used.
asmr_e_cannot_override_es_segment=07132_E_Kann ES Segment nicht überschreiben
% The ES segment in the ES:[EDI] reference of the x86 string instructions
% cannot be overridden.
asmr_w_invalid_reference=07133_W_Referenz ist hier ungültig (erwartete "$1")
% Certain x86 instructions require a fixed source or destination reference
% (e.g. [ESI] or [EDI] for the string instructions source and destination)
asmr_e_address_sizes_do_not_match=07134_E_Adressgrößen passen nicht
% Caused by using two memory operands in the same instruction with mismatched
% address sizes (e.g. movs byte ptr [EDI], byte ptr [SI] )
asmr_e_pop_cs_not_valid=07135_E_Die Instruktion "POP CS" ist für das aktuelle Ziel ungültig
% The 'pop cs' instruction works only on the 8086 and 8088 CPUs, which are not
% supported on the i386 or x86_64 targets.
asmr_w_pop_cs_not_portable=07136_W_Die Instruktion "POP CS" ist nicht portabel (es funktioniert nur mit 8086 und 8088 CPUs)
% The 'pop cs' instruction doesn't work on any CPU, except 8086 and 8088.
asmr_e_public_must_be_used_before_label_definition=07137_E_Das Label $1 kann nur "public" deklariert werden, bevor es definiert wird
asmr_e_local_label_cannot_be_declared_public=07138_E_Das lokale Label $1 kann nicht als "public" deklariert werden
asmr_e_multiple_segment_overrides=07139_E_Kann "multiple segment overrides" nicht benutzen
asmr_w_multiple_segment_overrides=07140_W_"Multiple segment overrides" (nur das letzte wird beachtet)
asmr_w_segment_override_ignored_in_64bit_mode=07141_W_Segment-Basis $1 wird erzeugt, aber von der CPU im 64-Bit-Modus ignoriert
%
% \end{description}
# EndOfTeX

#
# Assembler/binary writers
#
# 08033 is the last used one
#
asmw_f_too_many_asm_files=08000_F_Zu viele Assembler-Dateien
% With smartlinking enabled, there are too many assembler
% files generated. Disable smartlinking.
asmw_f_assembler_output_not_supported=08001_F_Gewählte Assemblerausgabe wird nicht unterstützt
asmw_f_comp_not_supported=08002_F_Comp nicht unterstützt
asmw_f_direct_not_supported=08003_F_Direct nicht unterstützt für binäres Schreiben
% Direct assembler mode is not supported for binary writers.
asmw_e_alloc_data_only_in_bss=08004_E_Allozieren von Daten ist nur in "bss"-Abschnitten zulässig
asmw_f_no_binary_writer_selected=08005_F_Kein Binärschreiber ausgewählt
asmw_e_opcode_not_in_table=08006_E_Asm: Opcode $1 nicht in Tabelle enthalten
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 ungültige Kombination von Opcode und Operanden
asmw_e_16bit_not_supported=08008_E_Asm: 16-Bit-Verweise werden nicht unterstützt
asmw_e_invalid_effective_address=08009_E_Asm: Ungültige effektive Adresse
asmw_e_immediate_or_reference_expected=08010_E_Asm: Konstanter Ausdruck oder Referenz erwartet
asmw_e_value_exceeds_bounds=08011_E_Asm: $1 Wert überschreitet Grenzen $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: "Short jump" ist außerhalb des Bereichs $1
asmw_e_undefined_label=08013_E_Asm: Undefiniertes Label: $1
asmw_e_comp_not_supported=08014_E_Asm: Comp wird für dieses Ziel nicht unterstützt
asmw_e_extended_not_supported=08015_E_Asm: Extended Typ wird für dieses Ziel nicht unterstützt
asmw_e_duplicate_label=08016_E_Asm: Doppeltes Label $1
asmw_e_redefined_label=08017_E_Asm: Neu definiertes Label $1
asmw_e_first_defined_label=08018_E_Asm: First beginnt hier
asmw_e_invalid_register=08019_E_Asm: Ungültiges Register $1
asmw_e_16bit_32bit_not_supported=08020_E_Asm: 16 oder 32 Bit Referenzen werden nicht unterstützt
asmw_e_64bit_not_supported=08021_E_Asm: 64 Bit Operanden werden nicht unterstützt
asmw_e_bad_reg_with_rex=08022_E_Asm: AH, BH, CH oder DH können nicht in einer Instruktion verwendt werden, die den Prefix REX benötigt
% x86_64 only: instruction encoding of this platform does not allow using
% 8086 high byte registers (AH, BH, CH or DH) together with REX prefix in a single instruction.
% The REX prefix is required whenever the instruction operand size is 64 bits, or
% when it uses one of extended x86_64 registers (R8-R15 or XMM8-XMM15).
asmw_e_missing_endprologue=08023_E_Die ".seh_endprologue"-Direktive fehlt
% x86_64-win64 only: Normally, SEH directives are handled internally by compiler.
% However, in pure assembler procedures .seh_endprologue directive is required
% if other SEH directives are present.
asmw_e_prologue_too_large=08024_E_Der Funktionsprolog überschreitet 255 Byte
% x86_64-win64: .seh_prologue directive must be placed within 255 bytes from function start.
asmw_e_handlerdata_no_handler=08025_E_".seh_handlerdata"-Direktive ohne vorangehendes ".seh_handler"
% x86_64-win64: If .seh_handlerdata directive is used, then a .seh_handler directive must be
% present earlier in the same function.
asmw_f_too_many_relocations=08026_F_Relocationzähler für Sektion $1 übersteigt 65535
% Legacy COFF targets limit number of relocations per section to 65535 because they use a 2-byte field
% to store the relocation count. Targets using newer PECOFF format do not have this limitation.
asmw_w_changing_bind_type=08027_N_Änderung des Bind-Typs des Symbols $1 von $2 nach $3 nach seiner Verwendung
asmw_h_changing_bind_type=08028_H_Änderung des Bind-Typs des Symbols $1 von $2 nach $3 nach seiner Verwendung
% An assembler symbol bind type has been altered after use, which can lead to wrong code.
% First version is reserved for changig to local label, which is the most probable cause
% of wrong code generation, but currently set to Note level as it appears inside
% the compiler compilation.
asmw_e_32bit_not_supported=08029_E_Asm: 32 Bit Referenzen werden nicht unterstützt
asmw_f_code_segment_too_large=08030_F_Code-Segment zu groß
asmw_f_data_segment_too_large=08031_F_Data-Segment zu groß
asmw_e_instruction_not_supported_by_cpu=08032_E_Befehl wird vom ausgewählten Befehlssatz nicht unterstützt
asmw_e_brxx_out_of_range=08033_E_Asm: Das Ziel des bedingten Zweigs ist außerhalb des Bereichs
%
% \end{description}
# EndOfTeX

#
# Executing linker/assembler
#
# 09035 is the last used one
#
# BeginOfTeX
%
% \section{Errors of assembling/linking stage}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
exec_w_source_os_redefined=09000_W_Quellbetriebssystem wurde neu definiert
% The source operating system is redefined.
exec_i_assembling_pipe=09001_I_Assembliere (pipe) $1
% Assembling using a pipe to an external assembler.
exec_d_cant_create_asmfile=09002_E_Kann Assemblerdatei nicht erzeugen: $1
% The mentioned file cannot be created. Check if you have
% access permissions to create this file.
exec_e_cant_create_objectfile=09003_E_Kann Objektdatei nicht erzeugen: $1 (Fehlercode: $2)
% The mentioned file cannot be created. Check if you have
% got access permissions to create this file.
exec_e_cant_create_archivefile=09004_E_Kann Archivdatei nicht erzeugen: $1
% The mentioned file cannot be created. Check if you have
% access permissions to create this file.
exec_e_assembler_not_found=09005_E_Assembler $1 nicht gefunden, schalte um zu externem Assemblieren
% The assembler program was not found. The compiler will produce a script that
% can be used to assemble and link the program.
exec_t_using_assembler=09006_T_Benutze Assembler: $1
% An informational message saying which assembler is being used.
exec_e_error_while_assembling=09007_E_Fehler beim Assemblieren des Exitcodes $1
% There was an error while assembling the file using an external assembler.
% Consult the documentation of the assembler tool to find out more information
% on this error.
exec_e_cant_call_assembler=09008_E_Assembler kann nicht aufgerufen werden, Fehler $1 schalte um zu externem Assemblieren
% An error occurred when calling an external assembler. The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_assembling=09009_I_Assembliere $1
% An informational message stating which file is being assembled.
exec_i_assembling_smart=09010_I_Assembliere mit Smartlinking $1
% An informational message stating which file is being assembled using smartlinking.
exec_w_objfile_not_found=09011_W_Objekt $1 nicht gefunden, Linken kann fehlschlagen!
% One of the object files is missing, and linking will probably fail.
% Check your paths.
exec_w_libfile_not_found=09012_W_Bibliothek $1 nicht gefunden, Linken kann fehlschlagen!
% One of the library file is missing, and linking will probably fail.
% Check your paths.
exec_e_error_while_linking=09013_E_Fehler beim Linken
% Generic error while linking.
exec_e_cant_call_linker=09014_E_Linker kann nicht aufgerufen werden, schalte um zu externem Linken
% An error occurred when calling an external linker. The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_linking=09015_I_Linke $1
% An informational message, showing which program or library is being linked.
exec_e_util_not_found=09016_E_Hilfsprogramm $1 nicht gefunden, schalte um zu externem Linken
% An external tool was not found. The compiler will produce a script that
% can be used to assemble and link or postprocess the program.
exec_t_using_util=09017_T_Benutze Hilfsprogramm $1
% An informational message, showing which external program (usually a postprocessor) is being used.
exec_e_exe_not_supported=09018_E_Erzeugen von ausführbaren Dateien nicht unterstützt
% Creating executable programs is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_e_dll_not_supported=09019_E_Dynamische Bibliotheken nicht unterstützt
% Creating dynamically loadable libraries is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_i_closing_script=09020_I_Schließe Skript $1
% Informational message showing when writing of the external assembling and linking script is finished.
exec_e_res_not_found=09021_E_Resource-Compiler "$1" nicht gefunden, schalte um auf externen Modus
% An external resource compiler was not found. The compiler will produce a script that
% can be used to assemble, compile resources and link or postprocess the program.
exec_i_compilingresource=09022_I_Übersetze Resource $1
% An informational message, showing which resource is being compiled.
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_Unit $1 kann nicht statisch gelinkt werden, schalte um zu smart Linken
% Static linking was requested, but a unit which is not statically linkable was used.
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_Unit $1 kann nicht smart gelinkt werden, schalte um zu statischem Linken
% Smart linking was requested, but a unit which is not smart-linkable was used.
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_Unit $1 kann nicht shared gelinkt werden, schalte um zu statischem Linken
% Shared linking was requested, but a unit which is not shared-linkable  was used.
exec_e_unit_not_smart_or_static_linkable=09026_E_Unit $1 kann weder smart noch statisch gelinkt werden
% Smart or static linking was requested, but a unit which cannot be used for either  was used.
exec_e_unit_not_shared_or_static_linkable=09027_E_Unit $1 kann weder shared noch statisch gelinkt werden
% Shared or static linking was requested, but a unit which cannot be used for either  was used.
exec_d_resbin_params=09028_D_Resource-Compiler "$1" wird mit "$2" als Kommandozeile aufgerufen
% An informational message showing which command line is used for the resource compiler.
exec_e_error_while_compiling_resources=09029_E_Fehler beim Übersetzen von Resourcen
% The resource compiler or converter returned an error.
exec_e_cant_call_resource_compiler=09030_E_Der Resource-Compiler "$1" kann nicht aufgerufen werden, schalte um auf externen Modus
% An error occurred when calling a resource compiler. The compiler will produce
% a script that can be used to assemble, compile resources and link or
% postprocess the program.
exec_e_cant_open_resource_file=09031_E_Kann die Resourcedatei "$1" nicht öffnen
% An error occurred resource file cannot be opened.
exec_e_cant_write_resource_file=09032_E_Kann die Resourcedatei "$1" nicht schreiben
% An error occurred resource file cannot be written.
exec_n_backquote_cat_file_not_found=09033_N_Die Datei "$1" wurde mit dem Kommando cat nicht gefunden
% The compiler did not find the file that should be expanded into linker parameters
exec_w_init_file_not_found=09034_W_"$1" nicht gefunden; dies wird wahrscheinlich zu einem Fehler beim Linken führen
% The compiler adds certain startup code files to the linker only when they are found.
% If they are not found, they are not added and this might cause a linking failure.
% If the system has gcc installed, running \var{gcc --print-file-name <filename>} might return the path to the file.
% Add this path in your \var{fpc.cfg} using the switch \var{-Fl} to fix this error. This requires though,
% that gcc targets the same target as FPC.
exec_e_static_lib_not_supported=09035_E_Statische Bibliotheken nicht unterstützt
% Creating static libraries is not supported for this platform, because it was
% not yet implemented in the compiler.
%
% \end{description}
# EndOfTeX

#
# Executable information
#
# 09134 is the last used one
#
# BeginOfTeX
%
% \section{Executable information messages.}
% This section lists all messages that the compiler emits when an executable program is produced,
% and only when the internal linker is used.
% \begin{description}
execinfo_f_cant_process_executable=09128_F_Kann ausführbare Datei nicht nachbearbeiten: $1
% Fatal error when the compiler is unable to post-process an executable.
execinfo_f_cant_open_executable=09129_F_Kann ausführbare Datei nicht öffnen: $1
% Fatal error when the compiler cannot open the file for the executable.
execinfo_x_codesize=09130_X_Größe des Codes: $1 Byte
% Informational message showing the size of the produced code section.
execinfo_x_initdatasize=09131_X_Größe der initialisierten Daten: $1 Byte
% Informational message showing the size of the initialized data section.
execinfo_x_uninitdatasize=09132_X_Größe der nicht initialisierten Daten: $1 Byte
% Informational message showing the size of the uninitialized data section.
execinfo_x_stackreserve=09133_X_Stack Bereich "reserved": $1 Byte
% Informational message showing the stack size that the compiler reserved for the executable.
execinfo_x_stackcommit=09134_X_Stack Bereich "committed": $1 Byte
% Informational message showing the stack size that the compiler committed for the executable.
%
% \end{description}
# EndOfTeX

#
# Internal linker messages
#
# 09220 is the last used one
#
# BeginOfTeX
%
% \section{Linker messages}
% This section lists messages produced by internal linker.
% \begin{description}
link_f_executable_too_big=09200_F_Das Programm - Image ist für das Target $1 zu groß
% Fatal error when resulting executable is too big.
link_w_32bit_absolute_reloc=09201_W_Objektdatei "$1" enthält eine 32-bit absolute Relocation auf Symbol "$2".
% Warning when 64-bit object file contains 32-bit absolute relocations.
% In such case an executable image can be loaded into lower 4Gb of
% address space only.
link_e_program_segment_too_large=09202_E_Program-Segment zu groß (übersteigt 64k um $1 Byte)
% Error when a 16-bit program is compiled in the tiny memory model, but its size exceeds 64k
link_e_code_segment_too_large=09203_E_Code-Segment "$1" zu groß (übersteigt 64k um $2 Byte)
% Error when a 16-bit program's code segment exceeds 64k bytes
link_e_data_segment_too_large=09204_E_Data-Segment "$1" zu groß (übersteigt 64k um $2 Byte)
% Error when a 16-bit program's data segment exceeds 64k bytes
link_e_segment_too_large=09205_E_Segment "$1" zu groß (übersteigt 64k um $2 Byte)
% Error when a 16-bit program contains a segment that exceeds 64k bytes
link_e_group_too_large=09206_E_Gruppe "$1" zu groß (übersteigt 64k um $2 Byte)
% Error when a 16-bit program's object modules define a segment group that
% exceeds 64k bytes
link_e_com_program_uses_segment_relocations=09207_E_Eine .COM-Datei kann nicht erzeugt werden, weil das Programm Segment-Relocations enthält
% Error occurs, when creating a tiny model DOS .COM file, but at least one of
% the program's object modules contains segment relocations. Segment relocations
% might be caused by the use of the Seg() function or by the SEG assembler
% directive (either in pascal's built-in inline assembler, or in an externally
% linked assembly module).
link_w_program_uses_checkpointer=09208_W_Das Programm "$1" benutzt die experimentelle CheckPointer-Option
link_e_duplicate_symbol=09209_E_Mehrfach definiertes Symbol "$1"
% The specified symbol is already defined inside the whole collection of object files.
link_e_comdat_select_unsupported=09210_E_COMDAT-Auswahlmodus $1 nicht untersützt (Sektion: "$1")
% The specified COMDAT selection mode is not supported.
link_e_comdat_associative_section_expected=09211_E_Assoziative Sektion für die COMDAT-Sektion "$1" erwartet
% The specified COMDAT section is specified as expecting an associative section,
% but none is specified.
link_e_comdat_not_matching=09212_E_COMDAT-Sektion-Auswahlmodus für Sektion "$1" und Symbol "$2" stimmen nicht überein
% All COMDAT symbols/sections need to use the same selection mode.
link_e_comdat_associative_section_not_found=09213_E_Assoziative COMDAT-Sektion für Sektion "$1" nicht gefunden
% The COMDAT section expects an associative section, but it was not found inside the object file.
link_d_comdat_discard_any=09214_D_Verwerfe das doppelte Symbol "$1" wegen des COMDAT-Auswahlmodus
% The COMDAT section specifies that any section with the same name might be selected and this
% specific section was selected to be discarded.
link_d_comdat_discard_size=09215_D_Verwerfe das doppelte Symbol "$1" mit der selben Größe wegen des COMDAT-Auswahlmodus
% The COMDAT section specifies that any section with the same name and size might be selected
% and this specific section was selected to be discarded.
link_d_comdat_discard_content=09216_D_Verwerfe das doppelte Symbol "$1" mit dem selben Inhalt wegen des COMDAT-Auswahlmodus
% The COMDAT section specifies that any section with the same name and content might be selected
% and this specific section was selected to be discarded.
link_d_comdat_replace_size=09217_D_Ersetze das doppelte Symbol "$1" mit kleinerer Größe wegen des COMDAT-Auswahlmodus
% The COMDAT section specifies that the largest section with the same name should be selected
% this specific section was larger than the previous largest one.
link_e_comdat_size_differs=09218_E_Größe des doppelten COMDAT-Symbols "$1" unterscheidet sich
% The COMDAT section specifies that all sections with the same name need to have the same size,
% but this section had a different size.
link_e_comdat_content_differs=09219_E_Inhalt des doppelten COMDAT-Symbols "$1" unterscheidet sich
% The COMDAT section specifies that all sections with the same name need to have the same content,
% but this section had a different size.
link_e_comdat_selection_differs=09220_E_COMDAT-Auswahlmodus für das Symbol "$1" unterscheidet sich%
%
% \end{description}
# EndOfTeX

#
# Unit loading
#
# 10066 is the last used one
#
# BeginOfTeX
%
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these mesages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_Suche Unit: $1
% When you use the \var{-vt} option, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_Lade PPU: $1
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_PPU-Name: $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_PPU-Flags: $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_PPU-CRC: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_PPU-Zeit: $1
% When you use the \var{-vu} flag, the unit time is shown.
unit_u_ppu_file_too_short=10006_U_PPU-Datei zu kurz
% When you use the \var{-vu} flag, the unit time is shown.
unit_u_ppu_invalid_header=10007_U_PPU Ungültiger Header (kein PPU am Anfang)
% A unit file contains as the first three bytes the ASCII codes of the characters \var{PPU}.
unit_u_ppu_invalid_version=10008_U_PPU Ungültige Version $1
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_PPU ist für einen anderen Prozessor übersetzt
% This unit file was compiled for a different processor type, and
% cannot be read.
unit_u_ppu_invalid_target=10010_U_PPU ist für ein anderes Zielsystem übersetzt
% This unit file was compiled for a different processor type, and
% cannot be read.
unit_u_ppu_source=10011_U_PPU Quelle: $1
% When you use the \var{-vu} flag, the unit source file name is shown.
unit_u_ppu_write=10012_U_Schreibe $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_Kann PPU-Datei nicht schreiben
% An error occurred when writing the unit file.
unit_f_ppu_read_error=10014_F_Kann PPU-Datei nicht lesen
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_read_unexpected_end=10015_F_Unerwartetes Ende der PPU-Datei
% Unexpected end of file. This may mean that the PPU file is
% corrupted.
unit_f_ppu_invalid_entry=10016_F_Ungültiger Eintrag in PPU-Datei: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_PPU Dbx Zähler-Problem
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_Ungültiger Unitname: $1 (erwartete $2)
% The name of the unit doesn't match the file name.
% There might to two reasons: either there is a spelling mistake in the unit name
% or there is a unit with a 8.3 name where the 8 characters are equal to first 8 characters of
% the name of a unit with a longer name. However, this unit is not found. Example: Program contains
% \var{uses mytestunit;}, the unit file or source of mytestunit are not available but there is a source
% with the name \var{mytestun}. Then compiler tries to compile and use that one, however the expected unit name does not match.
unit_f_too_much_units=10019_F_Zu viele Units
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{files.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_Gegenseitige Abhängigkeit von Units zwischen $1 und $2
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_Kann Unit "$1" nicht übersetzen, keine Quellen vorhanden
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_Kann Unit "$1", die von "$2" benutzt wird, nicht finden
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your configuration file for the unit paths.
unit_w_unit_name_error=10023_W_Unit "$1" wurde nicht gefunden, aber "$2" existiert
% This error message is no longer used.
unit_f_unit_name_error=10024_F_Unit "$1" gesucht, aber "$2" gefunden
% \dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_Übersetzen der Systemunit erfordert den Schalter -Us
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} switch must be specified.
unit_f_errors_in_unit=10026_F_Es traten $1 Fehler beim Übersetzen des Moduls auf, halte an
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_Lade aus $1 ($2) die Unit $3
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_Übersetze $1 erneut, Prüfsumme für $2 hat sich geändert
% The unit is recompiled because the checksum of a unit it depends on has
% changed.
unit_u_recompile_source_found_alone=10029_U_Übersetze "$1", nur Quellcode gefunden
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_Übersetze Unit erneut, statische Bibliothek ist älter als PPU-Datei
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit is older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_Übersetze Unit erneut, gemeinsame Bibliothek ist älter als PPU-Datei
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit is older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_Übersetze Unit erneut, obj und asm sind älter als PPU-Datei
% When you use the \var{-vu} flag, the compiler warns if the assembler of
% object file of the unit is older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_Übersetze Unit erneut, obj ist älter als asm
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_parsing_interface=10034_U_Analysiere Interface von $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit.
unit_u_parsing_implementation=10035_U_Analysiere Implementation von $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit.
unit_u_second_load_unit=10036_U_Zweites Laden für Unit "$1"
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happen with
% interdependent units.
unit_u_check_time=10037_U_PPU prüfe Datei $1 Zeit $2
% When you use the \var{-vu} flag, the compiler shows the filename and
% date and time of the file on which a recompile depends.
### The following two error msgs is currently disabled.
#unit_h_cond_not_set_in_last_compile=10038_H_Conditional $1 wurde beim Start beim letzten Übersetzen von $2 nicht gesetzt
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompilation. The compiler has
#% found a conditional that currently is defined, but was not used the last
#% time the unit was compiled.
#unit_h_cond_set_in_last_compile=10039_H_Conditional $1 wurde beim Start beim letzten Übersetzen von $2 gesetzt
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompilation. The compiler has
#% found a conditional that was used the last time the unit was compiled, but
#% the conditional is currently not defined.
unit_w_cant_compile_unit_with_changed_incfile=10040_W_Kann Unit $1 nicht erneut übersetzen, aber geänderte Include-Datei gefunden
% A unit was found to have modified include files, but
% some source files were not found, so recompilation is impossible.
unit_u_source_modified=10041_U_Datei $1 ist neuer als die, aus der die PPU Datei $2 erzeugt wird
% A modified source file for a compiler unit was found.
unit_u_ppu_invalid_fpumode=10042_U_Versuch eine Unit zu verwenden, die in einem anderen FPU Mode übersetzt wurde
% Trying to compile code while using units which were not compiled with
% the same floating point format mode. Either all code should be compiled
% with FPU emulation on, or with FPU emulation off.
unit_u_loading_interface_units=10043_U_Interface Units werden von $1 geladen
% When you use the \var{-vu} flag, the compiler warns that it is starting
% to load the units defined in the interface part of the unit.
unit_u_loading_implementation_units=10044_U_Implementation Units werden von $1 geladen
% When you use the \var{-vu} flag, the compiler warns that it is starting
% to load the units defined in the implementation part of the unit.
unit_u_interface_crc_changed=10045_U_Geänderte Interface CRC für Unit $1
% When you use the \var{-vu} flag, the compiler warns that the
% CRC calculated for the interface has been changed after the implementation
% has been parsed.
unit_u_implementation_crc_changed=10046_U_Geänderte Implementation CRC für Unit $1
% When you use the \var{-vu} flag, the compiler warns that the
% CRC calculated has been changed after the implementation
% has been parsed.
unit_u_finished_compiling=10047_U_Übersetzen der Unit $1 beendet
% When you use the \var{-vu} flag, the compiler warns that it
% has finished compiling the unit.
unit_u_add_depend_to=10048_U_Abhängigkeit hinzufügen: $1 hängt von $2 ab
% When you use the \var{-vu} flag, the compiler warns that it
% has added a dependency between the two units.
unit_u_no_reload_is_caller=10049_U_Kein erneutes Laden, Unit $1 ist die Aufrufende
% When you use the \var{-vu} flag, the compiler warns that it
% will not reload the unit because it is the unit that wants
% to load this unit.
unit_u_no_reload_in_second_compile=10050_U_Kein erneutes Laden der Unit, bereits beim zweiten Übersetzen: $1
% When you use the \var{-vu} flag, the compiler warns that it
% will not reload the unit because it is already in a second recompile.
unit_u_flag_for_reload=10051_U_Flag für erneutes Laden: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to reload the unit.
unit_u_forced_reload=10052_U_Erzwungenes erneutes Laden
% When you use the \var{-vu} flag, the compiler warns that it
% is reloading the unit because it was required.
unit_u_previous_state=10053_U_Vorhergehender Status von $1: $2
% When you use the \var{-vu} flag, the compiler shows the
% previous state of the unit.
unit_u_second_compile_unit=10054_U_$1 wird bereits übersetzt, zweites Übersetzen gesetzt
% When you use the \var{-vu} flag, the compiler warns that it is starting
% to recompile a unit for the second time. This can happen with interdependent
% units.
unit_u_loading_unit=10055_U_Unit $1 wird geladen
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the unit.
unit_u_finished_loading_unit=10056_U_Laden der Unit $1 beendet
% When you use the \var{-vu} flag, the compiler warns that it finished
% loading the unit.
unit_u_registering_new_unit=10057_U_Registrierung der neuen Unit $1
% When you use the \var{-vu} flag, the compiler warns that it has
% found a new unit and is registering it in the internal lists.
unit_u_reresolving_unit=10058_U_Erneutes resolving der Unit $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to recalculate the internal data of the unit.
unit_u_skipping_reresolving_unit=10059_U_Erneutes Resolving der Unit $1 wird übersprungen, benutzte Units werden noch geladen
% When you use the \var{-vu} flag, the compiler warns that it is
% skipping the recalculation of the internal data of the unit
% because there is no data to recalculate.
unit_u_unload_resunit=10060_U_Entlade die Resource-Unit $1 (wird nicht benötigt) 
% When you use the \var{-vu} flag, the compiler warns that it is unloading the 
% resource handling unit, since no resources are used.
unit_e_different_wpo_file=10061_E_Unit $1 wurde mit einer anderen Feedback-Eingabe ($2, $3) für die Gesamtprogramm-Optimierung (wpo) übersetzt. Bitte erneut ohne wpo oder mit der gleichen wpo-Feedback-Eingabe-Datei übersetzen
% When a unit has been compiled using a particular whole program optimization (wpo) feedback file (\var{-FW<x>} \var{-OW<x>}),
% this compiled version of the unit is specialised for that particular compilation scenario and cannot be used in
% any other context. It has to be recompiled before you can use it in another program or with another wpo feedback input file.
unit_u_indirect_crc_changed=10062_U_Die CRC des indirekten Interface (Objekte/Klassen) für die unit $1 hat sich geändert
% When you use the \var{-vu} flag, the compiler warns that the
% indirect CRC calculated for the unit (this is the CRC of all classes/objects/interfaces/$\ldots$
% in the interfaces of units directly or indirectly used by this unit in the interface) has been changed after the
% implementation has been parsed.
unit_u_ppu_invalid_memory_model=10063_U_PPU ist für ein anderes i8086 Speichermodell kompiliert
% This unit file was compiled for a different i8086 memory model and
% cannot be read.
unit_u_loading_from_package=10064_U_Unit $1 wird von Paket $2 geladen
% The unit is loaded from a package.
cg_f_internal_type_not_found=10065_F_Interner Typ "$1" wurde nicht gefunden. Überprüfen sie, ob sie die richtige Laufzeit-Bibliothek verwenden.
% The compiler expects that the runtime library contains certain types. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_f_internal_type_does_not_match=10066_F_Interner Typ "$1" ist unerwartet. Überprüfen sie, ob sie die richtige Laufzeit-Bibliothek verwenden.
% The compiler expects that the runtime library contains certain types. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you changed a type which the compiler needs for internal use and which needs to have a certain structure.
%
% \end{description}
# EndOfTeX

#
# Options
#
# 11061 is the last used one
#
# BeginOfTeX
%
% \section{Command line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_usage=11000_O_$1 [Optionen] <Eingabedatei> [Optionen]
option_only_one_source_support=11001_W_Es wird nur eine Quelldatei unterstützt. Wechsel für das Kompilieren von Quelldatei "$1" zu Quelldatei "$2"
% You can specify only one source file on the command line. The last
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_DEF-Datei kann nur für OS/2 erzeugt werden
% This option can only be specified when you're compiling for OS/2.
option_no_nested_response_file=11003_E_Verschachtelte Response-Dateien werden nicht unterstützt
% You cannot nest response files with the \var{@file} command line option.
option_no_source_found=11004_F_Kein Name für Quelldatei auf der Kommandzeile
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_Keine Angaben in Konfigurationsdatei "$1" gefunden
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_Ungültiger Parameter: $1
% You specified an unknown option.
option_help_pages_para=11007_H_-? zeigt Hilfetext an
% When an unknown option is given, this message is displayed.
option_too_many_cfg_files=11008_F_Zu viele verschachtelte Konfigurationsdateien
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_Kann Datei nicht öffnen $1
% The option file cannot be found.
option_reading_further_from=11010_D_Lese weitere Optionen aus $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_Zielsystem ist bereits gesetzt: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_Gemeinsame Bibliotheken sind auf der DOS Plattform nicht verfügbar, verwende stattdessen statische Bibliotheken
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos.
option_too_many_ifdef=11013_F_Zu viele \#IF(N)DEFs in Zeile $2 der Optionen-Datei $1
% The \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_Unerwartete \#ENDIFs in Zeile $2 der Optionen-Datei $1
% The \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_Offene Bedingung am Ende der Optionen-Datei
% The \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_Erzeugung von Debug-Informationen wird von dieser ausführbaren Datei nicht unterstützt
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_Versuchen Sie mit -dGDB erneut zu übersetzen
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_W_Sie verwenden den nun überholten Schalter $1
% This warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the meaning of the switch may change.
option_obsolete_switch_use_new=11019_W_Sie benutzen den nun überholten Schalter $1, bitte benutzen Sie $2
% This warns you when you use a switch that is not supported anymore. You
% must now use the second switch instead.
% It is recommended that you change the switch to overcome problems in the
% future, when the meaning of the switch may change.
option_switch_bin_to_src_assembler=11020_N_Schalte Assembler auf den Standard-Assembler-Quellcodeschreiber
% This notifies you that the assembler has been changed because you used the
% -a switch, which cannot be used with a binary assembler writer.
option_incompatible_asm=11021_W_Das gewählte Assembler-Ausgabeformat "$1" ist nicht mit "$2" kompatibel
option_asm_forced=11022_W_Verwendung des Assemblers "$1" erzwungen
% The assembler output selected cannot generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
option_using_file=11026_T_Optionen werden aus der Datei $1 gelesen
% Options are also read from this file.
option_using_env=11027_T_Optionen werden aus dem environment $1 gelesen
% Options are also read from this environment string.
option_handling_option=11028_D_Handling der Option "$1"
% Debug info that an option is found and will be handled.
option_help_press_enter=11029_O_*** Drücken Sie die ENTER-Taste ***
% Message shown when help is shown page per page. When pressing the ENTER
% Key, the next page of help is shown. If you press q and then ENTER, the
% compiler exits.
option_start_reading_configfile=11030_H_Beginn des Lesens der Konfigurationsdatei $1
% Start of configuration file parsing.
option_end_reading_configfile=11031_H_Ende des Lesens der Konfigurationsdatei $1
% End of configuration file parsing.
option_interpreting_option=11032_D_Option "$1" interpretieren
% The compiler is interpreting an option
option_interpreting_firstpass_option=11036_D_Firstpass Option "$1" interpretieren
% The compiler is interpreting an option for the first time.
option_interpreting_file_option=11033_D_Datei Option "$1" interpretieren
% The compiler is interpreting an option which it read from the configuration file.
option_read_config_file=11034_D_Konfigurationsdatei "$1" lesen
% The compiler is starting to read the configuration file.
option_found_file=11035_D_Name der Quelldatei "$1" gefunden
% Additional information about options.
% Displayed when you have the debug option turned on.
option_code_page_not_available=11039_E_Unbekannte Codepage "$1"
% An unknown code page for the source files was requested.
% The compiler is compiled with support for several code pages built-in.
% The requested code page is not in that list. You will need to recompile
% the compiler with support for the codepage you need.
option_config_is_dir=11040_F_Konfigurationsdatei $1 ist ein Verzeichnis
% Directories cannot be used as configuration files.
option_confict_asm_debug=11041_W_Die gewählte Assembler-Ausgabe "$1" kann kein Debug-Info erzeugen, Debugging ist ausgeschaltet
% The selected assembler output cannot generate
% debugging information, debugging option is therefore disabled.
option_ppc386_deprecated=11042_W_Die Verwendung von ppc386.cfg wird beendet. Bitte statt dessen fpc.cfg benutzen
% Using ppc386.cfg is still supported for historical reasons, however, for a multiplatform
% system the naming makes no sense anymore. Please continue to use fpc.cfg instead.
option_else_without_if=11043_F_Zur \#ELSE Direktive in Zeile $2 der Optionen-Datei $1 gibt es kein entsprechendes \#IF(N)DEF
% An \var{\#ELSE} statement was found in the options file without a matching \var{\#IF(N)DEF} statement.
option_unsupported_target=11044_F_Die Option "$1" wird auf der Zielplattform nicht oder noch nicht unterstützt
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is incompatible with the currently selected target platform.
option_unsupported_target_for_feature=11045_F_Das Feature "$1" wird für die ausgewählte Zielplattform nicht oder noch nicht unterstützt
% Not all features are supported or implemented for all target platforms. This message informs you that a chosen
% feature is incompatible with the currently selected target platform.
option_dwarf_smart_linking=11046_N_DWARF Debug-Information kann auf dieser Zielplattform nicht zusammen mit Smartlinking benutzt werden, es wird auf statisches Linken umgeschaltet
% Smart linking is currently incompatible with DWARF debug information on most
% platforms, so smart linking is disabled in such cases.
option_ignored_target=11047_W_Option "$1" wird für die ausgewählte Zielplattform ignoriert
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is ignored for the currently selected target platform.
option_debug_external_unsupported=11048_W_Schalte externe Debuginformation aus, weil es für die gewählte Kombination Ziel/Debugformat nicht unterstützt wird
% Not all debug formats can be stored in an external file on all platforms. In particular, on
% Mac OS X only DWARF debug information can be stored externally.
option_dwarf_smartlink_creation=11049_N_DWARF Debug-Information kann nicht benutzt werden, wenn das Smartlinken mit dem externen Assembler erfolgt. Das Erstellen einer statischen Bibliothek wird ausgeschaltet.
% Smart linking is currently incompatble with DWARF debug information on most
% platforms, so smart linking is disabled in such cases.
option_invalid_macosx_deployment_target=11050_E_Ungültiger Wert für die Umgebungsvariable MACOSX_DEPLOYMENT_TARGET: $1
option_invalid_iphoneos_deployment_target=11051_E_Ungültiger Wert für die Umgebungsvariable IPHONEOS_DEPLOYMENT_TARGET: $1
% On Mac OS X, the MACOSX\_DEPLOYMENT\_TARGET/IPHONEOS\_DEPLOYMENT\_TARGET environment variable can be
% used to set the default target OS version. In case of Mac OS X, it has to be of the format
% XY.Z or XY.Z.AB with X, Y,Z , A and B all digits from 0-9.
% In case of iOS, it has to be X.Z.A, where X, Z and A can all be either 1 or 2
% digits from 0-9.
option_illegal_fpu_eabihf=11052_E_Sie müssen beim Ziel EABIHF ABI einen der FPU Typen VFPV2, VFPV3 oder VFPV3_D16 verwenden
% The EABIHF (VFP hardfloat) ABI target can only be used with VFP FPUs.
option_w_unsupported_debug_format=11053_W_Das ausgewählte Debugformat wird auf dem aktuellen Ziel nicht unterstützt. Die aktuelle Einstellung wird beibehalten
% Not all targets support all debug formats (in particular, Stabs is not supported on 64 bit targets).
option_missing_arg=11054_E_Argument für "$1" fehlt
% Displayed when parameter must be followed by an argument.
option_malformed_para=11055_E_Ungültiges Argument für Parameter: $1
% Given argument is not valid for parameter.
option_smart_link_requires_external_linker=11056_W_Smart-linking erfordert externen Linker
option_com_files_require_tiny_model=11057_E_Das Erstellen von .COM Dateien wird für das aktuelle Speichermodell nicht unterstützt. Nur das winzige (tiny) Speichermodell unterstützt .COM Dateien
% Do not enable experimental -gc option if -Ur option is given.
option_gc_incompatible_with_release_flag=11058_W_Die experimentelle Option CheckPointer wird nicht eingeschaltet, weil sie zur Option -Ur inkompatibel ist
% The compiler binary only supports a single target architecture. Invoke the fpc binary if you wish to select a compiler binary for a different target architecture.
option_invalid_target_architecture=11059_E_Nicht unterstützte Zielarchitektur -P$1, rufe statt dessen den Kompilertreiber "fpc" auf.
% The ppc<target> executables support only a single target architecture. They do not support the -P switch. The compiler driver "fpc"
% handles this switch, so you have to invoke compiling by using "fpc" instead
option_features_only_for_system_unit=11060_E_Feature-Schalter werden nur beim Übersetzen der Unit "System" unterstützt
% To selected a certain feature, the system unit must be compiled with this feature enabled. All other units inherited the features set by the
% system unit through the ppu of the system unit.
option_debug_info_requires_external_linker=11061_N_Das ausgewählte Debug-Format wird vom internen Linker nicht unterstützt, schalte um auf externen Linker
%
% \end{description}
# EndOfTeX

#
# Whole program optimization
#
# 12019 is the last used one
#
# BeginOfTeX
%
% \section{Whole program optimization messages}
% This section lists errors that occur when the compiler is performing
% whole program optimization.
% \begin{description}
wpo_cant_find_file=12000_F_Feedback-Datei "$1" für die Gesamtprogramm-Optimierung kann nicht geöffnet werden
% The compiler cannot open the specified feedback file with whole program optimization information.
wpo_begin_processing=12001_D_Bearbeite die Informationen zur Gesamtprogramm-Optimierung aus der wpo-Feedback-Datei "$1"
% The compiler starts processing whole program optimization information found in the named file.
wpo_end_processing=12002_D_Bearbeitung der Informationen zur Gesamtprogramm-Optimierung aus der wpo-Feedback-Datei "$1" beendet
% The compiler has finished processing the whole program optimization information found in the named file.
wpo_expected_section=12003_E_Erwarte einen Sektions-Header, statt dessen "$2" in Zeile $1 der wpo-Feedback-Datei erhalten
% The compiler expected a section header in the whole program optimization file (starting with \%),
% but did not find it.
wpo_no_section_handler=12004_W_Kein Handler für die Sektion "$2" der Gesamtprogramm-Optimierung registriert (Zeile $1 der wpo-Feedback-Datei). Wird ignoriert
% The compiler has no handler to deal with the mentioned whole program optimization information
% section, and will therefore ignore it and skip to the next section.
wpo_found_section=12005_D_Sektion "$1" der Gesamtprogramm-Optimierung mit Informationen über "$2" gefunden
% The compiler encountered a section with whole program optimization information, and according
% to its handler this section contains information usable for the mentioned purpose.
wpo_no_input_specified=12006_F_Die ausgewählte Gesamtprogramm-Optimierung erfordert eine bereits erzeugte Feedback-Datei (bitte mit -Fw angeben)
% The compiler needs information gathered during a previous compilation run to perform the selected
% whole program optimizations. You can specify the location of the feedback file containing this
% information using the -Fw switch.
wpo_not_enough_info=12007_E_Keine Informationen für "$1" Gesamtprogramm-Optimierung gefunden
% While you pointed the compiler to a file containing whole program optimization feedback, it
% did not contain the information necessary to perform the selected optimizations. You most likely
% have to recompile the program using the appropriate -OWxxx switch.
wpo_no_output_specified=12008_F_Gebe eine Feedback-Datei an, um die erzeugte Information für die Gesamtprogramm-Optimierung zu speichern (mit der Option -FW)
% You have to specify the feedback file in which the compiler has to store the whole program optimization
% feedback that is generated during the compilation run. This can be done using the -FW switch.
wpo_output_without_info_gen=12009_E_Erzeuge keine Information für die Gesamtprogramm-Optimierung, obwohl eine Feedback-Datei dafür angegeben wurde (mit der Option -FW)
% The compiler was instructed to store whole program optimization feedback into a file specified using -FW,
% but not to actually generated any whole program optimization feedback. The classes of to be
% generated information can be specified using -OWxxx.
wpo_input_without_info_use=12010_E_Gesamtprogramm-Optimierung wird nicht durchgeführt, obwohl eine Feedback-Datei angegeben wurde (mit der Option -FW)
% The compiler was not instructed to perform any whole program optimizations (no -Owxxx parameters),
% but nevertheless an input file with such feedback was specified (using -Fwyyy). Since this can
% indicate that you forgot to specify an -Owxxx parameter, the compiler generates an error in this case.
wpo_skipping_unnecessary_section=12011_D_Überspringe die Sektion "$1" der Gesamtprogramm-Optimierung, weil sie nicht benötigt wird
% The whole program optimization feedback file contains a section with information that is not
% required by the selected whole program optimizations.
wpo_duplicate_wpotype=12012_W_Überschreibe bereits gelesene Information "$1" aus der Feedback-Eingabe-Datei mit der Information in Sektion "$2"
% The feedback file contains multiple sections that provide the same class of information (e.g.,
% information about which virtual methods can be devirtualized). In this case, the information in last encountered
% section is used. Turn on debugging output (-vd) to see which class of information is provided by each section.
wpo_cannot_extract_live_symbol_info_strip=12013_E_Symbol "liveness" Information kann nicht aus dem Programm erhalten werden, wenn Symbole mit "strip" entfernt wurden. Benutze -Xs-
% Certain symbol liveness collectors extract the symbol information from the linked program. If the symbol information
% is stripped (option -Xs), this is not possible.
wpo_cannot_extract_live_symbol_info_no_link=12014_E_Symbol "liveness" Information kann nicht aus dem Programm erhalten werden, wenn es nicht "linked" wird
% Certain symbol liveness collectors extract the symbol information from the linked program. If the program is not
% linked by the compiler, this is not possible.
wpo_cannot_find_symbol_progs=12015_F_"$1" oder "$2" wurden nicht gefunden, um die Symbol "liveness" Information aus dem Programm zu erhalten
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% This helper program is normally 'nm', which is part of the GNU binutils.
wpo_error_reading_symbol_file=12016_E_Fehler beim Lesen der "symbol liveness" Information durch "$1" erzeugt
% An error occurred during the reading of the symbol liveness file that was generated using the 'nm' or 'objdump' program. The reason
% can be that it was shorter than expected, or that its format was not understood.
wpo_error_executing_symbol_prog=12017_F_Fehler bei der Ausführung von "$1" (exitcode: $2) um Symbolinformationen aus dem "gelinkten" Programm zu erhalten
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% The helper program produced the reported error code when it was run on the linked program.
wpo_symbol_live_info_needs_smart_linking=12018_E_Die Sammlung der "symbol liveness" Information hilft nur bei smart linking, benutze -CX -XX
% Whether or not a symbol is live is determined by looking whether it exists in the final linked program.
% Without smart linking/dead code stripping, all symbols are always included, regardless of whether they are
% actually used or not. So in that case all symbols will be seen as live, which makes this optimization ineffective.
wpo_cant_create_feedback_file=12019_E_Die angegebene Feedback-Eingabe-Datei "$1" für die Gesamtprogramm-Optimierung kann nicht erzeugt werden
% The compiler is unable to create the file specified using the -FW parameter to store the whole program optimisation information.
%\end{description}
%
# EndOfTeX

#
# Package loading and handling
#
# 13029 is the last used one
#
# BeginOfTeX
%
% \section{Package loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a package from disk into memory, saving a package from memory
% to disk or when parsing packages in general. Many of these messages are
% informational messages.
% \begin{description}
package_f_cant_find_pcp=13001_F_Kann Paket $1 nicht finden
% You tried to use a package of which the PCP file isn't found by the
% compiler. Check your configuration file for the package paths.
package_u_pcp_found=13002_U_PCP-Datei für Paket $1 gefunden
% The PCP file for the specified package was found.
package_e_duplicate_package=13003_E_Das Paket $1 darf nicht mehrmals spezifiziert werden
% The package was already specified as required package and may not be specified
% a second time.
package_e_unit_deny_package=13004_E_Unit $1 kann nicht Teil eines Pakets sein
% The unit can not be part of a package because the DenyPackageUnit directive is enabled for the unit.
package_n_implicit_unit_import=13005_N_Unit $1 wird implizit in Paket $2 importiert
% The unit was not specified as part of the \var{contains} section and is also not included in one of the
% required packages. Add the unit to the \var{contains} section to increase compatibility with other packages.
package_f_cant_create_pcp=13006_F_PCP-Datei $2 für Paket $1 konnte nicht erzeugt werden
% The PCP file for the package could not be created.
package_f_cant_read_pcp=13007_F_PCP-Datei für Paket $1 konnte nicht gelesen werden
% The PCP file for the package could not be read.
package_t_pcp_loading=13008_T_PCP lädt $1
% When the \var{-vt} switch is used, the compiler tells you
% what packages it loads.
package_u_pcp_name=13009_U_PCP-Name: $1
% When you use the \var{-vu} flag, the package name is shown.
package_u_pcp_flags=13010_U_PCP-Flags: $1
% When you use the \var{-vu} flag, the package flags are shown.
package_u_pcp_crc=13011_U_PCP-CRC: $1
% When you use the \var{-vu} flag, the package CRC check is shown.
package_u_pcp_time=13012_U_PCP-Erstell-Zeit: $1
% When you use the \var{-vu} flag, the time the package was compiled is shown.
package_u_pcp_file_too_short=13013_U_PCP-Datei ist zu kurz
% The PCP file is too short, not all declarations are present.
package_u_pcp_invalid_header=13014_U_Ungültiger PCP-Header (Am Anfang fehlt PCP)
% A package file contains as the first three bytes the ASCII codes of the characters \var{PCP}.
package_u_pcp_invalid_version=13015_U_Ungültige PCP-Version $1
% This package file was compiled with a different version of the compiler, and
% cannot be read.
package_u_pcp_invalid_processor=13016_U_PCP ist für einen anderen Prozessor kompiliert
% This package file was compiled for a different processor type, and
% cannot be read.
package_u_pcp_invalid_target=13017_U_PCP ist für ein anderes Zielsystem kompiliert
% This package file was compiled for a different target, and
% cannot be read.
package_u_pcp_write=13018_U_$1 schreiben
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the package file.
package_f_pcp_cannot_write=13019_F_PCP-Datei kann nicht geschrieben werden
% An error occurred when writing the package file.
package_f_pcp_read_error=13020_F_Fehler beim Lesen der PCP-Datei
% This means that the package file was corrupted, and contains invalid
% information. Recompilation will be necessary.
package_f_pcp_read_unexpected_end=13021_F_Unerwartetes Ende der PCP-Datei
% Unexpected end of file. This may mean that the PCP file is
% corrupted.
package_f_pcp_invalid_entry=13022_F_Ungültiger Eintrag in der PCP-Datei: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
package_u_pcp_invalid_fpumode=13023_U_Versuche eine Unit zu nutzen, die mit einem andern FPU-Modus kompiliert wurde
% Trying to compile code while using units which were not compiled with
% the same floating point format mode. Either all code should be compiled
% with FPU emulation on, or with FPU emulation off.
package_t_packagesearch=13024_T_Paketsuche: $1
% When you use the \var{-vt} option, the compiler tells you where it tries to find
% package files.
package_u_required_package=13025_U_Benötigtes Paket $1
% When you specify the \var{-vu} switch, the compiler will tell you which
% packages a package requires.
package_u_contained_unit=13026_U_Enthält Unit $1
% When you specify the \var{-vu} switch, the compiler will tell you which
% units a package contains.
package_e_unit_already_contained_in_package=13027_E_Unit $1 ist bereits in Paket $2 enthalten
% A unit specified in a contains sections must not be part of a required package. Note that
% a unit might have become part of another package by indirectly including it.
package_w_unit_from_indirect_package=13028_W_Unit $1 wird vom indirekt benötigtem Paket $2 importiert
% If a unit from a package that is not part of the \var{requires} section is used then the package
% should require this unit directly to avoid confusion.
package_u_ppl_filename=13029_U_PPL Dateiname $1
% The name of the binary package library that is stored in the PCP.
%
% \end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_logo=11023_[
Free Pascal Compiler Version $FPCFULLVERSION [$FPCDATE] für $FPCTARGET
Copyright (c) 1993-2020 Florian Klämpfl und andere
]

#
# Info (option -i)
#
option_info=11024_[
Free Pascal Compiler Version $FPCVERSION

Compiler-Datum:      $FPCDATE
Compiler-Zielsystem: $FPCCPU

Unterstützte Zielbetriebssysteme (Mit '{*}' markierte Ziele sind in Entwicklung):
  $OSTARGETS

Unterstützte CPU Instruktionen:
  $INSTRUCTIONSETS

Unterstützte FPU Instruktionen:
  $FPUINSTRUCTIONSETS

Unterstützte Inline-Assembler Modi:
  $ASMMODES

Bekannte Compiler und RTL Feature:
  $FEATURELIST

Unterstützte ABI Ziele:
  $ABITARGETS

Unterstützte Optimierungen:
  $OPTIMIZATIONS

Unterstützte Gesamtprogramm-Optimierungen:
  All
  $WPOPTIMIZATIONS

Unterstützte Mikro-Kontroller:
  $CONTROLLERTYPES

Dieses Programm unterliegt der GNU General Public Licence
Weitere Informationen sind in COPYING.v2 zu finden

Fehlerberichte bitte senden an:
             http://bugs.freepascal.org

Weitere Informationen stehen auf unseren WWW Seiten (einschließlich Links zu
Mailing-Listen, auf denen Fragen gestellt und potentielle neue Features 
diskutiert werden können, usw.):
                 http://www.freepascal.org
]

#
# Help pages (option -? and -h)
#
# The first character on the line indicates who will display this
# line, the current possibilities are :
#    * = every target
#    3 = 80x86 targets
#    4 = x86_64
#    6 = 680x0 targets
#    8 = 8086 (16-bit) targets
#    a = AArch64
#    A = ARM
#    e = in extended debug mode only
#    F = help for the 'fpc' binary (independent of the target compiler)
#    I = VIS
#    J = JVM
#    M = MIPS (MIPSEB) targets
#    m = MIPSEL targets
#    P = PowerPC targets
#    p = PowerPC64 targets
#    S = Sparc targets
#    s = Sparc64 targets
#    V = AVR
# The second character also indicates who will display this line,
# (if the above character was TRUE) the current possibilities are :
#    * = everyone
#    g = with GDB info supported by the compiler
#    O = OS/2
#    L = UNIX systems
#  The third character represents the indentation level.
#
option_help_pages=11025_[
# It is also possible to insert comments in that section
F*0*_Es werden nur Optionen aufgelistet, die für die voreingestellte oder ausgewählte Plattform gültig sind.
**0*_Nach booleschen Optionen geben Sie + zum Ein- bzw. - zum Ausschalten an
**1@<x>_Lese zusätzlich zu den voreingestellten Compiler-Optionen aus fpc.cfg auch die aus <x>
# Assembler related options
**1a_Erzeugte Assembler-Datei nicht löschen
**2a5_Erzeuge keine Big-Obj-COFF-Dateien für GNU Binutils, die älter als Version 2.25 sind (Windows, NativeNT)
**2al_Liste Quellcode-Zeilen in der Assembler-Datei
**2an_Liste "node info" in der Assembler-Datei (-dEXTDEBUG Compiler)
**2ao_Füge eine zusätzliche Option zum Aufruf des externen Assemblers hinzu (ignoriert für den internen Assembler)
*L2ap_Benutze Pipes anstelle temporärer Assembler-Dateien
**2ar_Liste Registerbelegungsinformation in Assembler-Datei
**2at_Liste Temp. Variablenbelegungsinfo in Assembler-Datei
# Choice of assembler used
**1A<x>_Ausgabe Format:
**2Adefault_Benutze den "default" Assembler
3*2Aas_Assembliere mit Hilfe von GNU AS
3*2Amacho_Mach-O (Darwin, Intel 32 bit) mit Hilfe des internen Schreibers
8*2Anasm_Assembliere mit Hilfe von Nasm
8*2Anasmobj_Assembliere mit Hilfe von Nasm
3*2Anasm_Assembliere mit Hilfe von Nasm
3*2Anasmcoff_COFF (Go32v2) Datei mit Hilfe von Nasm
3*2Anasmelf_ELF32 (Linux) Datei mit Hilfe von Nasm
3*2Anasmwin32_Win32 Objektdatei mit Hilfe von Nasm
3*2Anasmwdosx_Win32/WDOSX Objektdatei mit Hilfe von Nasm
3*2Anasmdarwin_Macho32 Objektdatei mit Hilfe von Nasm (experimentell)
3*2Awasm_Obj Datei mit Hilfe von Wasm (Watcom)
3*2Anasmobj_Obj Datei mit Hilfe von Nasm
3*2Amasm_Obj Datei mit Hilfe von Masm (Microsoft)
3*2Atasm_Obj Datei mit Hilfe von Tasm (Borland)
3*2Aelf_ELF32 (Linux) mit Hilfe des internen Schreibers
3*2Acoff_COFF (Go32v2) mit Hilfe des internen Schreibers
3*2Apecoff_PE_COFF (Win32) mit Hilfe des internen Schreibers
3*2Ayasm_Assembliere mit Hilfe von Yasm (experimentell)
4*2Aas_Assembliere mit Hilfe von GNU AS
4*2Agas_Assembliere mit Hilfe von GNU GAS
4*2Agas-darwin_Assembliere darwin Mach-O64 mit Hilfe von GNU GAS
4*2Amasm_Win64 Objektdatei mit Hilfe von ml64 (Microsoft)
4*2Apecoff_PE-COFF (Win64) mit Hilfe des internen Schreibers
4*2Aelf_ELF (Linux-64bit) mit Hilfe des internen Schreibers
4*2Ayasm_Assembliere mit Hilfe von Yasm (experimentell)
4*2Anasm_Assembliere mit Hilfe von Nasm (experimentell)
4*2Anasmwin64_Assembliere Win64 Objektdatei mit Hilfe von Nasm (experimentell)
4*2Anasmelf_Assembliere Linux-64bit Objektdatei mit Hilfe von Nasm (experimentell)
4*2Anasmdarwin_Assembliere darwin Macho64 Objektdatei mit Hilfe von Nasm (experimentell)
6*2Aas_Unix o-Datei mit Hilfe von GNU AS
6*2Agas_GNU Motorola Assembler
6*2Amit_MIT Syntax (old GAS)
6*2Amot_Standard Motorola Assembler
A*2Aas_Assembliere mit Hilfe von GNU AS
P*2Aas_Assembliere mit Hilfe von GNU AS
S*2Aas_Assembliere mit Hilfe von GNU AS
# Used only internally by IDE
**1b_Erzeuge Browser-Info
**2bl_Erzeuge Info zu lokalen Symbolen
**1B_Erzeuge alle Module (Build)
**1C<x>_Optionen für Code-Erzeugung:
**2C3<x>_Schalte ieee-Prüfung von Konstanten ein
**2Ca<x>_Wähle ABI aus; fpc -i oder fpc -ia geben die möglichen Werte aus
**2Cb_Erzeuge Code für die "big-endian"-Variante der Zielarchitektur
**2Cc<x>_Setze "default calling convention" zu <x>
**2CD_Erzeuge auch eine dynamische Bibliothek (nicht unterstützt)
**2Ce_Übersetze mit emulierten Fliesskomma opcodes
**2CE_Erzeuge FPU Code, der Ausnahmen (exceptions) auslösen kann
**2Cf<x>_Wähle den Fliesskomma-Befehlssatz aus; fpc -i oder fpc -if geben die möglichen Werte aus
**2CF<x>_Minimale Präzission von Fliesskommakonstanten (default, 32, 64)
**2Cg_Erzeuge PIC Code
**2Ch<n>[,m]_<n> Minimale Größe des Heap in Byte (zwischen 1023 und 67107840) und optional [m] maximale Größe des Heap
**2Ci_I/O-Prüfung
A*2CI<x>_Wähle den Befehlssatz für ARM aus: ARM oder THUMB
**2Cn_Lasse die Linkstufe aus
P*2CN_Erzeuge Überprüfungen auf Nil-Zeiger (nur AIX)
**2Co_Prüfe auf Überlauf von Integer-Operationen
**2CO_Prüfe auf möglichen Überlauf von Integer-Operationen
**2Cp<x>_Wähle Befehlssatz aus; fpc -i oder fpc -ic geben die möglichen Werte aus
**2CP<x>=<y>_ Einstellungen für packing
**3CPPACKSET=<y>_ <y> Belegung von Sets: 0, 1 oder DEFAULT oder NORMAL, 2, 4 und 8
**3CPPACKENUM=<y>_ <y> Packen von Aufzählungen: 0, 1, 2 und 4 oder DEFAULT oder NORMAL
**3CPPACKRECORD=<y>_ <y> Packen von Records: 0 oder DEFAULT oder NORMAL, 1, 2, 4, 8, 16 und 32
**2Cr_Führe Bereichsprüfung durch
**2CR_Verifiziere die Gültigkiet des Aufrufs der Objektmethoden
**2Cs<n>_Setze die Prüfgröße des Stacks auf <n>
**2Ct_Führe Stackprüfung durch (nur zum Testen, siehe Handbuch)
8*2CT<x>_Optionen für die ziel-spezifische Code-Erzeugung
3*2CT<x>_Optionen für die ziel-spezifische Code-Erzeugung
4*2CT<x>_Optionen für die ziel-spezifische Code-Erzeugung
p*2CT<x>_Optionen für die ziel-spezifische Code-Erzeugung
P*2CT<x>_Optionen für die ziel-spezifische Code-Erzeugung
J*2CT<x>_Optionen für die ziel-spezifische Code-Erzeugung
A*2CT<x>_Optionen für die ziel-spezifische Code-Erzeugung
p*3CTsmalltoc_ Erzeuge kleinere TOCs auf Kosten der Ausführungsgeschwindigkeit (AIX)
P*3CTsmalltoc_ Erzeuge kleinere TOCs auf Kosten der Ausführungsgeschwindigkeit (AIX)
J*3CTautogetterprefix=X_  Erzeuge automatisch Getter für Eigenschaften mit dem Prefix X (Leerstring schaltet aus)
J*3CTautosetterprefix=X_  Erzeuge automatisch Setter für Eigenschaften mit dem Prefix X (Leerstring schaltet aus)
8*3CTcld_                 Emittiere eine CLD-Instruktion vor der x86-String-Instruktion
3*3CTcld_                 Emittiere eine CLD-Instruktion vor der x86-String-Instruktion
4*3CTcld_                 Emittiere eine CLD-Instruktion vor der x86-String-Instruktion
8*3CTfarprocspushoddbp_ Erhöhe BP vor dem Verschieben in den Prolog von "far"-Funktionen
J*3CTcompactintarrayinit_ Erzeuge kleineren (aber möglicherweise langsameren) Code für die Initialisierung von Integer-Array-Konstanten
J*3CTenumfieldinit_       Initialisiere Aufzählungsfelder in Konstruktoren mit enumtype(0), nach dem Aufruf geerbter Konstruktoren
J*3CTinitlocals_          Initialisiere lokale Variablen, die einen JVM bytecode Verifikationfehler auslösen, wenn uninitialisiert verwendet (verzögert den Code)
J*3CTlowercaseprocstart_  Kleinschreibung des ersten Buchstabens bei Prozedur/Funktions/Methoden-Namen.
A*3CTthumbinterworking_ Erzeuge "Thumb interworking-safe code", wenn möglich
J*2Cv_Var/out Parameter copy-out Überprüfung
**2CX_Benutze Smartlinking
**1d<x>_Definiere das Symbol <x>
**1D_Erzeuge eine DEF-Datei
**2Dd<x>_Setze Beschreibung zu <x>
**2Dv<x>_Setze DLL Version zu <x>
*O2Dw_Erzeuge PM-Anwendung
**1e<x>_Setze Pfad zur ausführbaren Datei
**1E_Genau wie -Cn
**1fPIC_Genau wie -Cg
**1F<x>_Dateinamen und Pfade:
**2Fa<x>[,y]_Ein Program soll die Units <x> und [y] laden, bevor uses analysiert wird
**2Fc<x>_Setze die Eingabe-Codepage zu <x>
**2FC<x>_Setze den Namen des RC Compiler-Binärprograms auf <x>
**2Fd_Schalte den internen Verzeichnis-Cache des Compilers aus
**2FD<x>_Setze das Verzeichnis für die Compiler-Hilfsprogramme
**2Fe<x>_Leite die Fehlerausgabe um nach <x>
**2Ff<x>_Ergänze <x> zum Framework-Pfad (nur Darwin)
**2FE<x>_Setze den Pfad für Exe/Unit-Dateien auf <x>
**2Fi<x>_Ergänze <x> zum Include-Pfad
**2Fl<x>_Ergänze <x> zum Bibliotheks-Pfad
**2FL<x>_Benutze <x> als dynamischen Linker
**2Fm<x>_Lade die Unicode-Konversionstabelle aus <x>.txt im Compiler-Verzeichnis
**2FM<x>_Setze das Verzeichnis für Unicode-Binär-Dateien
**2FN<x>_Ergänze <x> zur Liste der voreingestellten Unit-Bereiche (namespaces)
**2Fo<x>_Ergänze <x> zum Objektdatei-Pfad
**2Fr<x>_Lade die Fehlermeldungs-Datei <x>
**2FR<x>_Setze den Resource (.res) Linker auf <x>
**2Fu<x>_Ergänze <x> zum Unit-Pfad
**2FU<x>_Units werden nach <x> ausgegeben, hat Vorrang vor -FE
**2FW<x>_Speichere das erzeugte Feedback für die Gesamtprogramm-Optimierung in <x>
**2Fw<x>_Lade das bereits gespeicherte Feedback für die Gesamtprogramm-Optimierung aus <x>
*g1g_Erzeuge Informationen zur Fehlersuche (im für das Zielsystem voreingestellten Format):
*g2gc_Zeigerüberprüfung (experimentell, nur auf einige Systemen, kann falschen Alarm erzeugen)
*g2gh_Heaptrace-Unit einbinden (für Speicherlecks und -korruption)
*g2gl_Line info Unit einbinden, um mehr backtrace Informationen anzuzeigen
*g2gm_Erzeuge Microsoft-CodeView Debug-Informationen (experimentell)
*g2go<x>_Setze Optionen für die Debug-Informationen
*g3godwarfsets_Schalte DWARF Debug-Informationen für Mengen (sets) ein (verhindert debugging mit gdb < 6.5)
*g3gostabsabsincludes_ Absolute/volle Include-Datei-Pfade in Stabs speichern
*g3godwarfmethodclassprefix_ Stelle Methodennamen in DWARF den Namen der Klasse voran
*g3godwarfcpp_ Simuliere C++ Debug-Informationen in DWARF
*g3godwarfomflinnum_ Erzeuge Zeilennummer-Informationen in den "OMF LINNUM"-Records im Format "MS LINK" zusätzlich zu Debug-Informationen in DWARF (Kompatibilität zum Open Watcom Debugger/Linker)
*g2gp_Erhalte Groß/Kleinschreibung in Stabs-Symbolnamen
*g2gs_Erzeuge Stabs-Debug-Informationen
*g2gt_Lösche lokale Variablen (um eine Verwendung ohne Initialisierung zu finden; mehrere 't' ändern den Wert, mit dem überschrieben wird)
*g2gv_Erzeuge ein mit Valgrind verfolgbares (traceable) Programm
*g2gw_Erzeuge DWARFv2-Debug-Informationen (wie -gw2)
*g2gw2_Erzeuge DWARFv2-Debug-Informationen
*g2gw3_Erzeuge DWARFv3-Debug-Informationen
*g2gw4_Erzeuge DWARFv4-Debug-Informationen (experimentell)
**1i_Zeige alle Informationen über den Compiler
**2iD_Zeige Compilerdatum
**2iSO_Zeige Compilerbetriebssystem
**2iSP_Zeige Compilerprozessor
**2iTO_Zeige Zielbetriebssystem
**2iTP_Zeige Zielprozessor
**2iV_Zeige kurze Compilerversion
**2iW_Zeige vollständige Compilerversion
**2ia_Zeige Liste der unterstützten ABI-Ziele
**2ic_Zeige Liste der unterstützten CPU-Befehlssatz
**2if_Zeige Liste der unterstützten Fliesskomma-Befehlssatz
**2ii_Zeige Liste der unterstützten Inline-Assembler-Modi
**2io_Zeige Liste der unterstützten Optimierungen
**2ir_Zeige Liste der erkannten Compiler- und RTL-Feature
**2it_Zeige Liste der unterstützten Ziele
**2iu_Zeige Liste der unterstützten Mikro-Kontrollertypen
**2iw_Zeige Liste der unterstützten Gesamtprogramm-Optimierungen
**1I<x>_Ergänze <x> zum Include-Pfad
**1k<x>_Übergebe <x> an den Linker
**1l_Zeige Logo
**1M<x>_Setze Sprachmodus zu <x>
**2Mfpc_Free Pascal Dialekt (Voreinstellung)
**2Mobjfpc_FPC Modus mit Unterstützung von Object-Pascal
**2Mdelphi_Kompatibilitäts-Modus für Delphi
**2Mtp_Kompatibilitäts-Modus für TP/BP 7.0
**2Mmacpas_Kompatibilitäts-Modus für MacIntosh Pascal-Dialekte
**2Miso_ISO 7185 Modus
**2Mextendedpascal_ISO 10206 Modus
**2Mdelphiunicode_Kompatibilitäts-Modus für Delphi 2009 und später
**1n_Standard-Konfigurationsdatei ignorieren
**1o<x>_Die erzeugte, ausführbare Datei bekommt den Namen <x>
**1O<x>_Optimierungen:
**2O-_Optimierungen ausschalten
**2O1_Level 1 Optimierung (schnell und Debugger freundlich)
**2O2_Level 2 Optimierung (-O1 + schnelle Optimierungen)
**2O3_Level 3 Optimierung (-O2 + langsame Optimierungen)
**2O4_Level 4 Optimierung (-O3 + Optimierungen, die unerwartete Nebeneffekte haben können)
**2Oa<x>=<y>_Ausrichtung (alignment) von Mengen
**2Oo[NO]<x>_Optimierungen ein- oder ausschalten; fpc -i oder fpc -io geben die möglichen Werte aus
**2Op<x>_Setze Zielprozessor für die Optimierung; fpc -i oder fpc -ic geben die möglichen Werte aus
**2OW<x>_Erzeuge Feedback für die Gesamtprogramm-Optimierung für Optimierung <x>; fpc -i oder fpc -iw geben die möglichen Werte aus
**2Ow<x>_Führe die Gesamtprogramm-Optimierung durch <x>; fpc -i oder fpc -iw  geben die möglichen Werte aus
**2Os_Erzeuge kürzeren Code
**1pg_Erzeuge Profiler-Code für gprof
F*1P<x>_Ziel CPU / Compiler bezogene Optionen:
F*2PB_Zeige die voreingestellte Compilerbinärdatei
F*2PP_Zeige die voreingestellte Ziel-CPU
F*2P<x>_Setze die Ziel-CPU (aarch64,arm,avr,i386,i8086,jvm,m68k,mips,mipsel,powerpc,powerpc64,sparc,x86_64)
**1R<x>_Assembler Code-Format:
**2Rdefault_Benutze den voreingestellten Assembler
3*2Ratt_Lese Assembler Code im AT&T-Format
3*2Rintel_Lese Assembler im Intel-Format
4*2Ratt_Lese Assembler Code im AT&T-Format
4*2Rintel_Lese Assembler im Intel-Format
8*2Ratt_Lese Assembler Code im AT&T-Format
8*2Rintel_Lese Assembler im Intel-Format
6*2RMOT_Lese Assembler im Motorola-Format
**1S<x>_Syntax-Optionen:
**2S2_Schalte einige der Delphi 2 Erweiterungen ein (wie -Mobjfpc)
**2Sc_Unterstütze spezielle C-Operatoren (*=,+=,/= and -=)
**2Sa_Erlaube assertion code.
**2Sd_Sei Delphi-kompatibel (wie -Mdelphi)
**2Se<x>_Fehler Optionen. <x> ist eine der folgenden Kombinationen:
**3*_<n> : Compiler hält nach <n> Fehlern (Voreinstellung ist 1)
**3*_w : Compiler hält auch nach Warnungen
**3*_n : Compiler hält auch nach Anmerkungen
**3*_h : Compiler hält auch nach Hinweisen
**2Sf_Schalte bestimmte Feature in Compiler und RTL ein; fpc -i oder fpc -ir geben die möglichen Werte aus
**2Sg_Erlaube LABEL und GOTO
**2Sh_Benutze ANSI-Strings
**2Si_Benutze C++ artige INLINE
**2Sj_Erlaube das Schreiben von typisierten Konstanten (Voreinstellung in allen Modi)
**2Sk_Lade fpcylix Unit
**2SI<x>_Setze den Stil des Interface zu <x>
**3SIcom_COM kompatibles Interface (Voreinstellung)
**3SIcorba_CORBA kompatibles Interface
**2Sm_Unterstütze Makros wie in C (global)
**2So_Sei TP/BP 7.0 kompatibel (wie -Mtp)
**2Sr_Transparente Dateinamen im ISO-Modus
**2Ss_Konstruktor- und Destruktorname müssen "Init" und "Done" sein
**2Sv_Unterstütze Vektor-Verarbeitung (soweit vorhanden benutze CPU-Vektorerweiterungen)
**2Sx_Exception Schlüsselwörter einschalten (Voreinstellung in Delphi/ObjFPC Moden)
**2Sy_@<pointer> gibt einen typisierten Zeiger zurück, genau wie $T+
**1s_Rufe weder Assembler noch Linker auf (nur mit -a)
**2sh_Erzeuge Script um auf dem Host zu linken
**2st_Erzeuge Script um auf dem Zielsystem zu linken
**2sr_Überspringe die Phase der "register allocation" (mit -alr benutzen)
**1T<x>_Zielbetriebssystem::
# i386 targets
3*2Tandroid_Android
3*2Taros_AROS
3*2Tbeos_BeOS
3*2Tdarwin_Darwin/Mac OS X
3*2Tembedded_Embedded
3*2Temx_OS/2 via EMX (einschließlich EMX/RSX extender)
3*2Tfreebsd_FreeBSD
3*2Tgo32v2_Version 2 des DJ Delorie DOS extender
3*2Thaiku_Haiku
3*2Tlinux_Linux
3*2Tnativent_Native NT API (experimentell)
3*2Tnetbsd_NetBSD
3*2Tnetware_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2Topenbsd_OpenBSD
3*2Tos2_OS/2 / eComStation
3*2Tsymbian_Symbian OS
3*2Tsolaris_Solaris
3*2Twatcom_Watcom compatible DOS extender
3*2Twdosx_WDOSX DOS extender
3*2Twin32_Windows 32 Bit
3*2Twince_Windows CE
# x86_64 targets
4*2Taros_AROS
4*2Tdarwin_Darwin/Mac OS X
4*2Tdragonfly_DragonFly BSD
4*2Tembedded_Embedded
4*2Tfreebsd_FreeBSD
4*2Tiphonesim_iPhoneSimulator
4*2Tlinux_Linux
4*2Tnetbsd_NetBSD
4*2Topenbsd_OpenBSD
4*2Tsolaris_Solaris
4*2Twin64_Win64 (64 bit Windows Systeme)
# m68k targets
6*2Tamiga_Commodore Amiga
6*2Tatari_Atari ST/STe/TT
6*2Tembedded_Embedded
6*2Tlinux_Linux
6*2Tnetbsd_NetBSD
6*2Tmacos_Mac OS
6*2Tpalmos_PalmOS
# i8086 targets
8*2Tembedded_Embedded
8*2Tmsdos_MS-DOS (und kompatible)
8*2Twin16_Windows 16 Bit
# arm targets
A*2Tandroid_Android
A*2Taros_AROS
A*2Tdarwin_Darwin/iPhoneOS/iOS
A*2Tembedded_Embedded
A*2Tgba_Game Boy Advance
A*2Tlinux_Linux
A*2Tnds_Nintendo DS
A*2Tnetbsd_NetBSD
A*2Tpalmos_PalmOS
A*2Tsymbian_Symbian
A*2Twince_Windows CE
# aarch64 targets
a*2Tdarwin_Darwin/iOS
a*2Tlinux_Linux
# jvm targets
J*2Tandroid_Android
J*2Tjava_Java
# mipsel targets
m*2Tandroid_Android
m*2Tembedded_Embedded
m*2Tlinux_Linux
# mipseb targets
M*2Tembedded_Embedded
M*2Tlinux_Linux
# powerpc targets
P*2Taix_AIX
P*2Tamiga_AmigaOS
P*2Tdarwin_Darwin und Mac OS X
P*2Tembedded_Embedded
P*2Tlinux_Linux
P*2Tmacos_Mac OS (classic)
P*2Tmorphos_MorphOS
P*2Tnetbsd_NetBSD
P*2Twii_Wii
# powerpc64 targets
p*2Taix_AIX
p*2Tdarwin_Darwin/Mac OS X
p*2Tembedded_Embedded
p*2Tlinux_Linux
# riscv32 targets
R*2Tlinux_Linux
R*2Tembedded_Embedded
# riscv64 targets
r*2Tlinux_Linux
r*2Tembedded_Embedded
# sparc targets
S*2Tlinux_Linux
S*2Tsolaris_Solaris
# sparc64 targets
s*2Tlinux_Linux
# not yet ready s*2Tsolaris_Solaris
# avr targets
V*2Tembedded_Embedded
# end of targets section
**1u<x>_Entferne die Definition für das Symbol <x>
**1U<x>_Unit-Optionen:
**2Un_Prüfe den Unitnamen nicht
**2Ur_Erzeuge "release unit"-Dateien
**2Us_Erzeuge eine Systemunit
**1v<x>_Meldungen, <x> ist eine Kombination der folgenden Zeichen:
**2*_e : Fehler (Standard)             0 : Nichts (außer Fehlern)
**2*_w : Warnungen                     u : Unit Informationen
**2*_n : Anmerkungen                   t : Angesprochene/benutzte Dateien
**2*_h : Hinweise                      c : Preprozessordirektiven
**2*_i : Allgemeine Informationen      d : Debug-Informationen
**2*_l : Zeilennummern                 r : Rhide/GCC kompatibler Modus
**2*_s : Zeitstempel                   q : Nummer der Meldung
**2*_a : Alles                         x : Zeige Infos über aufgerufene Tools
**2*_b : Schreibe bei Meldungen mit    p : Schreibe tree.log mit Analysenbaum (parse tree)
**2*_    Dateinamen den vollständigen  v : Schreibe fpcdebug.txt mit 
**2*_    Pfad                              ganz viel Information
**2*_z : Schreibe output nach stderr
**2*_m<x>,<y> : Zeige die Meldungen mit den Nummern <x> und <y> nicht
F*1V<x>_Hänge '-<x>' an den Namen der Compilerbinary an (z.B. für die Version)
**1W<x>_Spezifiziere ein natives Programm (Windows)
3*2WA_Spezifiziere ein natives Programm (Windows)
4*2WA_Spezifiziere ein natives Programm (Windows)
A*2WA_Spezifiziere ein natives Programm (Windows)
3*2Wb_Erzeuge statt einer Bibliothek ein Bundle (Darwin)
P*2Wb_Erzeuge statt einer Bibliothek ein Bundle (Darwin)
p*2Wb_Erzeuge statt einer Bibliothek ein Bundle (Darwin)
a*2Wb_Erzeuge statt einer Bibliothek ein Bundle (Darwin)
A*2Wb_Erzeuge statt einer Bibliothek ein Bundle (Darwin)
4*2Wb_Erzeuge statt einer Bibliothek ein Bundle (Darwin)
3*2WB_Erzeuge ein relozierbares Image (Windows, Symbian)
3*2WB<x>_Setze die Imagebasis auf <x> (Windows, Symbian)
4*2WB_Erzeuge ein relozierbares Image (Windows)
4*2WB<x>_Setze die Imagebasis auf <x> (Windows, Symbian)
A*2WB_Erzeuge ein relozierbares Image (Windows, Symbian)
A*2WB<x>_Setze die Imagebasis auf <x> (Windows)
3*2WC_Spezifiziere "console type application" (EMX, OS/2, Windows)
4*2WC_Spezifiziere "console type application" (Windows)
A*2WC_Spezifiziere "console type application" (Windows)
P*2WC_Spezifiziere "console type application" (Classic Mac OS)
3*2WD_Benutze DEFFILE um Funktionen der DLL oder EXE zu exportieren (Windows)
4*2WD_Benutze DEFFILE um Funktionen der DLL oder EXE zu exportieren (Windows)
A*2WD_Benutze DEFFILE um Funktionen der DLL oder EXE zu exportieren (Windows)
3*2We_Benutze externe Resourcen (Darwin)
4*2We_Benutze externe Resourcen (Darwin)
a*2We_Benutze externe Resourcen (Darwin)
A*2We_Benutze externe Resourcen (Darwin)
P*2We_Benutze externe Resourcen (Darwin)
p*2We_Benutze externe Resourcen (Darwin)
3*2WF_Spezifiziere "full-screen type application" (EMX, OS/2)
3*2WG_Spezifiziere "graphic type application" (EMX, OS/2, Windows)
4*2WG_Spezifiziere "graphic type application" (Windows)
A*2WG_Spezifiziere "graphic type application" (Windows)
P*2WG_Spezifiziere "graphic type application" (Classic Mac OS)
3*2Wi_Benutze interne Resourcen (Darwin)
4*2Wi_Benutze interne Resourcen (Darwin)
a*2Wi_Benutze interne Resourcen (Darwin)
A*2Wi_Benutze interne Resourcen (Darwin)
P*2Wi_Benutze interne Resourcen (Darwin)
p*2Wi_Benutze interne Resourcen (Darwin)
3*2WI_Die Verwendung der "import"-Abschnitte ein/ausschalten (Windows)
4*2WI_Die Verwendung der "import"-Abschnitte ein/ausschalten (Windows)
A*2WI_Die Verwendung der "import"-Abschnitte ein/ausschalten (Windows)
8*2Wh_Benutze "huge"-Code für Units (Ignoriert für Modelle mit CODE in einem separaten Segment)
8*2Wm<x>_Setze Speichermodell
8*3WmTiny_Winziges (tiny) Speichermodell
8*3WmSmall_Kleines (small) Speichermodell (Voreinstellung)
8*3WmMedium_Mittleres (medium) Speichermodell
8*3WmCompact_Kompaktes (compact) Speichermodell
8*3WmLarge_Großes (large) Speichermodell
8*3WmHuge_Riesiges (huge) Speichermodell
3*2WM<x>_Minimale Mac OS X Deployment Version: 10.4, 10.5.1, ... (Darwin)
4*2WM<x>_Minimale Mac OS X Deployment Version: 10.4, 10.5.1, ... (Darwin)
p*2WM<x>_Minimale Mac OS X Deployment Version: 10.4, 10.5.1, ... (Darwin)
P*2WM<x>_Minimale Mac OS X Deployment Version: 10.4, 10.5.1, ... (Darwin)
3*2WN_Erzeuge keinen "relocation code" (notwendig für debugging) (Windows)
4*2WN_Erzeuge keinen "relocation code" (notwendig für debugging) (Windows)
A*2WN_Erzeuge keinen "relocation code" (notwendig für debugging) (Windows)
A*2Wp<x>_Spezifiziere den Kontrollertyp; fpc -i oder fpc -iu geben mögliche Werte aus
m*2Wp<x>_Spezifiziere den Kontrollertyp; fpc -i oder fpc -iu geben mögliche Werte aus
V*2Wp<x>_Spezifiziere den Kontrollertyp; fpc -i oder fpc -iu geben mögliche Werte aus
3*2WP<x>_Minimale iOS Deployment Version: 3.0, 5.0.1, ... (iphonesim)
4*2WP<x>_Minimale iOS Deployment Version: 8.0, 8.0.2, ... (iphonesim)
a*2WP<x>_Minimale iOS Deployment Version: 7.0, 7.1.2, ... (Darwin)
A*2WP<x>_Minimale iOS Deployment Version: 3.0, 5.0.1, ... (Darwin)
3*2WR_Erzeuge "relocation code" (Windows)
4*2WR_Erzeuge "relocation code" (Windows)
A*2WR_Erzeuge "relocation code" (Windows)
8*2Wt<x>_Setze das Format der Ziel-Executable
8*3Wtexe_Erzeuge eine DOS .EXE Datei (Voreinstellung)
8*3Wtcom_Erzeuge eine DOS .COM Datei (erfordert das winzige (tiny) Speichermodell)
P*2WF_Spezifiziere "MPW tool type application" (Classic Mac OS)
**2WX_Ermögliche den executable stack (Linux)
**1X_Programm-Optionen:
**2X9_Erzeuge Linkerscript für GNU Binutils ld älter als Version 2.19.1 (Linux)
**2Xc_Übergebe --shared an den Linker (BeOS, Darwin, FreeBSD, Linux)
**2Xd_Den Standard Bibliotheks-Suchpfad NICHT nutzen (benötigt für cross compile, wenn nicht -XR verwendet wird)
**2Xe_Verwende den externen Linker
**2Xf_Ersetze beim Linken den Namen der pthread-Bibliothek (BSD)
**2Xg_Erstelle die Debug-Informationen in einer separaten Datei und einen "Debug-Link"-Abschnitt im ausführbaren Programm
**2XD_Versuche Units dynamisch zu linken             (definiert FPC_LINK_DYNAMIC)
**2Xi_Verwende den internen Linker
**2XLA_Definiere Bibliotheksersetzungen für das Linken
**2XLO_Definiere die Reihenfolge der Bibliotheken für das Linken
**2XLD_Schließe die voreingestellte Reihenfolge der Standard-Bibliotheken aus
**2Xm_Erzeuge die "link map"
**2XM<x>_Setze den Namen der 'main' Programm-Routine (Voreinstellung ist 'main')
**2Xn_Nutze den plattformeigenen Linker des Zielsystem anstelle des GNU ld (Solaris, AIX)
F*2Xp<x>_Suche nach der Compilerbinary zuerst im Verzeichnis <x>
**2XP<x>_Stelle dem Namen der Compiler-Hilfsprogramme (binutils) den Prefix <x> voran
**2Xr<x>_Setze den rlink-Pfad des Linker zu <x> (benötigt für cross compile, siehe ld-Manual für mehr Informationen) (BeOS, Linux)
**2XR<x>_Stelle allen Linker-Suchpfaden den Namen <x> voran (BeOS, Darwin, FreeBSD, Linux, Mac OS, Solaris)
**2Xs_Entferne alle Symbole aus der ausführbaren Datei
**2XS_Versuche Units statisch zu linken (default)    (definiert FPC_LINK_STATIC)
**2Xt_Linke mit statischen Bibliotheken              (-static wird an den Linker übergeben)
**2Xv_Erzeuge eine Tabelle mit den virtuellen Entry-Aufrufen
**2XV_Benutze VLink als externen linker              (Voreinstellung für Amiga, MorphOS)
**2XX_Versuche Units smart zu linken                 (definiert FPC_LINK_SMART)
**1*_
**1?_Zeigt diese Hilfe an
**1h_Zeigt diese Hilfe ohne Warten an
]

#
# The End...
#
