%%% Reordering of errorda.msg respective to errore.msg
%%% Contains all comments from errorda.msg
#$
#   $Id: errorda.msg 621 2006-07-02 15:44:03Z chrivers 
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1999-2018 by the Free Pascal Development team
#
#   Danish Language File for Free Pascal
#
#   Translation by Christian Iversen <chrivers@iversen-net.dk>
#
#   This file is encoded in UTF-8, and based on r4084 of errore.eng
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
# 01016 is the last used one
#
# BeginOfTeX
% \section{General compiler messages}
% This section gives the compiler messages which are not fatal, but which
% display useful information. The number of such messages can be
% controlled with the various verbosity level \var{-v} switches.
% \begin{description}
general_t_compilername=01000_T_Oversætter: $1
% When the \var{-vt} switch is used, this line tells you what compiler
% is used.
general_d_sourceos=01001_D_Oversætter OS: $1
% When the \var{-vd} switch is used, this line tells you what the source
% operating system is.
general_i_targetos=01002_I_Compilerer til OS: $1
% When the \var{-vd} switch is used, this line tells you what the target
% operating system is.
general_t_exepath=01003_T_Sti til program: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for it's binaries.
general_t_unitpath=01004_T_Sti til units: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for compiled units. You can set this path with the \var{-Fu}
general_t_includepath=01005_T_Sti til include-filer: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for it's include files (files used in \var{\{\$I xxx\}} statements).
% You can set this path with the \var{-I} option.
general_t_librarypath=01006_T_Sti til biblioteker: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for the libraries. You can set this path with the \var{-Fl} option.
general_t_objectpath=01007_T_Sti til objekter: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for object files you link in (files used in \var{\{\$L xxx\}} statements).
% You can set this path with the \var{-Fo} option.
general_i_abslines_compiled=01008_I_$1 linier kompileret, $2 sekunder
% When the \var{-vi} switch is used, the compiler reports the number
% of lines compiled, and the time it took to compile them (real time,
% not program time).
general_f_no_memory_left=01009_F_Ikke mere hukommelse!
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
general_i_writingresourcefile=01010_I_Skriver resourcestrengstabel til fil: $1
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_Skriver resourcestrengtabel til fil: $1
% This message is shown when the compiler encountered an error when writing
% the Resource String Table file
general_i_fatal=01012_I_Fatal fejl:
% Prefix for Fatal Errors
general_i_error=01013_I_Fejl:
% Prefix for Errors
general_i_warning=01014_I_Advarsel:
% Prefix for Warnings
general_i_note=01015_I_Bemærkning:
% Prefix for Notes
general_i_hint=01016_I_Hint:
% Prefix for Hints
general_e_path_does_not_exist=01017_E_Stien "$1" blev ikke fundet
% The specified path does not exist.
general_f_compilation_aborted=01018_F_Compilering afbrudt
% \end{description}
#
# Scanner
#
# 02063 is the last used one
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compiling handling.
% \begin{description}
general_text_bytes_code=01019_bytes code
% The size of the generated executable code, in bytes.
general_text_bytes_data=01020_bytes data
% The size of the generated program data, in bytes.
general_i_number_of_warnings=01021_I_$1 warning(s) issued
% Total number of warnings issued during compilation.
general_i_number_of_hints=01022_I_$1 hint(s) issued
% Total number of hints issued during compilation.
general_i_number_of_notes=01023_I_$1 note(s) issued
% Total number of notes issued during compilation.
general_f_ioerror=01024_F_I/O error: $1
% During compilation an I/O error happened which allows no further compilation.
general_f_oserror=01025_F_Operating system error: $1
% During compilation an operating system error happened which allows no further compilation.
% \end{description}
#
# Scanner
#
# 02098 is the last used one
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compilation handling.
% \begin{description}
scan_f_end_of_file=02000_F_Slutning af fil fundet mod forventning
% this typically happens in one of the following cases :
% \begin{itemize}
% \item The source file ends before the final \var{end.} statement. This
% happens mostly when the \var{begin} and \var{end} statements aren't
% balanced;
% \item An include file ends in the middle of a statement.
% \item A comment was not closed
% \end{itemize}
scan_f_string_exceeds_line=02001_F_Streng overskrider slutning af linie
% There is a missing closing ' in a string, so it occupies
% multiple lines.
scan_f_illegal_char=02002_F_Ugyldig karakter "$1" ($2)
% An illegal character was encountered in the input file.
scan_f_syn_expected=02003_F_Syntaxfejl, "$1" forventet men "$2" fundet
% This indicates that the compiler expected a different token than
% the one you typed. It can occur almost everywhere where you make a
% mistake against the pascal language.
scan_t_start_include_file=02004_TL_Læser includefil $1
% When you provide the \var{-vt} switch, the compiler tells you
% when it starts reading an included file.
scan_w_comment_level=02005_W_Kommentarniveau $1 fundet
% When the \var{-vw} switch is used, then the compiler warns you if
% it finds nested comments. Nested comments are not allowed in Turbo Pascal
% and can be a possible source of errors.
scan_n_ignored_switch=02008_N_Ignorerer compilerindstilling "$1"
% With \var{-vn} on, the compiler warns if it ignores a switch
scan_w_illegal_switch=02009_W_Ugyldig compilerindstilling "$1"
% You included a compiler switch (i.e. \var{\{\$... \}}) which the compiler
% does not recognise
scan_w_switch_is_global=02010_W_Fejlplaceret global compilerindstilling
% The compiler switch is misplaced, and should be located at
% the start of the unit or program.
scan_e_illegal_char_const=02011_E_Ugyldig karakterkonstant
% This happens when you specify a character with its ASCII code, as in
% \var{\#96}, but the number is either illegal, or out of range.
scan_f_cannot_open_input=02012_F_Kan ikke åbne filen "$1"
% \fpc cannot find the program or unit source file you specified on the
% command line.
scan_f_cannot_open_includefile=02013_F_Kan ikke åbne includefilen "$1"
% \fpc cannot find the source file you specified in a \var{\{\$include ..\}}
% statement.
scan_e_illegal_pack_records=02015_E_Ugyldig strukturopstillingsspecifikation "$1"
% You are specifying the \var{\{\$PACKRECORDS n\} } or \var{\{\$ALIGN n\} }
% with an illegal value for \var{n}. For \$PACKRECORDS valid alignments are 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, and for \$ALIGN valid alignment are 1, 2, 4, 8, 16, 32, ON,
% OFF. Under mode MacPas \$ALIGN also supports MAC68K, POWER and RESET.
scan_e_illegal_pack_enum=02016_E_Ugyldig enumerations-minimumsstørrelsespecifikation "$1"
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2,4, NORMAL or DEFAULT are valid here.
scan_e_endif_expected=02017_E_$ENDIF forventet til $1 $2 defineret i $3 linie $4
% Your conditional compilation statements are unbalanced.
scan_e_preproc_syntax_error=02018_E_Syntaxfejl under læsning af betinget kompileret udtryk
% There is an error in the expression following the \var{\{\$if ..\}}, $ifc or $setc compiler
% directives.
scan_e_error_in_preproc_expr=02019_E_Evaluerer betinget kompileret udtryk
% There is an error in the expression following the \var{\{\$if ..\}}, $ifc or $setc compiler
% directives.
scan_w_macro_cut_after_255_chars=02020_W_Makroindhold er begrænset til 255 tegns længde
% The contents of macros cannot be longer than 255 characters.
scan_e_endif_without_if=02021_E_ENDIF uden IF(N)DEF
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements aren't balanced.
scan_f_user_defined=02022_F_Brugerdefineret: $1
% A user defined fatal error occurred. see also the \progref
scan_e_user_defined=02023_E_Brugerdefineret: $1
% A user defined error occurred. see also the \progref
scan_w_user_defined=02024_W_Brugerdefineret: $1
% A user defined warning occurred. see also the \progref
scan_n_user_defined=02025_N_Brugerdefineret: $1
% A user defined note was encountered. see also the \progref
scan_h_user_defined=02026_H_Brugerdefineret: $1
% A user defined hint was encountered. see also the \progref
scan_i_user_defined=02027_I_Brugerdefineret: $1
% User defined information was encountered. see also the \progref
scan_e_keyword_cant_be_a_macro=02028_E_Makroer kan ikke omdefinere nøgleord
% You cannot redefine keywords with macros.
scan_f_macro_buffer_overflow=02029_F_Overløb under læsning eller udvidelse af makro
% Your macro or it's result was too long for the compiler.
scan_w_macro_too_deep=02030_W_Makroudfoldning har oversteget 16 niveauer
% When expanding a macro, macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_w_wrong_styled_switch=02031_W_Compilerindstillinger understøttes ikke i // denne slags kommentarer
% Compiler switches should be in normal pascal style comments.
scan_d_handling_switch=02032_DL_Håndterer indstilling "$1"
% When you set debugging info on (\var{-vd}) the compiler tells you when it
% is evaluating conditional compile statements.
scan_c_endif_found=02033_CL_ENDIF $1 fundet
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifdef_found=02034_CL_IFDEF $1 fundet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifopt_found=02035_CL_IFOPT $1 fundet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_if_found=02036_CL_IF $1 fundet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifndef_found=02037_CL_IFNDEF $1 fundet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_else_found=02038_CL_ELSE $1 fundet, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_skipping_until=02039_CL_Springer over indtil...
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements, and whether it is skipping or
% compiling parts.
scan_i_press_enter=02040_I_Tryk <return> for at fortsætte
% When the \var{-vi} switch is used, the compiler stops compilation
% and waits for the \var{Enter} key to be pressed when it encounters
% a \var{\{\$STOP\}} directive.
scan_w_unsupported_switch=02041_W_Ugyldig compilerindstilling "$1"
% When warnings are turned on (\var{-vw}) the compiler warns you about
% unsupported switches. This means that the switch is used in Delphi or
% Turbo Pascal, but not in \fpc
scan_w_illegal_directive=02042_W_Ugyldig compilerindstilling "$1"
% When warings are turned on (\var{-vw}) the compiler warns you about
% unrecognised switches. For a list of recognised switches, \progref
scan_t_back_in=02043_TL_Tilbage i $1
% When you use (\var{-vt}) the compiler tells you when it has finished
% reading an include file.
scan_w_unsupported_app_type=02044_W_Ugyldig programtype: "$1"
% You get this warning, ff you specify an unknown application type
% with the directive \var{\{\$APPTYPE\}}
scan_w_app_type_not_support=02045_W_APPTYPE understøttes ikke af det OS der kompileres til
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_w_description_not_support=02046_W_DESCRIPTION understøttes ikke af det OS der kompileres til
% The \var{\{\$DESCRIPTION\}} directive is not supported on this target OS
scan_n_version_not_support=02047_N_VERSION understøttes ikke af det OS der kompileres til
% The \var{\{\$VERSION\}} directive is not supported on this target OS
scan_n_only_exe_version=02048_N_VERSION er kun til programmer og DLL-filer
% The \var{\{\$VERSION\}} directive is only used for executable or DLL sources.
scan_w_wrong_version_ignored=02049_W_Direktivet VERSION har forkert format "$1"
% The \var{\{\$VERSION\}} directive format is majorversion.minorversion
% where majorversion and minorversion are words.
scan_e_illegal_asmmode_specifier=02050_E_Ugyldig assemblerstil angivet "$1"
% When you specify an assembler mode with the \var{\{\$ASMMODE xxx\}}
% the compiler didn't recognize the mode you specified.
scan_w_no_asm_reader_switch_inside_asm=02051_W_Kan ikke skifte assemblerlæser midt i en assemblerblok. Skifter til "$1" i næste blok
% It is not possible to switch from one assembler reader to another
% inside an assembler block. The new reader will be used for next
% assembler statements only.
scan_e_wrong_switch_toggle=02052_E_Forkert indstillingsparameter. Brug ON/OFF eller +/-
% You need to use ON or OFF or a + or - to toggle the switch
scan_e_resourcefiles_not_supported=02053_E_Resourcefiler understøttes ikke af mål-operativsystemet
% The target you are compiling for doesn't support resource files.
scan_w_include_env_not_found=02054_W_Environment-værdien "$1" er ikke defineret
% The included environment variable cannot be found in the environment, it will
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_Ugyldig værdi for FPU-registergrænse
% Valid values for this directive are 0..8 and NORMAL/DEFAULT
scan_w_only_one_resourcefile_supported=02056_W_Mål-systemet understøtter kun én resourcefil
% The target you are compiling for supports only one resource file.
% The first resource file found is used, the others are discarded.
scan_w_macro_support_turned_off=02057_W_Makrosupport er slået fra
% A macro declaration has been found, but macro support is currently off,
% so the declaration will be ignored. To turn macro support on compile with
% -Sm on the commandline or add {$MACRO ON} in the source
scan_e_invalid_interface_type=02058_E_Ugyldig interface-type angivet. Gyldige værdier er COM, CORBA eller DEFAULT.
% The interface type that was specified is not supported
scan_w_appid_not_support=02059_W_APPID understøttes kun til PalmOS
% The \var{\{\$APPID\}} directive is only supported for the PalmOS target.
scan_w_appname_not_support=02060_W_APPNAME understøttes kun til PalmOS
% The \var{\{\$APPNAME\}} directive is only supported for the PalmOS target.
scan_e_string_exceeds_255_chars=02061_E_Strengkonstanter kan ikke være længere end 255 tegn
% A single string constant can contain at most 255 chars. Try splitting up the
% string in multiple smaller parts and concatenate them with a + operator.
scan_f_include_deep_ten=02062_F_Include-filer overskrider dypdegrænsen på 16 niveauer
% When including include files the files have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_too_many_push=02063_F_For mange niveauer af PUSH
% A maximum of 20 levels is allowed.  This error occur only in mode MacPas.
scan_e_too_many_pop=02064_E_POP uden forudgående PUSH
% This error occur only in mode MacPas.
scan_e_error_macro_lacks_value=02065_E_Makro "$1" har ingen værdi
% Thus the conditional compiling expression cannot be evaluated.
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
scan_e_wrong_switch_toggle_default=02066_E_Forkert indstillingsværdi, brug ON/OFF/DEFAULT eller +/-/*
% You need to use ON or OFF or DEFAULT or a + or - or * to toggle the switch
scan_e_mode_switch_not_allowed=02067_E_Tilstandstypeindstilling "$1" ikke tilladt her
% A mode switch has already been encountered, or, in case of option -Mmacpas,
% a mode switch occur after UNIT.
scan_e_error_macro_undefined=02068_E_Kompileringstidsvariablen eller makroen "$1" er ikke defineret
% Thus the conditional compile time expression cannot be evaluated. Only in mode MacPas.
scan_e_utf8_bigger_than_65535=02069_E_UTF-8 kode større end 65535 fundet
% \fpc handles utf-8 strings internally as widestrings e.g. the char codes are limited to 65535
scan_e_utf8_malformed=02070_E_Ugyldig UTF-8-streng
% The given string isn't a valid UTF-8 string
scan_c_switching_to_utf8=02071_C_UTF-8-signatur fundet, benytter UTF-8-kodning
% The compiler found an UTF-8 encoding signature (\$ef, \$bb, \$bf) at the beginning of a file,
% so it interprets it as an UTF-8 file
scan_e_compile_time_typeerror=02072_E_Kompileringstidsudtryk: Forventede $1, men fik $2 ved $3
% Type check of a compile time expression failed.
scan_n_app_type_not_support=02073_N_APPTYPE understøttes ikke af mål-operativsystemet
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
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
scan_e_illegal_optimization_specifier=02074_E_Illegal optimization specified "$1"
% You specified an optimization with the \var{\{\$OPTIMIZATION xxx\}} directive,
% and the compiler didn't recognize the optimization you specified.
scan_w_setpeflags_not_support=02075_W_SETPEFLAGS is not supported by the target OS
% The \var{\{\$SETPEFLAGS\}} directive is not supported by the target OS.
scan_w_imagebase_not_support=02076_W_IMAGEBASE is not supported by the target OS
% The \var{\{\$IMAGEBASE\}} directive is not supported by the target OS.
scan_w_minstacksize_not_support=02077_W_MINSTACKSIZE is not supported by the target OS
% The \var{\{\$MINSTACKSIZE\}} directive is not supported by the target OS.
scan_w_maxstacksize_not_support=02078_W_MAXSTACKSIZE is not supported by the target OS
% The \var{\{\$MAXSTACKSIZE\}} directive is not supported by the target OS.
scanner_e_illegal_warn_state=02079_E_Illegal state "$1" for $WARN directive
% Only ON and OFF can be used as state with a \var{\{\$WARN\}} compiler directive.
scan_e_only_packset=02080_E_Illegal set packing value
% Only 0, 1, 2, 4, 8, DEFAULT and NORMAL are allowed as packset parameters.
scan_w_pic_ignored=02081_W_PIC directive or switch ignored
% Several targets, such as \windows, do not support nor need PIC,
% so the PIC directive and switch are ignored.
scan_w_unsupported_switch_by_target=02082_W_The switch "$1" is not supported by the currently selected target
% Some compiler switches like \$E are not supported by all targets.
scan_w_frameworks_darwin_only=02084_W_Framework-related options are only supported for Darwin/Mac OS X
% Frameworks are not a known concept, or at least not supported by FPC,
% on operating systems other than Darwin/Mac OS X.
scan_e_illegal_minfpconstprec=02085_E_Illegal minimal floating point constant precision "$1"
% Valid minimal precisions for floating point constants are default, 32 and 64,
% which mean respectively minimal (usually 32 bit), 32 bit and 64 bit precision.
scan_w_multiple_main_name_overrides=02086_W_Overriding name of "main" procedure multiple times, was previously set to "$1"
% The name for the main entry procedure is specified more than once. Only the last
% name will be used.
scanner_w_illegal_warn_identifier=02087_W_Illegal identifier "$1" for $WARN directive
% Identifier is not known by a \var{\{\$WARN\}} compiler directive.
scanner_e_illegal_alignment_directive=02088_E_Illegal alignment directive
% The alignment directive is not valid. Either the alignment type is not known or the alignment
% value is not a power of two.
scanner_f_illegal_utf8_bom=02089_F_It is not possible to include a file that starts with an UTF-8 BOM in a module that uses a different code page
% All source code that is part of a single compilation entity (program, library, unit) must be encoded
% in the same code page
scanner_w_directive_ignored_on_target=02090_W_Directive "$1" is ignored for the current target platform
% Some directives are ignored for certain targets, such as changing the
% packrecords and packenum settings on managed platforms.
scan_w_unavailable_system_codepage=02091_W_Current system codepage "$1" is not available for the compiler. Switching default codepage back to "$2".
% The current system codepage is not known by the compiler.
% The compiler is compiled with support for several codepages built-in.
% The codepage of the operation system is not in that list. You will need to recompile
% the compiler with support for this codepage.
scan_w_setpeoptflags_not_support=02092_W_SETPEOPTFLAGS is not supported by the target OS
% The \var{\{\$SETPEOPTFLAGS\}} directive is not supported by the target OS.
scan_e_illegal_peflag=02093_E_Illegal argument for SETPEFLAGS
% The given argument for SETPEFLAGS is neither a correct named value nor an
% ordinal value
scan_e_illegal_peoptflag=02094_E_Illegal argument for SETPEOPTFLAGS
% The given argument for SETPEOPTFLAGS is neither a correct named value nor an
% ordinal value
scan_e_unsupported_switch=02095_E_Directive $1 is not supported on this target
% Not all compiler directives are supported on all targets.
scan_w_invalid_stacksize=02096_W_The specified stack size is not within the valid range for the platform. Setting the stack size ignored.
% The valid range for the stack size is 1024 - 67107839 on 32-bit and 64-bit
% platforms and 1024 - 65520 on 16-bit platforms. Additionally, for Turbo Pascal 7
% compatibility reasons, specifying a stack size of 65521 on 16-bit platforms
% actually sets the stack size to 65520.
scan_w_heapmax_lessthan_heapmin=02097_W_The specified HeapMax value is smaller than the HeapMin value. Setting HeapMax ignored.
% The HeapMax value (if specified) must be greater than or equal to the HeapMin
% value. Otherwise, the HeapMax value is ignored.
scan_e_illegal_hugepointernormalization=02098_E_Illegal argument for HUGEPOINTERNORMALIZATION
% The only allowed values for HUGEPOINTERNORMALIZATION are BORLANDC, MICROSOFTC
% and WATCOMC.
% \end{description}
#
# Parser
#
# 03339 is the last used one
#
% \section{Parser messages}
% This section lists all parser messages. The parser takes care of the
% semantics of you language, i.e. it determines if your Pascal constructs
% are correct.
% \begin{description}
parser_e_syntax_error=03000_E_Parser - syntaxfejl
% An error against the Turbo Pascal language was encountered. This happens
% typically when an illegal character is found in the sources file.
parser_e_dont_nest_interrupt=03004_E_INTERRUPT-procedure kan ikke indlejres
% An \var{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_Proceduretype "$1" ignoreret
% The specified is ignored by FPC programs.
parser_e_no_overload_for_all_procs=03006_E_Ikke alle erklæringer af "$1" er angivet med OVERLOAD
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_export_name_double=03008_E_Funktionsnavnet "$1" er angivet flere gange i en EXPORT-erklæring
% Exported function names inside a specific DLL must all be different
parser_e_export_ordinal_double=03009_E_Funktionsindex $1 er angivet flere gange i en EXPORT-erklæring
% Exported function names inside a specific DLL must all be different
parser_e_export_invalid_index=03010_E_Ugyldigt index for eksporteret funktion
% DLL function index must be in the range \var{1..\$FFFF}
parser_w_parser_reloc_no_debug=03011_W_Relokérbar DLL eller program $1 har ikke-fungerende debug-info. Slået fra.
% It is currently not possible to include debug information in a relocatable DLL.
parser_w_parser_win32_debug_needs_WN=03012_W_For at tillade debugging i win32-kode, skal relokérbarhed slås fra med -WN
% Stabs info is wrong for relocatable DLL or EXES use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_Constructor-navnet skal være INIT
% You are declaring an object constructor with a name which is not \var{init}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_destructorname_must_be_done=03014_E_Destructor-navnet skal være DONE
% You are declaring an object destructor with a name which is not \var{done}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_proc_inline_not_supported=03016_E_Proceduretypen INLINE er ikke understøttet
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler does not support C++
% styled inlining by default.
parser_w_constructor_should_be_public=03018_W_Constructor bør være public
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_Destructor bør være public
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_En klasse skal kun have én destructor
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_Lokale klassedefinitioner er ikke tilladt
% Classes must be defined globally. They cannot be defined inside a
% procedure or function
parser_f_no_anonym_objects=03022_F_Anonyme klassedefinitioner er ikke tilladt
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_n_object_has_no_vmt=03023_N_Objektet "$1" har ingen VMT
% This is a note indicating that the declared object has no
% virtual method table.
parser_e_illegal_parameter_list=03024_E_Ugyldig parameterliste
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_size=03026_E_Forkert antal parametre angivet
% There is an error in the parameter list of the function or procedure,
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_Overloaded navn "$1" er ikke en funktion
% The compiler encountered a symbol with the same name as an overloaded
% function, but it is not a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_Overloadede funktioner har samme parametre
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_Functionserklæring passer ikke med forward-erklæringen $1
% You declared a function with same parameters but
% different result type or function modifiers.
parser_e_header_different_var_names=03030_E_Funktionserklæringen "$1" passer ikke med forward - var-navn ændrer $2 => $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but define it with a different parameter list.
parser_n_duplicate_enum=03031_N_Værdier i enumerate-typer skal være stigende
% \fpc allows enumeration constructions as in C. Given the following
% declaration two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_e_no_with_for_variable_in_other_segments=03033_E_"With" kan ikke bruges til variabler i et andet segment
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_Funktionsindlejring over 31 niveauer
% You can nest function definitions only 31 times.
parser_e_range_check_error=03035_E_Værdiområdefejl under evaluering af konstanter
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_Værdiområdefejl under evaluering af konstanter
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_Gentaget case-værdi
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_Øvre grænse for case-værdiområde er mindre end den nedre grænse
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless
parser_e_type_const_not_possible=03039_E_Typegivne konstanter er ikke tilladt i klasser
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_Tildeling af overloaded funktion til procedurel variabel ikke tilladt
% You are trying to assign an overloaded function to a procedural variable.
% This is not allowed
parser_e_invalid_string_size=03041_E_Strenglængdegrænse skal være mellem 1 og 255
% The length of a shortstring in Pascal is limited to 255 characters. You are
% trying to declare a string with length lower than 1 or greater than 255
parser_w_use_extended_syntax_for_objects=03042_W_Brug udvidet syntax NEW og DISPOSE til instantiering af objekter
% If you have a pointer \var{a} to a class type, then the statement
% \var{new(a)} will not initialize the class (i.e. the constructor isn't
% called), although space will be allocated. you should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the object
parser_w_no_new_dispose_on_void_pointers=03043_W_Brug af NEW eller DISPOSE på typeløse pointere er meningsløst
parser_e_no_new_dispose_on_void_pointers=03044_E_Brug af NEW eller DISPOSE på typeløse pointere ikke muligt
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% Accepted for compatibility in \var{tp} and \var{delphi} modes.
parser_e_class_id_expected=03045_E_Klassenavn forventet
% This happens when the compiler scans a procedure declaration that contains
% a dot,
% i.e., a object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_Typenavn ikke tilladt her
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_Metodenavn forventet
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_Funktionserklæring passer ikke med nogen metoder i klassen "$1"
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_d_procedure_start=03049_DL_procedure/funktion $1
% When using the \var{-vd} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_Ugyldig kommetalskonstant
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_FAIL kan kun bruges i en constructor
% You are using the \var{fail} keyword outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_En destructor kan ikke have parametre
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_members_via_class_ref=03053_E_Only class methods, class properties and class variables can be referred with class references
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
parser_e_only_class_members=03054_E_Only class methods, class properties and class variables can be accessed in class methods
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
parser_e_case_mismatch=03055_E_Constant-type of CASE-type stemmer ikke overens
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_Symbolet kan ikke eksporteres fra et bibliotek
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_En nedarvet metode skjules af "$1"
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendent class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_Der er ingen metode i super-klassen at override: "$1"
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_no_procedure_to_access_property=03059_E_Ingen læseadgang til egenskaben
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_Gemte egenskaber er endnu ikke implementeret
% The \var{stored} directive is not yet implemented
parser_e_ill_property_access_sym=03061_E_Ugyldig værdi for egenskabsadgang
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code woud cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Kan ikke tilgå et protected felt i et objekt her
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module wher the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_Kan ikke tilgå et private felt i et objekt her
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_e_overridden_methods_not_same_ret=03066_E_Metoder med "override" skal have samme retur-type: "$2" er overridet af "$1" der har en anden retur-type
% If you declare overridden methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_EXPORT-erklærede funktioner kan ikke indlejres
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_Metoder kan ikke EXPORT'es
% You cannot declare a procedure that is a method for an object as
% \var{export}ed.
parser_e_call_by_ref_without_typeconv=03069_E_Kald med var-parametre skal passe præcist: Fandt "$1", "$2" forventet
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_Klassen er ikke en super-klasse af den angivne klasse
% When calling inherited methods, you are trying to call a method of a non-related
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_SELF kan kun bruges i objekter
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_methods can be only in other methods called direct with type identifier of the class
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_Ugyldig brug af ':'
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_Værdigrænsefejl i set-constructor, eller genanvendt element-værdi
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, either two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_Pointer til objekt forventet
% You specified an illegal type in a \var{new} statement.
% The extended syntax of \var{new} needs an  object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_Udtryk skal være constructor-kald
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_Udtryk skal være destructor-kald
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_Ugyldig rækkefølge af strukturelementer
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_Udtrykket skal være en klasse eller struktur
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_Procedurer kan ikke returnere værdier
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures  cannot return a value.
parser_e_only_methods_allowed=03081_E_constructors destructors og class operators skal være metoder
% You're declaring a procedure as destructor, constructor or class operator, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_Operatoren er ikke overloaded
% You're trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_no_such_assignment=03083_E_Umuligt at overloade tildeling for ensartede typer
% You can not overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_Umuligt at overloade operatoren
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_Re-raise er ikke muligt her
% You are trying to raise an exception where it is not allowed. You can only
% raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_Den udvidede syntax af new og dispose er ikke gyldig for klasser
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_procedure_overloading_is_off=03088_E_Procedure overloading er slået fra
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_Det er ikke muligt at overloade denne operator (overload "=" i stedet)
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_Sammenligningsoperationer skal returnere en sandhedsværdi
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_Kun virtuelle metoder kan være abstrakte
% You are declaring a method as abstract, when it is not declared to be
% virtual.
parser_f_unsupported_feature=03092_F_Ikke-understøttet konstruktion
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_Det er ikke gyldigt at blande forskellige typer objekter (class, object, interface, etc)
% You cannot derive \var{objects}, \var{classes}, \var{cppclasses} and \var{interfaces} interttwined . E.g.
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_Ukendt procedure-direktev blev ignoreret: "$1"
% The procedure directive you specified is unknown.
parser_e_directive_only_one_var=03095_E_$1 can be associated with only one variable
% You cannot specify more than one variable before the \var{absolute}, \var{export}, \var{external},
% \var{weakexternal}, \var{public} and \var{cvar} directives.
% As a result, for example the following construct will provide this error:
% \begin{verbatim}
% Var Z : Longint;
%     X,Y : Longint absolute Z;
% \end{verbatim}
parser_e_absolute_only_to_var_or_const=03096_E_Absolute kan kun associeres med én variabel eller konstant
% The address of a \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_Kun én variabel kan initialiseres
% You cannot specify more than one variable with a initial value
% in Delphi mode.
parser_e_abstract_no_definition=03098_E_Abstrakte metoder må ikke implementeres
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_Denne overloadede funktion kan ikke være lokal (skal eksporteres)
% You are defining a overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_Virtuelle metoder er brugt uden at have en constructor: "$1"
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_c_macro_defined=03101_CL_Makro defineret: $1
% When \var{-vc} is used, the compiler tells you when it defines macros.
parser_c_macro_undefined=03102_CL_Makro fjernet: $1
% When \var{-vc} is used, the compiler tells you when it undefines macros.
parser_c_macro_set_to=03103_CL_Makro $1 sat til $2
% When \var{-vc} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_Compilerer $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_UL_Parser interface-delen af unit $1
% This tells you that the reading of the interface
% of the current unit starts
parser_u_parsing_implementation=03106_UL_Parser implementation-delen af unit $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_DL_Compilerer $1 for anden gang
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_property_found_to_override=03109_E_Ingen egenskab at override
% You want to overrride a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_Kun én default-egenskab tilladt
% You specified a property as \var{Default}, but the class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_Default-egenskaben skal være et array
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_Virtuelle constructors er kun tilladt i klassemodellen
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_Ingen default-egenskab tilgængelig
% You are trying to access a default property of a class, but this class (or one of
% it's ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_Klassen kan ikke have en published-sektion, brug {$M+}-indstillingen
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, whch turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_Forward-erklæring af klasse "$1" skal løses her, for at bruge klassen som super-klasse
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% Where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_Lokale operatorer ikke understøttet
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_Procedure-direktivet "$1" er ikke tilladt i interface-sektionen
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_Procedure-direktivet "$1" er ikke tilladt i implementation-sektionen
% This procedure directive is not defined in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_Procedure-direktivet "$1" er ikke tilladt i procvar-erklæring
% This procedure directive cannot be part of a procedural or function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_Funktionen "$1" er allerede erklæret Public/Forward
% You will get this error if a function is defined as \var{forward} twice.
% Or it is once in the \var{interface} section, and once as a \var{forward}
% declaration in the \var{implmentation} section.
parser_e_not_external_and_export=03121_E_Kan ikke bruge både EXPORT og EXTERNAL
% These two procedure directives are mutually exclusive
parser_n_not_supported_for_inline=03123_N_Deklarationen "$1" er endnu ikke understøttet i inline procedurer/funktioner
% Inline procedures don't support this declaration.
parser_h_inlining_disabled=03124_H_Inlining slået fra
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_Skriver browser-log $1
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+ \}} switch).
parser_h_maybe_deref_caret_missing=03126_H_Måske mangler en pointer-dereference
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_Valgt assembler-læser er ikke understøttet
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_Procedure-direktivet "$1" er i konflikt med andre direktiver
% You specified a procedure directive that conflicts with other directives.
% for instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_Kaldkonvention passer ikke med forward-erklæring
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_property_cant_have_a_default_value=03131_E_Egenskaben kan ikke have default-værdi
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_Default-værdien for en egenskab skal være konstant
% The value of a \var{default} declared property must be known at compile
% time. The value you specified is only known at run time. This happens
% .e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_Variabler kan ikke være published, med mindre de er klasser
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_Egenskabs-typen er ikke understøttet i published
% Properties in a \var{published} section cannot be array properties.
% they must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_e_empty_import_name=03136_E_Et import-navn er krævet
% Some targets need a name for the imported procedure or a \var{cdecl} specifier
parser_e_division_by_zero=03138_E_Division med nul
% There is a divsion by zero encounted
parser_e_invalid_float_operation=03139_E_Ugyldig floating-point operation
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_Øvre grænse er mindre end nedre grænse
% The upper bound of a an array declaration is less than the lower bound and this
% is not possible
parser_w_string_too_long=03141_W_Strengen "$1" er længere end "$2"
% The size of the constant string is larger than the size you specified in
% string type definition
parser_e_string_larger_array=03142_E_Strenglængde er større end længden af karakter-arrayet
% The size of the constant string is larger than the size you specified in
% the array[x..y] of char definition
parser_e_ill_msg_expr=03143_E_Ugyldigt udtryk efter beskedddirektiv
% \fpc supports only integer or string values as message constants
parser_e_ill_msg_param=03144_E_Procedurerer til beskedhåndtering kan kun tage én parameter, der skal være en reference (var)
% A method declared with the \var{message}-directive as message handler
% can take only one parameter which must be declared as call by reference
% Parameters are declared as call by reference using the \var{var}-directive
parser_e_duplicate_message_label=03145_E_Gentaget besked-værdi: "$1"
% A label for a message is used twice in one object/class
parser_e_self_in_non_message_handler=03146_E_"Self" kan kun være en eksplicit parameter i metoder der er beskedhandlere
% The self parameter can only be passed explicitly to a method which
% is declared as message handler.
parser_e_threadvars_only_sg=03147_E_Trådvariabler kan kun erklæres static eller globalt
% Threadvars must be static or global, you can't declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has it's own stack and local variables
% are stored on the stack
parser_f_direct_assembler_not_allowed=03148_F_Direkte assembler er ikke understøttet i det binære udformat
% You can't use direct assembler when using a binary writer, choose an
% other outputformat or use another assembler reader
parser_w_no_objpas_use_mode=03149_W_Inkludér ikke unit'en OBJPAS manuelt, brug i stedet {$mode objfpc} eller {$mode delphi}
% You are trying to load the ObjPas unit manually from a uses clause. This is
% not a good idea. Use the \var{\{\$mode objfpc\}} or
% \var{\{\$mode delphi\}}
% directives which load the unit automatically
parser_e_no_object_override=03150_E_OVERRIDE kan ikke bruges i objekter
% Override is not supported for objects, use \var{virtual} instead to override
% a method of a parent object
parser_e_cant_use_inittable_here=03151_E_Datatyper der kræver initialisering/afslutning kan ikke bruges i en variantstruktur
% Some data type (e.g. \var{ansistring}) needs initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% can't be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_Strengresourcer kan kun være statiske eller globale
% Resourcestring can not be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_Exit med parametre kan ikke bruges her
% an exit statement with an argument for the return value can't be used here, this
% can happen e.g. in \var{try..except} or \var{try..finally} blocks
parser_e_stored_property_must_be_boolean=03154_E_Storage-symbolet skal være en sandhedsværdi
% If you specify a storage symbol in a property declaration, it must be of
% the type boolean
parser_e_ill_property_storage_sym=03155_E_Dette symbol er ikke tilladt som storage-symbol
% You can't use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants
parser_e_only_publishable_classes_can_be_published=03156_E_Only classes which are compiled in $M+ mode can be published
% A class-typed field in the published section of a class can only be a class which was
% compiled in \var{\{\$M+\}} or which is derived from such a class. Normally
% such a class should be derived from \var{TPersistent}.
parser_e_proc_directive_expected=03157_E_Procedure-direktiv forventet
% When declaring a procedure in a const block you used a ; after the
% procedure declaration after which a procedure directive must follow.
% Correct declarations are:
% \begin{verbatim}
% const
%   p : procedure;stdcall=nil;
%   p : procedure stdcall=nil;
% \end{verbatim}
parser_e_invalid_property_index_value=03158_E_Værdien for egenskabs-indexet skal være en ordinal-type
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_Procedurenavn for kort til at eksportere
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correct with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_Intet DEFFILE-opslag kan genereres for variabler der er globale i en unit
parser_e_dlltool_unit_var_problem2=03161_E_Kompilér uden -WD indstillingen
% You need to compile this file without the -WD switch on the
% commandline
parser_f_need_objfpc_or_delphi_mode=03162_F_Du skal bruge ObjFpc (-S2) eller Delphi (-Sd)-mode for at kompilere denne fil
% You need to use {$mode objfpc} or {$mode delphi} to compile this file.
% Or use the equivalent commandline switches -S2 or -Sd.
parser_e_no_export_with_index_for_target=03163_E_Kan ikke eksportere med index under $1
% Exporting of functions or procedures with a specified index is not
% supported on this target.
parser_e_no_export_of_variables_for_target=03164_E_Eksport af variabler ikke understøttet af målsystemet $1
% Exporting of variables is not supported on this target.
parser_e_improper_guid_syntax=03165_E_Ugyldig GUID-syntax
% The GUID indication does not have the proper syntax. It should be of the form
% \begin{verbatim}
% {XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}
% \end{verbatim}
% Where each \var{X} represents a hexadecimal digit.
parser_w_interface_mapping_notfound=03168_W_En procedure kaldet $1 der kan implementere $2 blev ikke fundet. $3
% The compiler cannot find a suitable procedure which implements the given method of an interface.
% A procedure with the same name is found, but the arguments do not match.
parser_e_interface_id_expected=03169_E_Interface-navn krævet
% This happens when the compiler scans a \var{class} declaration that contains
% \var{interface} function name mapping code like this:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% and the \var{interface} before the dot not listed in the inheritance list.
parser_e_type_cant_be_used_in_array_index=03170_E_Typen "$1" kan ikke bruges som index-type i et array
% Types like \var{qword} or \var{int64} aren't allowed as array index type
parser_e_no_con_des_in_interfaces=03171_E_Con- og destructors er ikke tilladt i interfaces
% Constructor and destructor declarations aren't allowed in interface
% In the most cases the method \var{QueryInterface} of \var{IUnknown} can
% be used to create a new interface.
parser_e_no_access_specifier_in_interfaces=03172_E_Adgangskrav kan ikke sættes i et interface
% The access specifiers \var{public}, \var{private}, \var{protected} and
% \var{pusblished} can't be used in interfaces because all methods
% of an interfaces must be public.
parser_e_no_vars_in_interfaces=03173_E_Et interface kan ikke indeholde felter
% Declarations of fields aren't allowed in interfaces. An interface
% can contain only methods
parser_e_no_local_proc_external=03174_E_Kan ikke erklære en lokal procedure som ekstern
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance of errors very high
parser_w_skipped_fields_before=03175_W_Visse felter i "$1" blev ikke initialiseret
% In Delphi mode, not all fields of a typed constant record have to be
% initialized, but the compiler warns you when it detects such situations.
parser_e_skipped_fields_before=03176_E_Visse felter i "$1" blev ikke initialiseret
% In all syntax modes but Delphi mode, you can't leave some fields uninitialized
% in the middle of a typed constant record
parser_w_skipped_fields_after=03177_W_Visse felter efter "$1" er ikke initialiseret
% You can leave some fields at the end of a type constant record uninitialized
% (the compiler will initialize them to zero automatically). This may be the cause
% of subtle problems.
parser_e_varargs_need_cdecl_and_external=03178_E_VarArgs-direktiv uden CDecl og External
% The varargs directive can only be used with procedures or functions
% that are declared with \var{cdecl} and \var{external} directives. The varargs directive
% is only meant to provide a compatible interface to C functions like printf.
parser_e_self_call_by_value=03179_E_Self skal være en almindelig (værdioverført) parameter
% You can't declare self as a const or var parameter, it must always be
% a call-by-value parameter
parser_e_interface_has_no_guid=03180_E_Interfacet "$1" har intet GUID
% When you want to assign an interface to a constant, then the interface
% must have a GUID value set.
parser_e_illegal_field_or_method=03181_E_Ukendt klassefelt eller metodenavn "$1"
% Properties must refer to a field or method in the same class.
parser_w_proc_overriding_calling=03182_W_Overrider kaldkonvention mellem "$1" og "$2"
% There are two directives in the procedure declaration that specify a calling
% convention. Only the last directive will be used
parser_e_no_procvarobj_const=03183_E_Konstanter af typen "procedure of object" kan kun initialiseres med NIL
% You can't assign the address of a method to a typed constant which has a
% 'procedure of object' type, because such a constant requires two addresses:
% that of the method (which is known at compile time) and that of the object or
% class instance it operates on (which can not be known at compile time).
parser_e_default_value_only_one_para=03184_E_Default-værdi kan kun tildeles én parameter
% It is not possible to specify a default value for several parameters at once.
% The following is invalid:
% \begin{verbatim}
% Procedure MyProcedure (A,B : Integer = 0);
% \end{verbatim}
% Instead, this should be declared as
% \begin{verbatim}
% Procedure MyProcedure (A : Integer = 0; B : Integer = 0);
% \end{verbatim}
parser_e_default_value_expected_for_para=03185_E_Default-værdi krævet for parameter "$1"
% The specified parameter requires a default value.
parser_w_unsupported_feature=03186_W_Feature ikke understøttet
% You're trying to force the compiler into doing something it cannot do yet.
parser_h_c_arrays_are_references=03187_H_C-arrays videregives som reference
% Any array passed to a C functions is passed
% by a pointer (i.e. by reference).
parser_e_C_array_of_const_must_be_last=03188_E_C-arrays af konstanter skal være det sidste element
% You can not add any other argument after an \var{array of const} for
% \var{cdecl} functions, as the size pushed on stack for this argument is
% not known.
parser_h_type_redef=03189_H_Type "$1" redefineret
% This is an indicator that a previously declared type is
% being redefined as something else. This may, or may not
% be, a cause for errors.
parser_w_cdecl_has_no_high=03190_W_cdecl'arerede funktioner har ingen "high"-parameter
% Functions declared with cdecl modifier do not pass an extra implicit parameter.
parser_w_cdecl_no_openstring=03191_W_cdecl'arerede funktioner understøtter ikke åbne strenge
% Openstring is not supported for cdecl'ared functions.
parser_e_initialized_not_for_threadvar=03192_E_Kan ikke initialisere variabler der er erklæret som threadvar
% Variables declared as threadvar can not be initialized with a default value.
% The variables will always be filled with zero at the start of a new thread.
parser_e_msg_only_for_classes=03193_E_Message-direktivet kun gyldigt i klasser
% The message directive is only supported for Class types.
parser_e_procedure_or_function_expected=03194_E_Procedure eller Function forventet
% A class method can only be specified for procedures and functions.
parser_e_illegal_calling_convention=03195_W_Kaldkonventions-direktiv ignoreret: "$1"
% Some calling conventions are supported only by certain CPUs. I.e. most non-i386 ports support
% only the standard ABI calling convention of the CPU.
parser_e_no_object_reintroduce=03196_E_REINTRODUCE kan ikke bruges i objekter
% \var{reintroduce} is not supported for objects.
parser_e_paraloc_only_one_para=03197_E_Hvert argument skal have sin egen pladsspecifikation
% If locations for arguments are specified explicitly as it is required by
% some syscall conventions, each argument must have it's only location, things
% like \var{procedure p(i,j : longint 'r1');} aren't allowed
parser_e_paraloc_all_paras=03198_E_Hvert argument skal have en specifik pladsspecifikatino
% If one argument has an explicit argument location, all arguments of a procedure
% must have one.
parser_e_illegal_explicit_paraloc=03199_E_Ukendt pladsspecifikation parameter
% The location specified for an argument isn't recognized by the compiler
parser_e_32bitint_or_pointer_variable_expected=03200_E_32-bit heltal eller pointer forventet
% The libbase for MorphOS/AmigaOS can be give only as \var{longint}, \var{dword} or any pointer variable.
parser_e_goto_outside_proc=03201_E_Goto-erklæring ikke tilladt mellem forskellige procedurer
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
parser_f_too_complex_proc=03202_F_Procedure for kompleks, den kræver for mange registre
% Your procedure body is too long for the compiler. You should split the
% procedure into multiple smaller procedures.
parser_e_illegal_expression=03203_E_Ugyldigt udtryk
% This can occur under many circumstances. Mostly when trying to evaluate
% constant expressions.
parser_e_invalid_integer=03204_E_Ugyldigt heltalsudtryk
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
parser_e_invalid_qualifier=03205_E_Ugyldigt udtryk
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
parser_e_upper_lower_than_lower=03206_E_Øvre grænse er mindre end nedre grænse
% You are declaring a subrange, and the lower limit is higher than the high
% limit of the range.
parser_e_macpas_exit_wrong_param=03207_E_Parametren til Exit skal være navnet på den kaldende procedure
% Non local exit is not allowed. This error occur only in mode MacPas.
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
parser_e_illegal_assignment_to_count_var=03208_E_Ugyldig tildeling til for-løkkevariablen "$1"
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings. You can also not assign values to
% loop variables inside the loop (except in Delphi and TP modes). Use a while or
% repeat loop instead if you need to do something like that, since those
% constructs were built for that.
parser_e_no_local_var_external=03209_E_Kan ikke erklære en lokal variabel som ekstern
% Declaring local variables as external is not allowed. Only global variables can reference
% to external variables.
parser_e_proc_already_external=03210_E_Proceduren er allerede erklæret ekstern
% The procedure is already declared with the EXTERNAL directive in an interface or
% forward declaration.
parser_w_implicit_uses_of_variants_unit=03211_W_Implicit brug af Variants-modulet.
% The Variant type is used in the unit without any used unit using the Variants unit. The
% compiler has implicitly added the Variants unit to the uses list. To remove this warning
% the Variants unit needs to be added to the uses statement.
parser_e_no_static_method_in_interfaces=03212_E_Klasse- og statiske metoder kan ikke benyttes i interfaces
% The specifier \var{class} and directive \var{static} can't be used in interfaces
% because all methods of an interfaces must be public.
parser_e_arithmetic_operation_overflow=03213_E_Overløb i aritmetisk operation
% An operation on two integers values produced an overflow
parser_e_protected_or_private_expected=03214_E_"Protected" eller "private" forventet
% \var{strict} can be only used together with \var{protected} or \var{private}.
parser_e_illegal_slice=03215_E_"Slice" kan kun bruges med parameterlister
% \var{slice} can be used only for arguments accepting an open array parameter
parser_e_dispinterface_cant_have_parent=03216_E_Et DispInterface kan ikke nedarve fra en anden klasse
% A DISPINMTERFACE is a special type of interface which can't have a parent class
parser_e_dispinterface_needs_a_guid=03217_E_Et DispInterface kræver et GUID
% A DISPINMTERFACE always needs an interface identification
parser_w_overridden_methods_not_same_ret=03218_W_Metoder med "override" skal have relaterede returtyper. Denne kode kan fejle, da den afhænger af en fejl i Delphis parser ("$2" overrides af "$1", der har en anden returtype)
% If you declare overridden methods in a class definition, they must
% have the same return type. Some versions of Delphi allow you to change the
% return type of interface methods, and even to change procedures into
% functions, but the resulting code may crash depending on the types used
% and the way the methods are called.
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
parser_e_dispid_must_be_ord_const=03219_E_Dispatch IDs must be ordinal constants
% The \var{dispid} keyword must be followed by an ordinal constant (the dispid index).
parser_e_array_range_out_of_bounds=03220_E_The range of the array is too large
% Regardless of the size taken up by its elements, an array cannot have more
% than high(ptrint) elements. Additionally, the range type must be a subrange
% of ptrint.
parser_e_packed_element_no_var_addr=03221_E_The address cannot be taken of bit packed array elements and record fields
% If you declare an array or record as \var{packed} in Mac Pascal mode
% (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. This means it becomes impossible to take addresses
% of individual array elements or record fields. The only exception to this rule
% is in the case of packed arrays elements whose packed size is a multiple of 8 bits.
parser_e_packed_dynamic_open_array=03222_E_Dynamic arrays cannot be packed
% Only regular (and possibly in the future also open) arrays can be packed.
parser_e_packed_element_no_loop=03223_E_Bit packed array elements and record fields cannot be used as loop variables
% If you declare an array or record as \var{packed} in Mac Pascal mode
% (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. For performance reasons, they cannot be
% used as loop variables.
parser_e_type_var_const_only_in_records_and_classes=03224_E_VAR, TYPE and CONST are allowed only in records, objects and classes
% The usage of VAR, TYPE and CONST to declare new types inside an object is allowed only inside
% records, objects and classes.
parser_e_cant_create_generics_of_this_type=03225_E_This type cannot be a generic
% Only Classes, Objects, Interfaces and Records are allowed to be used as generic.
parser_w_no_lineinfo_use_switch=03226_W_Don't load LINEINFO unit manually, Use the -gl compiler switch instead
% Do not use the \file{lineinfo} unit directly, Use the \var{-gl} switch which
% automatically adds the correct unit for reading the selected type of debugging
% information. The unit that needs to be used depends on the type of
% debug information used when compiling the binary.
parser_e_no_funcret_specified=03227_E_No function result type specified for function "$1"
% The first time you declare a function you have to declare it completely,
% including all parameters and the result type.
parser_e_special_onlygenerics=03228_E_Specialization is only supported for generic types
% Types which are not generics cannot be specialized.
parser_e_no_generics_as_params=03229_E_Generics cannot be used as parameters when specializing generics
% When specializing a generic, only non-generic types can be used as parameters.
parser_e_type_object_constants=03230_E_Constants of objects containing a VMT are not allowed
% If an object requires a VMT either because it contains a constructor or virtual methods,
% it's not allowed to create constants of it. In TP and Delphi mode this is allowed
% for compatibility reasons.
parser_e_label_outside_proc=03231_E_Taking the address of labels defined outside the current scope isn't allowed
% It isn't allowed to take the address of labels outside the
% current procedure.
parser_e_initialized_not_for_external=03233_E_Cannot initialize variables declared as external
% Variables declared as external cannot be initialized with a default value.
parser_e_illegal_function_result=03234_E_Illegal function result type
% Some types like file types cannot be used as function result.
parser_e_no_common_type=03235_E_No common type possible between "$1" and "$2"
% To perform an operation on integers, the compiler converts both operands
% to their common type, which appears to be an invalid type. To determine the
% common type of the operands, the compiler takes the minimum of the minimal values
% of both types, and the maximum of the maximal values of both types. The common
% type is then minimum..maximum.
parser_e_no_generics_as_types=03236_E_Generics without specialization cannot be used as a type for a variable
% Generics must be always specialized before being used as variable type.
parser_w_register_list_ignored=03237_W_Register list is ignored for pure assembler routines
% When using pure assembler routines, the list with modified registers is ignored.
parser_e_implements_must_be_class_or_interface=03238_E_Implements property must have class or interface type
% A property which implements an interface must be of type class or interface.
parser_e_implements_must_have_correct_type=03239_E_Implements-property must implement interface of correct type, found "$1" expected "$2"
% A property which implements an interface actually implements a different interface.
parser_e_implements_must_read_specifier=03240_E_Implements-property must have read specifier
% A property  which implements an interface must have at least a read specifier.
parser_e_implements_must_not_have_write_specifier=03241_E_Implements-property must not have write-specifier
% A property  which implements an interface may not have a write specifier.
parser_e_implements_must_not_have_stored_specifier=03242_E_Implements-property must not have stored-specifier
% A property  which implements an interface may not have a stored specifier.
parser_e_implements_uses_non_implemented_interface=03243_E_Implements-property used on unimplemented interface: "$1"
% The interface which is implemented by a property is not an interface implemented by the class.
parser_e_unsupported_real=03244_E_Floating point not supported for this target
% The compiler parsed a floating point expression, but it is not supported.
parser_e_class_doesnt_implement_interface=03245_E_Class "$1" does not implement interface "$2"
% The delegated interface is not implemented by the class given in the implements clause.
parser_e_class_implements_must_be_interface=03246_E_Type used by implements must be an interface
% The \var{implements} keyword must be followed by an interface type.
parser_e_cant_export_var_different_name=03247_E_Variables cannot be exported with a different name on this target, add the name to the declaration using the "export" directive (variable name: $1, declared export name: $2)
% On most targets it is not possible to change the name under which a variable
% is exported inside the \var{exports} statement of a library.
% In that case, you have to specify the export name at the point where the
% variable is declared, using the \var{export} and \var{alias} directives.
parser_e_weak_external_not_supported=03248_E_Weak external symbols are not supported for the current target
% A "weak external" symbol is a symbol which may or may not exist at (either static
% or dynamic) link time. This concept may not be available (or implemented yet)
% on the current cpu/OS target.
parser_e_forward_mismatch=03249_E_Forward type definition does not match
% Classes and interfaces being defined forward must have the same type
% when being implemented. A forward interface cannot be changed into a class.
parser_n_ignore_lower_visibility=03250_N_Virtual method "$1" has a lower visibility ($2) than parent class $3 ($4)
% The virtual method overrides an method that is declared with a higher visibility. This might give
% unexpected results. E.g., in case the new visibility is private then a call to ``inherited'' in a
% new child class will call the higher-visible method in a parent class and ignores the private method.
parser_e_field_not_allowed_here=03251_E_Fields cannot appear after a method or property definition, start a new visibility section first
% Once a method or property has been defined in a class or object, you cannot define any fields afterwards
% without starting a new visibility section (such as \var{public}, \var{private}, etc.). The reason is
% that otherwise the source code can appear ambiguous to the compiler, since it is possible to use modifiers
% such as \var{default} and \var{register} also as field names.
parser_e_no_local_para_def=03252_E_Parameters or result types cannot contain local type definitions. Use a separate type definition in a type block.
% In Pascal, types are not considered to be identical simply because they are semantically equivalent.
% Two variables or parameters are only considered to be of the same type if they refer to the
% same type definition.
% As a result, it is not allowed to define new types inside parameter lists, because then it is impossible to
% refer to the same type definition in the procedure headers of the interface and implementation of a unit
% (both procedure headers would define a separate type). Keep in mind that expressions such as
% ``file of byte'' or ``string[50]'' also define a new type.
parser_e_abstract_and_sealed_conflict=03253_E_ABSTRACT and SEALED conflict
% ABSTRACT and SEALED cannot be used together in one declaration
parser_e_sealed_descendant=03254_E_Cannot create a descendant of the sealed class "$1"
% Sealed means that class cannot be derived by another class.
parser_e_sealed_class_cannot_have_abstract_methods=03255_E_SEALED class cannot have an ABSTRACT method
% Sealed means that class cannot be derived. Therefore no one class is able to override an abstract method in a sealed class.
parser_e_only_virtual_methods_final=03256_E_Only virtual methods can be final
% You are declaring a method as final, when it is not declared to be
% virtual.
parser_e_final_can_no_be_overridden=03257_E_Final method cannot be overridden: "$1"
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_multiple_messages=03258_E_Only one message can be used per method.
% It is not possible to associate multiple messages with a single method.
parser_e_invalid_enumerator_identifier=03259_E_Invalid enumerator identifier: "$1"
% Only "MoveNext" and "Current" enumerator identifiers are supported.
parser_e_enumerator_identifier_required=03260_E_Enumerator identifier required
% "MoveNext" or "Current" identifier must follow the \var{enumerator} modifier.
parser_e_enumerator_movenext_is_not_valid=03261_E_Enumerator MoveNext pattern method is not valid. Method must be a function with the Boolean return type and no required arguments.
% "MoveNext" enumerator pattern method must be a function with Boolean return type and no required arguments
parser_e_enumerator_current_is_not_valid=03262_E_Enumerator Current pattern property is not valid. Property must have a getter.
% "Current" enumerator pattern property must have a getter
parser_e_only_one_enumerator_movenext=03263_E_Only one enumerator MoveNext method is allowed per class/object
% Class or Object can have only one enumerator MoveNext declaration.
parser_e_only_one_enumerator_current=03264_E_Only one enumerator Current property is allowed per class/object
% Class or Object can have only one enumerator Current declaration.
parser_e_for_in_loop_cannot_be_used_for_the_type=03265_E_For in loop cannot be used for the type "$1"
% For in loop can be used not for all types. For example it cannot be used for the enumerations with jumps.
parser_e_objc_requires_msgstr=03266_E_Objective-C messages require their Objective-C selector name to be specified using the "message" directive.
% Objective-C messages require their Objective-C name (selector name) to be specified using the \var{message `someName:'} procedure directive.
% While bindings to other languages automatically generate such names based on the identifier you use (by replacing
% all underscores with colons), this is unsafe since nothing prevents an Objective-C method name to contain actual
% colons.
parser_e_objc_no_constructor_destructor=03267_E_Objective-C does not have formal constructors nor destructors. Use the alloc, initXXX and dealloc messages.
% The Objective-C language does not have any constructors or destructors. While there are some messages with a similar
% purpose (such as \var{init} and \var{dealloc}), these cannot be identified using automatic parsers and do not
% guarantee anything like Pascal constructors/destructors (e.g., you have to take care of only calling ``designated''
% inherited ``constructors''). For these reasons, we have opted to follow the standard Objective-C patterns for
% instance creation/destruction.
parser_e_message_string_too_long=03268_E_Message name is too long (max. 255 characters)
% Due to compiler implementation reasons, message names are currently limited to 255 characters.
parser_e_objc_message_name_too_long=03269_E_Objective-C message symbol name for "$1" is too long
% Due to compiler implementation reasons, mangled message names (i.e., the symbol names used in the assembler
% code) are currently limited to 255 characters.
parser_h_no_objc_parent=03270_H_Defining a new Objective-C root class. To derive from another root class (e.g., NSObject), specify it as the parent class.
% If no parent class is specified for an Object Pascal class, then it automatically derives from TObject.
% Objective-C classes however do not automatically derive from NSObject, because one can have multiple
% root classes in Objective-C. For example, in the Cocoa framework both NSObject and NSProxy are root classes.
% Therefore, you have to explicitly define a parent class (such as NSObject) if you want to derive your
% Objective-C class from it.
parser_e_no_objc_published=03271_E_Objective-C classes cannot have published sections.
% In Object Pascal, ``published'' determines whether or not RTTI is generated. Since the Objective-C runtime always needs
% RTTI for everything, this specified does not make sense for Objective-C classes.
parser_f_need_objc=03272_F_This module requires an Objective-C mode switch to be compiled
% This error indicates the use of Objective-C language features without an Objective-C mode switch
% active. Enable one via the -M command line switch, or the {\$modeswitch x} directive.
parser_e_must_use_override=03273_E_Inherited methods can only be overridden in Objective-C and Java, add "override" (inherited method defined in $1)
parser_h_should_use_override=03274_H_Inherited methods can only be overridden in Objective-C and Java, add "override" (inherited method defined in $1).
% It is not possible to \var{reintroduce} methods in Objective-C or Java like in Object Pascal. Methods with the same
% name always map to the same virtual method entry. In order to make this clear in the source code,
% the compiler always requires the \var{override} directive to be specified when implementing overriding
% Objective-C or Java methods in Pascal. If the implementation is external, this rule is relaxed because Objective-C and Java
% do not have any \var{override}-style keyword (since it's the default and only behaviour in these languages),
% which makes it hard for automated header conversion tools to include it everywhere.
% The type in which the inherited method is defined is explicitly mentioned, because this may either
% be an objcclass or an objccategory in case of Objective-C.
parser_e_objc_message_name_changed=03275_E_Message name "$1" in inherited class is different from message name "$2" in current class.
% An overriding Objective-C method cannot have a different message name than an inherited method. The reason
% is that these message names uniquely define the message to the Objective-C runtime, which means that
% giving them a different message name breaks the ``override'' semantics.
parser_e_unique_unsupported=03276_E_It is not yet possible to make unique copies of Objective-C or Java types
% Duplicating an Objective-C or Java type using \var{type x = type y;} is not yet supported. You may be able to
% obtain the desired effect using \var{type x = objcclass(y) end;} resp.{} \var{type x = class(y) end;} instead.
parser_e_no_category_as_types=03277_E_Objective-C categories and Object Pascal class helpers cannot be used as types
% It is not possible to declare a variable as an instance of an Objective-C
% category or an Object Pascal class helper. A category/class helper adds
% methods to the scope of an existing class, but does not define a type by
% itself. An exception of this rule is when inheriting an Object Pascal class
% helper from another class helper.
parser_e_no_category_override=03278_E_Categories do not override, but replace methods. Use "reintroduce" instead.
parser_e_must_use_reintroduce_objc=03279_E_Replaced methods can only be reintroduced in Objective-C, add "reintroduce" (replaced method defined in $1).
parser_h_should_use_reintroduce_objc=03280_H_Replaced methods can only be reintroduced in Objective-C, add "reintroduce" (replaced method defined in $1).
% A category replaces an existing method in an Objective-C class, rather than that it overrides it.
% Calling an inherited method from an category method will call that method in
% the extended class' parent, not in the extended class itself. The
% replaced method in the original class is basically lost, and can no longer be
% called or referred to. This behaviour corresponds somewhat more closely to
% \var{reintroduce} than to \var{override} (although in case of \var{reintroduce}
% in Object Pascal, hidden methods are still reachable via inherited).
% The type in which the inherited method is defined is explicitly mentioned, because this may either
% be an objcclass or an objccategory.
parser_e_implements_getter_not_default_cc=03281_E_Getter for implements interface must use the target's default calling convention.
% Interface getters are called via a helper in the run time library, and hence
% have to use the default calling convention for the target (\var{register} on
% i386 and x86\_64, \var{stdcall} on other architectures).
parser_e_no_refcounted_typed_file=03282_E_Typed files cannot contain reference-counted types.
% The data in a typed file cannot be of a reference counted type (such as
% \var{ansistring} or a record containing a field that is reference counted).
parser_e_operator_not_overloaded_2=03283_E_Operator is not overloaded: $2 "$1"
% You are trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_operator_not_overloaded_3=03284_E_Operator is not overloaded: "$1" $2 "$3"
% You are trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_more_array_elements_expected=03285_E_Expected another $1 array elements
% When declaring a typed constant array, you provided to few elements to initialize the array
parser_e_string_const_too_long=03286_E_String constant too long while ansistrings are disabled
% Only when a piece of code is compiled with ansistrings enabled (\var{\{\$H+\}}), string constants
% longer than 255 characters are allowed.
parser_e_invalid_univ_para=03287_E_Type cannot be used as univ parameter because its size is unknown at compile time: "$1"
% \var{univ} parameters are compatible with all values of the same size, but this
% cannot be checked in case a parameter's size is unknown at compile time.
parser_e_only_one_class_constructor_allowed=03288_E_Only one class constructor can be declared in class: "$1"
% You are trying to declare more than one class constructor but only one class constructor can be declared.
parser_e_only_one_class_destructor_allowed=03289_E_Only one class destructor can be declared in class: "$1"
% You are trying to declare more than one class destructor but only one class destructor can be declared.
parser_e_no_paras_for_class_constructor=03290_E_Class constructors cannot have parameters
% You are declaring a class constructor with a parameter list. Class constructor methods
% cannot have parameters.
parser_e_no_paras_for_class_destructor=03291_E_Class destructors cannot have parameters
% You are declaring a class destructor with a parameter list. Class destructor methods
% cannot have parameters.
parser_f_modeswitch_objc_required=03292_F_This construct requires the \{\$modeswitch objectivec1\} mode switch to be active
% Objective-Pascal constructs are not supported when \{\$modeswitch ObjectiveC1\}
% is not active.
parser_e_widestring_to_ansi_compile_time=03293_E_Unicodechar/string constants cannot be converted to ansi/shortstring at compile-time
% It is not possible to use unicodechar and unicodestring constants in
% constant expressions that have to be converted into an ansistring or shortstring
% at compile time, for example inside typed constants. The reason is that the
% compiler cannot know what the actual ansi encoding will be at run time.
parser_e_objc_enumerator_2_0=03294_E_For-in Objective-Pascal loops require \{\$modeswitch ObjectiveC2\} to be active
% Objective-C ``fast enumeration'' support was added in Objective-C 2.0, and
% hence the appropriate modeswitch has to be activated to expose this feature.
% Note that Objective-C 2.0 programs require Mac OS X 10.5 or later.
parser_e_objc_missing_enumeration_defs=03295_E_The compiler cannot find the NSFastEnumerationProtocol or NSFastEnumerationState type in the CocoaAll unit
% Objective-C for-in loops (fast enumeration) require that the compiler can
% find a unit called CocoaAll that contains definitions for the
% NSFastEnumerationProtocol and NSFastEnumerationState types. If you get this
% error, most likely the compiler is finding and loading an alternate CocoaAll
% unit.
parser_e_no_procvarnested_const=03296_E_Typed constants of the type 'procedure is nested' can only be initialized with NIL and global procedures/functions
% A nested procedural variable consists of two components: the address of the
% procedure/function to call (which is always known at compile time), and also
% a parent frame pointer (which is never known at compile time) in case the
% procedural variable contains a reference to a nested procedure/function.
% Therefore such typed constants can only be initialized with global
% functions/procedures since these do not require a parent frame pointer.
parser_f_no_generic_inside_generic=03297_F_Declaration of generic class inside another generic class is not allowed
% At the moment, scanner supports recording of only one token buffer at the time
% (guarded by internal error 200511173 in tscannerfile.startrecordtokens).
% Since generics are implemented by recording tokens, it is not possible to
% have declaration of generic class inside another generic class.
parser_e_forward_intf_declaration_must_be_resolved=03298_E_Forward declaration "$1" must be resolved before a class can conform to or implement it
% An Objective-C protocol or Java Interface must be fully defined before classes can conform to it.
% This error occurs in the following situation (example for Objective-C, but the same goes for Java interfaces):
% \begin{verbatim}
%  Type MyProtocol = objcprotoocl;
%       ChildClass = Class(NSObject,MyProtocol)
%         ...
%       end;
% \end{verbatim}
% where \var{MyProtocol} is declared but not defined.
parser_e_no_record_published=03299_E_Record types cannot have published sections
% Published sections can be used only inside classes.
parser_e_no_destructor_in_records=03300_E_Destructors are not allowed in records or helpers
% Destructor declarations are not allowed in records or helpers.
parser_e_class_methods_only_static_in_records=03301_E_Class methods must be static in records
% Class methods declarations are not allowed in records without static modifier.
% Records have no inheritance and therefore non static class methods have no sense for them.
parser_e_no_parameterless_constructor_in_records=03302_E_Parameterless constructors are not allowed in records or record/type helpers
% Constructor declarations with no arguments are not allowed in records or record/type helpers.
parser_e_at_least_one_argument_must_be_of_type=03303_E_Either the result or at least one parameter must be of type "$1"
% It is required that either the result of the routine or at least one of its parameters be of the specified type.
% For example class operators either take an instance of the structured type in which they are defined, or they return one.
parser_e_cant_use_type_parameters_here=03304_E_Type parameters may require initialization/finalization - cannot be used in variant records
% Type parameters may be specialized with types which (e.g. \var{ansistring}) need initialization/finalization
% code which is implicitly generated by the compiler.
parser_e_externals_no_section=03305_E_Variables being declared as external cannot be in a custom section
% A section directive is not valid for variables being declared as external.
parser_e_section_no_locals=03306_E_Non-static and non-global variables cannot have a section directive
% A variable placed in a custom section is always statically allocated so it must be either a static or global variable.
parser_e_not_allowed_in_helper=03307_E_"$1" is not allowed in helper types
% Some directives and specifiers like "virtual", "dynamic", "override" are not
% allowed inside helper types in mode ObjFPC (they are ignored in mode Delphi),
% because they have no meaning within helpers. Also "abstract" isn't allowed in
% either mode.
parser_e_no_class_constructor_in_helpers=03308_E_Class constructors are not allowed in helpers
% Class constructor declarations are not allowed in helpers.
parser_e_inherited_not_in_record=03309_E_The use of "inherited" is not allowed in a record
% As records don't support inheritance the use of "inherited" is prohibited for
% these as well as for record helpers (in mode "Delphi" only).
parser_e_no_types_in_local_anonymous_records=03310_E_Type declarations are not allowed in local or anonymous records
% Records with types must be defined globally. Types cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_duplicate_implements_clause=03311_E_Duplicate implements clause for interface "$1"
% A class may delegate an interface using the "implements" clause only to a single property. Delegating it multiple times
% is a error.
parser_e_mapping_no_implements=03312_E_Interface "$1" cannot be delegated by "$2", it already has method resolutions
% Method resolution clause maps a method of an interface to a method of the current class. Therefore the current class
% has to implement the interface directly. Delegation is not possible.
parser_e_implements_no_mapping=03313_E_Interface "$1" cannot have method resolutions, "$2" already delegates it
% Method resolution is only possible for interfaces that are implemented directly, not by delegation.
parser_e_invalid_codepage=03314_E_Invalid codepage
% When declaring a string with a given codepage, the range of valid codepages values is limited
% to 0 to 65535.
parser_e_final_only_const_var=03315_E_Only fields (var-sections) and constants can be final in object types
% A final (class) field must be assigned a single value in the (class) constructor, and cannot
% be overwritten afterwards. A final (typed) constant is read-only.
parser_e_final_only_external=03316_E_Final fields are currently only supported for external classes
% Support for final fields in non-external classes requires a full data flow
% analysis implementation in FPC, which it currently still lacks.
parser_e_no_typed_const=03317_E_Typed constants are not allowed here, only formal constants are
% Java interfaces define a namespace in which formal constant can be defined,
% but since they define no storage it is not possible to define typed constants
% in them (those are more or less the same as initialised class fields).
parser_e_java_no_inherited_constructor=03318_E_Constructors are not automatically inherited in the JVM; explicitly add a constructor that calls the inherited one if you need it
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% For compatibility with external Java code, FPC does the same. If you require access to the same
% constructors in a child class, define them in the child class and call the inherited one from
% there.
parser_d_internal_parser_string=03319_D_Parsing internally generated code: $1
% The compiler sometimes internally constructs Pascal code that is subsequently
% injected into the program. These messages display such code, in order to help
% with debugging errors in them.
parser_e_feature_unsupported_for_vm=03320_E_This language feature is not supported on managed VM targets
% Certain language features are not supported on targets that are managed virtual machines.
parser_e_jvm_invalid_virtual_constructor_call=03321_E_Calling a virtual constructor for the current instance inside another constructor is not possible on the JVM target
% The JVM does not natively support virtual constructor. Unforunately, we are not aware of a way to
% emulate them in a way that makes it possible to support calling virtual constructors
% for the current instance inside another constructor.
parser_e_method_lower_visibility=03322_E_Overriding method "$1" cannot have a lower visibility ($2) than in parent class $3 ($4)
% The JVM does not allow lowering the visibility of an overriding method.
parser_e_nostackframe_without_assembler=03323_E_Procedure/Function declared with call option NOSTACKFRAME but without ASSEMBLER
% nostackframe call modifier is supposed to be used in conjunction with assembler.
parser_e_nostackframe_with_locals=03324_E_Procedure/Function declared with call option NOSTACKFRAME but local stack size is $1
% nostackframe call modifier used without assembler modifier
% might still generate local stack needs.
parser_e_cannot_generate_property_getter_setter=03325_E_Cannot generate property getter/setter $1 because its name clashes with existing identifier $2
% Automatically generated getters/setters cannot have the same name as existing
% identifiers, because this may change the behaviour of existing code.
parser_w_overriding_property_getter_setter=03326_W_Automatically generated property getter/setter $1 overrides the same-named getter/setter in class $2
% Automatically generated property getters/setters on the JVM platform are virtual methods, because
% the JVM does not support non-virtual methods that can be changed in child classes. This means
% that if a child class changes an inherited property definition, the behaviour of that property
% can change compared to native targets since even if a variable is declared as the parent type,
% by calling the virtual method the getter from the child will be used. This is different from
% the behaviour on native targets or when not activating automatically generated setters/getters,
% because in that case only the declared type of a variable influences the property behaviour.
parser_w_case_difference_auto_property_getter_setter_prefix=03327_W_Case mismatch between declared property getter/setter $1 and automatically constructed name $2, not changing declared name
% If a property's specified getter/setter already corresponded to the naming convention specified
% by the automatic getter/setter generation setting except in terms of upper/lowercase, the
% compiler will print a warning because it cannot necessarily change that other declaration itself
% not can it add one using the correct case (it could conflict with the original declaration).
% Manually correct the case of the getter/setter to conform to the desired coding rules.
% \var{TChild} overrides
parser_e_no_consts_in_local_anonymous_records=03328_E_Constants declarations are not allowed in local or anonymous records
% Records with constants must be defined globally. Constants cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_methods_in_local_anonymous_records=03329_E_Method declarations are not allowed in local or anonymous records
% Records with methods must be defined globally. Methods cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_properties_in_local_anonymous_records=03330_E_Property declarations are not allowed in local or anonymous records
% Records with properties must be defined globally. Properties cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_no_class_in_local_anonymous_records=03331_E_Class member declarations are not allowed in local or anonymous records
% Records with class members must be defined globally. Class members cannot be defined inside records which are defined in a
% procedure or function or in anonymous records.
parser_e_not_allowed_in_record=03332_E_Visibility section "$1" not allowed in records
% The visibility sections \var(protected) and \var(strict protected) are only
% useful together with inheritance. Since records do not support that they are
% forbidden.
parser_e_dir_not_allowed=03333_E_Directive "$1" not allowed here
% This directive is not allowed in the given context. E.g. "static"
% is not allowed for instance methods or class operators.
parser_e_no_assembler_in_generic=03334_E_Assembler blocks not allowed inside generics
% The use of assembler blocks/routines is not allowed inside generics.
parser_e_property_only_sgr=03335_E_Properties can be only static, global or inside structured types
% Properties cannot be declared local, only global, using the static
% directive or inside structured types.
parser_e_overloaded_have_same_mangled_name=03336_E_Overloaded routines have the same mangled name
% Some platforms, such as the JVM platform, encode the parameters in the routine name in
% a prescribed way, and this encoding may map different Pascal types to the same encoded
% (a.k.a.\ ``mangled'') name. This error can only be solved by removing or changing the
% conflicting definitions' parameter declarations or routine names.
parser_e_default_value_val_const=03337_E_Default values can only be specified for value, const and constref parameters
% A default parameter value allows you to not specify a value for this parameter
% when calling the routine, and the compiler will instead pass the specified
% default (constant) value. As a result, default values can only be specified
% for parameters that can accept constant values.
parser_w_ptr_type_ignored=03338_W_Pointer type "$1" ignored
% The specified pointer type modifier is ignored, because it is not supported on
% the current platform. This happens, for example, when a far pointer is
% declared on a non-x86 platform.
parser_e_global_generic_references_static=03339_E_Global Generic template references static symtable
% A generic declared in the interface section of a unit must not reference symbols that belong
% solely to the implementation section of that unit.
parser_u_already_compiled=03340_UL_Unit $1 has been already compiled meanwhile.
% This tells you that the recursive reading of the uses clauses triggered already
% a compilation of the current unit, so the current compilation can be aborted.
%
%
%
% \end{description}
%
# Type Checking
#
# 04123 is the last used one
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_Typefejl
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_Inkompatible typer: fandt "$1" forventede "$2"
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
type_e_not_equal_types=04002_E_Typeinkompatabilitet mellem "$1" og "$2"
% The types are not equal
type_e_type_id_expected=04003_E_Typenavn forventet
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_Variabelnavn forventet
% This happens when you pass a constant to a routine (such as \var{Inc} var or \var{Dec})
% when it expects a variable. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_Heltalsudtryk forventet, men "$1" fundet
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_Sandhedsudtryk forventet, men "$1" fundet
% The expression must be a boolean type, it should be return true or
% false.
type_e_ordinal_expr_expected=04007_E_Ordinaludtryk forventet
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_Pointertype forventet, men "$1" fundet
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_Klasse forventet, men "$1" fundet
% The variable of expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_cant_eval_constant_expr=04011_E_Kan ikke evaluere konstantudtryk
% This error can occur when the bounds of an array you declared does
% not evaluate to ordinal constants
type_e_set_element_are_not_comp=04012_E_Setelementer er ikke kompatible
% You are trying to make an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union
type_e_set_operation_unknown=04013_E_Operationen er ikke implementeret for set
% several binary operations are not defined for sets
% like div mod ** (also >= <= for now)
type_w_convert_real_2_comp=04014_W_Automatisk typekonversion fra kommatal til COMP, der er en heltalstype
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_Brug DIV i stedet, for at få et heltalsresultat
% When hints are on, then an integer division with the '/' operator will
% procuce this message, because the result will then be of type real
type_e_strict_var_string_violation=04016_E_Strengtyper passer ikke sammen, på grund af {$V+}-indstilling
% When compiling in \var{\{\$V+\}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_succ() eller pred() på enumerate-typer med tildelte værdier er ikke gyldigt
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use the \var{Succ} or \var{Pred} functions on them.
type_e_cant_read_write_type=04018_E_Kan ikke læse eller skrive variabler af denne type
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that. Only integer types,
% reals, pchars and strings can be read from/written to a text file.
% Booleans can only be written to text files.
type_e_no_readln_writeln_for_typed_file=04019_E_Kan ikke bruge readln og writeln på typede filer
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_Kan ikke bruge read eller write på utypede filer
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_Typekonflikt mellem setelementer
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(dword/qword) returnerer det øvre word/dword
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword/int64/qword}
% which returns the lower/upper word/dword of the argument. TP always uses
% a 16 bit \var{lo/hi} which returns always bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the TP behavior you have
% to type cast the argument to \var{word/integer}
type_e_integer_or_real_expr_expected=04023_E_Heltal eller kommatal forventet
% The first argument to \var{str} must a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_Forkert type "$1" i array-constructor
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_Inkompatible typer for $1. parameter: "$2" fundet men "$3" forventet
% You are trying to pass an invalid type for the specified parameter.
type_e_no_method_and_procedure_not_compatible=04026_E_Metode (variabel) og Procedure (variabel) er ikke kompatible
% You can't assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_Ugyldig konstant givet til intern matematik-funktion
% The constant argument passed to a ln or sqrt function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_Kan ikke beregne adressen af konstanter
% It is not possible to get the address of a constant, because they
% aren't stored in memory, you can try making it a typed constant.
type_e_argument_cant_be_assigned=04029_E_Ugyldig tildeling
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference argument
% Remark: Properties can be only
% used on the left side of an assignment, but they cannot be used as arguments
type_e_cannot_local_proc_to_procvar=04030_E_Kan ikke tildele lokal procedure/funktion til procedure-variabel
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling of local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_Kan ikke tildele værdi til en adresse
% It is not allowed to assign a value to an address of a variable,constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_Kan ikke tildele til konstant-erklæret variabel
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const, to allow
% changing the value make the parameter as a value parameter or a var.
type_e_array_required=04033_E_Array-type krævet
% If you are accessing a variable using an index '[<x>]' then
% the type must be an array. In FPC mode also a pointer is allowed.
type_e_interface_type_expected=04034_E_interface-type forventet, men "$1" fundet
% The compiler expected to encounter an interface type name, but got something else.
% The following code would produce this error:
% \begin{verbatim}
% Type
%   TMyStream = Class(TStream,Integer)
% \end{verbatim}
type_h_mixed_signed_unsigned=04035_H_Mixing signed expressions and longwords gives a 64bit result
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetic
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64-bit arithmetic which is slower than normal
% 32-bit arithmetic. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_Blanding af fortegnsudtryk og fortegnsfri udtryk her kan give værdiområdefejl
% If you use a binary operator (and, or, xor) and one of
% the operands is a longword while the other one is a signed expression, then,
% if range checking is turned on, you may get a range check error because in
% such a case both operands are converted to longword before the operation is
% carried out. You can avoid this by typecasting one operand so it
% matches the resulttype of the other one.
type_e_typecast_wrong_size_for_assignment=04037_E_Typecast har forskellig størrelse ($1 -> $2) i tildeling
% Type casting to a type with a different size is not allowed when the variable is
% used for assigning.
type_e_array_index_enums_with_assign_not_possible=04038_E_Enumerate-typer med tildelinger kan ikke bruges som index i arrays
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use it as index of an array.
type_e_classes_not_related=04039_E_Klasse- eller objekt-typerne "$1" og "$2" er ikke relaterede
%  There is a typecast from one class or object to another while the class/object
%  are not related. This will probably lead to errors
type_w_classes_not_related=04040_W_Klasse- eller objekt-typerne "$1" og "$2" er ikke relaterede
%  There is a typecast from one class or object to another while the class/object
%  are not related. This will probably lead to errors
type_e_class_or_interface_type_expected=04041_E_Klasse eller interface-type forventet, men "$1" fundet
% The compiler expected a class or interface name, but got another type or identifier.
type_e_type_is_not_completly_defined=04042_E_Typen "$1" er ikke fuldt ud defineret
% This error occurs when a type is not complete: i.e. a pointer type which points to
% an undefined type.
type_w_string_too_long=04043_W_Strengkonstant har flere karakterer end længden af en kort streng
% The size of the constant string, which is assigned to a shortstring,
% is longer than the maximum size of the shortstring
type_w_comparison_always_false=04044_W_Comparison might be always false due to range of constant and expression
% There is a comparison between a constant and an expression where the constant is out of the
% valid range of values of the expression. Because of type promotion, the statement will always evaluate to
% false. Explicitly typecast the constant or the expression to the correct range to avoid this warning
% if you think the code is correct.
type_w_comparison_always_true=04045_W_Comparison might be always true due to range of constant and expression
% There is a comparison between a constant and an expression where the constant is out of the
% valid range of values of the expression. Because of type promotion, the statement will always evaluate to
% true. Explicitly typecast the constant or the expression to the correct range to avoid this warning
% if you think the code is correct.
type_w_instance_with_abstract=04046_W_Konstruerer klassen "$1" med abstrakte metoder
% An instance of a class is created which contains non-implemented abstract
% methods. This will probably lead to a runtime error 211 in the code if that
% routine is ever called. All abstract methods should be overridden.
type_h_in_range_check=04047_H_Den venstre operand til IN-operatoren skal være én byte stor
% The left operand of the \var{in} operator is not an ordinal or enumeration which fits
% within 8-bits, this may lead to range check errors. The  \var{in} operator
% currently only supports a left operand which fits within a byte. In the case of
% enumerations, the size of an element of an enumeration can be controlled with
% the \var{\{\$PACKENUM\}} or \var{\{\$Zn\}} switches.
type_w_smaller_possible_range_check=04048_W_Forskel i typestørrelser. Muligt datatab eller værdigrænsefejl
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_h_smaller_possible_range_check=04049_H_Forskel i typestørrelser. Muligt datatab eller værdigrænsefejl
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_e_cant_take_address_of_abstract_method=04050_E_Kan ikke beregne adressen af en abstrakt metode
% An abstract method has no body, so the address of an abstract method can't be taken.
type_e_assignment_not_allowed=04051_E_Assignments to formal parameters and open arrays are not possible
% You are trying to assign a value to a formal (untyped var, const or out)
% parameter, or to an open array.
type_e_constant_expr_expected=04052_E_Konstant udtryk forventet
% The compiler expects an constant expression, but gets a variable expression.
type_e_operator_not_supported_for_types=04053_E_Operationen "$1" ikke understøttet for typerne "$2" og "$3"
% The operation is not allowed for the supplied types
type_e_illegal_type_conversion=04054_E_Ugyldig typekonvertering: "$1" til "$2"
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
type_h_pointer_to_longint_conv_not_portable=04055_H_Konvertering mellem ordinaltyper og pointere virker ikke på alle platforme
% If you typecast a pointer to a longint (or vice-versa), this code will not compile
% on a machine using 64-bit for pointer storage.
type_w_pointer_to_longint_conv_not_portable=04056_W_Konvertering mellem ordinaltyper og pointere virker ikke på alle platforme
% If you typecast a pointer to a ordinal type of a different size (or vice-versa), this can
% cause problems. This is a warning to help finding the 32bit specific code where cardinal/longint is used
% to typecast pointers to ordinals. A solution is to use the ptrint/ptruint types instead.
type_e_cant_choose_overload_function=04057_E_Kan ikke bestemme hvilken overloaded funktion der skal kaldes
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
type_e_illegal_count_var=04058_E_Ugyldig tællevariabel
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
% \end{description}
#
# Symtable
#
# 05055 is the last used one
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedurek and variable names.
% \begin{description}
type_w_double_c_varargs=04059_W_Converting constant real value to double for C variable argument, add explicit typecast to prevent this.
% In C, constant real values are double by default. For this reason, if you
% pass a constant real value to a variable argument part of a C function, FPC
% by default converts this constant to double as well. If you want to prevent
% this from happening, add an explicit typecast around the constant.
type_e_class_or_cominterface_type_expected=04060_E_Class or COM interface type expected, but got "$1"
% Some operators, such as the AS operator, are only applicable to classes or COM interfaces.
type_e_no_const_packed_array=04061_E_Constant packed arrays are not yet supported
% You cannot declare a (bit)packed array as a typed constant.
type_e_got_expected_packed_array=04062_E_Incompatible type for arg no. $1: Got "$2" expected "(Bit)Packed Array"
% The compiler expects a (bit)packed array as the specified parameter.
type_e_got_expected_unpacked_array=04063_E_Incompatible type for arg no. $1: Got "$2" expected "(not packed) Array"
% The compiler expects a regular (i.e., not packed) array as the specified parameter.
type_e_no_packed_inittable=04064_E_Elements of packed arrays cannot be of a type which need to be initialised
% Support for packed arrays of types that need initialization
% (such as ansistrings, or records which contain ansistrings) is not yet implemented.
type_e_no_const_packed_record=04065_E_Constant packed records and objects are not yet supported
% You cannot declare a (bit)packed array as a typed constant at this time.
type_w_untyped_arithmetic_unportable=04066_W_Arithmetic "$1" on untyped pointer is unportable to {$T+}, suggest typecast
% Addition/subtraction from an untyped pointer may work differently in \var{\{\$T+\}}.
% Use a typecast to a typed pointer.
type_e_cant_take_address_of_local_subroutine=04076_E_Can't take address of a subroutine marked as local
% The address of a subroutine marked as local cannot be taken.
type_e_cant_export_local=04077_E_Can't export subroutine marked as local from a unit
% A subroutine marked as local cannot be exported from a unit.
type_e_not_automatable=04078_E_Type is not automatable: "$1"
% Only byte, integer, longint, smallint, currency, single, double, ansistring,
% widestring, tdatetime, variant, olevariant, wordbool and all interfaces are automatable.
type_h_convert_add_operands_to_prevent_overflow=04079_H_Converting the operands to "$1" before doing the add could prevent overflow errors.
% Adding two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the addition.
type_h_convert_sub_operands_to_prevent_overflow=04080_H_Converting the operands to "$1" before doing the subtract could prevent overflow errors.
% Subtracting two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the subtraction.
type_h_convert_mul_operands_to_prevent_overflow=04081_H_Converting the operands to "$1" before doing the multiply could prevent overflow errors.
% Multiplying two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the multiplication.
type_w_pointer_to_signed=04082_W_Converting pointers to signed integers may result in wrong comparison results and range errors, use an unsigned type instead.
% The virtual address space on 32-bit machines runs from \$00000000 to \$ffffffff.
% Many operating systems allow you to allocate memory above \$80000000.
% For example both \windows and \linux allow pointers in the range \$0000000 to \$bfffffff.
% If you convert pointers to signed types, this can cause overflow and range check errors,
% but also \$80000000 < \$7fffffff. This can cause random errors in code like "if p>q".
type_e_interface_has_no_guid=04083_E_Interface type $1 has no valid GUID
% When applying the as-operator to an interface or class, the desired interface (i.e. the right operand of the
% as-operator) must have a valid GUID.
type_e_invalid_objc_selector_name=04084_E_Invalid selector name "$1"
% An Objective-C selector cannot be empty, must be a valid identifier or a single colon,
% and if it contains at least one colon it must also end in one.
type_e_expected_objc_method_but_got=04085_E_Expected Objective-C method, but got $1
% A selector can only be created for Objective-C methods, not for any other kind
% of procedure/function/method.
type_e_expected_objc_method=04086_E_Expected Objective-C method or constant method name
% A selector can only be created for Objective-C methods, either by specifying
% the name using a string constant, or by using an Objective-C method identifier
% that is visible in the current scope.
type_e_no_type_info=04087_E_No type info available for this type
% Type information is not generated for some types, such as enumerations with gaps
% in their value range (this includes enumerations whose lower bound is different
% from zero).
type_e_ordinal_or_string_expr_expected=04088_E_Ordinal or string expression expected
% The expression must be an ordinal or string type.
type_e_string_expr_expected=04089_E_String expression expected
% The expression must be a string type.
type_w_zero_to_nil=04090_W_Converting 0 to NIL
% Use NIL rather than 0 when initialising a pointer.
type_e_protocol_type_expected=04091_E_Objective-C protocol type expected, but got "$1"
% The compiler expected a protocol type name, but found something else.
type_e_objc_type_unsupported=04092_E_The type "$1" is not supported for interaction with the Objective-C and the blocks runtime.
% Objective-C and Blocks make extensive use of run time type information (RTTI). This format
% is defined by the maintainers of the run time and can therefore not be adapted
% to all possible Object Pascal types. In particular, types that depend on
% reference counting by the compiler (such as ansistrings and certain kinds of
% interfaces) cannot be used as fields of Objective-C classes, cannot be
% directly passed to Objective-C methods or Blocks, and cannot be encoded using \var{objc\_encode}.
type_e_class_or_objcclass_type_expected=04093_E_Class or objcclass type expected, but got "$1"
% It is only possible to create class reference types of \var{class} and \var{objcclass}
type_e_objcclass_type_expected=04094_E_Objcclass type expected
% The compiler expected an \var{objcclass} type
type_w_procvar_univ_conflicting_para=04095_W_Coerced univ parameter type in procedural variable may cause crash or memory corruption: $1 to $2
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
type_e_generics_cannot_reference_itself=04096_E_Type parameters of specializations of generics cannot reference the currently specialized type
% Recursive specializations of generics like \var{Type MyType = specialize MyGeneric<MyType>;} are not possible.
type_e_type_parameters_are_not_allowed_here=04097_E_Type parameters are not allowed on non-generic class/record/object procedure or function
% Type parameters are only allowed for methods of generic classes, records or objects
type_e_generic_declaration_does_not_match=04098_E_Generic declaration of "$1" differs from previous declaration
% Generic declaration does not match the previous declaration
type_e_helper_type_expected=04099_E_Helper type expected
% The compiler expected a \var{class helper} type.
type_e_record_type_expected=04100_E_Record type expected
% The compiler expected a \var{record} type.
type_e_class_helper_must_extend_subclass=04101_E_Derived class helper must extend a subclass of "$1" or the class itself
% If a class helper inherits from another class helper the extended class must
% extend either the same class as the parent class helper or a subclass of it
type_e_record_helper_must_extend_same_record=04102_E_Derived record or type helper must extend "$1"
% If a record helper inherits from another record helper it must extend the same
% record that the parent record helper extended.
type_e_procedures_return_no_value=04103_E_Invalid assignment, procedures return no value
% This error occurs when one tries to assign the result of a procedure or destructor call.
% A procedure or destructor returns no value so this is not
% possible.
type_w_implicit_string_cast=04104_W_Implicit string type conversion from "$1" to "$2"
% An implicit type conversion from an ansi string type to an unicode string type is
% encountered. To avoid this warning perform an explicit type conversion.
type_w_implicit_string_cast_loss=04105_W_Implicit string type conversion with potential data loss from "$1" to "$2"
% An implicit type conversion from an unicode string type to an ansi string type is
% encountered. This conversion can lose data since not all unicode characters may be represented in the codepage of
% destination string type.
type_w_explicit_string_cast=04106_-W_Explicit string typecast from "$1" to "$2"
% An explicit typecast from an ansi string type to an unicode string type is
% encountered. This warning is off by default. You can turn it on to see all suspicious string conversions.
type_w_explicit_string_cast_loss=04107_-W_Explicit string typecast with potential data loss from "$1" to "$2"
% An explicit typecast from an unicode string type to an ansi string type is
% encountered. This conversion can lose data since not all unicode characters may be represented in the codepage of
% destination string type. This warning is off by default. You can turn it on to see all the places with lossy string
% conversions.
type_w_unicode_data_loss=04108_W_Unicode constant cast with potential data loss
% Conversion from a WideChar to AnsiChar can lose data since now all unicode characters may be represented in the current
% system codepage
% You can nest function definitions only 31 levels deep.
type_e_range_check_error_bounds=04109_E_range check error while evaluating constants ($1 must be between $2 and $3)
type_w_range_check_error_bounds=04110_W_range check error while evaluating constants ($1 must be between $2 and $3)
% The constants are outside their allowed range.
type_e_type_not_allowed_for_default=04111_E_This type is not supported for the Default() intrinsic
% Some types like for example Text and File Of X are not supported by the Default intrinsic.
type_e_java_class_method_not_static_virtual=04112_E_JVM virtual class methods cannot be static
% Virtual class methods cannot be static when targeting the JVM platform, because
% the self pointer is required for correct dispatching.
type_e_invalid_final_assignment=04113_E_Final (class) fields can only be assigned in their class' (class) constructor
% It is only possible to assign a value to a final (class) field inside a (class) constructor of its owning class.
type_e_no_managed_formal_assign_typecast=04114_E_It is not possible to typecast untyped parameters on managed platforms, simply assign a value to them instead.
% On managed platforms, untyped parameters are translated by the compiler into
% the equivalent of \var{var x: BaseClassType}. Non-class-based types passed to
% such parameters are automatically wrapped (or boxed) in a class, and after the
% call the potentially modified value is assigned back to the original variable.
% On the caller side, changing untyped var/out parameters happens by simply assigning
% values to them (either class-based or primitive ones). On the caller side,
% they will be extracted and if their type does not match the original variable's,
% an exception will be raised.
type_e_no_managed_assign_generic_typecast=04115_E_The assignment side of an expression cannot be typecasted to a supertype on managed platforms
% Managed platforms guarantee type safety at the bytecode level. This means that the virtual machine must be able
% to statically determine that no type-unsafe assignments or operations occur. By assigning a parent class type to a
% variable of a child type by typecasting the assignment side to the parent class type, the type safety would no
% longer be guaranteed and the generated code would fail verification at run time time.
type_w_interface_lower_visibility=04116_-W_The interface method "$1" raises the visibility of "$2" to public when accessed via an interface instance
type_e_interface_lower_visibility=04117_E_The interface method "$1" has a higher visibility (public) than "$2"
% All methods in an interface have always public visibility. That means that if
% an interface method is implemented using a (strict) protected or private method,
% this method is actually publicly accessible via the interface. On the JVM
% target this situation results in an error because the JVM rejects such
% attempts to circumvent the visibility rules. On other targets this is a
% warning that is disabled by default because such situations are common
% practice, but it can be enabled in case you are concerned with keeping your
% code compilable for the JVM target.
type_e_typeof_requires_vmt=04118_E_TYPEOF can only be used on object types with VMT
% Typeof() intrinsic returns pointer to VMT of its argument. It cannot be used on object types that do not have VMT.
type_e_invalid_default_value=04119_E_It is not possible to define a default value for a parameter of type "$1"
% Parameters declared as structured types, such as files, variants, non-dynamic
% arrays and TP-style objects, cannot have a default value.
type_e_type_not_allowed_for_type_helper=04120_E_Type "$1" cannot be extended by a type helper
% Types like procedural variables cannot be extended by type helpers
type_e_procedure_must_be_far=04121_E_Procedure or function must be far in order to allow taking its address: "$1"
% In certain i8086 memory models (medium, large and huge), procedures and functions
% have to be declared 'far' in order to allow their address to be taken.
type_w_instance_abstract_class=04122_W_Creating an instance of abstract class "$1"
% The specified class is declared as \var{abstract} and thus no instance of this class
% should be created. This is merely a warning for Delphi compatibility.
type_e_function_reference_kind=04123_E_Subroutine references cannot be declared as "of object" or "is nested", they can always refer to any kind of subroutine
% Subroutine references can refer to any kind of subroutine and hence do not
% require specialisation for methods or nested subroutines.
type_e_anonymous_function_unsupported=04999_E_Function references are not yet supported, only blocks (add "cdecl;" at the end)
% Remove this error message once Delphi-style anonymous are implemented. It has
% number 4999 so as not to result in a gap in the error message numbering once
% it's removed.
% \end{description}
#
# Symtable
#
# 05095 is the last used one
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Ukendt navn: "$1"
% The compiler doesn't know this symbol. Usually happens when you misspel
% the name of a variable or procedure, or when you forgot to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_Intern fejl i SymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide  an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_Generklæret navn "$1"
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_Navnet er allerede brugt in $1, linie $2
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_Ukendt navn: "$1"
% The identifier encountered has not been declared, or is used outside the
% scope where it is defined.
sym_e_forward_not_resolved=05005_E_Forward-erklæring ikke løst: "$1"
% This can happen in two cases:
% \begin{itemize}
% \item This happens when you declare a function (in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_e_error_in_type_def=05007_E_Fejl i typeerklæring
% There is an error in your definition of a new array type:
% \item One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_forward_type_not_resolved=05009_E_Forward-type ikke erklæret: "$1"
% A symbol was forward defined, but no declaration was encountered.
sym_e_only_static_in_static=05010_E_Kun statiske variabler kan bruges i statiske metoder, eller uden for metoder
% A static method of an object can only access static variables.
sym_e_type_must_be_rec_or_object_or_class=05012_E_Record or object or class type expected
% The variable or expression isn't of the type \var{record} or \var{object} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Instancer af klasser eller objekter med abstrakte metoder er ikke tilladt
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_Label er ikke defineret "$1"
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_Label brugt men ikke defineret "$1"
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_Ugyldig label-erklæring
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO og LABEL er ikke understøttet (brug -Sg)
% You must compile a program which has \var{label}s and \var{goto} statements
% with the  \var{-Sg} switch. By default, \var{label} and \var{goto} aren't
% supported.
sym_e_label_not_found=05018_E_Label ikke fundet
% A \var{goto label} was encountered, but the label isn't declared.
sym_e_id_is_no_label_id=05019_E_Navnet er ikke en label
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_Label er allerede defineret
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_Ugyldig typeerklæring af setelementer
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_Forward-erklæring af klassen "$1" ikke løst
% You declared a class, but you did not implement it.
sym_n_unit_not_used=05023_H_Unit "$1" ikke brugt i "$2"
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_Parameter "$1" ikke brugt
% The identifier was declared (locally or globally) but
% was not used (locally or globally).
sym_n_local_identifier_not_used=05025_N_Lokal variabel "$1" ikke brugt
% You have declared, but not used a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_Værdiparameter "$1" er tildelt, men aldrig brugt
% The identifier was declared (locally or globally)
% set but not used (locally or globally).
sym_n_local_identifier_only_set=05027_N_Lokal variabel "$1" er tildelt, men aldrig brugt
% The variable in a procedure or function
% implementation is declared, set but never used.
sym_h_local_symbol_not_used=05028_H_Lokal $1 "$2" aldrig brugt
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_Private-feltet "$1.$2" aldrig brugt
% The indicated private field is defined, but is never used in the code.
sym_n_private_identifier_only_set=05030_N_Private-feltet "$1.$2" tildelt men aldrig brugt
% The indicated private field is declared and assigned to, but never read.
sym_n_private_method_not_used=05031_N_Private-metoden "$1.$2" aldrig brugt
% The indicated private method is declared but is never used in the code.
sym_e_set_expected=05032_E_Set-type forventet
% The variable or expression is not of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_Funktionsresultat ser ikke ud til at være sat
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_Typen "$1" er ikke korrekt opstillet i C-struktur
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_Ukendt feltnavn i struktur, "$1"
% The field doesn't exist in the record/object definition.
sym_w_uninitialized_local_variable=05036_W_Den lokale variabel "$1" er ikke initialiseret
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_w_uninitialized_variable=05037_W_Variablen "$1" er ikke initialiseret
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_e_id_no_member=05038_E_Strukturen har intet felt ved navn "$1"
% This error is generated when an identifier of a record,
% field, or method is accessed while it is not defined.
sym_h_param_list=05039_H_Fandt erklæring: $1
% You get this when you use the \var{-vh} switch. In case an overloaded
% procedure is not found, then all candidate overloaded procedures are
% listed, with their parameter lists.
sym_e_segment_too_large=05040_E_Dataelement for stort
% You get this when you declare a data element whose size exceeds the
% prescribed limit (2 Gb on 80386+/68020+ processors)
sym_e_no_matching_implementation_found=05042_E_Ingen passende implementation for interface-metoden "$1" fundet
% There was no matching method found which could implement the interface
% method. Check argument types and result type of the methods.
sym_w_deprecated_symbol=05043_W_Symbolet "$1" er markeret forældet, og bør ikke anvendes
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Usage of this symbol
% should be avoided as much as possible.
sym_w_non_portable_symbol=05044_W_Symbolet "$1" er ikke portabelt
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{platform} is used. This symbol's value, usage
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_non_implemented_symbol=05055_W_Symbolet "$1" er ikke implementeret
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{unimplemented} is used. This symbol is defined,
% but is not yet implemented on this specific platform.
sym_e_cant_create_unique_type=05056_E_Kan ikke lave unik type fra denne type
% Only simple types like ordinal, float and string types are supported when
% redefining a type with \var{type newtype = type oldtype;}.
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
sym_h_uninitialized_local_variable=05057_H_Den lokale variabel "$1" er ikke initialiseret
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_h_uninitialized_variable=05058_H_Variablen "$1" er ikke initialiseret
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_w_function_result_uninitialized=05059_W_Funktionsresultatet er ikke initialiseret
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. appears in the right-hand-side of an expression)
% before it is initialized (i.e. appeared in the left-hand side of an
% assigment)
sym_h_function_result_uninitialized=05060_H_Funktionsresultatet er ikke initialiseret
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. appears in the right-hand-side of an expression)
% before it is initialized (i.e. appeared in the left-hand side of an
% assigment)
sym_w_identifier_only_read=05061_W_Variablen "$1" bliver læst men aldrig tildelt
% You have read the value of a variable, but nowhere assigned a value to
% it.

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
sym_h_abstract_method_list=05062_H_Found abstract method: $1
% When getting a warning about constructing a class/object with abstract methods
% you get this hint to assist you in finding the affected method.
sym_w_experimental_symbol=05063_W_Symbol "$1" is experimental
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{experimental} is used. Experimental symbols
% might disappear or change semantics in future versions. Usage of this symbol
% should be avoided as much as possible.
sym_w_forward_not_resolved=05064_W_Forward declaration "$1" not resolved, assumed external
% This happens if you declare a function in the \var{interface} of a unit in macpas mode,
% but do not implement it.
sym_w_library_symbol=05065_W_Symbol "$1" is belongs to a library
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{library} is used. Library symbols may not be
% available in other libraries.
sym_w_deprecated_symbol_with_msg=05066_W_Symbol "$1" is deprecated: "$2"
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Use of this symbol
% should be avoided as much as possible.
sym_e_no_enumerator=05067_E_Cannot find an enumerator for the type "$1"
% This means that compiler cannot find an appropriate enumerator to use in the for-in loop.
% To create an enumerator you need to define an operator enumerator or add a public or published
% GetEnumerator method to the class or object definition.
sym_e_no_enumerator_move=05068_E_Cannot find a "MoveNext" method in enumerator "$1"
% This means that compiler cannot find a public MoveNext method with the Boolean return type in
% the enumerator class or object definition.
sym_e_no_enumerator_current=05069_E_Cannot find a "Current" property in enumerator "$1"
% This means that compiler cannot find a public Current property in  the enumerator class or object
% definition.
sym_e_objc_para_mismatch=05070_E_Mismatch between number of declared parameters and number of colons in message string.
% In Objective-C, a message name automatically contains as many colons as parameters.
% In order to prevent mistakes when specifying the message name in FPC, the compiler
% checks whether this is also the case here. Note that in case of messages taking a
% variable number of arguments translated to FPC via an \var{array of const} parameter,
% this final \var{array of const} parameter is not counted. Neither are the hidden
% \var{self} and \var{\_cmd} parameters.
sym_n_private_type_not_used=05071_N_Private type "$1.$2" never used
% The indicated private type is declared but is never used in the code.
sym_n_private_const_not_used=05072_N_Private const "$1.$2" never used
% The indicated private const is declared but is never used in the code.
sym_n_private_property_not_used=05073_N_Private property "$1.$2" never used
% The indicated private property is declared but is never used in the code.
sym_w_deprecated_unit=05074_W_Unit "$1" is deprecated
% This means that a unit which is
% declared as \var{deprecated} is used. Deprecated units may no longer
% be available in newer versions of the library. Use of this unit
% should be avoided as much as possible.
sym_w_deprecated_unit_with_msg=05075_W_Unit "$1" is deprecated: "$2"
% This means that a unit which is
% declared as \var{deprecated} is used. Deprecated units may no longer
% be available in newer versions of the library. Use of this unit
% should be avoided as much as possible.
sym_w_non_portable_unit=05076_W_Unit "$1" is not portable
% This means that a unit which is
% declared as \var{platform} is used. This unit use
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_library_unit=05077_W_Unit "$1" is belongs to a library
% This means that a unit which is
% declared as \var{library} is used. Library units may not be
% available in other libraries.
sym_w_non_implemented_unit=05078_W_Unit "$1" is not implemented
% This means that a unit which is
% declared as \var{unimplemented} is used. This unit is defined,
% but is not yet implemented on this specific platform.
sym_w_experimental_unit=05079_W_Unit "$1" is experimental
% This means that a unit which is
% declared as \var{experimental} is used. Experimental units
% might disappear or change semantics in future versions. Usage of this unit
% should be avoided as much as possible.
sym_e_formal_class_not_resolved=05080_E_No complete definition of the formally declared class "$1" is in scope
% Objecive-C and Java classes can be imported formally, without using the unit in which it is fully declared.
% This enables making forward references to such classes and breaking circular dependencies amongst units.
% However, as soon as you wish to actually do something with an entity of this class type (such as
% access one of its fields, send a message to it, or use it to inherit from), the compiler requires the full definition
% of the class to be in scope.
sym_e_interprocgoto_into_init_final_code_not_allowed=05081_E_Gotos into initialization or finalization blocks of units are not allowed
% Gotos into initialization or finalization blockse of units are not allowed.
sym_e_external_class_name_mismatch1=05082_E_Invalid external name "$1" for formal class "$2"
sym_e_external_class_name_mismatch2=05083_E_Complete class definition with external name "$1" here
% When a class is declared using a formal external definition, the actual external
% definition (if any) must specify the same external name as the formal definition
% (since both definitions refer to the same actual class type).
sym_w_library_overload=05084_W_Possible library conflict: symbol "$1" from library "$2" also found in library "$3"
% Some OS do not have library specific namespaces, for those
% OS, the function declared as "external 'libname' name 'funcname'",
% the 'libname' part is only a hint, funcname might also be loaded
% by another library. This warning appears if 'funcname' is used twice
% with two different library names.
sym_e_duplicate_id_create_java_constructor=05085_E_Cannot add implicit constructor 'Create' because identifier already used by "$1"
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% However, if a class does not explicitly declare at least one constructor, the compiler is
% required to add a public, parameterless constructor. In Java, constructors are nameless,
% but in FPC they are all called ``Create''. Therefore, if you do not add a constructor to
% a Java class and furthermore use the ``Create'' identifier for another entity (e.g., a field,
% or a parameterless method), the compiler cannot satisfy this requirement.
sym_e_no_matching_inherited_parameterless_constructor=05086_E_Cannot generate default constructor for class, because parent has no parameterless constructor
% Java does not automatically add inherited constructors to child classes, so that they can be hidden.
% However, if a class does not explicitly declare at least one constructor, the compiler is
% required to add a public, parameterless constructor. This compiler must then call
% the parameterless constructor from the parent class inside this added constructor.
% This is however impossible if the parent class does not declare such a constructor.
% In this case you must add a valid constructor yourself.
sym_d_adding_helper_for=05087_D_Adding helper for $1
% A helper for the mentioned type is added to the current scope
sym_e_param_list=05088_E_Found declaration: $1
% This message shows all overloaded declarations in case of an error.
sym_w_uninitialized_managed_local_variable=05089_W_Local variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_uninitialized_managed_variable=05090_W_Variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_uninitialized_managed_local_variable=05091_H_Local variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. it did not appear in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_uninitialized_managed_variable=05092_H_Variable "$1" of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. t did not appear in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_managed_function_result_uninitialized=05093_W_function result variable of a managed type does not seem to initialized
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. before it appeared in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_h_managed_function_result_uninitialized=05094_H_Function result variable of a managed type does not seem to be initialized
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. it appears in the left-hand side of an
% assignment). Since the variable is managed, i. e. implicitly initialized by the compiler, this might be intended behaviour and
% does not necessarily mean that the code is wrong.
sym_w_duplicate_id=05095_W_Duplicate identifier "$1"
% The identifier was already declared in an Objective-C category that's in the
% same scope as the current identifier. This is a warning instead of an error,
% because while this hides the identifier from the category, there are often
% many unused categories in scope.
% \end{description}
#
# Codegenerator
#
# 06049 is the last used one
#
% \section{Code generator messages}
% This section lists all messages that can be displayed if the code
% generator encounters an error condition.
% \begin{description}
cg_e_parasize_too_big=06009_E_Den samlede parameterstørrelse overstiger 64KB
% The I386 processor limits the parameter list to 65535 bytes (the \var{RET}
% instruction causes this)
cg_e_file_must_call_by_reference=06012_E_File-typer skal være var-parametre
% You cannot specify files as value parameters, i.e. they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_Brugen af FAR-pointere er ikke tilladt her
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_dont_call_exported_direct=06015_E_EXPORT-erklæringer kan ikke kaldes
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_Mulig ugyldigt kald af constructor eller destructor
% The compiler detected that a constructor or destructor is called within a
% a method. This will probably lead to problems, since constructors / destructors
% require parameters on entry.
cg_n_inefficient_code=06017_N_Utilstrækkelig kode
% Your statement seems dubious to the compiler.
cg_w_unreachable_code=06018_W_Koden vil aldrig blive kørt
% You specified a construct which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_cant_call_abstract_method=06020_E_Abstrakte metoder kan ikke kaldes direkte
% You cannot call an abstract method directy, instead you must call a
% overriding child method, because an abstract method isn't implemented.
cg_d_register_weight=06027_DL_Register $1 vægt $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_d_stackframe_omited=06029_DL_Stakramme er udeladt
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_e_unable_inline_object_methods=06031_E_Objekt eller klassemetode kan ikke være inline
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_Procvar kald kan ikke være inline
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_Koden til inline-procedure ikke gemt
% The compiler couldn't store code for the inline procedure.
cg_e_can_access_element_zero=06035_E_Element nul af en ansi/wide/lang-strengtype kan ikke tilgås. Brug i stedet (set)length
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such a string types
cg_e_cannot_call_cons_dest_inside_with=06037_E_En constructor eller destructor kan ikke kaldes i en "with"-erklæring
% Inside a \var{with} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_Kan ikke kalde beskedhåndteringsprocedure direkte
% A message method handler method cannot be called directly if it contains an
% explicit self argument
cg_e_goto_inout_of_exception_block=06039_E_Jump ind eller ud af en untagelsesblok
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
cg_e_control_flow_outside_finally=06040_E_Forløbs-ændrende erklæringer er ikke tilladt i en finally-blok
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
% \end{description}
cg_w_parasize_too_big=06041_W_Parameterstørrelsen overstiger grænsen for visse CPUer
% This indicates that you are declaring more than 64K of parameters, which
% might not be supported on other processor targets.
cg_w_localsize_too_big=06042_W_Størrelsen af lokale variabler overstiger grænsen for visse CPUer
% This indicates that you are declaring more than 32K of lcoal variables, which
% might not be supported on other processor targets.
cg_e_localsize_too_big=06043_E_Størrelsen af lokale variabler overstiger understøttet størrelse
% This indicates that you are declaring more than 32K of lcoal variables, which
% is not supported by this processor.
cg_e_break_not_allowed=06044_E_BREAK ikke tilladt uden for løkke-erklæringer
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06045_E_CONTINUE ikke tilladt uden for løkke-erklæringer
% You're trying to use \var{continue} outside a loop construction.
# EndOfTeX

#
# Assembler reader
#
# 07097 is the last used one
#
cg_f_unknown_compilerproc=06046_F_Ukendt compilerfunktion "$1". Undersøg om du bruger det korrekt køretidsbibliotek.
% The compiler expects that the runtime library contains some subrountines. If you see this error
% and you didn't mess with the runtime library, it's very likely that the runtime library
% you're using doesn't match the used compiler. If you changed the runtime library this error means
% that you removed a subroutine which the compiler needs for internal use.
% \end{description}
# EndOfTeX

#
# Assembler reader
#
# 07097 is the last used one
#
cg_f_unknown_system_type=06047_F_Cannot find system type "$1". Check if you use the correct run time library.
% The compiler expects that the runtime library contains certain type definitions. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_h_inherited_ignored=06048_H_Inherited call to abstract method ignored
% This message appears only in Delphi mode when you call an abstract method
% of a parent class via \var{inherited;}. The call is then ignored.
cg_e_goto_label_not_found=06049_E_Goto label "$1" not defined or optimized away
% The label used in the goto definition is not defined or optimized away by the
% unreachable code elemination.
cg_f_unknown_type_in_unit=06050_F_Cannot find type "$1" in unit "$2". Check if you use the correct run time library.
% The compiler expects that the runtime library contains certain type definitions. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_e_interprocedural_goto_only_to_outer_scope_allowed=06051_E_Interprocedural gotos are allowed only to outer subroutines
% Gotos between subroutines are only allowed if the goto jumps from an inner to an outer subroutine or
% from a subroutine to the main program
cg_e_labels_cannot_defined_outside_declaration_scope=06052_E_Label must be defined in the same scope as it is declared
% In ISO mode, labels must be defined in the same scope as they are declared.
cg_e_goto_across_procedures_with_exceptions_not_allowed=06053_E_Leaving procedures containing explicit or implicit exceptions frames using goto is not allowed
% Non-local gotos might not be used to leave procedures using exceptions either implicitly or explicitly. Procedures
% which use automated types like ansistrings or class constructurs are affected by this too.
cg_e_mod_only_defined_for_pos_quotient=06054_E_In ISO mode, the mod operator is defined only for positive quotient
% In ISO pascal, only positive values are allowed for the quotient: \var{n mod m} is only valid if \var{m>0}.
cg_d_autoinlining=06055_DL_Auto inlining: $1
% Due to auto inlining turned on, the compiler auto inlines this subroutine.
cg_e_function_not_support_by_selected_instruction_set=06056_E_The function used, is not supported by the selected instruction set: $1
% Some functions cannot be implemented efficiently for certain instruction sets, one example is fused multiply/add.
% To avoid very inefficient code, the compiler complains in this case, so either select another instruction set
% or replace the function call by alternative code
cg_f_max_units_reached=06057_F_Maximum number of units ($1) reached for the current target
% Depending of target architecture, the number of units is limited. This limit
% has been reached. A unit counts only if it contains initialization or finalization count.
%
% \end{description}
# EndOfTeX
#
# Assembler reader
#
# 07125 is the last used one
#
asmr_d_start_reading=07000_DL_Begynder assembler-parsing ($1)
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_DL_Afslutter assembler-parsing ($1)
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_Ikke-label navn indeholder @
% A identifier which isn't a label can't contain a @.
asmr_e_building_record_offset=07004_E_Fejl under opbygning af struktur-offset
% There has an error occured while building the offset of a record/object
% structure, this can happend when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_OFFSET brugt uden navn
% You can only use OFFSET with an identifier. Other syntaxes aren't
% supported
asmr_e_type_without_identifier=07006_E_TYPE brugt uden navn
% You can only use TYPE with an identifier. Other syntaxes aren't
% supported
asmr_e_no_local_or_para_allowed=07007_E_Kan ikke bruge lokale variabler eller parametre her
% You can't use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the frame pointer register so the
% address can't be obtained directly.
asmr_e_need_offset=07008_E_Nødvendigt at bruge OFFSET her
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_Nødvendigt at bruge $ her
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Kan ikke bruge flere samtidige relokérbare symboler
% You can't have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_Relokérbare symboler kan kun adderes
% Relocatable symbols (variable/typed constant) can't be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_Ugyldigt konstantudtryk
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_Relokérbart symbol ikke tilladt her
% You can't use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_Ugyldig referencesyntax
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_Kan ikke tilgå $1 fra denne position
% You can not read directly the value of local or para
% of a higher level in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_Lokale symboler/labels er ikke gyldige referencer
% You can't use local symbols/labels as references
asmr_e_wrong_base_index=07017_E_Ugyldigt base- eller indexregister
% There is an error with the base and index register, they are
% probably incorrect
asmr_w_possible_object_field_bug=07018_W_Mulig fejl i objektfelthåndtering
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_Forkert skaleringsfaktor angivet
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_Mere end ét indexregister angivet
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_Ugyldig operandtype
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_Ugyldig streng som operand: $1
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE og @DATA ikke understøttet
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_Tomme labelreferencer er ikke tilladt
asmr_e_expr_zero_divide=07025_E_Division med nul i assembler-evaluatoren
% There is a division by zero in a constant expression
asmr_e_expr_illegal=07026_E_Ugyldigt konstantudtryk
% There is an illegal expression in a constant expression
asmr_e_escape_seq_ignored=07027_E_Escape-sekvens ignoreret: $1
% There is a C-styled string, but the escape sequence in the string
% is unknown, and is therefore ignored
asmr_e_invalid_symbol_ref=07028_E_Ugyldig symbolreference
asmr_w_fwait_emu_prob=07029_W_FWAIT kan føre til emulationsproblemer med emu387
asmr_w_fadd_to_faddp=07030_W_$1 uden operand oversat til $1P
asmr_w_enter_not_supported_by_linux=07031_W_ENTER-instruktionen er ikke understøttet af Linux-kernen
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_Kald til overloaded funktion i assembler
% There is a call to an overloaded method in the assembler block,
% this might be the sign there is a problem
asmr_e_unsupported_symbol_type=07033_E_Operandens symboltype er ikke understøttet
asmr_e_constant_out_of_bounds=07034_E_Konstantværdi overskrider værdigrænse
asmr_e_error_converting_decimal=07035_E_Fejl under konvertering af decimal $1
% A constant decimal value does not have the correct syntax
asmr_e_error_converting_octal=07036_E_Fejl under konvertering af octal $1
% A constant octal value does not have the correct syntax
asmr_e_error_converting_binary=07037_E_Fejl under konvertering af binær $1
% A constant binary value does not have the correct syntax
asmr_e_error_converting_hexadecimal=07038_E_Fejl under konvertering af hexadecimal $1
% A constant hexadecimal value does not have the correct syntax
asmr_h_direct_global_to_mangled=07039_H_$1 oversat til $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 er associeret med en overloadet funktion
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Kan ikke tilgå SELF uden for en metode
% There is a reference to the \var{self} symbol while it is not
% allowed. \var{self} can only be referenced inside methods
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Kan ikke tilgå OLDEBP uden for en indlejret procedure
% There is a reference to the \var{oldebp} symbol while it is not
% allowed. \var{oldebp} can only be referenced inside nested routines
asmr_e_void_function=07043_W_En procedure kan ikke returnere en værdi i assembler-kode
% Trying to return a value while in a procedure. A procedure
% does not have any return value
asmr_e_SEG_not_supported=07044_E_SEG er ikke understøttet
asmr_e_size_suffix_and_dest_dont_match=07045_E_Størrelses-suffix og destination- eller kildestørrelse stemmer ikke overens
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_w_size_suffix_and_dest_dont_match=07046_W_Størrelses-suffix og destination- eller kildestørrelse stemmer ikke overens
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_e_syntax_error=07047_E_Syntaxfejl i assemblerkode
% There is an assembler syntax error
asmr_e_invalid_opcode_and_operand=07048_E_Ugyldig kombination af operationskode og operand
% The opcode cannot be used with this type of operand
asmr_e_syn_operand=07049_E_Syntaxfejl i assembleroperand
asmr_e_syn_constant=07050_E_Syntaxfejl i assemblerkonstant
asmr_e_invalid_string_expression=07051_E_Ugyldigt strengudtryk
asmr_w_const32bit_for_address=07052_W_adressekonstanten $1 er ikke en gyldig pointer
% A constant expression represents an address which does not fit
% into a pointer. The address is probably incorrect
asmr_e_unknown_opcode=07053_E_Ukendt operationskode $1
% This opcode is not known
asmr_e_invalid_or_missing_opcode=07054_E_Ugyldig eller manglende operationskode
asmr_e_invalid_prefix_and_opcode=07055_E_Ugyldig kombination af prefix og operationskode: $1
asmr_e_invalid_override_and_opcode=07056_E_Ugyldig kombination af override og operationskode: $1
asmr_e_too_many_operands=07057_E_For mange operander på linien
% There are too many operands for this opcode. Check your
% assembler syntax
asmr_w_near_ignored=07058_W_NEAR ignoreret
asmr_w_far_ignored=07059_W_FAR ignoreret
asmr_e_dup_local_sym=07060_E_Gentaget lokalt symbol $1
asmr_e_unknown_local_sym=07061_E_Ukendt lokalt symbol $1
asmr_e_unknown_label_identifier=07062_E_Unknown labelnavn $1
asmr_e_invalid_register=07063_E_Ugyldigt registernavn
% There is an unknown register name used as operand.
asmr_e_invalid_fpu_register=07064_E_Ugyldigt kommatalsregister-navn
% There is an unknown register name used as operand.
asmr_w_modulo_not_supported=07066_W_Modulo ikke understøttet
asmr_e_invalid_float_const=07067_E_Ugyldig kommatalskonstant $1
% The floating point constant declared in an assembler block is
% invalid.
asmr_e_invalid_float_expr=07068_E_Ugyldig kommatalsudtryk
% The floating point expression declared in an assembler block is
% invalid.
asmr_e_wrong_sym_type=07069_E_Forkert symboltype
asmr_e_cannot_index_relative_var=07070_E_Kan ikke indexere en lokal variabel eller parameter med et register
% Trying to index using a base register a symbol which is already relative
% to a register. This is not possible, and will probably lead to crashes.
asmr_e_invalid_seg_override=07071_E_Ugyldigt segment-override-udtryk
asmr_w_id_supposed_external=07072_W_Navnet $1 antages at være eksternt
% There is a reference to an undefined symbol. This will not result
% in an error, since the symbol might be external, but may cause
% problems at link time if the symbol is not defined anywhere.
asmr_e_string_not_allowed_as_const=07073_E_Strenge er ikke tilladte som konstanter
% Character strings are not allowed as constants.
asmr_e_no_var_type_specified=07074_Ingen variabeltype specificeret
% The syntax expects a type idenfitifer after the dot, but
% none was found.
asmr_w_assembler_code_not_returned_to_text=07075_E_Assemblerkode returnerer ikke til tekst-sektionen
% There was a directive in the assembler block to change sections,
% but there is a missing return to the text section at the end
% of the assembler block. This might cause errors during link time.
asmr_e_not_directive_or_local_symbol=07076_E_Ikke et direktiv eller lokalt symbol $1
% This symbol is unknown.
asmr_w_using_defined_as_local=07077_E_Brug af defineret navn som lokal label
asmr_e_dollar_without_identifier=07078_E_Dollar-token brugt uden navn
% A constant expression has an identifier which does not start with
% the $ symbol.
asmr_w_32bit_const_for_address=07079_W_32-bit konstant brugt som adresse
% A constant was used as an address. This is probably an error,
% since using absolute addresses will probably not work.
asmr_n_align_is_target_specific=07080_N_.align er mål-specifikt. Brug  .balign eller .p2align
% Using the .align directive is platform specific, and its meaning will vary
% from one platform to another.
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Kan ikke tilgå felter fra parametre direkte
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_Kan ikke tilgå felter fra objekter/klasser direkte
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
asmr_e_unable_to_determine_reference_size=07083_E_Ingen størrelse angivet, og det var ikke muligt at fastslå størrelsen af operanderne
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference.
asmr_e_cannot_use_RESULT_here=07084_E_Kan ikke bruge RESULT i denne funktion
% Some functions which return complex types cannot use the \var{result}
% keyword.
asmr_w_adding_explicit_args_fXX=07086_W_"$1" uden operand oversat til "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" oversat til "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" oversat til "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_Karakter "<" ikke tilladt her (brug "<<" for logisk skift)
% The shift operator requires the << characters. Only one
% of those characters was found.
asmr_e_invalid_char_greater=07090_E_Karakter ">" ikke tilladt her (brug ">>" for logisk skift)
% The shift operator requires the >> characters. Only one
% of those characters was found.
asmr_w_align_not_supported=07093_W_ALIGN ikke understøttet
asmr_e_no_inc_and_dec_together=07094_E_Inc og Dec kan ikke bruges sammen
% Trying to use an increment and a decrement within the same
% opcode on the 680x0. This is impossible.
asmr_e_invalid_reg_list_in_movem=07095_E_Ugyldig registerliste til movem
% Trying to use the \var{movem} opcode with invalid registers
% to save or restore.
asmr_e_invalid_reg_list_for_opcode=07096_E_Registerliste ikke gyldig for denne operationskode
asmr_e_higher_cpu_mode_required=07097_E_Højere cpu-mode krævet ($1)
% Trying to use an instruction which is not supported in the current
% cpu mode. Use a higher cpu generation to be able to use this
% opcode in your assembler block
asmr_w_unable_to_determine_reference_size_using_dword=07098_W_Ingen størrelse angivet, og det var ikke muligt at fastslå størrelsen af operanderne. Bruger DWORD. 
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use DWORD as default.
#
# Assembler/binary writers
#
# 08018 is the last used one
#
asmr_e_illegal_shifterop_syntax=07099_E_Syntaxfejl under parsning af skifteroperand
% ARM only; ARM assembler supports a so called shifter operand. The used syntax isn't
% a valid shifter operand. Example for an operation with shifter operand:
% \begin{verbatim}
% asm
%   orr     r2,r2,r2,lsl #8
% end;
% \end{verbatim}
#
# Assembler/binary writers
#
# 08018 is the last used one
#
asmr_e_packed_element=07100_E_Address of packed component is not at a byte boundary
% Packed components (record fields and array elements) may start at an arbitrary
% bit inside a byte. On CPU which do not support bit-addressable memory (which
% includes all currently supported CPUs by FPC) you will therefore get an error
% message when trying to index arrays with elements whose size is not a multiple
% of 8 bits. The same goes for accessing record fields with such an address.
% multiple of 8 bits.
asmr_w_unable_to_determine_reference_size_using_byte=07101_W_No size specified and unable to determine the size of the operands, using BYTE as default
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use BYTE as default.
asmr_w_no_direct_ebp_for_parameter=07102_W_Use of +offset(%ebp) for parameters invalid here
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_for_parameter_regcall=07103_W_Use of +offset(%ebp) is not compatible with regcall convention
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_neg_offset=07104_W_Use of -offset(%ebp) is not recommended for local variable access
% Using -8(%ebp) to access a local variable is not recommended
asmr_w_direct_esp_neg_offset=07105_W_Use of -offset(%esp), access may cause a crash or value may be lost
% Using -8(%esp) to access a local stack is not recommended, as
% this stack portion can be overwritten by any function calls or interrupts.
asmr_e_no_vmtoffset_possible=07106_E_VMTOffset must be used in combination with a virtual method, and "$1" is not virtual
% Only virtual methods have VMT offsets
asmr_e_need_pic_ref=07107_E_Generating PIC, but reference is not PIC-safe
% The compiler has been configured to generate position-independent code
% (PIC), but there are position-dependent references in the current
% handwritten assembler instruction.
asmr_e_mixing_regtypes=07108_E_All registers in a register set must be of the same kind and width
% Instructions on the ARM architecture that take a register set as argument require that all registers
% in this set are of the same kind (e.g., integer, vfp) and width (e.g., single precision, double precision).
asmr_e_empty_regset=07109_E_A register set cannot be empty
% Instructions on the ARM architecture that take a register set as argument require that such a set
% contains at least one register.
asmr_w_useless_got_for_local=07110_W_@GOTPCREL is useless and potentially dangerous for local symbols
% The use of @GOTPCREL supposes an extra indirection that is
% not present if the symbol is local, which might lead to wrong assembler code
asmr_w_general_segment_with_constant=07111_W_Constant with general purpose segment register
% General purpose register should not have constant offsets
% as OS memory allocation might not be compatible with that.
asmr_e_bad_seh_directive_offset=07112_E_Invalid offset value for $1
% Win64 SEH directives have certain restrictions on possible offset values, e.g. they should
% be positive and have 3 or 4 low bits clear.
asmr_e_bad_seh_directive_register=07113_E_Invalid register for $1
% Win64 SEH directives accept only 64-bit integer registers or XMM registers.
asmr_e_seh_in_pure_asm_only=07114_E_SEH directives are allowed only in pure assembler procedures
% Win64 SEH directives are allowed only in pure assembler procedures, not in assembler
% blocks of regular procedures.
asmr_e_unsupported_directive=07115_E_Directive "$1" is not supported for the current target
asmr_e_complex_function_result_location=07116_E_This function's result location cannot be encoded directly in a single operand when "nostackframe" is used
% Functions declared with the \var{nostackframe} modifier do not have a stack frame, and hence
% do not have a local variable to hold the temporary function result.  Accesses to the function
% result in such routines directly go to the registers or memory location that will be used
% to return their result to the caller.  In some cases this result may be spread over multiple
% registers, in which case it is not possible to access the location via its symbolic name.
% You have to directly use the appropriate register names in this case.
asmr_e_wrong_gotpcrel_intel_syntax=07117_E_GOTPCREL references in Intel assembler syntax cannot contain a base or index register, and their offset must 0.
% The syntax for a gotpcrel PIC memory expression in Intel assembler syntax is
% \verb*[global_symbol wrt ..gotpcrel]*
asmr_e_no_gotpcrel_support=07118_E_The current target does not support GOTPCREL relocations
% Not all targets support position-independent code using a global offset table.
% Use a different way to access symbols in a position-indepent way in these cases.
asmr_w_global_access_without_got=07119_W_Exported/global symbols should be accessed via the GOT
% Global symbols (symbols from the unit interface, or defined in a program
% or library) should be accessed via the GOT when generating position-indepent code.
asmr_w_check_mem_operand_size=07120_W_Check size of memory operand "$1"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_size3=07121_W_Check size of memory operand "$1: memory-operand-size is $2 bits, but expected [$3 bits]"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_size_offset=07122_W_Check size of memory operand "$1: memory-operand-size is $2 bits, but expected [$3 bits + $4 byte offset]"
% The size of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_negative_offset=07123_W_Check "$1: offset of memory operand is negative "$2 byte"
% The offset of memory operand is possible invalid. This is
% probably an error in the assembler statement
asmr_w_check_mem_operand_automap_multiple_size=07124_W_Check "$1: size of memory operand is empty, but es exists different definitions of the memory size =>> map to $2 (smallest option)"
% The size of memory operand is empty and we have different definitions of possible memory sizes. Check it!
asmr_e_invalid_ref_register=07125_E_Invalid register used in memory reference expression: "$1"
% FPU, vector and sometimes integer registers cannot be used in memory reference
% expressions, due to limitations of the cpu architecture or simple because
% it is not meaningful.

#
# Assembler/binary writers
#
# 08029 is the last used one
#
asmw_f_too_many_asm_files=08000_F_For mange assemblerfiler (forsøg uden smartlinking)
% With smartlinking enabled, there are too many assembler
% files generated. Disable smartlinking.
asmw_f_assembler_output_not_supported=08001_F_Den valgte assemblerform understøttes ikke
asmw_f_comp_not_supported=08002_F_Typen "Comp" er ikke understøttet
asmw_f_direct_not_supported=08003_F_Direkte assembler-tilstand ikke understøttet for indbyggede filskrivere
% Direct assembler mode is not supported for binary writers.
asmw_e_alloc_data_only_in_bss=08004_E_Allokering af data kun tilladt i bss-sektionen
asmw_f_no_binary_writer_selected=08005_F_Ingen binær filskriver valgt
asmw_e_opcode_not_in_table=08006_E_Asm: Operationskode $1 ikke fundet i tabellen
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 ugyldig kombination af register og operationskode
asmw_e_16bit_not_supported=08008_E_Asm: 16-bit referencer ikke understøttet
asmw_e_invalid_effective_address=08009_E_Asm: Ugyldig effektiv adressering
asmw_e_immediate_or_reference_expected=08010_E_Asm: Konstant eller reference forventet
asmw_e_value_exceeds_bounds=08011_E_Asm: $1 værdi overstiger grænse $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: Kort hop er uden for rækkevidde $1
asmw_e_undefined_label=08013_E_Asm: Ukendt labelnavn $1
asmw_e_comp_not_supported=08014_E_Asm: Typen "Comp" er ikke understøttet på dette målsystem
asmw_e_extended_not_supported=08015_E_Asm: Typen "Extended" er ikke understøttet på dette målsystem
asmw_e_duplicate_label=08016_E_Asm: Genbrugt labelnavn $1
asmw_e_redefined_label=08017_E_Asm: Redefineret labelnavn $1
asmw_e_first_defined_label=08018_E_Asm: Først defineret her
asmw_e_invalid_register=08019_E_Asm: Ugyldigt register $1

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
asmw_e_16bit_32bit_not_supported=08020_E_Asm: 16 or 32 Bit references not supported
asmw_e_64bit_not_supported=08021_E_Asm: 64 Bit operands not supported
asmw_e_bad_reg_with_rex=08022_E_Asm: AH,BH,CH or DH cannot be used in an instruction requiring REX prefix
% x86_64 only: instruction encoding of this platform does not allow using
% 8086 high byte registers (AH,BH,CH or DH) together with REX prefix in a single instruction.
% The REX prefix is required whenever the instruction operand size is 64 bits, or
% when it uses one of extended x86_64 registers (R8-R15 or XMM8-XMM15).
asmw_e_missing_endprologue=08023_E_Missing .seh_endprologue directive
% x86_64-win64 only: Normally, SEH directives are handled internally by compiler.
% However, in pure assembler procedures .seh_endprologue directive is required
% if other SEH directives are present.
asmw_e_prologue_too_large=08024_E_Function prologue exceeds 255 bytes
% x86_64-win64: .seh_prologue directive must be placed within 255 bytes from function start.
asmw_e_handlerdata_no_handler=08025_E_.seh_handlerdata directive without preceding .seh_handler
% x86_64-win64: If .seh_handlerdata directive is used, then a .seh_handler directive must be
% present earlier in the same function.
asmw_f_too_many_relocations=08026_F_Relocation count for section $1 exceeds 65535
% Legacy COFF targets limit number of relocations per section to 65535 because they use a 2-byte field
% to store the relocation count. Targets using newer PECOFF format do not have this limitation.
asmw_w_changing_bind_type=08027_N_Change of bind type of symbol $1 from $2 to $3 after use
asmw_h_changing_bind_type=08028_H_Change of bind type of symbol $1 from $2 to $3 after use
% An assembler symbol bind type has been altered after use, which can lead to wrong code.
% First version is reserved for changig to local label, which is the most probable cause
% of wrong code generation, but currently set to Note level as it appears inside
% the compiler compilation.
asmw_e_32bit_not_supported=08029_E_Asm: 32 Bit references not supported
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
exec_w_source_os_redefined=09000_W_Kilde-operativsystem redefineret
% The source operating system is redefined.
exec_i_assembling_pipe=09001_I_Assembler (pipe) $1
% Assembling using a pipe to an external assembler.
exec_d_cant_create_asmfile=09002_E_Kan ikke oprette assemblerfil: $1
% The mentioned file can't be created. Check if you have got
% access permissions to create this file
exec_e_cant_create_objectfile=09003_E_Kan ikke oprette objektfil: $1
% The mentioned file can't be created. Check if you've
% got access permissions to create this file
exec_e_cant_create_archivefile=09004_E_Kan ikke oprette arkivfil: $1
% The mentioned file can't be created. Check if you've
% access permissions to create this file
exec_e_assembler_not_found=09005_E_Assembler $1 ikke fundet, skifter til ekstern assembler
% The assembler program was not found. The compiler will produce a script that
% can be used to assemble and link the program.
exec_t_using_assembler=09006_T_Bruger assembler: $1
% An informational message saying which assembler is being used.
exec_e_error_while_assembling=09007_E_Fejl under assembling (returkode $1)
% There was an error while assembling the file using an external assembler.
% Consult the documentation of the assembler tool to find out more information
% on this error.
exec_e_cant_call_assembler=09008_E_Kan ikke kalde assembleren, fejl $1 under skift til ekstern assembler
% An error occurred when calling an external assembler. The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_assembling=09009_I_Assembler $1
% An informational message stating which file is being assembled.
exec_i_assembling_smart=09010_I_Assembler med smartlinking $1
% An informational message stating which file is being assembled using smartlinking.
exec_w_objfile_not_found=09011_W_Objektet $1 ikke fundet - linkningen kan fejle!
% One of the object file is missing, and linking will probably fail.
% Check your paths.
exec_w_libfile_not_found=09012_W_Biblioteket $1 blev ikke fundet - linkningen kan fejle!
% One of the library file is missing, and linking will probably fail.
% Check your paths.
exec_e_error_while_linking=09013_E_Fejl under linkning
% Generic error while linking.
exec_e_cant_call_linker=09014_E_Kan ikke starte linkeren, skifter til ekstern linker
% An error occurred when calling an external linker. The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_linking=09015_I_Linker $1
% An informational message, showing which program or library is being linked.
exec_e_util_not_found=09016_E_Hjælpeprogram $1 ikke fundet, skifter til ekstern linker
% An external tool was not found. The compiler will produce a script that
% can be used to assemble and link or postprocess the program.
exec_t_using_util=09017_T_Bruger hjælpeprogram $1
% An informational message, showing which external program (usually a postprocessor) is being used.
exec_e_exe_not_supported=09018_E_Programbygning ikke understøttet
% Creating executable programs is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_e_dll_not_supported=09019_E_Dynamisk/Delt bibliotek-bygning ikke understøttet
% Creating dynamically loadable libraries is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_e_static_lib_not_supported=09035_E_Creation of Static Libraries not supported
% Creating static libraries is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_i_closing_script=09020_I_Lukker script $1
% Informational message showing when writing of the external assembling and linking script is finished.
exec_e_res_not_found=09021_E_resourcecompiler ikke fundet, skifter til ekstern
% An external resource compiler was not found. The compiler will produce a script that
% can be used to assemble, compile resources and link or postprocess the program.
exec_i_compilingresource=09022_I_Compilerer resourcen $1
% An informational message, showing which resource is being compiled.
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_unit $1 kan ikke linkes statisk. Skifter til smartlinking
% Static linking was requested, but a unit which is not statically linkable was used.
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_unit $1 kan ikke smartlinkes. Skifter til statisk linking
% Smart linking was requested, but a unit which is not smart-linkable was used.
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_unit $1 kan ikke linkes delt, skifter til statisk linking
% Shared linking was requested, but a unit which is not shared-linkable was used.
exec_e_unit_not_smart_or_static_linkable=09026_E_unit $1 kan ikke linkes smart eller statisk
% Smart or static linking was requested, but a unit which cannot be used for either  was used.
exec_e_unit_not_shared_or_static_linkable=09027_E_unit $1 kan ikke linkes delt eller statisk
%\end{description}
# EndOfTeX

#
# Executable information
#
exec_d_resbin_params=09028_D_Kalder resourcecompiler "$1" med "$2" som kommandolinie
%\end{description}
# EndOfTeX

#
# Executable information
#
exec_e_error_while_compiling_resources=09029_E_Error while compiling resources
% The resource compiler or converter returned an error.
exec_e_cant_call_resource_compiler=09030_E_Can't call the resource compiler "$1", switching to external mode
% An error occurred when calling a resource compiler. The compiler will produce
% a script that can be used to assemble, compile resources and link or
% postprocess the program.
exec_e_cant_open_resource_file=09031_E_Can't open resource file "$1"
% An error occurred resource file cannot be opened.
exec_e_cant_write_resource_file=09032_E_Can't write resource file "$1"
% An error occurred resource file cannot be written.
exec_n_backquote_cat_file_not_found=09033_N_File "$1" not found for backquoted cat command
% The compiler did not find the file that should be expanded into linker parameters
exec_w_init_file_not_found=09034_W_"$1" not found, this will probably cause a linking failure
% The compiler adds certain startup code files to the linker only when they are found.
% If they are not found, they are not added and this might cause a linking failure.
%
%\end{description}
# EndOfTeX

#
# Executable information
#
# 09134 is the last used one
#
# BeginOfTeX
% \section{Executable information messages.}
% This section lists all messages that the compiler emits when an executable program is produced,
% and only when the internal linker is used.
% \begin{description}
execinfo_f_cant_process_executable=09128_F_Can't post process executable $1
% Fatal error when the compiler is unable to post-process an executable.
execinfo_f_cant_open_executable=09129_F_Kan ikke åbne programmet $1
% Fatal error when the compiler cannot open the file for the executable.
execinfo_x_codesize=09130_X_Størrelsen af koden: $1 bytes
% Informational message showing the size of the produced code section.
execinfo_x_initdatasize=09131_X_Størrelse af initialiseret data: $1 bytes
% Informational message showing the size of the initialized data section.
execinfo_x_uninitdatasize=09132_X_Størrelse af uinitialiseret data: $1 bytes
% Informational message showing the size of the uninitialized data section.
execinfo_x_stackreserve=09133_X_Reserveret stakstørrelse: $1 bytes
% Informational message showing the stack size that the compiler reserved for the executable.
execinfo_x_stackcommit=09134_X_Brugt stakstørrelse: $1 bytes

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
link_f_executable_too_big=09200_F_Executable image size is too big for $1 target.
% Fatal error when resulting executable is too big.
link_w_32bit_absolute_reloc=09201_W_Object file "$1" contains 32-bit absolute relocation to symbol "$2".
% Warning when 64-bit object file contains 32-bit absolute relocations.
% In such case an executable image can be loaded into lower 4Gb of
% address space only.
%\end{description}
# EndOfTeX

#
# Unit loading
#
# 10062 is the last used one
#
# BeginOfTeX
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these messages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_Leder efter unit: $1
% When you use the \var{-vt}, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_Indlæser PPU $1
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_PPU Navn: $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_PPU Flag: $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_PPU CRC: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_PPU Tid: $1
% When you use the \var{-vu} flag, the time the unit was compiled is shown.
unit_u_ppu_file_too_short=10006_U_PPU-filen er for kort
% The ppufile is too short, not all declarations are present.
unit_u_ppu_invalid_header=10007_U_PPU-filen har ugyldigt startinformation (ingen PPU i begyndelsen af filen)
% A unit file contains as the first three bytes the ascii codes of \var{PPU}
unit_u_ppu_invalid_version=10008_U_PPU ugyldig version $1
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_PPU er kompileret til en anden processor
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_invalid_target=10010_U_PPU er kompileret til et andet mål
% This unit file was compiled for a different target, and
% cannot be read
unit_u_ppu_source=10011_U_PPU Kilde: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_write=10012_U_Skriver $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_Kan ikke skrive PPU-fil
% An error occurred when writing the unit file.
unit_f_ppu_read_error=10014_F_Fejl under læsning af PPU-fil
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_read_unexpected_end=10015_F_PPU-filen sluttede uventet
% Unexpected end of file. This may mean that the PPU file is
% corrupted.
unit_f_ppu_invalid_entry=10016_F_Ugyldigt element $1 i PPU-fil
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_Problem med størrelsen af Dbx i PPU-filen
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_Ugyldigt unitnavn: $1
% The name of the unit does not match the file name.
unit_f_too_much_units=10019_F_For mange units
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{fmodule.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_Cirkulær unit-reference mellem $1 og $2
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_Kan ikke gencompilere $1, ingen kildekode tilgængelig
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_Kan ikke finde unit'en $1
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your configuration file for the unit paths
unit_w_unit_name_error=10023_W_Unit $1 blev ikke fundet, men $2 findes
% This error message is no longer used.
unit_f_unit_name_error=10024_F_Unit $1 blev eftersøgt, men $2 fundet
% Dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_Compilering af systemunit'en kræver -Us indstillingen
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} must be specified.
unit_f_errors_in_unit=10026_F_Der var $1 fejl under compileringen af modulet, stopper
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_Indlæser fra $1 ($2) unit $3
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_Gencompilerer $1, checksum ændret for $2
% The unit is recompiled because the checksum of a unit it depends on has
% changed.
unit_u_recompile_source_found_alone=10029_U_Gencompilerer $1, kun kilden fundet
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_Gencompilerer unit, statisk bibliotek er ældre end PPU-fil
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit are older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_Gencompilerer unit, delt bibliotek er ældre end PPU-fil
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit are older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_Gencompilerer unit, objekt- og assemblerfil er ældre end PPU-fil
% When you use the \var{-vu} flag, the compiler warns if the assembler or
% object file of the unit are older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_Gencompilerer unit, objekt-filen er ældre end assemblerfilen
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_parsing_interface=10034_U_Parser interfacet af $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit
unit_u_parsing_implementation=10035_U_Parser implementationen af $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit
unit_u_second_load_unit=10036_U_Anden indlæsning for unit $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_check_time=10037_U_PPU Check filen $1 tid $2
% When you use the \var{-vu} flag, the compiler show the filename and
% date and time of the file which a recompile depends on
unit_w_cant_compile_unit_with_changed_incfile=10040_W_Kan ikke gencompilere unit $1, men der blev fundet ændrede includefiler
% A unit was found to have modified include files, but
% some source files were not found, so recompilation is impossible.
unit_u_source_modified=10041_U_File $1 is newer than the one used for creating PPU file $2
% A modified source file for a compiler unit was found.
unit_u_ppu_invalid_fpumode=10042_U_Bruger en unit der der ikke er compileret i den korrekte FPU-mode
% Trying to compile code while using units which were not compiled with
% the same floating point format mode. Either all code should be compiled
% with FPU emulation on, or with FPU emulation off.
unit_u_loading_interface_units=10043_U_Indlæser interface-units fra $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the interface part of the unit.
unit_u_loading_implementation_units=10044_U_Indlæser implementation-units fra $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the implementation part of the unit.
unit_u_interface_crc_changed=10045_U_Interface-CRC ændret for unit $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated for the interface has been changed after the implementation
% has been parsed.
unit_u_implementation_crc_changed=10046_U_Implementation-CRC ændret for unit $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated has been changed after the implementation
% has been parsed.
unit_u_finished_compiling=10047_U_Færdiggjorde compileringen af unit $1
% When you use the \var{-vu} flag, the compiler warns that it
% has finished compiling the unit.
unit_u_add_depend_to=10048_U_Tilføjede afhængighed af $1 til $2
% When you use the \var{-vu} flag, the compiler warns that it
% has added a dependency between the two units.
unit_u_no_reload_is_caller=10049_U_Ingen genindlæsning, $1 er kalder
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is the unit that wants
% to load this unit
unit_u_no_reload_in_second_compile=10050_U_Ingen genindlæsning, allerede anden kompilering: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is already in a second recompile
unit_u_flag_for_reload=10051_U_Indstillinger for genindlæsning: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to reload the unit
unit_u_forced_reload=10052_U_Tvungen genindlæsning
% When you use the \var{-vu} flag, the compiler warns that it
% has is reloading the unit because it was required
unit_u_previous_state=10053_U_Tidligere tilstand af $1: $2
% When you use the \var{-vu} flag, the compiler shows the
% previous state of the unit
unit_u_second_compile_unit=10054_U_Compilerer allerede $1, starter anden compilering
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_loading_unit=10055_U_Indlæser unit $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the unit.
unit_u_finished_loading_unit=10056_U_Indlæsning af unit $1 færdig
% When you use the \var{-vu} flag, the compiler warns that it finished
% loading the unit.
unit_u_registering_new_unit=10057_U_Registrering af unit $1
% When you use the \var{-vu} flag, the compiler warns that it has
% found a new unit and registers it in the internal lists.
unit_u_reresolving_unit=10058_U_Genberegner unit $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to recalculate the internal data of the unit
unit_u_skipping_reresolving_unit=10059_U_Springer genberegning over for unit $1, indlæser stadig brugte units
% When you use the \var{-vu} flag, the compiler warns that it
% skips to recalculate the internal data of the unit because there
% is no data to recalculate
% \end{description}
# EndOfTeX

#
#  Options
#
# 11039 is the last used one
#
unit_u_unload_resunit=10060_U_Unloading resource unit $1 (not needed)
% When you use the \var{-vu} flag, the compiler warns that it is unloading the
% resource handling unit, since no resources are used.
unit_e_different_wpo_file=10061_E_Unit $1 was compiled using a different whole program optimization feedback input ($2, $3); recompile it without wpo or use the same wpo feedback input file for this compilation invocation
% When a unit has been compiled using a particular whole program optimization (wpo) feedback file (\var{-FW<x>} \var{-OW<x>}),
% this compiled version of the unit is specialised for that particular compilation scenario and cannot be used in
% any other context. It has to be recompiled before you can use it in another program or with another wpo feedback input file.
unit_u_indirect_crc_changed=10062_U_Indirect interface (objects/classes) CRC changed for unit $1
% When you use the \var{-vu} flag, the compiler warns that the
% indirect CRC calculated for the unit (this is the CRC of all classes/objects/interfaces/$\ldots$
% in the interfaces of units directly or indirectly used by this unit in the interface) has been changed after the
% implementation has been parsed.
unit_u_ppu_invalid_memory_model=10063_U_PPU is compiled for another i8086 memory model
% This unit file was compiled for a different i8086 memory model and
% cannot be read.
% \end{description}
# EndOfTeX

#
#  Options
#
# 11057 is the last used one
#
option_usage=11000_O_$1 [indstillinger] <inputfil> [indstillinger]
# BeginOfTeX
%
% \section{Command-line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_only_one_source_support=11001_W_Kun én kildefil understøttet
% You can specify only one source file on the command line. The first
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_DEF-fil kan kun laves på OS/2
% This option can only be specified when you're compiling for OS/2
option_no_nested_response_file=11003_E_indlejrede svarfiler ikke understøttet
% you cannot nest response files with the \var{@file} command-line option.
option_no_source_found=11004_F_Ingen kildefil angivet på kommandolinien
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_Ingen indstillinger i konfigurationsfilen $1
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_Ugyldig parameter: $1
% You specified an unknown option.
option_help_pages_para=11007_H_-? udskriver hjælpe-beskeder
% When an unknown option is given, this message is diplayed.
option_too_many_cfg_files=11008_F_for mange indlejrede niveauer af konfigurationsfiler
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_Kan ikke åbne filen $1
% The option file cannot be found.
option_reading_further_from=11010_D_Læser yderligere indstillinger fra $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_Mål er allerede sat til: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_Delte biblioteker er ikke understøttet på DOS-platformen, går tilbage til statisk
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos
option_too_many_ifdef=11013_F_for mange IF(N)DEF
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_for mange ENDIFs
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_åbne betingelser i slutningen af filen
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_generering af debug-information understøttes ikke at dette programformat
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_Prøv at gencompilere med -dGDB
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_E_Du bruger $1 der er en forældet indstilling
% this warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the switch meaning may change.
option_obsolete_switch_use_new=11019_E_Du bruger $1 der er en forældet indstilling. Vær venlig at bruge $2
% this warns you when you use a switch that is not supported anymore. You
% must now use the second switch instead.
% It is recommended that you change the switch to overcome problems in the
% future, when the switch meaning may change.
option_switch_bin_to_src_assembler=11020_N_Skifter til den standard-assembler der skriver direkte til assemblerfilerne
% this notifies you that the assembler has been changed because you used the
% -a switch which can't be used with a binary assembler writer.
option_incompatible_asm=11021_W_Det valgte udformat for assemblerfiler "$1" er ikke kompatibelt med "$2"
option_asm_forced=11022_W_brug af "$1" assembler gennemtvunget
% The assembler output selected can not generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
option_using_file=11026_T_Læser indstillinger fra filen $1
% Options are also read from this file
option_using_env=11027_T_Læser indstillinger fra environment $1
% Options are also read from this environment string
option_handling_option=11028_D_Håndterer indstilling "$1"
% Debug info that an option is found and will be handled
option_help_press_enter=11029__*** tryk enter ***
% Message shown when help is shown page per page. When pressing the ENTER
% Key, the next page of help is shown. If you press q and then ENTER, the
% compiler exits.
option_start_reading_configfile=11030_H_Begynder at læse konfigurationsfil $1
% Starting of config file parsing.
option_end_reading_configfile=11031_H_Er færdig med at læse konfigurationsfil $1
% End of config file parsing.
option_interpreting_option=11032_D_fortolker indstilling "$1"
% The compiler is interpreting an option
option_interpreting_firstpass_option=11036_D_fortolker indstilling til første gennemløb "$1" 
% The compiler is interpreting an option for the first time.
option_interpreting_file_option=11033_D_fortolker filindstilling "$1"
% The compiler is interpreting an option which it read from the configuration file.
option_read_config_file=11034_D_Læser konfigurationsfil "$1"
% The compiler is starting to read the configuration file.
option_found_file=11035_D_fandt kildefilen "$1"
% Additional infos about options, displayed
% when you have debug option turned on.
option_code_page_not_available=11039_E_Ukendt tegnsæt
%\end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_config_is_dir=11040_F_Konfigurationsfilen $1 er et directory
% Directories can not be used as configuration files.
%\end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_confict_asm_debug=11041_W_Assembler output selected "$1" cannot generate debug info, debugging disabled
% The selected assembler output cannot generate
% debugging information, debugging option is therefore disabled.
option_ppc386_deprecated=11042_W_Use of ppc386.cfg is deprecated, please use fpc.cfg instead
% Using ppc386.cfg is still supported for historical reasons, however, for a multiplatform
% system the naming makes no sense anymore. Please continue to use fpc.cfg instead.
option_else_without_if=11043_F_In options file $1 at line $2 #ELSE directive without #IF(N)DEF found
% An \var{\#ELSE} statement was found in the options file without a matching \var{\#IF(N)DEF} statement.
option_unsupported_target=11044_F_Option "$1" is not, or not yet, supported on the current target platform
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is incompatible with the currently selected target platform.
option_unsupported_target_for_feature=11045_F_The feature "$1" is not, or not yet, supported on the selected target platform
% Not all features are supported or implemented for all target platforms. This message informs you that a chosen
% feature is incompatible with the currently selected target platform.
option_dwarf_smart_linking=11046_N_DWARF debug information cannot be used with smart linking on this target, switching to static linking
% Smart linking is currently incompatible with DWARF debug information on most
% platforms, so smart linking is disabled in such cases.
option_ignored_target=11047_W_Option "$1" is ignored for the current target platform.
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is ignored for the currently selected target platform.
option_debug_external_unsupported=11048_W_Disabling external debug information because it is unsupported for the selected target/debug format combination.
% Not all debug formats can be stored in an external file on all platforms. In particular, on
% Mac OS X only DWARF debug information can be stored externally.
option_dwarf_smartlink_creation=11049_N_DWARF debug information cannot be used with smart linking with external assembler, disabling static library creation.
% Smart linking is currently incompatble with DWARF debug information on most
% platforms, so smart linking is disabled in such cases.
option_invalid_macosx_deployment_target=11050_E_Invalid value for MACOSX_DEPLOYMENT_TARGET environment variable: $1
option_invalid_iphoneos_deployment_target=11051_E_Invalid value for IPHONEOS_DEPLOYMENT_TARGET environment variable: $1
% On Mac OS X, the MACOSX\_DEPLOYMENT\_TARGET/IPHONEOS\_DEPLOYMENT\_TARGET environment variable can be
% used to set the default target OS version. In case of Mac OS X, it has to be of the format
% XY.Z or XY.Z.AB with X, Y,Z , A and B all digits from 0-9.
% In case of iOS, it has to be X.Z.A, where X, Z and A can all be either 1 or 2
% digits from 0-9.
option_illegal_fpu_eabihf=11052_E_You must use a FPU type of VFPV2, VFPV3 or VFPV3_D16 when using the EABIHF ABI target
% The EABIHF (VFP hardfloat) ABI target can only be used with VFP FPUs.
option_w_unsupported_debug_format=11053_W_The selected debug format is not supported on the current target, not changing the current setting
% Not all targets support all debug formats (in particular, Stabs is not supported on 64 bit targets).
option_missing_arg=11054_E_argument to "$1" is missing
% Displayed when parameter must be followed by an argument.
option_malformed_para=11055_E_malformed parameter: $1
% Given argument is not valid for parameter.
option_smart_link_requires_external_linker=11056_W_Smart linking requires external linker
option_com_files_require_tiny_model=11057_E_Creating .COM files is not supported in the current memory model. Only the tiny memory model supports making .COM files.


%\end{description}
# EndOfTeX

#
#  Whole program optimization
#
# 12019 is the last used one
#
# BeginOfTeX
%
% \section{Whole program optimization messages}
% This section lists errors that occur when the compiler is performing
% whole program optimization.
% \begin{description}
wpo_cant_find_file=12000_F_Cannot open whole program optimization feedback file "$1"
% The compiler cannot open the specified feedback file with whole program optimization information.
wpo_begin_processing=12001_D_Processing whole program optimization information in wpo feedback file "$1"
% The compiler starts processing whole program optimization information found in the named file.
wpo_end_processing=12002_D_Finished processing the whole program optimization information in wpo feedback file "$1"
% The compiler has finished processing the whole program optimization information found in the named file.
wpo_expected_section=12003_E_Expected section header, but got "$2" at line $1 of wpo feedback file
% The compiler expected a section header in the whole program optimization file (starting with \%),
% but did not find it.
wpo_no_section_handler=12004_W_No handler registered for whole program optimization section "$2" at line $1 of wpo feedback file, ignoring
% The compiler has no handler to deal with the mentioned whole program optimization information
% section, and will therefore ignore it and skip to the next section.
wpo_found_section=12005_D_Found whole program optimization section "$1" with information about "$2"
% The compiler encountered a section with whole program optimization information, and according
% to its handler this section contains information usable for the mentioned purpose.
wpo_no_input_specified=12006_F_The selected whole program optimizations require a previously generated feedback file (use -Fw to specify)
% The compiler needs information gathered during a previous compilation run to perform the selected
% whole program optimizations. You can specify the location of the feedback file containing this
% information using the -Fw switch.
wpo_not_enough_info=12007_E_No collected information necessary to perform "$1" whole program optimization found
% While you pointed the compiler to a file containing whole program optimization feedback, it
% did not contain the information necessary to perform the selected optimizations. You most likely
% have to recompile the program using the appropriate -OWxxx switch.
wpo_no_output_specified=12008_F_Specify a whole program optimization feedback file to store the generated info in (using -FW)
% You have to specify the feedback file in which the compiler has to store the whole program optimization
% feedback that is generated during the compilation run. This can be done using the -FW switch.
wpo_output_without_info_gen=12009_E_Not generating any whole program optimization information, yet a feedback file was specified (using -FW)
% The compiler was instructed to store whole program optimization feedback into a file specified using -FW,
% but not to actually generated any whole program optimization feedback. The classes of to be
% generated information can be specified using -OWxxx.
wpo_input_without_info_use=12010_E_Not performing any whole program optimizations, yet an input feedback file was specified (using -Fw)
% The compiler was not instructed to perform any whole program optimizations (no -Owxxx parameters),
% but nevertheless an input file with such feedback was specified (using -Fwyyy). Since this can
% indicate that you forgot to specify an -Owxxx parameter, the compiler generates an error in this case.
wpo_skipping_unnecessary_section=12011_D_Skipping whole program optimization section "$1", because not needed by the requested optimizations
% The whole program optimization feedback file contains a section with information that is not
% required by the selected whole program optimizations.
wpo_duplicate_wpotype=12012_W_Overriding previously read information for "$1" from feedback input file using information in section "$2"
% The feedback file contains multiple sections that provide the same class of information (e.g.,
% information about which virtual methods can be devirtualized). In this case, the information in last encountered
% section is used. Turn on debugging output (-vd) to see which class of information is provided by each section.
wpo_cannot_extract_live_symbol_info_strip=12013_E_Cannot extract symbol liveness information from program when stripping symbols, use -Xs-
% Certain symbol liveness collectors extract the symbol information from the linked program. If the symbol information
% is stripped (option -Xs), this is not possible.
wpo_cannot_extract_live_symbol_info_no_link=12014_E_Cannot extract symbol liveness information from program when when not linking
% Certain symbol liveness collectors extract the symbol information from the linked program. If the program is not
% linked by the compiler, this is not possible.
wpo_cannot_find_symbol_progs=12015_F_Cannot find "$1" or "$2" to extract symbol liveness information from linked program
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% This helper program is normally 'nm', which is part of the GNU binutils.
wpo_error_reading_symbol_file=12016_E_Error during reading symbol liveness information produced by "$1"
% An error occurred during the reading of the symbol liveness file that was generated using the 'nm' or 'objdump' program. The reason
% can be that it was shorter than expected, or that its format was not understood.
wpo_error_executing_symbol_prog=12017_F_Error executing "$1" (exitcode: $2) to extract symbol information from linked program
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% The helper program produced the reported error code when it was run on the linked program.
wpo_symbol_live_info_needs_smart_linking=12018_E_Collection of symbol liveness information can only help when using smart linking, use -CX -XX
% Whether or not a symbol is live is determined by looking whether it exists in the final linked program.
% Without smart linking/dead code stripping, all symbols are always included, regardless of whether they are
% actually used or not. So in that case all symbols will be seen as live, which makes this optimization ineffective.
wpo_cant_create_feedback_file=12019_E_Cannot create specified whole program optimisation feedback file "$1"
% The compiler is unable to create the file specified using the -FW parameter to store the whole program optimisation information.
%\end{description}
# EndOfTeX


#
# Logo (option -l)
#
option_logo=11023_[
Free Pascal Compiler version $FPCFULLVERSION [$FPCDATE] til $FPCTARGET
Copyright (c) 1993-2020 Florian Klaempfl and others
]

#
# Info (option -i)
#
option_info=11024_[
Free Pascal Compiler version $FPCVERSION

Compilerdato     : $FPCDATE
Compilerplatform : $FPCTARGET

Understøttede platforme:
  $OSTARGETS

Dette program dækkes af reglerne for GNU General Public License
Læs COPYING.v2 for mere information

Indsend fejlbeskrivelser, forslag, etc til
                 http://bugs.freepascal.org

For kommentarer vedrørende den danske oversættelse, skriv til
                 chrivers@iversen-net.dk
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
**0*_sæt + efter en indstilling for at slå den til, - for at slå den fra
**1a_compileren sletter ikke den genererede assemblerfil
**2al_lav en liste over linienummer fra kildekoden i assemblerfilen
**2ar_indsæt information om register(de)allokering i assemblerfilen
**2at_indsæt information om midlertidige allokeringer i assemblerfilen
**1b_generér browser-information
**2bl_generér information om lokale symboler
**1B_byg alle moduler
**1C<x>_Indstillinger for kodegenerering
**2CD_Byg også et dynamisk bibliotek (ikke understøttet)
**2Ce_Compilér med emulerede kommatalsoperationer
**2Cf_Vælg hvilket fpu-instruktionssæt der skal bruges
**2Cg_Generér PIC-kode
**2Ch<n>_Sæt heapstørrelsen til <n> bytes (mellem 1023 og 67107840)
**2Ci_IO-checking
**2Cn_spring link-stadiet over
**2Co_check overflow af heltalsoperationer
**2Cr_værdiområdecheck
**2CR_verificér gyldigheden af objekters metodekald
**2Cs<n>_set stakstørrelsen til <n>
**2Ct_stakchecking
**2CX_byg også et smartlinket bibliotek
**1d<x>_definerer symbolet <x>
*O1D_genererer en DEF-fil
*O2Dd<x>_sætter beskrivelsen til <x>
*O2Dw_PM-applikation
**1e<x>_sæt stien til programmet
**1E_samme som -Cn
**1F<x>_sæt filnavne og stier:
**2FD<x>_sætter mappen hvor der søges efter hjælpeprogrammer til compileren
**2Fe<x>_videresend fejlbeskeder til <x>
**2FE<x>_sæt exe/unit sti til <x>
**2Fi<x>_tilføjer <x> til include-stien
**2Fl<x>_tilføjer <x> til library-stien
*L2FL<x>_bruger <x> som dynamisk linker
**2Fo<x>_tilføjer <x> til object-stien
**2Fr<x>_indlæser beskedfilen <x>
**2Fu<x>_tilføjer <x> til unit-stien
**2FU<x>_sæt unit-output-stien til <x>, overskriver -FE
*g1g_Generér debug-information:
*g2gg_brug gsym
*g2gd_brug dbx
*g2gh_brug heap-sporingsunit (til at finde memory leaks)
*g2gl_brug line-infounit for at vise backtraces
*g2gc_generér checks for pointere
*g2gv_generér programmer der kan spores med valgrind
*g2gw_generér dwarf debug-information
**1i_information:
**2iD_returnerer compilerdato
**2iV_returnerer compilerversion
**2iSO_returnerer compiler-OS
**2iSP_returnerer compiler-processor
**2iTO_returnerer mål-system
**2iTP_returnerer mål-processor
**1I<x>_tilføjer <x> til include-stien
**1k<x>_videregiv <x> til linkeren
**1l_skriv logo under kørsel
**1M<x>_sæt sprogtilstand til <x>
**2Mfpc_free pascal dialekt (standard)
**2Mobjfpc_slå visse udvidelser fra Delphi 2 til
**2Mdelphi_forsøg at være Delphi-kompatibel
**2Mtp_forsøg at være Turbo/Borland Pascal 7.0-kompatibel
**2Mgpc_forsøg at være GNU-Pascal-kompatibel
**2Mmacpas_forsøg at være kompatibel med pascal-dialekter fra macintosh 
**1n_spring læsning af standardkonfigurationsfil over
**1o<x>_ændr navnet af det producerede program til <x>
**1pg_generér profileringskode til gprof (definerer FPC_PROFILE)
*L1P_brug pipes i stedet for midlertidige assembler-filer
**1S<x>_Syntaxindstillinger:
**2S2_samme som -Mobjfpc
**2Sc_understøttet operatorer som i C (*=,+=,/= and -=)
**2Sa_bruger assertion-kode
**2Sd_samme som -Mdelphi
**2Se<x>_compileren stopper efter den <x>te fejl (standard er efter første fejl)
**2Sg_tillad LABEL og GOTO
**2Sh_brug ansistrenge
**2Si_brug C++-lignende INLINE
**2Sm_understøt makroer som I C (globalt)
**2So_samme som -Mtp
**2Sp_samme som -Mgpc
**2Ss_constructor-navn skal være "init" (destructor-navn skal være "done")
**1s_kør hverken assembler eller linker
**2sh_generér script til linkning på værtscomputeren
**2st_generér script til linkning på mål-computeren
**2sr_spring registerallokerings-fasen over (optimeringer vil være slået fra)
**1u<x>_fjerner definitionen af symbolet <x>
**1U_Unit-indstillinger:
**2Un_undersøg ikke om unit-navnet er korrekt
**2Ur_generér release-unit-filer
**2Us_compilér en system-enhed
**1v<x>_Ekstra information. <x> er en kombination af følgende bogstaver:
**2*_e : Vis fejl (standard)         d : Vis debug-information
**2*_w : Vis advarsler               u : Vis unit-information
**2*_n : Vis bemærkninger            t : Vis forsøgte/benyttede filer
**2*_h : Vis hints                   m : Vis definerede makroer
**2*_i : Vis generel information     p : Vis kompilerede procedurer
**2*_l : Vis linienumre              c : Vis betingelser
**2*_a : Vis alt                     0 : Vis intet (bortset fra fejl)
**2*_b : Vis alle procedure-         r : Rhide/GCC kompatabilitetstilstand
**2*_    erklæringer hvis der        x : Programinformation (kun på windows)
**2*_    opstår en fejl
**1V_skriv filen fpcdebug.txt med store mængder debug-information
*L2Xc_link med c-biblioteket
**2Xs_fjern alle symboler fra programfilen
**2XP<x>_sæt <x> foran binutils-filnavnene
**2XD_forsøg at linke dynamisk           (definerer FPC_LINK_DYNAMIC)
**2XS_forsøg at linke statisk (standard) (definerer FPC_LINK_STATIC)
**2XX_forsøg at linke intelligent        (definerer FPC_LINK_SMART)
**0*_Processor-specifikke indstillinger:
3*1A<x>_Udfilformat:
3*2Aas_brug GNU AS som assembler
3*2Anasmcoff_brug Nasm til at lave en coff-fil (Go32v2)
3*2Anasmelf_brug Nasm til at lave en elf32-fil (Linux)
3*2Awasm_brug Wasm til at lave en obj-fil (Watcom)
3*2Anasmobj_brug Nasm til at lave en obj-fil
3*2Amasm_brug Masm til at lave en obj-fil (Microsoft)
3*2Atasm_brug Tasm til at lave en obj-fil (Borland)
3*2Acoff_brug intern filgenerator til at lave coff-fil (Go32v2)
3*2Apecoff_brug intern filgenerator til at lave pecoff-fil (Win32)
3*1R<x>_Læsestil for assembler:
3*2Ratt_Læs assemblerkode i AT&T-stil
3*2Rintel_Læs assemblerkode i Intel-stil
4*2Ratt_Læs assemblerkode i AT&T-stil
4*2Rintel_Læs assemblerkode i Intel-stil
8*2Ratt_Læs assemblerkode i AT&T-stil
8*2Rintel_Læs assemblerkode i Intel-stil
3*2Rdirect_Kopier assemblerkoden direkte ind i assemblerkildefilen
3*1O<x>_Optimeringer:
3*2Og_generér lille kode
3*2OG_generér hurtig kode (standard)
3*2Or_gem visse variabler i registre
3*2Ou_slå usikre optimeringer til (se dokumentation)
3*2O1_niveau-1 optimeringer (hurtige optimeringer)
3*2O2_niveau-2 optimeringer (-O1 + langsommere optimeringer)
3*2O3_niveau-3 optimeringer (-O2 gentaget, højst 5 gange)
3*2Op<x>_Målprocessor:
3*3Op1_sæt målprocessor til 386/486
3*3Op2_sæt målprocessor til Pentium/PentiumMMX (tm)
3*3Op3_sæt målprocessor til PPro/PII/c6x86/K6 (tm)
3*1T<x>_Mål-operativsystem:
3*2Temx_OS/2 via EMX (bruger EMX/RSX extender)
3*2Tgo32v2_Version 2 af DJ Delorie DOS extender
3*2Tlinux_Linux
3*2Tnetware_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2Tos2_OS/2 / eComStation
3*2Twatcom_Watcom kompatibel DOS extender
3*2Twdosx_WDOSX DOS extender
3*2Twin32_Windows 32 Bit
3*1W<x>_Indstillinger for Win32-lignende mål:
3*2WB<x>_sæt image base til <x> (hexadecimal)
3*2WC_specificér konsolprogram
3*2WD_brug DEFFILE til at eksportere funktioner fra DLL eller EXE
3*2WF_specificér grafisk fuldskærmsprogram (kun OS/2)
3*2WG_specificér grafisk programtype
3*2WN_lav ikke-relokérbar kode (nødvendig til debugging)
3*2WR_lav relokérbar kode
6*1A<x>_Udfilformat
6*2Aas_Brug GNU AS til at lave Unix o-filer
6*2Agas_GNU Motorola assembler
6*2Amit_MIT Syntax (old GAS)
6*2Amot_Standard Motorola assembler
6*1O_Optimeringer:
6*2Oa_slå optimeringer til
6*2Og_generér lille kode
6*2OG_generér hurtig kode (standard)
6*2Ox_optimer maksimalt (stadig ikke fejlfri!!)
6*2O0_sæt målprocessor til MC68000
6*2O2_sæt målprocessor til MC68020+ (standard)
6*1R<x>_Læsestil for assembleren:
6*2RMOT_Læs assemblerkode i Motorola-stil
6*1T<x>_Mål-operativsystem:
6*2Tamiga_Commodore Amiga
6*2Tatari_Atari ST/STe/TT
6*2Tlinux_Linux-68k
6*2Tmacos_Macintosh m68k (ikke understøttet)
6*2Tpalmos_PalmOS
P*1T<x>_Mål-operativsystem:
P*2Tdarwin_Darwin og MacOS X på PowerPC
P*2Tlinux_Linux på PowerPC
P*2Tmacos_MacOS (klassisk) på PowerPC
P*2Tmorphos_MorphOS
P*2WC_Vælg konsoleapplikation (MacOS only)
P*2WG_Vælg grafisk applikation (MacOS only)
P*2WT_Vælg værktøjsapplikation (MPW tool, MacOS only)
**1*_
**1?_vis denne hjælp
**1h_vis denne hjælp uden at vente på tastetryk
]

#
# Slut :)
#
%%% general_e_path_does_not_exists=01017_E_Sti "$1" ikke fundet
% The specified path does not exists.
% \end{description}
#
# Scanner
#
# 02061 is the last used one
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compiling handling.
% \begin{description}
%%% scan_w_only_pack_records=02015_W_Strukturfelter kan kun opstilles på 1,2,4,8,16 eller 32 bytes
% You are specifying the \var{\{\$PACKRECORDS n\} } with an illegal value for
% \var{n}. Only 1, 2, 4, 8, 16 and 32 are valid in this case.
%%% scan_w_only_pack_enum=02016_W_Enumerate-typer kan kun gemmes i 1,2 eller 4 bytes
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2 or 4 are valid in this case.
%%% scan_w_macro_deep_ten=02030_W_Makroudvidelse har en dybdegrænse på 16 niveauer
% When expanding a macro, macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
%%% parser_e_no_local_external=03174_E_Kan ikke erklære lokal procedure som EXTERNAL
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance of errors very high
%%% parser_e_too_complex_expr=03202_E_Udtryk for komplekst - stakoverløb på FPU
% Your expression is too long for the compiler. You should try dividing the
% construct over multiple assignments.
%%% sym_n_uninitialized_local_variable=05036_W_Lokal variabel "$1" ser ikke ud til at være initialiseret
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
%%% sym_n_uninitialized_variable=05037_W_Variablen "$1" ser ikke ud til at være initialiseret
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
%%% unit_h_cond_not_set_in_last_compile=10038_H_Betingelsen $1 var slået fra ved sidste compilering af $2
% when recompilation of an unit is required the compiler will check that
% the same conditionals are set for the recompiliation. The compiler has
% found a conditional that currently is defined, but was not used the last
% time the unit was compiled.
%%% unit_h_cond_set_in_last_compile=10039_H_Betingelsen $1 var slået til ved sidste compilering af $2
% when recompilation of an unit is required the compiler will check that
% the same conditionals are set for the recompiliation. The compiler has
% found a conditional that was used the last time the unit was compiled, but
% the conditional is currently not defined.
%%% option_defining_symbol=11037_D_Definerer symbolet $1
%%% option_undefining_symbol=11038_D_Fjerner symbolet $1
% Additional infos about options, displayed
% when you have debug option turned on.
%%% parser_e_only_class_methods_via_class_ref=03053_E_Kun klassemetoder kan refereres med en klassereference
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
%%% parser_e_only_class_methods=03054_E_Kun klassemetoder kan tilgås i klassemetoder
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
%%% parser_e_absolute_only_one_var=03095_E_Absolute kan kun associere én variabel
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
%%% parser_e_only_publishable_classes_can__be_published=03156_E_Kun klasser der er kompileret i med {$M+}-indstillingen kan bruge published
% In the published section of a class can be only class as fields used which
% are compiled in \var{\{\$M+\}} or which are derived from such a class. Normally
% such a class should be derived from TPersitent
%%% type_w_mixed_signed_unsigned=04035_W_Blanding af fortegnsudtryk og fortegnsfri udtryk giver 64-bit resultat
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetic
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64bit which is slower than normal
% 32bit arithmetic. You can avoid this by typecasting one operand so it
% matches the resulttype of the other one.
%%% type_w_signed_unsigned_always_false=04044_W_Sammenligning er altid falsk, grundet værdiernes rækkevidde
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% false. Exlicitly typecast the constant to the correct range to avoid this problem.
%%% type_w_signed_unsigned_always_true=04045_W_Sammenligning er altid sand, grundet værdiernes rækkevidde
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% true. Exlicitly typecast the constant to the correct range to avoid this problem.
%%% type_e_operator_not_allowed=04051_E_Operatoren kan ikke bruges på denne operand-type
% You are trying an operator that is not available for the type of the
% operands
%%% unit_h_source_modified=10041_H_Filen $1 er nyere end PPU-filen $2 (Release-udgave)
% A modified source file for a unit was found that was compiled with the
% release flag (-Ur). The unit will not implicitly be recompiled because
% this release flag is set.
