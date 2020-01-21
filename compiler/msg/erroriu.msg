#
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1999-2018 by the Free Pascal Development team
#
#   Italian Language File for Free Pascal
#   Contributed by Massimo Soricetti <notturno at quipo.it>
#   Translation for FPC 2.4.2, april 16, 2011, Tolentino, Italy
#
#   See the file COPYING.v2, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#

# CodePage 1252

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
#   exec_     calls to assembler, external linker, binder
#   link_     internal linker
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
general_t_compilername=01000_T_Compilatore: $1
% When the \var{-vt} switch is used, this line tells you what compiler
% is used.
general_d_sourceos=01001_D_Sistema operativo del compilatore: $1
% When the \var{-vd} switch is used, this line tells you what the source
% operating system is.
general_i_targetos=01002_I_Sistema operativo di destinazione: $1
% When the \var{-vd} switch is used, this line tells you what the target
% operating system is.
general_t_exepath=01003_T_Path degli eseguibili: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its binaries.
general_t_unitpath=01004_T_Path delle unit: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for compiled units. You can set this path with the \var{-Fu} option.
general_t_includepath=01005_T_Path degli include: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its include files (files used in \var{\{\$I xxx\}} statements).
% You can set this path with the \var{-Fi} option.
general_t_librarypath=01006_T_Path delle library: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for the libraries. You can set this path with the \var{-Fl} option.
general_t_objectpath=01007_T_Path dei file oggetto: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for object files you link in (files used in \var{\{\$L xxx\}} statements).
% You can set this path with the \var{-Fo} option.
general_i_abslines_compiled=01008_I_$1 righe compilate, $2 sec $3 min
% When the \var{-vi} switch is used, the compiler reports the number
% of lines compiled, and the time it took to compile them (real time,
% not program time).
general_f_no_memory_left=01009_F_Memoria esaurita
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
general_i_writingresourcefile=01010_I_Scrivo il file di risorse della stringtable: $1
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_Scrivo il file di risorse della stringtable: $1
% This message is shown when the compiler encounters an error when writing
% the Resource String Table file.
#
#
# La finestra dei messaggi di Lazarus filtra via tutto quello che non comincia
# con uno di questi cinque prefissi in inglese (o meglio Lazarus versione 0.9.30 lo fa) 
# per cui, almeno per ora, ce li dobbiamo tenere :-(
# Fanno eccezione 'Fatale' e 'Errore' perché sono identici alle versioni inglesi, ma più lunghi.
#
#
general_i_fatal=01012_I_Fatale:
% Prefix for Fatal Errors.
general_i_error=01013_I_Errore:
% Prefix for Errors.
general_i_warning=01014_I_Warning:
% Prefix for Warnings.
general_i_note=01015_I_Note:
% Prefix for Notes.
general_i_hint=01016_I_Hint:
% Prefix for Hints.
general_e_path_does_not_exist=01017_E_Il path "$1" non esiste
% The specified path does not exist.
general_f_compilation_aborted=01018_F_Compilazione abortita
% Compilation was aborted.
general_text_bytes_code=01019_I_$1 byte di codice
% The size of the generated executable code, in bytes.
general_text_bytes_data=01020_I_$1 byte di dati
% The size of the generated program data, in bytes.
general_i_number_of_warnings=01021_I_$1 attenzioni date
% Total number of warnings issued during compilation.
general_i_number_of_hints=01022_I_$1 consigli dati
% Total number of hints issued during compilation.
general_i_number_of_notes=01023_I_$1 note date
% Total number of notes issued during compilation.
% \end{description}
#
# Scanner
#
# 02086 is the last used one
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compilation handling.
% \begin{description}
scan_f_end_of_file=02000_F_Il file è terminato troppo presto
% This typically happens in one of the following cases:
% \begin{itemize}
% \item The source file ends before the final \var{end.} statement. This
% happens mostly when the \var{begin} and \var{end} statements aren't
% balanced;
% \item An include file ends in the middle of a statement.
% \item A comment was not closed.
% \end{itemize}
scan_f_string_exceeds_line=02001_F_La stringa continua su più righe
% There is a missing closing ' in a string, so it occupies
% multiple lines.
scan_f_illegal_char=02002_F_il carattere "$1" ($2) non è ammesso
% An illegal character was encountered in the input file.
scan_f_syn_expected=02003_F_Errore di sintassi: era atteso "$1" ma non "$2"
% This indicates that the compiler expected a different token than
% the one you typed. It can occur almost anywhere it is possible to make an error
% against the Pascal language.
scan_t_start_include_file=02004_TL_Inizio lettura del file include $1
% When you provide the \var{-vt} switch, the compiler tells you
% when it starts reading an included file.
scan_w_comment_level=02005_W_Trovato commento di livello $1
% When the \var{-vw} switch is used, then the compiler warns you if
% it finds nested comments. Nested comments are not allowed in Turbo Pascal
% and Delphi, and can be a possible source of errors.
scan_n_ignored_switch=02008_N_Ignorato lo switch "$1" del compilatore
% With \var{-vn} on, the compiler warns if it ignores a switch.
scan_w_illegal_switch=02009_W_Lo switch "$1" del compilatore non è ammesso
% You included a compiler switch (i.e. \var{\{\$... \}}) which the compiler
% does not recognise.
scan_w_switch_is_global=02010_W_Switch globale del compilatore fuori posto
% The compiler switch is misplaced, and should be located at
% the start of the unit or program.
scan_e_illegal_char_const=02011_E_Costante carattere illegale
% This happens when you specify a character with its ASCII code, as in
% \var{\#96}, but the number is either illegal, or out of range.
scan_f_cannot_open_input=02012_F_Impossibile aprire il file "$1"
% \fpc cannot find the program or unit source file you specified on the
% command line.
scan_f_cannot_open_includefile=02013_F_Impossibile aprire il file include "$1"
% \fpc cannot find the source file you specified in a \var{\{\$include ..\}}
% statement.
scan_e_illegal_pack_records=02015_E_L'allineamento dei record a "$1" è illegale
% You are specifying \var{\{\$PACKRECORDS n\} } or \var{\{\$ALIGN n\} }
% with an illegal value for \var{n}. For \$PACKRECORDS valid alignments are 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, and for \$ALIGN valid alignments are 1, 2, 4, 8, 16, 32, ON,
% OFF. Under mode MacPas \$ALIGN also supports MAC68K, POWER and RESET.
scan_e_illegal_pack_enum=02016_E_La dimensione minima "$1" per gli enum è illegale
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2,4, NORMAL or DEFAULT is valid here.
scan_e_endif_expected=02017_E_$ENDIF previsto per $1 $2 definito in $3 riga $4
% Your conditional compilation statements are unbalanced.
scan_e_preproc_syntax_error=02018_E_Errore di sintassi nel parsing di una espressione di compilazione condizionale
% There is an error in the expression following the \var{\{\$if ..\}}, \var{\{\$ifc \}}
% or \var{\{\$setc \}} compiler directives.
scan_e_error_in_preproc_expr=02019_E_Espressione di compilazione condizionale errata
% There is an error in the expression following the \var{\{\$if ..\}}, $ifc or $setc compiler
% directives.
scan_w_macro_cut_after_255_chars=02020_W_Le macro sono limitate a 255 caratteri di lunghezza
% The contents of macros cannot be longer than 255 characters.
scan_e_endif_without_if=02021_E_ENDIF senza IF(N)DEF
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements aren't balanced.
scan_f_user_defined=02022_F_Definito dall'utente: $1
% A user defined fatal error occurred. See also the \progref.
scan_e_user_defined=02023_E_Definito dall'utente: $1
% A user defined error occurred. See also the \progref.
scan_w_user_defined=02024_W_Definito dall'utente: $1
% A user defined warning occurred. See also the \progref.
scan_n_user_defined=02025_N_Definito dall'utente: $1
% A user defined note was encountered. See also the \progref.
scan_h_user_defined=02026_H_Definito dall'utente: $1
% A user defined hint was encountered. See also the \progref.
scan_i_user_defined=02027_I_Definito dall'utente: $1
% User defined information was encountered. See also the \progref.
scan_e_keyword_cant_be_a_macro=02028_E_Ridefinire parole chiave nelle macro non ha alcun effetto
% You cannot redefine keywords with macros.
scan_f_macro_buffer_overflow=02029_F_Buffer overflow di una macro durante la lettura o l'espansione
% Your macro or its result was too long for the compiler.
scan_w_macro_too_deep=02030_W_L'espansione della macro ha superato profondità 16
% When expanding a macro, macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_w_wrong_styled_switch=02031_W_Gli switch del compilatore non funzionano nei commenti che iniziano con // 
% Compiler switches should be in normal Pascal style comments.
scan_d_handling_switch=02032_DL_Valutazione dello switch "$1"
% When you set debugging info on (\var{-vd}) the compiler tells you when it
% is evaluating conditional compile statements.
scan_c_endif_found=02033_CL_ENDIF $1 trovato
% When you turn on conditional messages (\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifdef_found=02034_CL_IFDEF $1 trovato, $2
% When you turn on conditional messages (\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifopt_found=02035_CL_IFOPT $1 trovato, $2
% When you turn on conditional messages (\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_if_found=02036_CL_IF $1 trovato, $2
% When you turn on conditional messages (\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifndef_found=02037_CL_IFNDEF $1 trovato, $2
% When you turn on conditional messages (\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_else_found=02038_CL_ELSE $1 trovato, $2
% When you turn on conditional messages (\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_skipping_until=02039_CL_Sarà ignorato tutto fino a...
% When you turn on conditional messages (\var{-vc}), the compiler tells you
% where it encounters conditional statements, and whether it is skipping or
% compiling parts.
scan_i_press_enter=02040_I_Premere <Invio> per continuare
% When the \var{-vi} switch is used, the compiler stops compilation
% and waits for the \var{Enter} key to be pressed when it encounters
% a \var{\{\$STOP\}} directive.
scan_w_unsupported_switch=02041_W_Switch "$1" non supportato
% When warnings are turned on (\var{-vw}), the compiler warns you about
% unsupported switches. This means that the switch is used in Delphi or
% Turbo Pascal, but not in \fpc.
scan_w_illegal_directive=02042_W_La direttiva "$1" del compilatore è illegale
% When warnings are turned on (\var{-vw}), the compiler warns you about
% unrecognised switches. For a list of recognised switches, see the \progref.
scan_t_back_in=02043_TL_Torno in $1
% When you use the \var{-vt} switch, the compiler tells you when it has finished
% reading an include file.
scan_w_unsupported_app_type=02044_W_Tipo di programma "$1" non supportato
% You get this warning if you specify an unknown application type
% with the directive \var{\{\$APPTYPE\}}.
scan_w_app_type_not_support=02045_W_APPTYPE non è supportato dall'OS destinazione
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_w_description_not_support=02046_W_DESCRIPTION non è supportata dall'OS di destinazione
% The \var{\{\$DESCRIPTION\}} direttiva non è supportata da questo OS destinazione.
scan_n_version_not_support=02047_N_VERSION non è supportata dall'OS di destinazione
% The \var{\{\$VERSION\}} direttiva non è supportata da questo OS destinazione.
scan_n_only_exe_version=02048_N_VERSION vale solo per exe o DLL
% The \var{\{\$VERSION\}} direttiva è usata solo per sorgenti eseguibili o DLL.
scan_w_wrong_version_ignored=02049_W_Formato errato per la direttiva VERSION "$1"
% The \var{\{\$VERSION\}} directive format is majorversion.minorversion
% where majorversion and minorversion are words.
scan_e_illegal_asmmode_specifier=02050_E_Lo stile assembler "$1" è illegale
% When you specify an assembler mode with the \var{\{\$ASMMODE xxx\}} directive,
% the compiler didn't recognize the mode you specified.
scan_w_no_asm_reader_switch_inside_asm=02051_W_Lo switch lettore ASM non è ammesso in una istruzione asm: "$1" sarà valido solo per la prossima
% It is not possible to switch from one assembler reader to another
% inside an assembler block. The new reader will be used for next
% assembler statements only.
scan_e_wrong_switch_toggle=02052_E_Interruttore switch errato, usare ON/OFF o +/-
% You need to use ON or OFF or a + or - to toggle the switch.
scan_e_resourcefiles_not_supported=02053_E_I resource file non sono supportati per questa destinazione
% The target you are compiling for doesn't support resource files.
scan_w_include_env_not_found=02054_W_La variabile ambiente "$1" per inclusione non esiste
% The included environment variable can't be found in the environment; it will
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_Valore illegale per limiti registri FPU
% Valid values for this directive are 0..8 and NORMAL/DEFAULT.
scan_w_only_one_resourcefile_supported=02056_W_Questa destinazione supporta solo un file di risorse
% The target you are compiling for supports only one resource file.
% The first resource file found is used, the others are discarded.
scan_w_macro_support_turned_off=02057_W_Il supporto alle macro è disabilitato: macro ignorata 
% A macro declaration has been found, but macro support is currently off,
% so the declaration will be ignored. To turn macro support on compile with
% -Sm on the command line or add \{\$MACRO ON\} in the source.
scan_e_invalid_interface_type=02058_E_Tipo di interfaccia illegale. Sono ammessi COM, CORBA o DEFAULT.
% The interface type that was specified is not supported.
scan_w_appid_not_support=02059_W_APPID è supportata solo da PalmOS
% The \var{\{\$APPID\}} directive is only supported for the PalmOS target.
scan_w_appname_not_support=02060_W_APPNAME è supportata solo da PalmOS
% The \var{\{\$APPNAME\}} directive is only supported for the PalmOS target.
scan_e_string_exceeds_255_chars=02061_E_Le costanti stringa devono essere lunghe non più di 255 caratteri
% A single string constant can contain at most 255 chars. Try splitting up the
% string into multiple smaller parts and concatenate them with a + operator.
scan_f_include_deep_ten=02062_F_La inclusione nidificata di file include ha superato profondità 16.
% When including include files the files have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_too_many_push=02063_F_Troppi livelli di PUSH
% A maximum of 20 levels is allowed. This error occurs only in mode MacPas.
scan_e_too_many_pop=02064_E_Trovata POP senza una PUSH precedente
% This error occurs only in mode MacPas.
scan_e_error_macro_lacks_value=02065_E_La variable "$1" in una macro o espressione a tempo di compilazione non ha valore
% Thus the conditional compile time expression cannot be evaluated.
scan_e_wrong_switch_toggle_default=02066_E_Interruttore switch errato, usare ON/OFF/DEFAULT o +/-/*
% You need to use ON or OFF or DEFAULT or a + or - or * to toggle the switch.
scan_e_mode_switch_not_allowed=02067_E_Lo switch modale "$1" non è ammesso qui
% A mode switch has already been encountered, or, in the case of option -Mmacpas,
% a mode switch occurs after UNIT.
scan_e_error_macro_undefined=02068_E_La variabile o macro "$1" a tempo di compilazione non è definita.
% Thus the conditional compile time expression cannot be evaluated. Only in mode MacPas.
scan_e_utf8_bigger_than_65535=02069_E_Trovato un codice UTF-8 maggiore di 65535
% \fpc handles UTF-8 strings internally as widestrings, i.e. the char codes are limited to 65535.
scan_e_utf8_malformed=02070_E_Stringa UTF-8 malformata
% The given string isn't a valid UTF-8 string.
scan_c_switching_to_utf8=02071_C_Trovata firma UTF-8, uso codifica UTF-8
% The compiler found a UTF-8 encoding signature (\$ef, \$bb, \$bf) at the beginning of a file,
% so it interprets it as a UTF-8 file.
scan_e_compile_time_typeerror=02072_E_Espressione a tempo di compilazione: previsto $1 ma trovato $2 in $3
% The type-check of a compile time expression failed.
scan_n_app_type_not_support=02073_N_APPTYPE non è supportato dall'OS di destinazione
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_e_illegal_optimization_specifier=02074_E_L'ottimizzazione specificata "$1" è illegale
% You specified an optimization with the \var{\{\$OPTIMIZATION xxx\}} directive,
% and the compiler didn't recognize the optimization you specified.
scan_w_setpeflags_not_support=02075_W_SETPEFLAGS non è supportato dall'OS di destinazione
% The \var{\{\$SETPEFLAGS\}} directive is not supported by the target OS.
scan_w_imagebase_not_support=02076_W_IMAGEBASE non è supportato dall'OS di destinazione
% The \var{\{\$IMAGEBASE\}} directive is not supported by the target OS.
scan_w_minstacksize_not_support=02077_W_MINSTACKSIZE non è supportato dall'OS di destinazione
% The \var{\{\$MINSTACKSIZE\}} directive is not supported by the target OS.
scan_w_maxstacksize_not_support=02078_W_MAXSTACKSIZE non è supportato dall'OS di destinazione
% The \var{\{\$MAXSTACKSIZE\}} directive non è supportato dall'OS di destinazione
scanner_e_illegal_warn_state=02079_E_Stato illegale per la direttiva $WARN
% Only ON and OFF can be used as state with a \var{\{\$WARN\}} compiler directive.
scan_e_only_packset=02080_E_Valore illegale di set packing
% Only 0, 1, 2, 4, 8, DEFAULT and NORMAL are allowed as packset parameters.
scan_w_pic_ignored=02081_W_Direttiva o switch PIC ignorati
% Several targets, such as \windows, do not support nor need PIC,
% so the PIC directive and switch are ignored.
scan_w_unsupported_switch_by_target=02082_W_Lo switch "$1" non è supportato dalla destinazione scelta
% Some compiler switches like \$E are not supported by all targets.
scan_w_frameworks_darwin_only=02084_W_Le opzioni relative al framework sono supportate solo da Darwin/Mac OS X
% Frameworks are not a known concept, or at least not supported by FPC,
% on operating systems other than Darwin/Mac OS X.
scan_e_illegal_minfpconstprec=02085_E_Precisione minima per costanti floating point "$1" illegale
% Valid minimal precisions for floating point constants are default, 32 and 64,
% which mean respectively minimal (usually 32 bit), 32 bit and 64 bit precision.
scan_w_multiple_main_name_overrides=02086_W_Nome della procedura "main" ridefinito: il vecchio nome era "$1"
% The name for the main entry procedure is specified more than once. Only the last
% name will be used.
% \end{description}
#
# Parser
#
# 03265 is the last used one
#
% \section{Parser messages}
% This section lists all parser messages. The parser takes care of the
% semantics of you language, i.e. it determines if your Pascal constructs
% are correct.
% \begin{description}
parser_e_syntax_error=03000_E_Analisi - Errore di sintassi
% An error against the Turbo Pascal language was encountered. This typically
% happens when an illegal character is found in the source file.
parser_e_dont_nest_interrupt=03004_E_Le procedure INTERRUPT non possono essere nidificate
% An \var{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_Tipo di procedura "$1" ignorato
% The specified procedure directive is ignored by FPC programs.
parser_e_no_overload_for_all_procs=03006_E_Non tutte le dichiarazioni di "$1" sono dichiarate con OVERLOAD
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_export_name_double=03008_E_Nome di funzione esportata "$1" duplicato
% Exported function names inside a specific DLL must all be different.
parser_e_export_ordinal_double=03009_E_Indice di funzione esportata $1 duplicato
% Exported function indexes inside a specific DLL must all be different.
parser_e_export_invalid_index=03010_E_Indice di funzione esportata non valido
% DLL function index must be in the range \var{1..\$FFFF}.
parser_w_parser_reloc_no_debug=03011_W_Impossibile creare info di debug per DLL o eseguibile $1, perché è rilocabile.
% It is currently not possible to include debug information in a relocatable DLL.
parser_w_parser_win32_debug_needs_WN=03012_W_Per generare info di debug per codice win32 bisogna disabilitare la rilocazione con l'ozione -WN
% Stabs debug info is wrong for relocatable DLL or EXES. Use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_Il nome del costruttore deve essere INIT
% You are declaring an object constructor with a name which is not \var{init}, and the
% \var{-Ss} switch is in effect. See the switch \seeo{Ss}.
parser_e_destructorname_must_be_done=03014_E_Il nome del distruttore deve essere DONE
% You are declaring an object destructor with a name which is not \var{done}, and the
% \var{-Ss} switch is in effect. See the switch \seeo{Ss}.
parser_e_proc_inline_not_supported=03016_E_Tipo di procedura INLINE non supportato
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler doesn't support C++
% styled inlining by default.
parser_w_constructor_should_be_public=03018_W_Il costruttore dovrebbe essere pubblico
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_Il distruttore dovrebbe essere pubblico
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_Una classe dovrebbe avere un solo distruttore
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_Non sono ammesse definizioni di classi locali
% Classes must be defined globally. They cannot be defined inside a
% procedure or function.
parser_f_no_anonym_objects=03022_F_Non sono ammesse definizioni di classi anonime
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_n_object_has_no_vmt=03023_N_L'oggetto "$1" non ha VMT
% This is a note indicating that the declared object has no
% virtual method table.
parser_e_illegal_parameter_list=03024_E_Lista di parametri illegale
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_size=03026_E_La chiamata a "$1" ha un numero errato di parametri
% There is an error in the parameter list of the function or procedure --
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_L'identificatore overloaded "$1" non è una funzione
% The compiler encountered a symbol with the same name as an overloaded
% function, but it is not a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_ Due o più funzioni overloaded hanno gli stessi parametri
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_La definizione della funzione è diversa dalla dichiarazione "$1"
% You declared a function with the same parameters but
% different result type or function modifiers.
parser_e_header_different_var_names=03030_E_La definizione della funzione "$1" è diversa dalla dichiarazione: $2 cambia in $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but defined it with a different parameter list.
parser_n_duplicate_enum=03031_N_I valori nei tipi enumerazioni devono essere ascendenti
% \fpc allows enumeration constructions as in C. Examine the following
% two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_e_no_with_for_variable_in_other_segments=03033_E_Non si può usare With per variabili in un segmento diverso
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_Nidificazione di funzioni supera profondità 31
% You can nest function definitions only 31 levels deep.
parser_e_range_check_error=03035_E_Costanti fuori dalla gamma di valori ammessa
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_Costanti fuori dalla gamma di valori ammessa
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_etichetta case duplicata
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_Il limite superiore di un intervallo case è minore del limite inferiore
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless.
parser_e_type_const_not_possible=03039_E_costanti di tipo classe o interfaccia non sono permesse
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_Variabili di funzioni in funzioni overloaded non sono permesse
% You are trying to assign an overloaded function to a procedural variable.
% This is not allowed.
parser_e_invalid_string_size=03041_E_La lunghezza di una stringa deve essere fra 1 e 255
% The length of a shortstring in Pascal is limited to 255 characters. You are
% trying to declare a string with length less than 1 or greater than 255.
parser_w_use_extended_syntax_for_objects=03042_W_usate la sintassi estesa di NEW e DISPOSE per le istanze di oggetti
% If you have a pointer \var{a} to an object type, then the statement
% \var{new(a)} will not initialize the object (i.e. the constructor isn't
% called), although space will be allocated. You should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the object.
parser_w_no_new_dispose_on_void_pointers=03043_W_non ha senso usare NEW o DISPOSE con puntatori senza tipo
parser_e_no_new_dispose_on_void_pointers=03044_E_usare NEW o DISPOSE con puntatori senza tipo non è permesso
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% It is accepted for compatibility in \var{TP} and \var{DELPHI} modes, but the
% compiler will still warn you if it finds such a construct.
parser_e_class_id_expected=03045_E_Qui era atteso un identificatore di classe (ma non c'è)
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., an object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_gli identificatori di tipo non sono permessi qui
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_Qui era atteso un identificatore di metodo (ma non c'è)
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., an object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_La definizione della funzione non corrisponde a nessun metodo di questa classe "$1"
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., an object or class method, but the procedure name is not a
% procedure of this type.
parser_d_procedure_start=03049_DL_procedura/funzione $1
% When using the \var{-vd} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_Costante in virgola mobile illegale
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_FAIL si può usare solo nei costruttori
% You are using the \var{fail} keyword outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_I distruttori non possono avere parametri
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_methods_via_class_ref=03053_E_Solo i metodi di classe si possono chiamare riferendosi a una classe
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
parser_e_only_class_methods=03054_E_Solo i metodi di classe si possono chiamare da altri metodi di classe
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
parser_e_case_mismatch=03055_E_I tipi di un CASE e della sua costante non corrispondono
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_Simbolo non è esportabile da una libreria
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_Un metodo ereditato è nascosto da "$1"
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendant class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_Non c'è un metodo "$1" da sovrascrivere nella classe padre
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_no_procedure_to_access_property=03059_E_Non sono forniti membri per accedere alla proprietà
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_Direttiva di proprietà stored non ancora implementata
% This message is no longer used, as the \var{stored} directive has been implemented.
parser_e_ill_property_access_sym=03061_E_Simbolo illegale per l'accesso alla proprietà
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code would cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Qui non si può accedere alla parte protetta di un oggetto
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module where the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_Qui non si può accedere alla parte privata di un oggetto
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_e_overridden_methods_not_same_ret=03066_E_I metodi overridden devono avere lo stesso tipo di ritorno: "$2" è overriden da "$1" che ritorna un tipo diverso
% If you declare overridden methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_Funzioni dichiarate EXPORT non possono essere nidificate
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_I metodi di classe non si possono dichiarare EXPORT
% You cannot declare a procedure that is a method for an object as
% \var{export}ed.
parser_e_call_by_ref_without_typeconv=03069_E_I tipi nelle chiamate a funzioni var $1 devono corrispondere esattamente: passato "$2" ma serviva "$3"
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_La classe non è una classe padre della classe corrente
% When calling inherited methods, you are trying to call a method of a non-related
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_SELF is only allowed in methods
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_Solo i metodi si possono chiamare direttamente da altri metodi con l'identificatore della classe
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_Uso illegale di ':'
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_Errore di intervallo in un costruttore di insiemi, oppure elemento di insieme duplicato
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, or two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_Qui era atteso un puntatore a un elemento
% You specified an illegal type in a \var{new} statement.
% The extended syntax of \var{new} needs an object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_L'espressione deve essere la chiamata di un costruttore
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_L'espressione deve essere la chiamata di un distruttore
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_Elementi del record disposti in ordine illegale
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_L'espressione deve essere un tipo classe o record
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_Le procedure non possono ritornare un valore
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures cannot return a value.
parser_e_constructors_always_objects=03081_E_Costruttori e distruttori devono essere metodi
% You're declaring a procedure as destructor or constructor, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_L'operatore non ha overload per il tipo specificato
% You're trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_no_such_assignment=03083_E_Impossibile fare l'overload dell'assegnazione fra tipi uguali
% You cannot overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_Overload di operatore impossibile
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_Il rilancio di un'eccezione non è permesso qui
% You are trying to re-raise an exception where it is not allowed. You can only
% re-raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_La sintassi estesa di new o dispose non è permessa per una classe
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_procedure_overloading_is_off=03088_E_L'overloading di procedure è disabilitato
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_Impossibile fare l'overload di questo operatore. Operatori overloadabili simili (se esistono): $1
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_L'operatore di confronto deve dare un valore booleano
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_Solo i metodi virtuali possono essere astratti
% You are declaring a method as abstract, when it is not declared to be
% virtual.
parser_f_unsupported_feature=03092_F_Uso di capacità non supportata!
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_The mix of different kind of objects (class, object, interface, etc) isn't allowed
% You cannot derive \var{objects}, \var{classes}, \var{cppclasses} and \var{interfaces} intertwined. E.g.
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_Ignorata una direttiva di procedura sconosciuta: "$1"
% The procedure directive you specified is unknown.
parser_e_absolute_only_one_var=03095_E_ABSOLUTE si può associare solo a una variabile
% You cannot specify more than one variable before the \var{absolute} directive.
% Thus, the following construct will provide this error:
% \begin{verbatim}
% Var Z : Longint;
%     X,Y : Longint absolute Z;
% \end{verbatim}
parser_e_absolute_only_to_var_or_const=03096_E_ABSOLUTE si può associare solo a variabili o costanti
% The address of an \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
parser_e_initialized_only_one_var=03097_E_Si può inizializzare solo una variabile alla volta
% You cannot specify more than one variable with a initial value in Delphi mode.
parser_e_abstract_no_definition=03098_E_I metodi astratti non dovrebbero avere definizioni (il corpo funzione)
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_Questa funzione overloaded non può essere locale: deve essere esportata
% You are defining an overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_In "$1" dei metodi virtuali sono usati senza costruttore
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_c_macro_defined=03101_CL_Macro definita: $1
% When \var{-vc} is used, the compiler tells you when it defines macros.
parser_c_macro_undefined=03102_CL_Macro non definita: $1
% When \var{-vc} is used, the compiler tells you when it undefines macros.
parser_c_macro_set_to=03103_CL_Macro $1 impostata a $2
% When \var{-vc} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_Compilando $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_UL_Interfaccia di analisi della unit $1
% This tells you that the reading of the interface
% of the current unit has started
parser_u_parsing_implementation=03106_UL_Analisi dell'implementazione di $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_DL_Compilando $1 per la seconda volta
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_property_found_to_override=03109_E_Non ci sono proprietà di cui fare l'override
% You want to override a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_E' permessa solo una proprietà di default
% You specified a property as \var{Default}, but the class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_La proprietà di default deve essere una proprietà array
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_Costruttori virtuali sono supportati solo nel modello di oggetto classe
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_Nessuna proprietà di default disponibile
% You are trying to access a default property of a class, but this class (or one of
% its ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_La classe non può avere una sezione pubblicata, usare lo switch {$M+}
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, which turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_La dichiarazione della classe "$1" deve essere risolta qui per poterla usare come padre
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_Operatori locali non supportati
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_Direttiva di procedura "$1" non è permessa nella sezione interfaccia
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_Direttiva di procedura "$1" non è permessa nella sezione implementazione
% This procedure directive is not allowed in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_Direttiva di procedura "$1" non è permessa in dichiarazioni procvar
% This procedure directive cannot be part of a procedural or function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_La funzione è già dichiarata Public/Forward "$1"
% You will get this error if a function is defined as \var{forward} twice.
% Or if it occurs in the \var{interface} section, and again as a \var{forward}
% declaration in the \var{implementation} section.
parser_e_not_external_and_export=03121_E_Non si possono usare sia EXPORT che EXTERNAL
% These two procedure directives are mutually exclusive.
parser_n_not_supported_for_inline=03123_N_"$1" non è ancora supportata all'interno di procedure o funzioni inline
% Inline procedures don't support this declaration.
parser_h_inlining_disabled=03124_H_Inlining disabilitato
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_Scrivo il log del browser $1
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+ \}} switch).
parser_h_maybe_deref_caret_missing=03126_H_Forse bisogna prima dereferenziare un puntatore
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_Il lettore assembler scelto non è supportato
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_Direttiva di procedura "$1" è in conflitto con altre direttive
% You specified a procedure directive that conflicts with other directives.
% For instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_La convenzione di chiamata non corrisponde alla dichiarazione
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_property_cant_have_a_default_value=03131_E_La proprietà non può avere un valore di default
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_Il valore di default di una proprietà deve essere costante
% The value of a \var{default} declared property must be known at compile
% time. The value you specified is only known at run time. This happens
% e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_Questo simbolo non può essere pubblicato, può essere solo una classe
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_This kind of property can't be published
% Properties in a \var{published} section cannot be array properties.
% They must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_e_empty_import_name=03136_E_E' necessario un nome di importazione
% Some targets need a name for the imported procedure or a \var{cdecl} specifier.
parser_e_division_by_zero=03138_E_Divisione per zero
% A division by zero was encounted.
parser_e_invalid_float_operation=03139_E_Operazione in virgola mobile non valida
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_Limite superiore dell'intervallo minore del limite inferiore
% The upper bound of an array declaration is less than the lower bound and this
% is not possible.
parser_w_string_too_long=03141_W_La stringa "$1" è più lunga di "$2"
% The size of the constant string is larger than the size you specified in
% string type definition.
parser_e_string_larger_array=03142_E_La stringa è più lunga dell'array di char
% The size of the constant string is larger than the size you specified in
% the \var{Array[x..y] of char} definition.
parser_e_ill_msg_expr=03143_E_Espressione illegale dopo la direttiva di messaggio
% \fpc supports only integer or string values as message constants.
parser_e_ill_msg_param=03144_E_I gestori di messaggi possono prendere un solo parametro, dichiarato come chiamata per riferimento
% A method declared with the \var{message} directive as message handler
% can take only one parameter which must be declared as call by reference.
% Parameters are declared as call by reference using the \var{var}-directive.
parser_e_duplicate_message_label=03145_E_Label di messaggio duplicata: "$1"
% A label for a message is used twice in one object/class.
parser_e_self_in_non_message_handler=03146_E_In metodi che gestiscono messaggi, il parametro Self può essere solo esplicito 
% The \var{Self} parameter can only be passed explicitly to a method which
% is declared as message handler.
parser_e_threadvars_only_sg=03147_E_Le threadvars possono essere solo statiche o globali
% Threadvars must be static or global; you can't declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has its own stack and local variables
% are stored on the stack.
parser_f_direct_assembler_not_allowed=03148_F_L'assembler diretto non è supportato dal formato binario di output
% You can't use direct assembler when using a binary writer. Choose an
% other output format or use another assembler reader.
parser_w_no_objpas_use_mode=03149_W_Non caricare manualmente la unit OBJPAS, usare \{\$mode objfpc\} oppure \{\$mode delphi\}
% You are trying to load the \file{ObjPas} unit manually from a \var{uses} clause.
% This is not a good idea. Use the \var{\{\$MODE OBJFPC\}} or
% \var{\{\$mode delphi\}} directives which load the unit automatically.
parser_e_no_object_override=03150_E_OVERRIDE non si può usare negli oggetti
% \var{Override} is not supported for objects, use \var{virtual} instead to override
% a method of a parent object.
parser_e_cant_use_inittable_here=03151_E_I Tipi di dato che richiedono inizializzazione/finalizzazione non si possono usare in record variant
% Some data types (e.g. \var{ansistring}) need initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% can't be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_Le resourcestring possono essere solo o statiche o globali
% Resourcestring cannot be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_L'istruzione Exit con argomenti non si può usare qui
% An exit statement with an argument for the return value can't be used here. This
% can happen for example in \var{try..except} or \var{try..finally} blocks.
parser_e_stored_property_must_be_boolean=03154_E_Il simbolo di storage deve essere di tipo boolean
% If you specify a storage symbol in a property declaration, it must be a
% boolean type.
parser_e_ill_property_storage_sym=03155_E_Questo simbolo non è ammesso come simbolo di storage
% You can't use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants.
parser_e_only_publishable_classes_can_be_published=03156_E_Si possono pubblicare solo classi compilate nel modo $M+
% A class-typed field in the published section of a class can only be a class which was
% compiled in \var{\{\$M+\}} or which is derived from such a class. Normally
% such a class should be derived from \var{TPersistent}.
parser_e_proc_directive_expected=03157_E_Qui era attesa una direttiva di procedura (ma non c'è)
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
parser_e_invalid_property_index_value=03158_E_Il valore di un indice di proprietà deve essere un tipo ordinale
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_Il nome di procedura è troppo corto per poterlo esportare
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correctly with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_Non si possono generare voci DEFFILE per la variabili globali di una unit
parser_e_dlltool_unit_var_problem2=03161_E_Questo file deve essere compilato senza l'opzione -WD
% You need to compile this file without the -WD switch on the
% command line.
parser_f_need_objfpc_or_delphi_mode=03162_F_Questo file deve essere compilato in modo ObjFpc (-S2) o Delphi (-Sd)
% You need to use \var{\{\$MODE OBJFPC\}} or \var{\{\$MODE DELPHI\}} to compile this file.
% Or use the corresponding command line switch, either \var{-Mobjfpc} or \var{-MDelphi.}
parser_e_no_export_with_index_for_target=03163_E_Impossibile esportare con indice sulla piattaforma $1
% Exporting of functions or procedures with a specified index is not
% supported on this target.
parser_e_no_export_of_variables_for_target=03164_E_L'esportazione di variabili non è supportata sulla piattaforma $1
% Exporting of variables is not supported on this target.
parser_e_improper_guid_syntax=03165_E_Sintassi GUID errata
% The GUID indication does not have the proper syntax. It should be of the form
% \begin{verbatim}
% {XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX}
% \end{verbatim}
% Where each \var{X} represents a hexadecimal digit.
parser_w_interface_mapping_notfound=03168_W_Non c'è nessuna procedura di nome "$1" adatta per implementare $2.$3
% The compiler cannot find a suitable procedure which implements the given method of an interface.
% A procedure with the same name is found, but the arguments do not match.
parser_e_interface_id_expected=03169_E_era atteso un identificatore di interfaccia (ma non c'è)
% This happens when the compiler scans a \var{class} declaration that contains
% \var{interface} function name mapping code like this:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% and the \var{interface} before the dot is not listed in the inheritance list.
parser_e_type_cant_be_used_in_array_index=03170_E_Il tipo "$1" non si può usare come indice di un array
% Types like \var{qword} or \var{int64} aren't allowed as array index type.
parser_e_no_con_des_in_interfaces=03171_E_Le interfacce non possono avere né costruttori né distruttori
% Constructor and destructor declarations aren't allowed in interfaces.
% In the most cases method \var{QueryInterface} of \var{IUnknown} can
% be used to create a new interface.
parser_e_no_access_specifier_in_interfaces=03172_E_Non si possono usare specificatori d'accesso in INTERFACES
% The access specifiers \var{public}, \var{private}, \var{protected} and
% \var{published} can't be used in interfaces because all methods
% of an interface must be public.
parser_e_no_vars_in_interfaces=03173_E_Una interfaccia non può contenere campi
% Declarations of fields aren't allowed in interfaces. An interface
% can contain only methods and properties with method read/write specifiers.
parser_e_no_local_proc_external=03174_E_Le procedure locali non si possono dichiarare EXTERNAL
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance of errors very high.
parser_w_skipped_fields_before=03175_W_Alcuni campi all'inizio di "$1" non sono stati inizializzati
% In Delphi mode, not all fields of a typed constant record have to be
% initialized, but the compiler warns you when it detects such situations.
parser_e_skipped_fields_before=03176_E_Alcuni campi all'inizio di "$1" non sono stati inizializzati
% In all syntax modes but Delphi mode, you can't leave some fields uninitialized
% in the middle of a typed constant record.
parser_w_skipped_fields_after=03177_W_Alcuni campi alla fine di "$1" non sono stati inizializzati
% You can leave some fields at the end of a type constant record uninitialized
% (The compiler will initialize them to zero automatically). This may be the cause
% of subtle problems.
parser_e_varargs_need_cdecl_and_external=03178_E_Trovata direttiva VarArgs (o '...' in MacPas) senza CDecl/CPPDecl/MWPascal ed External
% The varargs directive (or the ``...'' varargs parameter in MacPas mode) can only be
% used with procedures or functions that are declared with \var{external} and one of
% \var{cdecl}, \var{cppdecl} and \var{mwpascal}. This functionality
% is only supported to provide a compatible interface to C functions like printf.
parser_e_self_call_by_value=03179_E_Self deve essere un normale parametro (chiamata per valore)
% You can't declare \var{Self} as a const or var parameter, it must always be
% a call-by-value parameter.
parser_e_interface_has_no_guid=03180_E_L'interfaccia "$1" non ha identificazione di interfaccia
% When you want to assign an interface to a constant, then the interface
% must have a GUID value set.
parser_e_illegal_field_or_method=03181_E_Identificatore di classe sconosciuto per campo o metodo "$1"
% Properties must refer to a field or method in the same class.
parser_w_proc_overriding_calling=03182_W_Forzatura della convenzione di chiamata "$1" a "$2"
% There are two directives in the procedure declaration that specify a calling
% convention. Only the last directive will be used.
parser_e_no_procvarobj_const=03183_E_Costanti tipizzate del tipo "procedura di oggetto" possono solo essere inizializzate con NIL
% You can't assign the address of a method to a typed constant which has a
% 'procedure of object' type, because such a constant requires two addresses:
% that of the method (which is known at compile time) and that of the object or
% class instance it operates on (which cannot be known at compile time).
parser_e_default_value_only_one_para=03184_E_I valori di default si possono assegnare solo a un parametro per volta
% It is not possible to specify a default value for several parameters at once.
% The following is invalid:
% \begin{verbatim}
% Procedure MyProcedure (A,B : Integer = 0);
% \end{verbatim}
% Instead, this should be declared as
% \begin{verbatim}
% Procedure MyProcedure (A : Integer = 0; B : Integer = 0);
% \end{verbatim}
parser_e_default_value_expected_for_para=03185_E_Parametro di default necessario per "$1"
% The specified parameter requires a default value.
parser_w_unsupported_feature=03186_W_Uso di capacità non supportata!
% You're trying to force the compiler into doing something it cannot do yet.
parser_h_c_arrays_are_references=03187_H_Array C sono passati per riferimento
% Any array passed to a C function is passed
% by a pointer (i.e. by reference).
parser_e_C_array_of_const_must_be_last=03188_E_Array C di const devono essere l'ultimo argomento
% You cannot add any other argument after an \var{array of const} for
% \var{cdecl} functions, as the size pushed on stack for this argument is
% not known.
parser_h_type_redef=03189_H_Ridefinizione del tipo "$1"
% This is an indicator that a previously declared type is
% being redefined as something else. This may, or may not
% be, a potential source of errors.
parser_w_cdecl_has_no_high=03190_W_Le funzioni dichiarate cdecl non passano parametri impliciti extra
% Functions declared with the \var{cdecl} modifier do not pass an extra implicit parameter.
parser_w_cdecl_no_openstring=03191_W_Le funzioni dichiarate cdecl non supportano stringhe aperte
% Openstring is not supported for functions that have the \var{cdecl} modifier.
parser_e_initialized_not_for_threadvar=03192_E_Le variabili dichiarate come threadvar non si possono inizializzare
% Variables declared as threadvar cannot be initialized with a default value.
% The variables will always be filled with zero at the start of a new thread.
parser_e_msg_only_for_classes=03193_E_La direttiva Message è permessa solo nelle classi
% The message directive is only supported for Class types.
parser_e_procedure_or_function_expected=03194_E_Era attesa una procedura o una funzione (ma non ci sono)
% A class method can only be specified for procedures and functions.
parser_e_illegal_calling_convention=03195_W_Direttiva di convenzione di chiamata "$1" ignorata
% Some calling conventions are supported only by certain CPUs. I.e. most non-i386 ports support
% only the standard ABI calling convention of the CPU.
parser_e_no_object_reintroduce=03196_E_REINTRODUCE non si può usare negli oggetti
% \var{reintroduce} is not supported for objects.
parser_e_paraloc_only_one_para=03197_E_Ogni argomento deve avere la sua collocazione
% If locations for arguments are specified explicitly as it is required by
% some syscall conventions, each argument must have its own location. Things
% like
% \begin{verbatim}
% procedure p(i,j : longint 'r1');
% \end{verbatim}
% aren't allowed.
parser_e_paraloc_all_paras=03198_E_Ogni argomento deve avere una collocazione esplicita
% If one argument has an explicit argument location, all arguments of a procedure
% must have one.
parser_e_illegal_explicit_paraloc=03199_E_Collocazione dell'argomento sconosciuta
% The location specified for an argument isn't recognized by the compiler.
parser_e_32bitint_or_pointer_variable_expected=03200_E_Qui era attesa una variabile intera o puntatore a 32 Bit (ma non c'è)
% The libbase for MorphOS/AmigaOS can be given only as \var{longint}, \var{dword} or any pointer variable.
parser_e_goto_outside_proc=03201_E_Le istruzioni Goto fra procedure diverse non sono permesse
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
parser_f_too_complex_proc=03202_F_Procedura troppo complessa, richiede troppi registri
% Your procedure body is too long for the compiler. You should split the
% procedure into multiple smaller procedures.
parser_e_illegal_expression=03203_E_Espressione illegale
% This can occur under many circumstances. Usually when trying to evaluate
% constant expressions.
parser_e_invalid_integer=03204_E_Espressione intera non valida
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
parser_e_invalid_qualifier=03205_E_Qualificatore illegale
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
parser_e_upper_lower_than_lower=03206_E_Limite superiore di intervallo minore del limite inferiore
% You are declaring a subrange, and the high limit is less than the low limit of
% the range.
parser_e_macpas_exit_wrong_param=03207_E_I parametri di Exit devono essere il nome della procedura in cui è usato
% Non local exit is not allowed. This error occurs only in mode MacPas.
parser_e_illegal_assignment_to_count_var=03208_E_Assegnazione illegale della variabile "$1" del ciclo for-loop 
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings. You also cannot assign values to
% loop variables inside the loop (Except in Delphi and TP modes). Use a while or
% repeat loop instead if you need to do something like that, since those
% constructs were built for that.
parser_e_no_local_var_external=03209_E_Le variabili locali non si possono dichiarare EXTERNAL
% Declaring local variables as external is not allowed. Only global variables can reference
% external variables.
parser_e_proc_already_external=03210_E_La procedura è già dichiarata EXTERNAL
% The procedure is already declared with the EXTERNAL directive in an interface or
% forward declaration.
parser_w_implicit_uses_of_variants_unit=03211_W_Uso implicito della unit Variants
% The Variant type is used in the unit without any used unit using the Variants unit. The
% compiler has implicitly added the Variants unit to the uses list. To remove this warning
% the Variants unit needs to be added to the uses statement.
parser_e_no_static_method_in_interfaces=03212_E_Classi e metodi statici non si possono usare in INTERFACES
% The specifier \var{class} and directive \var{static} can't be used in interfaces
% because all methods of an interface must be public.
parser_e_arithmetic_operation_overflow=03213_E_Overflow in una operazione aritmetica
% An operation on two integer values produced an overflow.
parser_e_protected_or_private_expected=03214_E_Qui erano attesi Protected oppure private (ma non ci sono)
% \var{strict} can be only used together with \var{protected} or \var{private}.
parser_e_illegal_slice=03215_E_SLICE non si può usare fuori dalla lista dei parametri
% \var{slice} can be used only for arguments accepting an open array parameter.
parser_e_dispinterface_cant_have_parent=03216_E_Una DISPINTERFACE non può avere una classe antenato
% A DISPINTERFACE is a special type of interface which can't have a parent class.
parser_e_dispinterface_needs_a_guid=03217_E_Una DISPINTERFACE deve avere un GUID
% A DISPINTERFACE always needs an interface identification (a GUID).
parser_w_overridden_methods_not_same_ret=03218_W_I metodi overloadati devono avere un tipo di ritorno analogo. Questo codice può crashare a causa di un bug nel parser Delphi ("$2" è overloadato da "$1" che ritorna un tipo diverso)
% If you declare overridden methods in a class definition, they must
% have the same return type. Some versions of Delphi allow you to change the
% return type of interface methods, and even to change procedures into
% functions, but the resulting code may crash depending on the types used
% and the way the methods are called.
parser_e_dispid_must_be_ord_const=03219_E_Gli ID Dispatch devono essere costanti ordinali
% The \var{dispid} keyword must be followed by an ordinal constant (the dispid index).
parser_e_array_range_out_of_bounds=03220_E_L'intervallo dell'array è troppo grande
% Regardless of the size taken up by its elements, an array cannot have more
% than high(ptrint) elements. Additionally, the range type must be a subrange
% of ptrint.
parser_e_packed_element_no_var_addr=03221_E_Non si può accedere agli indirizzi di elementi di array o campi di record bit packed
% If you declare an array or record as \var{packed} in Mac Pascal mode
% (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. This means it becomes impossible to take addresses
% of individual array elements or record fields. The only exception to this rule
% is in the case of packed arrays elements whose packed size is a multple of 8 bits.
parser_e_packed_dynamic_open_array=03222_E_Gli array dinamici non possono essere packed
% Only regular (and possibly in the future also open) arrays can be packed.
parser_e_packed_element_no_loop=03223_E_Non si possono usare elementi di array o campi di record bit packed come variabili di un loop
% If you declare an array or record as \var{packed} in Mac Pascal mode
% (or as \var{packed} in any mode with \var{\{\$bitpacking on\}}), it will
% be packed at the bit level. For performance reasons, they cannot be
% used as loop variables.
parser_e_type_and_var_only_in_generics=03224_E_VAR e TYPE sono permessi solo nei generic
% The usage of VAR and TYPE to declare new types inside an object is allowed only inside
% generics.
parser_e_cant_create_generics_of_this_type=03225_E_Questo tipo non può essere un generic
% Only Classes, Objects, Interfaces and Records are allowed to be used as generic.
parser_w_no_lineinfo_use_switch=03226_W_Non caricare la unit LINEINFO manualmente, Usare lo switch -gl del compilatore per farlo
% Do not use the \file{lineinfo} unit directly, Use the \var{-gl} switch which
% automatically adds the correct unit for reading the selected type of debugging
% information. The unit that needs to be used depends on the type of
% debug information used when compiling the binary.
parser_e_no_funcret_specified=03227_E_La funzione "$1" deve dichiarare il tipo del suo risultato
% The first time you declare a function you have to declare it completely,
% including all parameters and the result type.
parser_e_special_onlygenerics=03228_E_La specializzazione è supportata solo per i tipi generici
% Types which are not generics can't be specialized.
parser_e_no_generics_as_params=03229_E_Non si possono usare generici come parametri al momento della loro specializzazione
% When specializing a generic, only non-generic types can be used as parameters.
parser_e_type_object_constants=03230_E_Costanti di oggetti contenenti un VMT non sono permesse
% If an object requires a VMT either because it contains a constructor or virtual methods,
% it's not allowed to create constants of it. In TP and Delphi mode this is allowed
% for compatibility reasons.
parser_e_label_outside_proc=03231_E_Non è ammesso prendere l'indirizzo di label definite fuori della visibilità corrente
% It isn't allowed to take the address of labels outside the
% current procedure.
parser_e_initialized_not_for_external=03233_E_Non si possono inizializzare variabili dichiarate external
% Variables declared as external cannot be initialized with a default value.
parser_e_illegal_function_result=03234_E_Il tipo del risultato della funzione è illegale
% Some types like file types cannot be used as function result.
parser_e_no_common_type=03235_E_Non esistono tipi in comune fra "$1" e "$2"
% To perform an operation on integers, the compiler converts both operands
% to their common type, which appears to be an invalid type. To determine the
% common type of the operands, the compiler takes the minimum of the minimal values
% of both types, and the maximum of the maximal values of both types. The common
% type is then minimum..maximum.
parser_e_no_generics_as_types=03236_E_Generici senza specializzazione non si possono usare come tipi per una variabile
% Generics must be always specialized before being used as variable type.
parser_w_register_list_ignored=03237_W_Le liste di registri sono ignorate per routines in assembler puro
% When using pure assembler routines, the list with modified registers is ignored.
parser_e_implements_must_be_class_or_interface=03238_E_La proprietà IMPLEMENTS deve avere un tipo classe o interfaccia
% A property which implements an interface must be of type class or interface.
parser_e_implements_must_have_correct_type=03239_E_Le proprietà IMPLEMENTS devono implementare interfacce del tipo giusto. Trovato tipo "$1" ma serviva "$2"
% A property which implements an interface actually implements a different interface.
parser_e_implements_must_read_specifier=03240_E_Le proprietà IMPLEMENTS devono avere specificatore di lettura
% A property  which implements an interface must have at least a read specifier.
parser_e_implements_must_not_have_write_specifier=03241_E_Le proprietà IMPLEMENTS non devono avere specificatore di scrittura
% A property  which implements an interface may not have a write specifier.
parser_e_implements_must_not_have_stored_specifier=03242_E_Le proprietà IMPLEMENTS non devono avere specificatore stored
% A property  which implements an interface may not have a stored specifier.
parser_e_implements_uses_non_implemented_interface=03243_E_Proprietà IMPLEMENTS usata su una interfaccia non implementata: "$1"
% The interface which is implemented by a property is not an interface implemented by the class.
parser_e_unsupported_real=03244_E_Questa destinazione non supporta la virgola mobile
% The compiler parsed a floating point expression, but it is not supported.
parser_e_class_doesnt_implement_interface=03245_E_La classe "$1" non implementa l'interfaccia "$2"
% The delegated interface is not implemented by the class given in the implements clause.
parser_e_class_implements_must_be_interface=03246_E_Il tipo usato da IMPLEMENTS deve essere un'interfaccia
% The \var{implements} keyword must be followed by an interface type.
parser_e_cant_export_var_different_name=03247_E_Non si possono esportare variabili con un nome diverso su questa destinazione, aggiungere il nome alla dichiarazione con la direttiva "export" (nome variabile: $1, nome di export dichiarato: $2)
% On most targets it is not possible to change the name under which a variable
% is exported inside the \var{exports} statement of a library.
% In that case, you have to specify the export name at the point where the
% variable is declared, using the \var{export} and \var{alias} directives.
parser_e_weak_external_not_supported=03248_E_La destinazione attuale non supporta simboli esterni deboli 
% A "weak external" symbol is a symbol which may or may not exist at (either static
% or dynamic) link time. This concept may not be available (or implemented yet)
% on the current cpu/OS target.
parser_e_forward_mismatch=03249_E_La dichiarazione e l'implementazione del tipo non corrispondono
% Classes and interfaces being defined forward must have the same type
% when being implemented. A forward interface cannot be changed into a class.
parser_n_ignore_lower_visibility=03250_N_Il metodo virtuale "$1" ha una visibilità minore ($2) della classe padre $3 ($4)
% The virtual method overrides a method that is declared with a higher visibility. This might give
% unexpected results. In case the new visibility is private than it might be that a call to inherited in a
% new child class will call the higher visible method in a parent class and ignores the private method.
parser_e_field_not_allowed_here=03251_E_I campi non possono comparire dopo la definizione di un metodo o di una proprietà: iniziare una nuova sezione di visibilità prima
% Once a method or property has been defined in a class or object, you cannot define any fields afterwards
% without starting a new visibility section (such as \var{public}, \var{private}, etc.). The reason is
% that otherwise the source code can appear ambiguous to the compiler, since it is possible to use modifiers
% such as \var{default} and \var{register} also as field names.
parser_e_no_local_para_def=03252_E_I parametri non possono contenere definizioni di tipo locali. Usate una definizione di tipo a parte in un blocco type.
% In Pascal, types are not considered to be identical simply because they are semantically equivalent.
% Two variables or parameters are only considered to be of the same type if they refer to the
% same type definition.
% As a result, it is not allowed to define new types inside parameter lists, because then it is impossible to
% refer to the same type definition in the procedure headers of the interface and implementation of a unit
% (both procedure headers would define a separate type). Keep in mind that expressions such as
% ``file of byte'' or ``string[50]'' also define a new type.
parser_e_abstract_and_sealed_conflict=03253_E_Conflitto fra ABSTRACT e SEALED
% ABSTRACT and SEALED cannot be used together in one declaration
parser_e_sealed_descendant=03254_E_Non posso creare discendenti dalla classe "$1", perché è SEALED
% Sealed means that class cannot be derived by another class.
parser_e_sealed_class_cannot_have_abstract_methods=03255_E_Le classi SEALED non possono avere metodi ABSTRACT
% Sealed means that class cannot be derived. Therefore no one class is able to override an abstract method in a sealed class.
parser_e_only_virtual_methods_final=03256_E_Solo i metodi virtuali possono essere final
% You are declaring a method as final, when it is not declared to be
% virtual.
parser_e_final_can_no_be_overridden=03257_E_I metodi final non si possono overloadare: "$1"
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_invalid_enumerator_identifier=03259_E_Identificatore di enumeratore non valido: "$1"
% Only "MoveNext" and "Current" enumerator identifiers are supported.
parser_e_enumerator_identifier_required=03260_E_E' necessario un identificatore di enumeratore
% "MoveNext" or "Current" identifier must follow the \var{enumerator} modifier.
parser_e_enumerator_movenext_is_not_valid=03261_E_Il metodo pattern enumerator MoveNext non è valido. Il metodo deve essere una funzione con tipo di ritorno boolean e senza argomenti.
% "MoveNext" enumerator pattern method must be a function with Boolean return type and no required arguments
parser_e_enumerator_current_is_not_valid=03262_E_La proprietà del pattern enumerator Current non è valida. La proprietà deve avere un getter.
% "Current" enumerator pattern property must have a getter
parser_e_only_one_enumerator_movenext=03263_E_E' permesso un solo metodo enumerator MoveNext per classe/oggetto
% Class or Object can have only one enumerator MoveNext declaration.
parser_e_only_one_enumerator_current=03264_E_E' permessa una sola proprietà enumerator Current per classe/oggetto
% Class or Object can have only one enumerator Current declaration.
parser_e_for_in_loop_cannot_be_used_for_the_type=03265_E_Non si possono usare for in loop per il tipo "$1"
% For in loop can be used not for all types. For example it cannot be used for the enumerations with jumps.
% \end{description}
#
# Type Checking
#
# 04087 is the last used one
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_Type mismatch
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_Incompatible types: got "$1" expected "$2"
% There is no conversion possible between the two types.
% Another possiblity is that they are declared in different
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
type_e_not_equal_types=04002_E_I tipi "$1" e "$2" devono essere uguali ma non lo sono
% The types are not equal.
type_e_type_id_expected=04003_E_Qui era atteso un identificatore di tipo (ma non c'è)
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_Qui era atteso un identificatore di variabile (ma non c'è)
% This happens when you pass a constant to a routine (such as \var{Inc} var or \var{Dec})
% when it expects a variable. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_Qui era attesa una espressione intera, invece c'è "$1"
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_Qui era attesa una espressione booleana, invece c'è "$1"
% The expression must be a boolean type. It should be return \var{True} or
% \var{False}.
type_e_ordinal_expr_expected=04007_E_Qui era attesa una espressione ordinale (ma non c'è)
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_Qui era atteso un tipo puntatore, invece c'è "$1"
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_Qui era atteso un tipo classe, invece c'è "$1"
% The variable or expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_cant_eval_constant_expr=04011_E_L'espressione da valutare non ritorna una costante
% This error can occur when the bounds of an array you declared do
% not evaluate to ordinal constants.
type_e_set_element_are_not_comp=04012_E_Gli elementi dell'insieme non sono compatibili
% You are trying to perform an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union.
type_e_set_operation_unknown=04013_E_questa operazione non è implementata per gli insiemi
% several binary operations are not defined for sets.
% These include: \var{div}, \var{mod}, \var{**}, \var{>=} and \var{<=}.
% The last two may be  defined for sets in the future.
type_w_convert_real_2_comp=04014_W_Conversione automatica di tipo da virgola mobile a COMP (che è un tipo intero)
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_L'operatore di divisione fra tipi interi è 'DIV' e non '/'
% When hints are on, then an integer division with the '/' operator will
% produce this message, because the result will then be of type real.
type_e_strict_var_string_violation=04016_E_I tipi stringa non corrispondono, a causa dello switch $V+
% When compiling in \var{\{\$V+\}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_Non si possono usare succ o pred su enums con assegnazione stile C
% If you declare an enumeration type which has C-like assignments
% in it, such as in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% then you cannot use the \var{Succ} or \var{Pred} functions with this enumeration.
type_e_cant_read_write_type=04018_E_Le variabili di questo tipo non si possono leggere o scrivere da file direttamente
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that variable's type.
% Only integer types, reals, pchars and strings can be read from or
% written to a text file. Booleans can only be written to text files.
type_e_no_readln_writeln_for_typed_file=04019_E_Non si possono usare readln o writeln su file che non sono di tipo testo
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_Non si possono usare read o write su file senza tipo.
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_Conflitto di tipo fra elementi dell'insieme
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(dword/qword) ritorna la word/dword inferiore/superiore
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword/int64/qword}
% which returns the lower/upper word/dword of the argument. \tp always uses
% a 16 bit \var{lo/hi} which always returns bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the \tp behavior you have
% to type cast the argument to a \var{word} or \var{integer}.
type_e_integer_or_real_expr_expected=04023_E_Qui era attesa una espressione intera o reale
% The first argument to \var{str} must be a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_Tipo "$1" errato nel costruttore dell'array
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_Tipo incompatibile per l'argomento n. $1: trovato "$2", serviva "$3"
% You are trying to pass an invalid type for the specified parameter.
type_e_no_method_and_procedure_not_compatible=04026_E_Non si può assegnare un metodo a una variabile procedura, né una procedura a un puntatore a metodo
% You can't assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_Passata una costante illegale a una funzione matematica interna
% The constant argument passed to a \var{ln} or \var{sqrt} function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_Non si può ottenere l'indirizzo di una espressione costante
% It is not possible to get the address of a constant expression, because they
% aren't stored in memory. You can try making it a typed constant. This error
% can also be displayed if you try to pass a property to a var parameter.
type_e_argument_cant_be_assigned=04029_E_A questo argomento non possono essere assegnati valori
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference arguments.
%
% Remark: Properties can be used on the left side of an assignment,
% nevertheless they cannot be used as arguments.
type_e_cannot_local_proc_to_procvar=04030_E_Non si possono assegnare procedure/funzioni locali a variabili procedura
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling convention of a local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_Non si possono assegnare valori a un indirizzo di memoria
% It is not allowed to assign a value to an address of a variable, constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_Non si possono assegnare valori a variabili dichiarate const
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const. To allow
% changing the value, pass the parameter by value, or a parameter by reference
% (using var).
type_e_array_required=04033_E_E' richiesto un tipo array
% If you are accessing a variable using an index '[<x>]' then
% the type must be an array. In FPC mode a pointer is also allowed.
type_e_interface_type_expected=04034_Qui era atteso un tipo interfaccia, invece c'è "$1"
% The compiler expected to encounter an interface type name, but got something else.
% The following code would produce this error:
% \begin{verbatim}
% Type
%   TMyStream = Class(TStream,Integer)
% \end{verbatim}
type_h_mixed_signed_unsigned=04035_H_Mescolare longword ed espressioni con segno dà un risultato a 64bit
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetic
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64-bit arithmetic which is slower than normal
% 32-bit arithmetic. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_In questo punto, mescolare espressioni con segno e tipi cardinali può provocare un errore di controllo range 
% If you use a binary operator (and, or, xor) and one of
% the operands is a longword while the other one is a signed expression, then,
% if range checking is turned on, you may get a range check error because in
% such a case both operands are converted to longword before the operation is
% carried out. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_e_typecast_wrong_size_for_assignment=04037_E_La conversione di tipo causa dimensioni diverse ($1 -> $2) nell'assegnazione
% Type casting to a type with a different size is not allowed when the variable is
% used in an assignment.
type_e_array_index_enums_with_assign_not_possible=04038_E_Gli enum con assegnazione non si possono usare come indici di array
% When you declared an enumeration type which has C-like
% assignments, such as in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use it as the index of an array.
type_e_classes_not_related=04039_E_I tipi classe o oggetto "$1" e "$2" non sono correlati. Conversione di tipo impossibile. 
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors.
type_w_classes_not_related=04040_W_I tipi classe "$1" e "$2" non sono correlati.
% There is a typecast from one class to another while the classes
% are not related. This will probably lead to errors.
type_e_class_or_interface_type_expected=04041_E_Qui era atteso un tipo classe o interfaccia, invece c'è il tipo "$1"
% The compiler expected a class or interface name, but got another type or identifier.
type_e_type_is_not_completly_defined=04042_E_Il tipo "$1" non è completamente definito
% This error occurs when a type is not complete: i.e. a pointer type which points to
% an undefined type.
type_w_string_too_long=04043_W_Il letterale stringa è troppo lungo per una short string (lunghezza maggiore di 255)
% The size of the constant string, which is assigned to a shortstring,
% is longer than the maximum size of the shortstring (255 characters).
type_w_signed_unsigned_always_false=04044_W_A causa dei range diversi, il confronto è sempre FALSE
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% false. Explicitly typecast the constant to the correct range to avoid this problem.
type_w_signed_unsigned_always_true=04045_W_A causa dei range diversi, il confronto è sempre TRUE
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% true. Explicitly typecast the constant to the correct range to avoid this problem.
type_w_instance_with_abstract=04046_W_Costruzione della classe "$1" con metodo "$2" astratto
% An instance of a class is created which contains non-implemented abstract
% methods. This will probably lead to a runtime error 211 in the code if that
% routine is ever called. All abstract methods should be overriden.
type_h_in_range_check=04047_H_L'operando a sinistra dell'operatore IN dovrebbe avere la dimensione di un byte
% The left operand of the \var{in} operator is not an ordinal or enumeration which fits
% within 8 bits. This may lead to range check errors. The \var{in} operator
% currently only supports a left operand which fits within a byte. In the case of
% enumerations, the size of an element of an enumeration can be controlled with
% the \var{\{\$PACKENUM\}} or \var{\{\$Zn\}} switches.
type_w_smaller_possible_range_check=04048_W_Differenza nella dimensione dei tipi: possibili perdite di dati o errori di overflow
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_h_smaller_possible_range_check=04049_H_Differenza nella dimensione dei tipi: possibili perdite di dati o errori di overflow
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_e_cant_take_address_of_abstract_method=04050_E_Non si può prendere l'indirizzo di un metodo astratto
% An abstract method has no body, so the address of an abstract method can't be taken.
type_e_assignment_not_allowed=04051_E_Non si possono assegnare valori a parametri formali o ad array aperti
% You are trying to assign a value to a formal (untyped var, const or out)
% parameter, or to an open array.
type_e_constant_expr_expected=04052_E_Qui era attesa una espressione costante
% The compiler expects an constant expression, but gets a variable expression.
type_e_operator_not_supported_for_types=04053_E_L'operazione "$1" non è possibile sui tipi "$2" e "$3"
% The operation is not allowed for the supplied types.
type_e_illegal_type_conversion=04054_E_La conversione dal tipo "$1" al tipo "$2" è illegale
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
type_h_pointer_to_longint_conv_not_portable=04055_H_La conversione fra ordinali e puntatori non è portabile
% If you typecast a pointer to a longint (or vice-versa), this code will not compile
% on a machine using 64 bits addressing.
type_w_pointer_to_longint_conv_not_portable=04056_W_La conversione fra ordinali e puntatori non è portabile
% If you typecast a pointer to an ordinal type of a different size (or vice-versa), this can
% cause problems. This is a warning to help in finding the 32-bit specific code where cardinal/longint is used
% to typecast pointers to ordinals. A solution is to use the ptrint/ptruint types instead.
type_e_cant_choose_overload_function=04057_E_La funzione non ha alcun overload per parametri di questo tipo
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
type_e_illegal_count_var=04058_E_La variabile contatore del loop deve essere un tipo ordinale
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
type_w_double_c_varargs=04059_W_Conversione di costanti real a double per variabili argomento C: aggiungere un typecast esplicito per prevenirlo.
% In C, constant real values are double by default. For this reason, if you
% pass a constant real value to a variable argument part of a C function, FPC
% by default converts this constant to double as well. If you want to prevent
% this from happening, add an explicit typecast around the constant.
type_e_class_or_cominterface_type_expected=04060_E_Qui era atteso un tipo interfaccia classe o COM, invece c'è il tipo "$1"
% Some operators, such as the AS operator, are only applicable to classes or COM interfaces.
type_e_no_const_packed_array=04061_E_Array packed costanti non sono ancora supportati
% You cannot declare a (bit)packed array as a typed constant.
type_e_got_expected_packed_array=04062_E_Tipo incompatibile: argomento n. $1 di tipo "$2", invece deve essere "(Bit)Packed Array"
% The compiler expects a (bit)packed array as the specified parameter.
type_e_got_expected_unpacked_array=04063_E_Tipo incompatibile: argomento n. $1 di tipo "$2", invece deve essere "(not packed) Array"
% The compiler expects a regular (i.e., not packed) array as the specified parameter.
type_e_no_packed_inittable=04064_E_Gli elementi di un array packed non possono essere di un tipo che deve essere inizializzato
% Support for packed arrays of types that need initialization
% (such as ansistrings, or records which contain ansistrings) is not yet implemented.
type_e_no_const_packed_record=04065_E_Oggetti o record packed costanti non sono ancora supportati
% You cannot declare a (bit)packed array as a typed constant at this time.
type_w_untyped_arithmetic_unportable=04066_W_L'aritmetica "$1" su puntatori senza tipo non è portabile su {$T+}, si suggerisce un typecast
% Addition/subtraction from an untyped pointer may work differently in \var{\{\$T+\}}.
% Use a typecast to a typed pointer.
type_e_cant_take_address_of_local_subroutine=04076_E_Non si può prendere l'indirizzo di una subroutine marcata come local
% The address of a subroutine marked as local can't be taken.
type_e_cant_export_local=04077_E_Una subroutine marcata come local non è esportabile fuori dalla unit
% A subroutine marked as local can't be exported from a unit.
type_e_not_automatable=04078_E_Il tipo "$1" non è automatizzabile
% Only byte, integer, longint, smallint, currency, single, double, ansistring,
% widestring, tdatetime, variant, olevariant, wordbool and all interfaces are automatable.
type_h_convert_add_operands_to_prevent_overflow=04079_H_Convertire gli operandi a "$1" prima dell'addizione può evitare errori di overflow
% Adding two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the addition.
type_h_convert_sub_operands_to_prevent_overflow=04080_H_Convertire gli operandi a "$1" prima della sottrazione può evitare errori di overflow
% Subtracting two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the subtraction.
type_h_convert_mul_operands_to_prevent_overflow=04081_H_Convertire gli operandi a "$1" prima della moltiplicazione può evitare errori di overflow
% Multiplying two types can cause overflow errors. Since you are converting the result to a larger type, you
% could prevent such errors by converting the operands to this type before doing the multiplication.
type_w_pointer_to_signed=04082_W_Convertire puntatori in interi con segno può portare a confronti errati ed errori di range: meglio usare un tipo senza segno
% The virtual address space on 32-bit machines runs from \$00000000 to \$ffffffff.
% Many operating systems allow you to allocate memory above \$80000000.
% For example both \windows and \linux allow pointers in the range \$0000000 to \$bfffffff.
% If you convert pointers to signed types, this can cause overflow and range check errors,
% but also \$80000000 < \$7fffffff. This can cause random errors in code like "if p>q".
type_interface_has_no_guid=04083_E_Il tipo interfaccia $1 non ha un GUID valido
% When applying the as-operator to an interface or class, the desired interface (i.e. the right operand of the
% as-operator) must have a valid GUID.
type_e_invalid_objc_selector_name=04084_E_Nome di selettore non valido
% An Objective-C selector cannot be empty, must be a valid identifier or a single colon,
% and if it contains at least one colon it must also end in one.
type_e_expected_objc_method_but_got=04085_E_Qui era atteso un metodo Objective-C, invece c'è $1
% A selector can only be created for Objective-C methods, not for any other kind
% of procedure/function/method.
type_e_expected_objc_method=04086_E_Qui era atteso un metodo Objective-C o un nome di un metodo costante
% A selector can only be created for Objective-C methods, either by specifying
% the name using a string constant, or by using an Objective-C method identifier
% that is visible in the current scope.
type_e_no_type_info=04087_E_Nessuna informazione sul tipo disponibile per questo tipo
% Type information is not generated for some types, such as enumerations with gaps
% in their value range (this includes enumerations whose lower bound is different
% from zero).
% \end{description}
#
# Symtable
#
# 05069 is the last used one
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Identificatore "$1" non trovato
% The compiler doesn't know this symbol. Usually happens when you misspell
% the name of a variable or procedure, or when you forget to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_Errore interno del compilatore in SymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_Identificatore "$1" duplicato
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_Identificatore già definito in $1 alla riga $2
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_Identificatore "$1" sconosciuto
% The identifier encountered has not been declared, or is used outside the
% scope where it is defined.
sym_e_forward_not_resolved=05005_E_Dichiarazione "$1" priva di implementazione
% This can happen in two cases:
% \begin{itemize}
% \item You declare a function in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_e_error_in_type_def=05007_E_Errore nella definizione del tipo
% There is an error in your definition of a new array type.
% One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_forward_type_not_resolved=05009_E_Tipo "$1" dichiarato ma non implementato
% A symbol was forward defined, but no declaration was encountered.
sym_e_only_static_in_static=05010_E_Al di fuori dei metodi o nei metodi statici si possono usare solo variabli statiche
% A static method of an object can only access static variables.
sym_e_type_must_be_rec_or_object_or_class=05012_E_Record or object or class type expected
% The variable or expression isn't of the type \var{record} or \var{object} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Non sono permesse istanze di classi o di oggetti con un metodo astratto
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_Label "$1" non definita
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_Label "$1" usata ma non definita
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_Dichiarazione di label illegale
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO e LABEL non sono supportati (usare lo switch -Sg)
% You must use the -Sg switch to compile a program which has \var{label}s
% and \var{goto} statements. By default, \var{label} and \var{goto} aren't
% supported.
sym_e_label_not_found=05018_E_Label non trovata
% A \var{goto label} was encountered, but the label wasn't declared.
sym_e_id_is_no_label_id=05019_E_L'identificatore non è una label
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_Label già definita
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_La dichiarazione del tipo di un elemento in un insieme è illegale
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_Classe "$1" dichiarata ma non implementata
% You declared a class, but you did not implement it.
sym_n_unit_not_used=05023_H_La unit "$1" non è usata in $2
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_Parametro "$1" non usato
% The identifier was declared (locally or globally) but
% was not used (locally or globally).
sym_n_local_identifier_not_used=05025_N_Variabile locale "$1" non usata
% You have declared, but not used, a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_Al parametro "$1" è assegnato un valore, ma poi non viene mai usato
% The identifier was declared (locally or globally) and
% assigned to, but is not used (locally or globally) after the assignment.
sym_n_local_identifier_only_set=05027_N_La variabile locale "$1" è assegnata ma non viene mai usata
% The variable in a procedure or function implementation is declared and
% assigned to, but is not used after the assignment.
sym_h_local_symbol_not_used=05028_H_Locale $1 "$2" non è mai usato
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_Il campo privato "$1.$2" non viene mai usato
% The indicated private field is defined, but is never used in the code.
sym_n_private_identifier_only_set=05030_N_Il campo privato "$1.$2" è assegnato ma poi non viene mai usato
% The indicated private field is declared and assigned to, but never read.
sym_n_private_method_not_used=05031_N_Il metodo privato "$1.$2" non è mai usato
% The indicated private method is declared but is never used in the code.
sym_e_set_expected=05032_E_Qui era atteso un tipo insieme
% The variable or expression is not of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_Il valore di ritorno della funzione non sembra essere mai assegnato
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_Il tipo "$1" non è allineato correttamente per il linguaggio C
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_L'identificatore del campo di un record "$1" è sconosciuto
% The field doesn't exist in the record/object definition.
sym_w_uninitialized_local_variable=05036_W_La variabile locale "$1" non sembra essere inizializzata
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment).
sym_w_uninitialized_variable=05037_W_La variabile "$1" non sembra essere inizializzata
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assignment).
sym_e_id_no_member=05038_E_L'identificatore "$1" non identifica alcun membro
% This error is generated when an identifier of a record,
% field or method is accessed while it is not defined.
sym_h_param_list=05039_H_Trovata dichiarazione: $1
% You get this when you use the \var{-vh} switch.In the case of an overloaded procedure
% not being found. Then all candidate overloaded procedures are
% listed, with their parameter lists.
sym_e_segment_too_large=05040_E_Elemento dati troppo grande
% You get this when you declare a data element whose size exceeds the
% prescribed limit (2 Gb on 80386+/68020+ processors).
sym_e_no_matching_implementation_found=05042_E_Non ci sono implementazioni corrispondenti per il metodo di interfaccia "$1"
% There was no matching method found which could implement the interface
% method. Check argument types and result type of the methods.
sym_w_deprecated_symbol=05043_W_Il simbolo "$1" è deprecato
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Use of this symbol
% should be avoided as much as possible.
sym_w_non_portable_symbol=05044_W_Il simbolo "$1" non è portabile
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{platform} is used. This symbol's value, use
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_non_implemented_symbol=05055_W_Il simbolo "$1" non è implementato
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{unimplemented} is used. This symbol is defined,
% but is not yet implemented on this specific platform.
sym_e_cant_create_unique_type=05056_E_Non si può creare un tipo univoco dal tipo fornito
% Only simple types like ordinal, float and string types are supported when
% redefining a type with \var{type newtype = type oldtype;}.
sym_h_uninitialized_local_variable=05057_H_La variabile locale "$1" non sembra essere inizializzata
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. it did not appear in the left-hand side of an
% assignment).
sym_h_uninitialized_variable=05058_H_La variabile "$1" non sembra essere inizializzata
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. it appears in the right-hand side of an expression) when it
% was not initialized first (i.e. t did not appear in the left-hand side of an
% assignment).
sym_w_function_result_uninitialized=05059_W_La variabile risultato della funzione sembra non essere ancora stata assegnata
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. before it appeared in the left-hand side of an
% assignment).
sym_h_function_result_uninitialized=05060_H_La variabile risultato della funzione sembra non essere ancora stata assegnata
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. it appears in the right-hand side of an expression)
% before it is initialized (i.e. it appears in the left-hand side of an
% assignment)
sym_w_identifier_only_read=05061_W_La variabile "$1" è letta ma mai assegnata
% You have read the value of a variable, but nowhere assigned a value to
% it.
sym_h_abstract_method_list=05062_H_Trovato metodo astratto: $1
% When getting a warning about constructing a class/object with abstract methods
% you get this hint to assist you in finding the affected method.
sym_w_experimental_symbol=05063_W_Il simbolo "$1" è sperimentale
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{experimental} is used. Experimental symbols
% might disappear or change semantics in future versions. Usage of this symbol
% should be avoided as much as possible.
sym_w_forward_not_resolved=05064_W_Dichiarazione "$1" non implementata, si assume che sia external
% This happens if you declare a function in the \var{interface} of a unit in macpas mode,
% but do not implement it.
sym_w_library_symbol=05065_W_Il simbolo "$1" appartiene a una libreria
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{library} is used. Library symbols may not be
% available in other libraries.
sym_w_deprecated_symbol_with_msg=05066_W_Il simbolo "$1" è deprecato: "$2"
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Use of this symbol
% should be avoided as much as possible.
sym_e_no_enumerator=05067_E_Non c'è un enumeratore per il tipo "$1"
% This means that compiler cannot find an apropriate enumerator to use in the for-in loop.
% To create an enumerator you need to defind an operator enumerator or add a public or published 
% GetEnumerator method to the class or object definition.
sym_e_no_enumerator_move=05068_E_Non c'è un metodo "MoveNext" nell'enumeratore "$1"
% This means that compiler cannot find a public MoveNext method with the Boolean return type in
% the enumerator class or object definition.
sym_e_no_enumerator_current=05069_E_Non c'è una proprietà "Current" nell'enumeratore "$1"
% This means that compiler cannot find a public Current property in  the enumerator class or object
% definition.
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
cg_e_parasize_too_big=06009_E_La lista dei parametri è più grande di 65535 bytes
% The I386 processor limits the parameter list to 65535 bytes. (The \var{RET}
% instruction causes this.)
cg_e_file_must_call_by_reference=06012_E_I tipi file devono essere parametri var
% You cannot specify files as value parameters, i.e., they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_Usare un puntatore far non è permesso qui
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_dont_call_exported_direct=06015_E_Le funzioni dichiarare EXPORT non possono essere chiamate
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_Possibile chiamata illegale di un costruttore o distruttore
% The compiler detected that a constructor or destructor is called within a
% a method. This will probably lead to problems, since constructors / destructors
% require parameters on entry.
cg_n_inefficient_code=06017_N_Codice inefficiente
% Your statement seems dubious to the compiler.
cg_w_unreachable_code=06018_W_Codice irraggiungibile
% You specified a construct which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_cant_call_abstract_method=06020_E_I metodi astratti non possono essere chiamati direttamente
% You cannot call an abstract method directly. Instead, you must call an
% overriding child method, because an abstract method isn't implemented.
cg_d_register_weight=06027_DL_Il Registro $1 pesa $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_d_stackframe_omited=06029_DL_Stack frame omesso
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_e_unable_inline_object_methods=06031_E_I metodi di oggetti e classi non possono essere inline
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_Chiamate procvar non possono essere inline
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_Non è stato memorizzato codice per la procedura inline
% The compiler couldn't store code for the inline procedure.
cg_e_can_access_element_zero=06035_E_Non si può accedere all'elemento zero di una ansi/wide/longstring, usare (set)length
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such string type.
cg_e_cannot_call_cons_dest_inside_with=06037_E_Non si possono chiamare costruttoro o distruttori all'interno di una clausola 'with'
% Inside a \var{with} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_I metodi gestori di messaggi non possono essere chiamati direttamente
% A message method handler method cannot be called directly if it contains an
% explicit \var{Self} argument.
cg_e_goto_inout_of_exception_block=06039_E_Non è permesso saltare da dentro a fuori un blocco di gestione delle eccezioni o viceversa
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
cg_e_control_flow_outside_finally=06040_E_Le istruzioni 'break', 'continue' ed 'exit' non sono ammesse in un blocco finally
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
cg_w_parasize_too_big=06041_W_La dimensione dei parametri eccede il limite massimo di alcune CPU
% This indicates that you are declaring more than 64K of parameters, which
% might not be supported on other processor targets.
cg_w_localsize_too_big=06042_W_La dimensione delle variabili locali eccede il limite massimo di alcune CPU
% This indicates that you are declaring more than 32K of local variables, which
% might not be supported on other processor targets.
cg_e_localsize_too_big=06043_E_La dimensione delle variabili locali è eccessiva
% This indicates that you are declaring more than 32K of local variables, which
% is not supported by this processor.
cg_e_break_not_allowed=06044_E_BREAK non è permesso fuori da un loop
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06045_E_CONTINUE non è permesso fuori da un loop
% You're trying to use \var{continue} outside a loop construction.
cg_f_unknown_compilerproc=06046_F_Compilerproc "$1" sconosciuta. Verificate di stare usando la giusta libreria di runtime
% The compiler expects that the runtime library contains certain subroutines. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a subroutine which the compiler needs for internal use.
cg_f_unknown_system_type=06047_F_Il tipo di sistema "$1" non esiste. Verificate di stare usando la giusta libreria di runtime.
% The compiler expects that the runtime library contains certain type definitions. If you see this error
% and you didn't change the runtime library code, it's very likely that the runtime library
% you're using doesn't match the compiler in use. If you changed the runtime library this error means
% that you removed a type which the compiler needs for internal use.
cg_h_inherited_ignored=06048_H_Ignorata una chiamata ereditata a un metodo astratto
% This message appears only in Delphi mode when you call an abstract method
% of a parent class via \var{inherited;}. The call is then ignored.
cg_e_goto_label_not_found=06049_E_La label goto "$1" non è definita o è stata eliminata dalle ottimizzazioni
% The label used in the goto definition is not defined or optimized away by the
% unreachable code elemination.
% \end{description}
# EndOfTeX

#
# Assembler reader
#
# 07107 is the last used one
#
asmr_d_start_reading=07000_DL_Inizio analisi assembler stile $1
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_DL_Fine analisi assembler stile $1
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_Il pattern contiene @ ma non è una label
% A identifier which isn't a label can't contain a @.
asmr_e_building_record_offset=07004_E_Errore nella costruzione dell'offset di un record
% There has an error occured while building the offset of a record/object
% structure, this can happend when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_usato OFFSET senza identificatore
% You can only use OFFSET with an identifier. Other syntaxes aren't
% supported
asmr_e_type_without_identifier=07006_E_usato TYPE senza identificatore
% You can only use TYPE with an identifier. Other syntaxes aren't
% supported
asmr_e_no_local_or_para_allowed=07007_E_Qui non si possono usare variabili o parametri locali
% You can't use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the frame pointer register so the
% address can't be obtained directly.
asmr_e_need_offset=07008_E_Qui è necessario usare OFFSET
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_Qui è necessario usare $
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Si può usare un solo simbolo rilocabile per ogni argomento
% You can't have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_Un simbolo rilocabile puà solo essere sommato
% Relocatable symbols (variable/typed constant) can't be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_Espressione costante non valida
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_I simboli rilocabili non sono permessi qui
% You can't use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_Sintassi del reference non valida
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_Impossibile raggiungere $1 da quel codice
% You cannot read directly the value of a local variable or parameter
% of a higher level procedure in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_Simboli o label locali non sono ammessi come reference
% You can't use local symbols/labels as references
asmr_e_wrong_base_index=07017_E_Uso errato dei registri base e indice
% There is an error with the base and index register, they are
% probably incorrect
asmr_w_possible_object_field_bug=07018_W_Possibile errore nella gestione dei campi di un oggetto
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_Specificato un fattore di scala errato
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_Uso di registri indice multipli
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_Tipo di operando non valido
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_Stringa non valida come operando dell'opcode: $1
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE e @DATA non supportati
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_Non sono ammessi riferimenti a label nulle
asmr_e_expr_zero_divide=07025_E_Divisione per zero in una espressione asm costante
% There is a division by zero in a constant expression
asmr_e_expr_illegal=07026_E_Espressione illegale
% There is an illegal expression in a constant expression
asmr_e_escape_seq_ignored=07027_E_Ignorata sequenza di escape: $1
% There is a C-styled string, but the escape sequence in the string
% is unknown, and is therefore ignored
asmr_e_invalid_symbol_ref=07028_E_Riferimento a simbolo non valido
asmr_w_fwait_emu_prob=07029_W_Fwait può creare problemi di emulazione con emu387
asmr_w_fadd_to_faddp=07030_W_$1 senza operandi tradotto in $1P
asmr_w_enter_not_supported_by_linux=07031_W_L'istruzione ENTER non è supportata dal kernel Linux
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_Chiamata da assembler di una funzione overloaded
% There is a call to an overloaded method in the assembler block,
% this might be the sign there is a problem
asmr_e_unsupported_symbol_type=07033_E_Tipo di simbolo non supportato per l'operando
asmr_e_constant_out_of_bounds=07034_E_Costante fuori dai limiti
asmr_e_error_converting_decimal=07035_E_Errore nella conversione decimale $1
% A constant decimal value does not have the correct syntax
asmr_e_error_converting_octal=07036_E_Errore nella conversione ottale $1
% A constant octal value does not have the correct syntax
asmr_e_error_converting_binary=07037_E_Errore nella conversione binaria $1
% A constant binary value does not have the correct syntax
asmr_e_error_converting_hexadecimal=07038_E_Errore nella conversione esadecimale $1
% A constant hexadecimal value does not have the correct syntax
asmr_h_direct_global_to_mangled=07039_H_$1 tradotto con $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 è associato a una funzione overloaded
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Non si può usare SELF al di fuori di un metodo
% There is a reference to the \var{self} symbol while it is not
% allowed. \var{self} can only be referenced inside methods
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Non si può usare OLDEBP al di fuori di una procedura nidificata
% There is a reference to the \var{oldebp} symbol while it is not
% allowed. \var{oldebp} can only be referenced inside nested routines
asmr_e_void_function=07043_W_Le procedure non possono ritornare alcun valore nel codice assembler
% Trying to return a value while in a procedure. A procedure
% does not have any return value
asmr_e_SEG_not_supported=07044_E_SEG non supportato
asmr_e_size_suffix_and_dest_dont_match=07045_E_Il suffisso di dimensione e la dimensione della destinazione non corrispondono
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_w_size_suffix_and_dest_dont_match=07046_W_Il suffisso di dimensione e la dimensione della destinazione non corrispondono
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_e_syntax_error=07047_E_Errore di sintassi nell'assembler
% There is an assembler syntax error
asmr_e_invalid_opcode_and_operand=07048_E_Combinazione di opcode e operandi non valida
% The opcode cannot be used with this type of operand
asmr_e_syn_operand=07049_E_Errore di sintassi nell'operando assembler
asmr_e_syn_constant=07050_E_Errore di sintassi nella costante assembler
asmr_e_invalid_string_expression=07051_E_Espressione stringa non valida
asmr_w_const32bit_for_address=07052_W_costante con simbolo di indirizzo $1 che non entra in un puntatore
% A constant expression represents an address which does not fit
% into a pointer. The address is probably incorrect
asmr_e_unknown_opcode=07053_E_Opcode $1 non riconosciuto
% This opcode is not known
asmr_e_invalid_or_missing_opcode=07054_E_Opcode non valido o mancante
asmr_e_invalid_prefix_and_opcode=07055_E_Combinazione di prefisso e opcode: $1
asmr_e_invalid_override_and_opcode=07056_E_Combinazione di override e opcode $1 non valida
asmr_e_too_many_operands=07057_E_Troppi operandi su una sola riga
% There are too many operands for this opcode. Check your
% assembler syntax
asmr_w_near_ignored=07058_W_NEAR ignorato
asmr_w_far_ignored=07059_W_FAR ignorato
asmr_e_dup_local_sym=07060_E_Simbolo locale $1 ignorato
asmr_e_unknown_local_sym=07061_E_Simbolo locale $1 non definito
asmr_e_unknown_label_identifier=07062_E_Identificatore label $1 non riconosciuto
asmr_e_invalid_register=07063_E_Nome di registro non valido
% There is an unknown register name used as operand.
asmr_e_invalid_fpu_register=07064_E_Nome di registro virgola mobile non valido
% There is an unknown register name used as operand.
asmr_w_modulo_not_supported=07066_W_Modulo non supportato
asmr_e_invalid_float_const=07067_E_Costante in virgola mobile $1 non valida
% The floating point constant declared in an assembler block is
% invalid.
asmr_e_invalid_float_expr=07068_E_Espressione in virgola mobile non valida
% The floating point expression declared in an assembler block is
% invalid.
asmr_e_wrong_sym_type=07069_E_Tipo di simbolo errato
asmr_e_cannot_index_relative_var=07070_E_Non si possono indicizzare una variabile o un parametro locali con un registro
% Trying to index using a base register a symbol which is already relative
% to a register. This is not possible, and will probably lead to crashes.
asmr_e_invalid_seg_override=07071_E_Espressione di override di segmento non valida
asmr_w_id_supposed_external=07072_W_L'identificatore $1 non è definito: sarà considerato esterno
% There is a reference to an undefined symbol. This will not result
% in an error, since the symbol might be external, but may cause
% problems at link time if the symbol is not defined anywhere.
asmr_e_string_not_allowed_as_const=07073_E_Le stringhe non sono ammesse come costanti
% Character strings are not allowed as constants.
asmr_e_no_var_type_specified=07074_Nessun tipo di variabile specificato
% The syntax expects a type identifier after the dot, but
% none was found.
asmr_w_assembler_code_not_returned_to_text=07075_E_Il codice assembler non è tornato nella sezione text
% There was a directive in the assembler block to change sections,
% but there is a missing return to the text section at the end
% of the assembler block. This might cause errors during link time.
asmr_e_not_directive_or_local_symbol=07076_E_$1 non è una direttiva né un simbolo locale
% This symbol is unknown.
asmr_w_using_defined_as_local=07077_E_Uso di un nome definito come label locale
asmr_e_dollar_without_identifier=07078_E_Il token '$' è usato senza identificatori
% A constant expression has an identifier which does not start with
% the $ symbol.
asmr_w_32bit_const_for_address=07079_W_Creata una costante a 32 bit per l'indirizzo
% A constant was used as an address. This is probably an error,
% since using absolute addresses will probably not work.
asmr_n_align_is_target_specific=07080_N_.align è specifico per la destinazione: usare .balign oppure .p2align
% Using the .align directive is platform specific, and its meaning will vary
% from one platform to another.
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Non si può accedere direttamente dal parametro ai campi
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_Non si può accedere direttamente dalla classe/oggetto ai campi
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
asmr_e_unable_to_determine_reference_size=07083_E_Dimensione degli operandi non specificata ed impossibile da determinare
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference.
asmr_e_cannot_use_RESULT_here=07084_E_Non si può usare RESULT in questa funzione
% Some functions which return complex types cannot use the \var{result}
% keyword.
asmr_w_adding_explicit_args_fXX=07086_W_"$1" senza operandi tradotto con "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" tradotto con "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" tradotto con "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_Il carattere '<' non è permesso qui
% The shift operator requires the << characters. Only one
% of those characters was found.
asmr_e_invalid_char_greater=07090_E_Il carattere '>' non è permesso qui
% The shift operator requires the >> characters. Only one
% of those characters was found.
asmr_w_align_not_supported=07093_W_ALIGN non è supportato
asmr_e_no_inc_and_dec_together=07094_E_Non si possono usare Inc e Dec insieme
% Trying to use an increment and a decrement within the same
% opcode on the 680x0. This is impossible.
asmr_e_invalid_reg_list_in_movem=07095_E_La reglist per movem non è valida
% Trying to use the \var{movem} opcode with invalid registers
% to save or restore.
asmr_e_invalid_reg_list_for_opcode=07096_E_La reglist per l'opcode non è valida
asmr_e_higher_cpu_mode_required=07097_E_Necessaria una classe di cpu più alta ($1)
% Trying to use an instruction which is not supported in the current
% cpu mode. Use a higher cpu generation to be able to use this
% opcode in your assembler block
asmr_w_unable_to_determine_reference_size_using_dword=07098_W_Dimensione degli operandi non specificata ed impossibile da determinare, assunta DWORD come default
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use DWORD as default.
asmr_e_illegal_shifterop_syntax=07099_E_Errore di sintassi nell'analisi di un operando shifter
% ARM only; ARM assembler supports a so called shifter operand. The used syntax isn't
% a valid shifter operand. Example for an operation with shifter operand:
% \begin{verbatim}
% asm
%   orr     r2,r2,r2,lsl #8
% end;
% \end{verbatim}
asmr_e_packed_element=07100_E_Indirizzo di componenti packed non allineato al byte
% Packed components (record fields and array elements) may start at an arbitrary
% bit inside a byte. On CPU which do not support bit-addressable memory (which
% includes all currently supported CPUs by FPC) you will therefore get an error
% message when trying to index arrays with elements whose size is not a multiple
% of 8 bits. The same goes for accessing record fields with such an address.
% multiple of 8 bits.
asmr_w_unable_to_determine_reference_size_using_byte=07101_W_Dimensione degli operandi non specificata ed impossibile da determinare, assunta BYTE come default
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use BYTE as default.
asmr_w_no_direct_ebp_for_parameter=07102_W_Qui non si può usare +offset(%ebp) per i parametri
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_for_parameter_regcall=07103_W_Usare +offset(%ebp) non è compatibile con la convenzione di chiamata regcall
% Using direct 8(%ebp) reference for function/procedure parameters is invalid
% if parameters are in registers.
asmr_w_direct_ebp_neg_offset=07104_W_Usare -offset(%ebp) non è consigliato per accedere a variabili locali
% Using -8(%ebp) to access a local variable is not recommended
asmr_w_direct_esp_neg_offset=07105_W_Usando -offset(%esp), l'accesso può causare un crash o perdite di dati
% Using -8(%esp) to access a local stack is not recommended, as
% this stack portion can be overwritten by any function calls or interrupts.
asmr_e_no_vmtoffset_possible=07106_E_VMTOffset deve essere usato in combinazione con un metodo virtuale, ma "$1" non lo è
% Only virtual methods have VMT offsets
asmr_e_need_pic_ref=07107_E_Non si può generare PIC: ci sono reference dipendenti dalla posizione
% The compiler has been configured to generate position-independent code
% (PIC), but there are position-dependent references in the current
% handwritten assembler instruction.
#
# Assembler/binary writers
#
# 08020 is the last used one
#
asmw_f_too_many_asm_files=08000_F_Troppi file assembler
% With smartlinking enabled, there are too many assembler
% files generated. Disable smartlinking.
asmw_f_assembler_output_not_supported=08001_F_L'assembler di output selezionato non è supportato
asmw_f_comp_not_supported=08002_F_Comp not supported
asmw_f_direct_not_supported=08003_F_Modo diretto non supportato perscrittori binari
% Direct assembler mode is not supported for binary writers.
asmw_e_alloc_data_only_in_bss=08004_E_Si possono allocare dati solo nella sezione bss
asmw_f_no_binary_writer_selected=08005_F_Nessuno scrittore binario selezionato
asmw_e_opcode_not_in_table=08006_E_Asm: L'opcode $1 non è in tabella
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 combinazione di opcode e operandi non valida
asmw_e_16bit_not_supported=08008_E_Asm: riferimenti a 16 Bit non sono supportati
asmw_e_invalid_effective_address=08009_E_Asm: indirizzo effettivo non valido
asmw_e_immediate_or_reference_expected=08010_E_Asm: era previsto un immediate o un reference
asmw_e_value_exceeds_bounds=08011_E_Asm: valore $1 fuori dal limite $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: lo short jump è fuori dal limite $1
asmw_e_undefined_label=08013_E_Asm: Label $1 non definita
asmw_e_comp_not_supported=08014_E_Asm: il tipo comp non è supportato per questa destinazione
asmw_e_extended_not_supported=08015_E_Asm: il tipo extended non è supportato per questa destinazione
asmw_e_duplicate_label=08016_E_Asm: label $1 duplicata
asmw_e_redefined_label=08017_E_Asm: label $1 ridefinita
asmw_e_first_defined_label=08018_E_Asm: definito qui per la prima volta
asmw_e_invalid_register=08019_E_Asm: registro $1 non valido
asmw_e_16bit_32bit_not_supported=08020_E_Asm: riferimenti a 16 o a 32 Bit non supportati
asmw_e_64bit_not_supported=08021_E_Asm: operandi a 64 Bit non supportati

#
# Executing linker/assembler
#
# 09032 is the last used one
#
# BeginOfTeX
%
% \section{Errors of assembling/linking stage}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
exec_w_source_os_redefined=09000_W_Sistema operativo sorgente ridefinito
% The source operating system is redefined.
exec_i_assembling_pipe=09001_I_Uso del'assemblatore $1  (tramite pipe)
% Assembling using a pipe to an external assembler.
exec_d_cant_create_asmfile=09002_E_Non posso creare il file assembler $1
% The mentioned file can't be created. Check if you have
% access permissions to create this file.
exec_e_cant_create_objectfile=09003_E_Non posso creare il file oggetto: $1
% The mentioned file can't be created. Check if you have
% got access permissions to create this file.
exec_e_cant_create_archivefile=09004_E_Non posso creare il file archivio: $1
% The mentioned file can't be created. Check if you have
% access permissions to create this file.
exec_e_assembler_not_found=09005_E_Assemblatore $1 non trovato, sarà usato un assemblatore esterno
% The assembler program was not found. The compiler will produce a script that
% can be used to assemble and link the program.
exec_t_using_assembler=09006_T_Assemblatore in uso: $1
% An informational message saying which assembler is being used.
exec_e_error_while_assembling=09007_E_Errore nell'assembaggio dell'exitcode $1
% There was an error while assembling the file using an external assembler.
% Consult the documentation of the assembler tool to find out more information
% on this error.
exec_e_cant_call_assembler=09008_E_Non è possibile chiamare l'assemblatore, errore $1: sarà usato un assemblatore esterno
% An error occurred when calling an external assembler. The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_assembling=09009_I_Assemblaggio di $1
% An informational message stating which file is being assembled.
exec_i_assembling_smart=09010_I_Assemblaggio con smartlinking di $1
% An informational message stating which file is being assembled using smartlinking.
exec_w_objfile_not_found=09011_W_Oggetto $1 non trovato, il linking potrebbe fallire!
% One of the object files is missing, and linking will probably fail.
% Check your paths.
exec_w_libfile_not_found=09012_W_Libreria $1 non trovata, il linking potrebbe fallire!
% One of the library files is missing, and linking will probably fail.
% Check your paths.
exec_e_error_while_linking=09013_E_Errore durante il linking
% Generic error while linking.
exec_e_cant_call_linker=09014_E_Impossibile chiamare il linker, sarà usato un linker esterno
% An error occurred when calling an external linker. The compiler will produce a script that
% can be used to assemble and link the program.
exec_i_linking=09015_I_Linking di $1
% An informational message, showing which program or library is being linked.
exec_e_util_not_found=09016_E_Utility $1 non trovata, sarà usato un linker esterno
% An external tool was not found. The compiler will produce a script that
% can be used to assemble and link or postprocess the program.
exec_t_using_util=09017_T_Uso di utility $1
% An informational message, showing which external program (usually a postprocessor) is being used.
exec_e_exe_not_supported=09018_E_La creazione di eseguibili non è supportata
% Creating executable programs is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_e_dll_not_supported=09019_E_La creazione di librerie dinamiche o condivise non è supportata
% Creating dynamically loadable libraries is not supported for this platform, because it was
% not yet implemented in the compiler.
exec_i_closing_script=09020_I_Chiusura dello script $1
% Informational message showing when writing of the external assembling and linking script is finished.
exec_e_res_not_found=09021_E_Compilatore di risorse "$1" non trovato, creazione di uno script esterno
% An external resource compiler was not found. The compiler will produce a script that
% can be used to assemble, compile resources and link or postprocess the program.
exec_i_compilingresource=09022_I_Compilazione della risorsa $1
% An informational message, showing which resource is being compiled.
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_La unit $1 non si può linkare staticamente, sarà usato lo smart linking
% Static linking was requested, but a unit which is not statically linkable was used.
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_La unit $1 deve essere linkata staticamente
% Smart linking was requested, but a unit which is not smart-linkable was used.
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_La unit $1 non si può linkare in modo shared o smart
% Shared linking was requested, but a unit which is not shared-linkable was used.
exec_e_unit_not_smart_or_static_linkable=09026_E_La unit $1 deve essere linkata in modo shared
% Smart or static linking was requested, but a unit which cannot be used for either  was used.
exec_e_unit_not_shared_or_static_linkable=09027_E_La unit $1 non si può linkare in modo shared o statico
% Shared or static linking was requested, but a unit which cannot be used for either  was used.
exec_d_resbin_params=09028_D_Chiamata al compilatore di risorse "$1" con riga di comando "$2"
% An informational message showing which command line is used for the resource compiler.
exec_e_error_while_compiling_resources=09029_E_Errore nella compilazione delle risorse
% The resource compiler or converter returned an error.
exec_e_cant_call_resource_compiler=09030_E_Impossibile chiamare il compilatore di risorse "$1", creazione di uno script esterno
% An error occurred when calling a resource compiler. The compiler will produce
% a script that can be used to assemble, compile resources and link or
% postprocess the program.
exec_e_cant_open_resource_file=09031_E_Impossibile aprire il file di risorse "$1"
% An error occurred resource file cannot be opened.
exec_e_cant_write_resource_file=09032_E_Impossibile scrivere il file di risorse "$1"
% An error occurred resource file cannot be written.
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
execinfo_f_cant_process_executable=09128_F_Impossibile postprocessare l'eseguibile $1
% Fatal error when the compiler is unable to post-process an executable.
execinfo_f_cant_open_executable=09129_F_Impossibile aprire l'eseguibile $1
% Fatal error when the compiler cannot open the file for the executable.
execinfo_x_codesize=09130_X_Dimnesione del codice: $1 byte
% Informational message showing the size of the produced code section.
execinfo_x_initdatasize=09131_X_Dimensione dei dati inizializzati: $1 byte
% Informational message showing the size of the initialized data section.
execinfo_x_uninitdatasize=09132_X_Dimensione dei dati non inizializzati: $1 byte
% Informational message showing the size of the uninitialized data section.
execinfo_x_stackreserve=09133_X_Spazio di stack riservato: $1 byte
% Informational message showing the stack size that the compiler reserved for the executable.
execinfo_x_stackcommit=09134_X_Spazio di stack impegnato: $1 byte
% Informational message showing the stack size that the compiler committed for the executable.
%\end{description}
# EndOfTeX

#
# Internal linker messages
#
# 09200 is the last used one
#
# BeginOfTeX
% \section{Linker messages}
% This section lists messages produced by internal linker.
% \begin{description}
link_f_executable_too_big=09200_F_L'immagine dell'eseguibile è troppo grande per la destinazione $1.
% Fatal error when resulting executable is too big.
link_w_32bit_absolute_reloc=09201_W_Il file oggetto "$1" contiene rilocazioni assolute a 32-bit al simbolo "$2".
% Warning when 64-bit object file contains 32-bit absolute relocations.
% In such case an executable image can be loaded into lower 4Gb of
% address space only.
%\end{description}
# EndOfTeX

#
# Unit loading
#
# 10061 is the last used one
#
# BeginOfTeX
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these messages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_Ricerca della unit: $1
% When you use the \var{-vt} option, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_Caricamento del PPU $1
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_Nome del PPU: $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_Flag del PPU: $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_Crc del PPU: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_Ora del PPU: $1
% When you use the \var{-vu} flag, the time the unit was compiled is shown.
unit_u_ppu_file_too_short=10006_U_File PPU troppo corto
% The ppufile is too short, not all declarations are present.
unit_u_ppu_invalid_header=10007_U_Header del PPU non valido (non c'è PPU all'inizio)
% A unit file contains as the first three bytes the ASCII codes of the characters \var{PPU}.
unit_u_ppu_invalid_version=10008_U_Versione del PPU $1 non valida
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_Il PPU è compilato per un altro processore
% This unit file was compiled for a different processor type, and
% cannot be read.
unit_u_ppu_invalid_target=10010_U_Il PPU è compilato per un'altra destinazione
% This unit file was compiled for a different target, and
% cannot be read.
unit_u_ppu_source=10011_U_Sorgente del PPU: $1
% When you use the \var{-vu} flag, the unit source file name is shown.
unit_u_ppu_write=10012_U_Scrittura di $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_Impossibile scrivere il file PPU
% An error occurred when writing the unit file.
unit_f_ppu_read_error=10014_F_Impossibile leggere il file PPU
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_read_unexpected_end=10015_F_Fine del file PPU inaspettata
% Unexpected end of file. This may mean that the PPU file is
% corrupted.
unit_f_ppu_invalid_entry=10016_F_Voce del file PPU non valida: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_Problemi nel conteggi Dbx nel PPU
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_Nome della unit $1 illegale
% The name of the unit does not match the file name.
unit_f_too_much_units=10019_F_Troppe unit 
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{fmodule.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_Riferimento circolare fra le unit $1 e $2
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_Impossibile compilare la unit $1, i sorgenti non sono disponibili
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_Impossibile trovare la unit unit $1 usata da $2
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your configuration file for the unit paths.
unit_w_unit_name_error=10023_W_La unit $1 non è stata trovata ma la $2 esiste
% This error message is no longer used.
unit_f_unit_name_error=10024_F_Cercata la unit $1 ma trovata la $2
% \dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_E' necessario lo switch -Us per compilare la unit 'system'
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} switch must be specified.
unit_f_errors_in_unit=10026_F_Riscontrati $1 errori nella compilazione del modulo: terminata.
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_Caricamento da $1 ($2) unit $3
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_Ricompilazione di $1, il checksum di $2 è cambiato
% The unit is recompiled because the checksum of a unit it depends on has
% changed.
unit_u_recompile_source_found_alone=10029_U_Ricompilazione di $1, trovato solo il sorgente
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_La libreria statica è più vecchia del file PPU: ricompilo la unit
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit is older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_La libreria condivisa è più vecchia del file PPU: ricompilo la unit
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit is older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_i file obj e asm sono più vecchi del file PPU: ricompilo la unit
% When you use the \var{-vu} flag, the compiler warns if the assembler or
% object file of the unit is older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_Il file obj è più vecchio del file asm: ricompilo la unit
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_parsing_interface=10034_U_Analisi dell'interfaccia di $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit.
unit_u_parsing_implementation=10035_U_Analisi dell'implementazione di $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit.
unit_u_second_load_unit=10036_U_Secondo caricamento della unit $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happen with
% interdependent units.
unit_u_check_time=10037_U_PPU Controllo del file $1 ora $2
% When you use the \var{-vu} flag, the compiler shows the filename and
% date and time of the file on which a recompile depends.
### The following two error msgs is currently disabled.
#unit_h_cond_not_set_in_last_compile=10038_H_Il condizionale $1 non era settato all'inizio dell'ultima compilazione di $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that currently is defined, but was not used the last
#% time the unit was compiled.
#unit_h_cond_set_in_last_compile=10039_H_Il condizionale $1 era settato all'inizio dell'ultima compilazione di $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that was used the last time the unit was compiled, but
#% the conditional is currently not defined.
unit_w_cant_compile_unit_with_changed_incfile=10040_W_Impossibile ricompilare la unit $1, ma sono stati trovati file include modificati
% A unit was found to have modified include files, but
% some source files were not found, so recompilation is impossible.
unit_u_source_modified=10041_U_Il file $1 è più recente del file PPU $2
% A modified source file for a compiler unit was found.
unit_u_ppu_invalid_fpumode=10042_U_Tentativo di usare una unit compilata con un modo FPU diverso
% Trying to compile code while using units which were not compiled with
% the same floating point format mode. Either all code should be compiled
% with FPU emulation on, or with FPU emulation off.
unit_u_loading_interface_units=10043_U_Caricamento delle unit dall'interfaccia di $1
% When you use the \var{-vu} flag, the compiler warns that it is starting
% to load the units defined in the interface part of the unit.
unit_u_loading_implementation_units=10044_U_Caricamento delle unit dall'implementazione di $1
% When you use the \var{-vu} flag, the compiler warns that it is starting
% to load the units defined in the implementation part of the unit.
unit_u_interface_crc_changed=10045_U_Il CRC dell'interfaccia per la unit $1 è cambiato
% When you use the \var{-vu} flag, the compiler warns that the
% CRC calculated for the interface has been changed after the implementation
% has been parsed.
unit_u_implementation_crc_changed=10046_U_Il CRC dell'implementazione per la unit $1 è cambiato
% When you use the \var{-vu} flag, the compiler warns that the
% CRC calculated has been changed after the implementation
% has been parsed.
unit_u_finished_compiling=10047_U_Compilazione della unit $1 terminata
% When you use the \var{-vu} flag, the compiler warns that it
% has finished compiling the unit.
unit_u_add_depend_to=10048_U_Aggiunta dipendenza da $1 a $2
% When you use the \var{-vu} flag, the compiler warns that it
% has added a dependency between the two units.
unit_u_no_reload_is_caller=10049_U_Nessun ricaricamento, chiamante: $1
% When you use the \var{-vu} flag, the compiler warns that it
% will not reload the unit because it is the unit that wants
% to load this unit.
unit_u_no_reload_in_second_compile=10050_U_Nessun ricaricamento, già in ricompilazione: $1
% When you use the \var{-vu} flag, the compiler warns that it
% will not reload the unit because it is already in a second recompile.
unit_u_flag_for_reload=10051_U_Flag per il ricaricamento: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to reload the unit.
unit_u_forced_reload=10052_U_Ricaricamento forzato
% When you use the \var{-vu} flag, the compiler warns that it
% is reloading the unit because it was required.
unit_u_previous_state=10053_U_Stato precedente di $1: $2
% When you use the \var{-vu} flag, the compiler shows the
% previous state of the unit.
unit_u_second_compile_unit=10054_U_Unit $1 già in fase di compilazione, sarà compilata una seconda volta
% When you use the \var{-vu} flag, the compiler warns that it is starting
% to recompile a unit for the second time. This can happen with interdependent
% units.
unit_u_loading_unit=10055_U_Caricamento della unit $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the unit.
unit_u_finished_loading_unit=10056_U_Finito di caricare la unit $1
% When you use the \var{-vu} flag, the compiler warns that it finished
% loading the unit.
unit_u_registering_new_unit=10057_U_Registrazione della nuova unit $1
% When you use the \var{-vu} flag, the compiler warns that it has
% found a new unit and is registering it in the internal lists.
unit_u_reresolving_unit=10058_U_Ri-risoluzione della unit $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to recalculate the internal data of the unit.
unit_u_skipping_reresolving_unit=10059_U_Abortita ri-risoluzione della unit $1, ancora caricamento delle unit usate
% When you use the \var{-vu} flag, the compiler warns that it is
% skipping the recalculation of the internal data of the unit
% because there is no data to recalculate.
unit_u_unload_resunit=10060_U_Unit di risorse $1 scaricata perché non necessaria
% When you use the \var{-vu} flag, the compiler warns that it is unloading the
% resource handling unit, since no resources are used.
unit_e_different_wpo_file=10061_E_La unit $1 è compilata con un diverso feedback di ottimizzazione globale ($2, $3); ricompilarla senza wpo o usare lo stesso file input wpo per questa compilazione
% When a unit has been compiled using a particular ottimizzazione globale (wpo) feedback file (\var{-FW<x>} \var{-OW<x>}),
% this compiled version of the unit is specialised for that particular compilation scenario and cannot be used in
% any other context. It has to be recompiled before you can use it in another program or with another wpo feedback input file.
% \end{description}
# EndOfTeX

#
#  Options
#
# 11047 is the last used one
#
option_usage=11000_O_$1 [options] <inputfile> [options]
# BeginOfTeX
%
% \section{Command line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_only_one_source_support=11001_W_Supportato un solo file sorgente, cambio il file sorgente da compilare da "$1" in "$2"
% You can specify only one source file on the command line. The last
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_I file DEF si possono creare solo per OS/2
% This option can only be specified when you're compiling for OS/2.
option_no_nested_response_file=11003_E_I file di risposta nidificati non sono supportati
% You cannot nest response files with the \var{@file} command line option.
option_no_source_found=11004_F_Nella riga di comando manca il nome del file sorgente
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_Non ci sono opzioni nel file di configurazione $1
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_Parametro $1 illegale
% You specified an unknown option.
option_help_pages_para=11007_H_-? fornisce le pagine di aiuto
% When an unknown option is given, this message is displayed.
option_too_many_cfg_files=11008_F_Troppi file di configurazione nidificati
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_Impossibile aprire il file $1
% The option file cannot be found.
option_reading_further_from=11010_D_Lettura di ulteriori opzioni da $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_Destinazione già impostata a: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_Le librerie condivise non sono supportate sotto DOS: passaggio a librerie statiche
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos.
option_too_many_ifdef=11013_F_Nel file opzioni $1 alla riga $2 ci sono troppi \var{\#IF(N)DEFs}
% The \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_Nel file opzioni $1 alla riga $2 c'è un \var{\#ENDIFs} che non dovrebbe esserci
% The \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_Condizionale aperto alla fine del file di opzioni
% The \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_La generazione delle informazioni di debug non è supportata da questo eseguibile
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_Provate a ricompilare il compilatore con -dGDB
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_W_State usando lo switch $1 che è obsoleto
% This warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the meaning of the switch may change.
option_obsolete_switch_use_new=11019_W_State usando lo switch $1, che è obsoleto: si prega usare $2 al suo posto
% This warns you when you use a switch that is not supported anymore. You
% must now use the second switch instead.
% It is recommended that you change the switch to overcome problems in the
% future, when the meaning of the switch may change.
option_switch_bin_to_src_assembler=11020_N_Cambio dell'assemblatore a quello di scrittora dei sorgenti di default
% This notifies you that the assembler has been changed because you used the
% -a switch, which can't be used with a binary assembler writer.
option_incompatible_asm=11021_W_L'output assembler selezionato "$1" non è compatibile con "$2"
option_asm_forced=11022_W_Forzato l'uso dell'assemblatore "$1"
% The assembler output selected cannot generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
option_using_file=11026_T_Lettura delle opzioni dal file $1
% Options are also read from this file.
option_using_env=11027_T_Lettura dell opzioni dall'ambiente $1
% Options are also read from this environment string.
option_handling_option=11028_D_Gestione dell'opzione "$1"
% Debug info that an option is found and will be handled.
option_help_press_enter=11029_O_*** premere invio ***
% Message shown when help is shown page per page. When pressing the ENTER
% Key, the next page of help is shown. If you press q and then ENTER, the
% compiler exits.
option_start_reading_configfile=11030_H_Inizio lettura del file di configurazione $1
% Start of configuration file parsing.
option_end_reading_configfile=11031_H_Fine lettura del file di configurazione $1
% End of configuration file parsing.
option_interpreting_option=11032_D_Interpretando l'opzione "$1"
% The compiler is interpreting an option
option_interpreting_firstpass_option=11036_D_Interpretando l'opzione  "$1" (primo passo)
% The compiler is interpreting an option for the first time.
option_interpreting_file_option=11033_D_Interpretando l'opzione da file "$1"
% The compiler is interpreting an option which it read from the configuration file.
option_read_config_file=11034_D_Lettura del file di configurazione "$1"
% The compiler is starting to read the configuration file.
option_found_file=11035_D_trovato nome di file sorgente "$1"
% Additional information about options.
% Displayed when you have the debug option turned on.
option_code_page_not_available=11039_E_Codepage (lingua del sorgente) sconosciuta
% An unknown code page for the source files was requested.
% The compiler is compiled with support for several code pages built-in.
% The requested code page is not in that list. You will need to recompile
% the compiler with support for the codepage you need.
option_config_is_dir=11040_F_Il file di configurazione $1 è una directory
% Directories cannot be used as configuration files.
option_confict_asm_debug=11041_W_L'assemblatore selezionato "$1" non può gneerare informazioni di debug: debugging disabilitato
% The selected assembler output cannot generate
% debugging information, debugging option is therefore disabled.
option_ppc386_deprecated=11042_W_L'uso di ppc386.cfg è deprecato, prego usate fpc.cfg al suo posto
% Using ppc386.cfg is still supported for historical reasons, however, for a multiplatform
% system the naming makes no sense anymore. Please continue to use fpc.cfg instead.
option_else_without_if=11043_F_Nel file opzioni $1 alla riga $2 c'è una direttiva \var{\#ELSE} senza il \var{\#IF(N)DEF} corrispondente
% An \var{\#ELSE} statement was found in the options file without a matching \var{\#IF(N)DEF} statement.
option_unsupported_target=11044_F_L'opzione "$1" non è supportata sulla destinazione corrente
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is incompatible with the currently selected target platform.
option_unsupported_target_for_feature=11045_F_La capacità "$1" non è supportata sulla destinazione corrente
% Not all features are supported or implemented for all target platforms. This message informs you that a chosen
% feature is incompatible with the currently selected target platform.
option_dwarf_smart_linking=11046_N_Le informazioni di debug DWARF non si possono usare con lo smart linking: passaggio al linking statico
% Smart linking is currently incompatble with DWARF debug information on most
% platforms, so smart linking is disabled in such cases.
option_ignored_target=11047_W_L'opzione "$1" è ignorata per la destinazione corrente
% Not all options are supported or implemented for all target platforms. This message informs you that a chosen
% option is ignored for the currently selected target platform.
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
% ottimizzazione globale.
% \begin{description}
wpo_cant_find_file=12000_F_Impossibile aprire il file di feedback "$1" per la ottimizzazione globale
% The compiler cannot open the specified feedback file with ottimizzazione globale information.
wpo_begin_processing=12001_D_Elaborazione dell'informazione per l'ottimizzazione globale dal file di feedback "$1"
% The compiler starts processing ottimizzazione globale information found in the named file.
wpo_end_processing=12002_D_Fine elaborazione dell'informazione per l'ottimizzazione globale dal file di feedback "$1"
% The compiler has finished processing the ottimizzazione globale information found in the named file.
wpo_expected_section=12003_E_Header di sezione errato: trovato "$2" alla riga $1 del file di feedback
% The compiler expected a section header in the ottimizzazione globale file (starting with \%),
% but did not find it.
wpo_no_section_handler=12004_W_Nessun handler registrato per la sezione ottimizzazione globale "$2" alla riga $1 del file feedback wpo: sezione ignorata
% The compiler has no handler to deal with the mentioned ottimizzazione globale information
% section, and will therefore ignore it and skip to the next section.
wpo_found_section=12005_D_Trovata sezione WPO "$1" con informazioni su "$2"
% The compiler encountered a section with ottimizzazione globale information, and according
% to its handler this section contains information usable for the mentioned purpose.
wpo_no_input_specified=12006_F_Le ottimizzazioni globali selezionate richiedono un file di feedback pre-generato (usare -Fw per specificarlo)
% The compiler needs information gathered during a previous compilation run to perform the selected
% ottimizzazione globales. You can specify the location of the feedback file containing this
% information using the -Fw switch.
wpo_not_enough_info=12007_E_Non sono state trovate le informazioni necessarie per eseguire le ottimizzazioni globali "$1"
% While you pointed the compiler to a file containing ottimizzazione globale feedback, it
% did not contain the information necessary to perform the selected optimizations. You most likely
% have to recompile the program using the appropriate -OWxxx switch.
wpo_no_output_specified=12008_F_Specificare un file di feedback (usando -FW) in cui memorizzare le informazioni per l'ottimizzazione globale
% You have to specify the feedback file in which the compiler has to store the ottimizzazione globale
% feedback that is generated during the compilation run. This can be done using the -FW switch.
wpo_output_without_info_gen=12009_E_Non saranno generate informazioni per l'ottimizzazione globale, ma è stato comunque specificato un file (con -FW)
% The compiler was instructed to store ottimizzazione globale feedback into a file specified using -FW,
% but not to actually generated any ottimizzazione globale feedback. The classes of to be
% generated information can be speciied using -OWxxx.
wpo_input_without_info_use=12010_E_Non saranno generate informazioni per l'ottimizzazione globale, ma è stato comunque specificato un file (con -Fw)
% The compiler was not instructed to perform any ottimizzazione globales (no -Owxxx parameters),
% but nevertheless an input file with such feedback was specified (using -Fwyyy). Since this can
% indicate that you forgot to specify an -Owxxx parameter, the compiler generates an error in this case.
wpo_skipping_unnecessary_section=12011_D_Sezione del file di feedback "$1" ignorata, perché non necessaria per le ottimizzazioni richieste
% The ottimizzazione globale feedback file contains a section with information that is not
% required by the selected ottimizzazione globales.
wpo_duplicate_wpotype=12012_W_Sovrascrittura delle informazioni già lette dal file di feedback "$1" con le informazioni nella sezione "$2"
% The feedback file contains multiple sections that provide the same class of information (e.g.,
% information about which virtual methods can be devirtualized). In this case, the information in last encountered
% section is used. Turn on debugging output (-vd) to see which class of information is provided by each section.
wpo_cannot_extract_live_symbol_info_strip=12013_E_Impossibile estrarre informazioni di vitalità dei simboli se il programma ne è privo: usate -Xs-
% Certain symbol liveness collectors extract the symbol information from the linked program. If the symbol information
% is stripped (option -Xs), this is not possible.
wpo_cannot_extract_live_symbol_info_no_link=12014_E_Impossibile estrarre informazioni di vitalità dei simboli se il programma non è in fase di linking
% Certain symbol liveness collectors extract the symbol information from the linked program. If the program is not
% linked by the compiler, this is not possible.
wpo_cannot_find_symbol_progs=12015_F_Impossibile trovare "$1" o "$2" per estrarre informazioni di vitalità dei simboli dal programma linkato
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% This helper program is normally 'nm', which is part of the GNU binutils.
wpo_error_reading_symbol_file=12016_E_Errore nella lettura delle informazioni di vitalità dei simboli prodotte da "$1"
% An error occurred during the reading of the symbol liveness file that was generated using the 'nm' or 'objdump' program. The reason
% can be that it was shorter than expected, or that its format was not understood.
wpo_error_executing_symbol_prog=12017_F_Errore nell'esecuzione di "$1" (exitcode: $2) per l'estrazione di informazioni sui simboli dal programma linkato
% Certain symbol liveness collectors need a helper program to extract the symbol information from the linked program.
% The helper program produced the reported error code when it was run on the linked program.
wpo_symbol_live_info_needs_smart_linking=12018_E_La raccolta di informazioni sulla vitalità dei simboli è utile solo in caso di smart linking: usate -CX -XX
% Whether or not a symbol is live is determined by looking whether it exists in the final linked program.
% Without smart linking/dead code stripping, all symbols are always included, regardless of whether they are
% actually used or not. So in that case all symbols will be seen as live, which makes this optimization ineffective.
wpo_cant_create_feedback_file=12019_E_Impossibile creare il file di feedback "$1" per l'ottimizzazione globale
% The compiler is unable to create the file specified using the -FW parameter to store the whole program optimisation information.
%\end{description}
# EndOfTeX


#
# Logo (option -l)
#
option_logo=11023_[
Compilatore Free Pascal, versione $FPCFULLVERSION [$FPCDATE] per $FPCCPU
Copyright (c) 1993-2020 di Florian Klaempfl and others
]

#
# Info (option -i)
#
option_info=11024_[
Compilatore Free Pascal, versione $FPCVERSION

Data creazione del compilatore : $FPCDATE
CPU nativa del compilatore     : $FPCCPU

Destinazioni supportate:
  $OSTARGETS

Set di istruzioni di CPU supportate:
  $INSTRUCTIONSETS

Set di istruzioni di FPU supportate:
  $FPUINSTRUCTIONSETS

Destinazioni ABI supportate:
  $ABITARGETS

Ottimizzazioni supportate:
  $OPTIMIZATIONS

Ottimizzazioni globali supportate:
  All
  $WPOPTIMIZATIONS

Tipi di microcontroller supportati:
  $CONTROLLERTYPES

Questo programma è distribuito sotto la GNU General Public Licence
Per ulteriori informazioni leggere il file COPYING.v2

Riferire difetti, suggerimenti ecc. a:
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
**0*_Mettere + dopo uno switch per attivarlo, - per disattivarlo
**1a_Il compilatore non cancella i file asm generati
**2al_Elenca le linee del codice sorgente nel file asm
**2an_Elenca informazioni sui nodi nel file assembler
*L2ap_Usa pipes invece di creare file assembler temporanei
**2ar_Elenca info allocaz./rilascio registri nel file asm
**2at_Elenca info allocaz./rilascio nei file asm
**1A<x>_Formato di uscita:
**2Adefault_Usa l'assemblatore di default
3*2Aas_Assembla usando GNU AS
3*2Anasmcoff_File COFF (Go32v2) con Nasm
3*2Anasmelf_File ELF32 (Linux) con Nasm
3*2Anasmwin32_File oggetto Win32 con Nasm
3*2Anasmwdosx_File oggetto Win32/WDOSX con Nasm
3*2Awasm_File oggetto con Wasm (Watcom)
3*2Anasmobj_File oggetto con Nasm
3*2Amasm_File oggetto con Masm (Microsoft)
3*2Atasm_File oggetto con Tasm (Borland)
3*2Aelf_File ELF (Linux) con assemblatore interno
3*2Acoff_File COFF (Go32v2) con assemblatore interno
3*2Apecoff_File PE-COFF (Win32) con assemblatore interno
4*2Aas_Assembla using GNU AS
6*2Aas_Unix o-file using GNU AS
6*2Agas_GNU Motorola assembler
6*2Amit_MIT Syntax (old GAS)
6*2Amot_Usa l'assemblatore standard Motorola
A*2Aas_Assembla con GNU AS
P*2Aas_Assembla con GNU AS
S*2Aas_Assembla con GNU AS
**1b_Genera info per il browser
**2bl_Genera info sui simboli locali
**1B_Costruisce tutti i moduli
**1C<x>_Opzioni di generazione del codice:
**2Ca<x>_Seleziona l'ABI, vedi fpc -i per i valori possibili
**2Cb_Genera codice big-endian
**2Cc<x>_Imposta la convenzione di chiamata di default a <x>
**2CD_Crea anche la libreria dinamica (non supportato)
**2Ce_Compilazione con opcode in virgola mobile emulati
**2Cf<x>_Sceglie il set di istruzioni FPU, (vedi fpc -i)
**2CF<x>_Precisione minima per costanti virgola mobile (default/32/64)
**2Cg_Genera codice position-independent (PIC)
**2Ch<n>_<n> dimensione in byte dello heap 
**3*_        (fra 1023 e 67107840)
**2Ci_controllo dell'I/O
**2Cn_Non fare il linking
**2Co_Controlla l'overflow di operazioni su interi
**2CO_Controlla il possibile overflow di operazioni su interi
**2Cp<x>_Scegli il set di istruzioni, 
**3*_        (vedi fpc -i per i valori possibili)
**2CP<x>=<y>_ impostazioni per il packing
**3CPPACKSET=<y>_ <y> imposta l'allocazione: 
**3*_        (0, 1 o DEFAULT o NORMAL, 2, 4 e 8)
**2Cr_Controllo degli intervalli (range)
**2CR_Controlla la validità di chiamate a metodi di oggetti
**2Cs<n>_Imposta controllo validità dello stack a <n>
**2Ct_Controllo stack (solo testing, vedi manuale)
**2CX_Crea anche librerie smartlinked
**1d<x>_Definisce il simbolo <x>
**1D_Genera un file DEF
**2Dd<x>_Imposta descrizione a <x>
**2Dv<x>_Imposta versione della DLL a <x>
*O2Dw_Applicazione PM
**1e<x>_Imposta il path all'eseguibile
**1E_Identico a -Cn
**1fPIC_Identico a -Cg
**1F<x>_Imposta nomi di file e path:
**2Fa<x>[,y]_(programma) carica unit <x> e [y] 
**3*_        (prima di parsare la sezione uses)
**2Fc<x>_Imposta codepage di input a <x>
**2FC<x>_Imposta il nome dell'exe del compilatore RC a <x>
**2Fd_Disabilita cache interna delle dir. del compilatore
**2FD<x>_Imposta la directory in cui cercare le utility
**2Fe<x>_Redirezione l'output degli errori a <x>
**2Ff<x>_Aggiunge <x> al path del framework (solo Darwin)
**2FE<x>_Imposta il path di output exe/unit a <x>
**2Fi<x>_Aggiunge <x> al path degli include
**2Fl<x>_Aggiunge <x> al path delle library
**2FL<x>_Usa <x> come linker dinamico
**2Fm<x>_Carica tabella conversione unicode da <x>.txt 
**3*_        (nella directory del compilatore)
**2Fo<x>_Aggiunge <x> al path dei file oggetto
**2Fr<x>_Carica file dei messaggi di errore <x>
**2FR<x>_Imposta linker delle risorse (.res) a <x>
**2Fu<x>_Aggiunge <x> al path delle unit
**2FU<x>_Setta output path delle unit a <x>, sovrascrive -FE
**2FW<x>_Memorizza il feedback per l'ottimizz. globale in <x>
**2Fw<x>_Carica info ottimizz. globale dal feedback file <x>
*g1g_Genera info di debug (formato di default per la destinaz.)
*g2gc_Genera controlli per i puntatori
*g2gh_Usa unit heaptrace (debug di memory leak o corruzioni)
*g2gl_Usa unit line info (mostra più info con i backtrace)
*g2go<x>_Imposta opzioni delle informazioni di debug
*g3godwarfsets_Abilita debug info tipo DWARF 'set' 
*g3*_        (attenzione: fa crashare gdb < 6.5)
*g3gostabsabsincludes_Memorizza path assoluto per 
*g3*_        gli include file in Stabs
*g3godwarfmethodclassprefix_Prefissa i nomi dei metodi 
**3*_        in DWARF con il nome di classe
*g2gp_Conserva maiuscole nei nomi di simboli stabs
*g2gs_Genera info di debug Stabs
*g2gt_Scarta variabili locali 
*g3*_        (rileva il loro uso non inizializzato)
*g2gv_Genera programmi tracciabili con Valgrind
*g2gw_Genera info di debug DWARFv2 (come -gw2)
*g2gw2_Genera info di debug DWARFv2 
*g2gw3_Genera info di debug DWARFv3 
**1i_Informazioni
**2iD_Ritorna la data del compilatore
**2iV_Ritorna la versione breve del compilatore
**2iW_Ritorna la versione completa del compilatore
**2iSO_Ritorna il sistema operativo del compilatore
**2iSP_Ritorna la CPU ospite del compilatore
**2iTO_Ritorna il sistema operativo di destinazione
**2iTP_Ritorna la CPU di destinazione
**1I<x>_Aggiunge <x> al path degli include file
**1k<x>_Passa <x> al linker
**1l_Scrive il logo
**1M<x>_Imposta il modo del linguaggio a <x>
**2Mfpc_Dialetto Free Pascal (default)
**2Mobjfpc_Modo FPC con supporto Object Pascal
**2Mdelphi_Modo compatibilità Delphi 7 
**2Mtp_Modo compatibilità TP/BP 7.0 
**2Mmacpas_Modo compatibilità Macintosh Pascal e dialetti
**1n_Non leggere i file configurazione di default
**1o<x>_Rinomina l'eseguibile prodotto in <x>
**1O<x>_Ottimizzazioni:
**2O-_Disabilita le ottimizzazioni
**2O1_Ottimizzazione livello 1 
**3*_        (rapide e trasparenti al debugger)
**2O2_Ottimizzazione livello 2 (-O1 + ottimizz. veloce)
**2O3_Ottimizzazione livello 3 (-O2 + ottimizz. lenta)
**2Oa<x>=<y>_Imposta allineamento
**2Oo[NO]<x>_Abilita/disabilita ottimizzazioni, 
**3*_        (vedi fpc -i per possibili valori)
**2Op<x>_Imposta ottimizzazione per una CPU specifica, 
**3*_        (vedi fpc -i per possibili valori)
**2OW<x>_Genera feedback per l'ottimizz. globale <x> (vedi fpc -i)
**2Ow<x>_Esegue ottimizzazione globale <x> (vedi fpc -i)
**2Os_Ottimizza per la dimensione e non per la velocità
**1pg_Genera codici di profiling per gprof (FPC_PROFILE: on)
**1R<x>_Stile di lettura assembler:
**2Rdefault_Usa assembler di default per la destinazione
3*2Ratt_Leggi assembler stile AT&T
3*2Rintel_Leggi assembler stile Intel
4*2Ratt_Leggi assembler stile AT&T
4*2Rintel_Leggi assembler stile Intel
8*2Ratt_Leggi assembler stile AT&T
8*2Rintel_Leggi assembler stile Intel
6*2RMOT_Leggi assembler stile Motorola
**1S<x>_Opzioni di sintassi:
**2S2_Come -Mobjfpc
**2Sc_Supporta operatori simil-C (*=,+=,/= e -=)
**2Sa_Abilita le asserzioni
**2Sd_Come -Mdelphi
**2Se<x>_Opzioni sugli errori. <x> è la combinazione di:
**3*_<n> : Il compilatore si ferma dopo <n> errori (default = 1)
**3*_w : Il compilatore si ferma anche con warning
**3*_n : Il compilatore si ferma anche con note
**3*_h : Il compilatore si ferma anche con consigli
**2Sg_Abilita LABEL e GOTO (default in -Mtp e -Mdelphi)
**2Sh_Usa ansistring per default invece di shortstring
**2Si_Abilita l'inlining di procedure e funzioni 
**3*_        dichiarate come "inline"
**2Sk_Carica unit fpcylix
**2SI<x>_Imposta stile di interfacce a <x>
**3SIcom_Interfaccia COM compatibile (default)
**3SIcorba_Interfaccia CORBA compatible
**2Sm_Supporta macro simil-C (global)
**2So_Come -Mtp
**2Ss_Il nome del costruttore deve essere 'init' 
**3*_        (e il nome del distruttore deve essere 'done')
**2St_Permetti la parola chiave static negli oggetti
**2Sx_Abilita la parola chiave exception 
**3*_        (default nei modi Delphi/ObjFPC)
**1s_Non chiamare assembler e linker
**2sh_Genera script per il linking sull'host
**2st_Genera script per il linking sulla destinazione
**2sr_Salta la fase di allocazione dei registri (usare con -alr)
**1T<x>_Sistema operativo di destinazione:
3*2Temx_OS/2 via EMX (inclusi extender EMX/RSX)
3*2Tfreebsd_FreeBSD
3*2Tgo32v2_DJ Delorie DOS extender versione 2
3*2Tlinux_Linux
3*2Tnetbsd_NetBSD
3*2Tnetware_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2Topenbsd_OpenBSD
3*2Tos2_OS/2 / eComStation
3*2Tsymbian_Symbian OS
3*2Twatcom_DOS extender compatibile Watcom
3*2Twdosx_WDOSX DOS extender
3*2Twin32_Windows 32 Bit
3*2Twince_Windows CE
4*2Tlinux_Linux
6*2Tamiga_Commodore Amiga
6*2Tatari_Atari ST/STe/TT
6*2Tlinux_Linux/m68k
6*2Tmacos_Macintosh m68k (non supportato)
6*2Tpalmos_PalmOS
A*2Tlinux_Linux
A*2Twince_Windows CE
P*2Tamiga_AmigaOS su PowerPC
P*2Tdarwin_Darwin e Mac OS X su PowerPC
P*2Tlinux_Linux su PowerPC
P*2Tmacos_Mac OS (classic) su PowerPC
P*2Tmorphos_MorphOS
S*2Tlinux_Linux
**1u<x>_Elimina il simbolo <x>
**1U_Opzioni per le unit:
**2Un_Non controllare se i nomi della unit e 
**3*_        del file che la contiene sono uguali
**2Ur_Genera file di release delle unit 
**3*_        (niente ricompilazione automatica)
**2Us_Compila una unit di sistema
**1v<x>_Prolisso. <x> è una combinazione dei seguenti:
**2*_e : Mostra errori (default)  0 : Mostra solo errori e basta
**2*_w : Mostra warning           u : Mostra info sulla unit
**2*_n : Mostra note              t : Mostra files usati/tentati
**2*_h : Mostra consigli          c : Mostra i condizionali
**2*_i : Mostra info generiche    d : Mostra info di debug
**2*_l : Mostra numri di riga     r : Modo compatibile Rhide/GCC
**2*_s : Mostra date/ore          q : Mostra numeri dei messaggi
**2*_a : Mostra tutto             x : Info degli exe (solo Win32)
**2*_b : Nei messaggi scrivi nomi p : Scrivi parse tree in tree.log
**2*_    file con path completo   v : Scrivi fpcdebug.txt con
**2*_                                 molte informazioni di debug
**2*_m<x>,<y> : Non mostrare messaggi con i numeri <x> e <y>
3*1W<x>_Opzioni specifiche della destinazione (destinazioni)
A*1W<x>_Opzioni specifiche della destinazione (destinazioni)
P*1W<x>_Opzioni specifiche della destinazione (destinazioni)
p*1W<x>_Opzioni specifiche della destinazione (destinazioni)
3*2Wb_Crea un bundle e non una libreria (Darwin)
P*2Wb_Crea un bundle e non una libreria (Darwin)
p*2Wb_Crea un bundle e non una libreria (Darwin)
3*2WB_Crea una immagine rilocabile (Windows)
A*2WB_Crea una immagine rilocabile (Windows, Symbian)
3*2WC_Specifica applicazione di tipo console 
3*3*_        (EMX, OS/2, Windows)
A*2WC_Specifica applicazione console (Windows)
P*2WC_Specifica applicazione console (Classic Mac OS)
3*2WD_Usa DEFFILE per esportare funz. di DLL o EXE (Windows)
A*2WD_Usa DEFFILE per esportare funz. di DLL o EXE (Windows)
3*2We_Usa risorse esterne (Darwin)
P*2We_Usa risorse esterne (Darwin)
p*2We_Usa risorse esterne (Darwin)
3*2WF_Specifica un'applicazione a tutto schermo (EMX, OS/2)
3*2WG_Specifica una applicazione grafica (EMX, OS/2, Windows)
A*2WG_Specifica una applicazione grafica (Windows)
P*2WG_Specifica una applicazione grafica (Classic Mac OS)
3*2Wi_Usa risorse interne (Darwin)
P*2Wi_Usa risorse interne (Darwin)
p*2Wi_Usa risorse interne (Darwin)
3*2WN_Non generare codice di rilocazione, 
3*3*_        (necessario per il debug sotto Windows)
A*2WN_Non generare codice di rilocazione, 
A*3*_(necessario per il debug sotto Windows)
3*2WR_Genera codice di rilocazione (Windows)
A*2WR_Genera codice di rilocazione (Windows)
P*2WT_Specifica applicazione tipo tool MPW (Classic Mac OS)
3*2WX_Abilita stack dell'eseguibile (Linux)
A*2WX_Abilita stack dell'eseguibile (Linux)
p*2WX_Abilita stack dell'eseguibile (Linux)
P*2WX_Abilita stack dell'eseguibile (Linux)
**1X_Opzioni per gli eseguibili:
**2Xc_Passa --shared/-dynamic al linker 
**3*_        (BeOS, Darwin, FreeBSD, Linux)
**2Xd_Non usare il path per la standard library 
**3*_        (per crosscompilazione)
**2Xe_Usa linker esterno
**2Xg_Metti le info di debug in un file diverso e inserisci
**3*_        una sezione debuglink nell'eseguibile
**2XD_Cerca di linkare le unit dinamicamente 
**3*_        (FPC_LINK_DYNAMIC: on)
**2Xi_Usa linker interno
**2Xm_Genera la mappa di linking
**2XM<x>_Rinomina la routine 'main' del programma (default: main)
**2XP<x>_Aggiunge il prefisso <x> ai nomi delle binutils
**2Xr<x>_Pone il path rlink del linker a <x> 
**3*_        per crosscompilazioni, vedi manuale di ld (BeOS, Linux)
**2XR<x>_Prepone <x> ai path del linker 
**3*_        (BeOS, Darwin, FreeBSD, Linux, Mac OS, Solaris)
**2Xs_Elimina tutti i simboli dall'eseguibile finale
**2XS_Prova a linkare tutte le unit staticamente 
**3*_        (default, FPC_LINK_STATIC: on)
**2Xt_Linka a librerie statiche (passa -static al linker)
**2XX_Prova a usare lo smartlinking (FPC_LINK_SMART: on)
**1*_
**1?_Mostra questo help
**1h_Mostra questo help senza interruzioni
]

#
# The End...
