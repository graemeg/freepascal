#
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1999-2018 by the Free Pascal Development team
#
#   Polish (ISO 8859-2) Language File for Free Pascal
#
#   See the file COPYING.v2, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#

# CodePage 28592

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
#   l_   linenumber
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
general_t_compilername=01000_T_Kompilator: $1
% When the \var{-vt} switch is used, this line tells you what compiler
% is used.
general_d_sourceos=01001_D_OS kompilatora: $1
% When the \var{-vd} switch is used, this line tells you what the source
% operating system is.
general_i_targetos=01002_I_Docelowy OS: $1
% When the \var{-vd} switch is used, this line tells you what the target
% operating system is.
general_t_exepath=01003_T_¦cie¿ka narzêdzi: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its binaries.
general_t_unitpath=01004_T_¦cie¿ka modu³ów: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for compiled units. You can set this path with the \var{-Fu}
general_t_includepath=01005_T_¦cie¿ka include'ów: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its include files (files used in \var{\{\$I xxx\}} statements).
% You can set this path with the \var{-I} option.
general_t_librarypath=01006_T_¦cie¿ka bibliotek: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for the libraries. You can set this path with the \var{-Fl} option.
general_t_objectpath=01007_T_¦cie¿ka obiektów: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for object files you link in (files used in \var{\{\$L xxx\}} statements).
% You can set this path with the \var{-Fo} option.
general_i_abslines_compiled=01008_I_$1 linii skompilowanych w $2 sek
% When the \var{-vi} switch is used, the compiler reports the number
% of lines compiled, and the time it took to compile them (real time,
% not program time).
general_f_no_memory_left=01009_F_Brak pamiêci
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
general_i_writingresourcefile=01010_I_Zapisywanie pliku zasobów: $1
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_B³±d podczas zapisywania pliku zasobów: $1
% This message is shown when the compiler encountered an error when writing
% the Resource String Table file
general_i_fatal=01012_I_B³±d krytyczny:
% Prefix for Fatal Errors
general_i_error=01013_I_B³±d:
% Prefix for Errors
general_i_warning=01014_I_Ostrze¿enie:
% Prefix for Warnings
general_i_note=01015_I_Nota:
% Prefix for Notes
general_i_hint=01016_I_Podpowied¼:
% Prefix for Hints
general_e_path_does_not_exist=01017_E_¦cie¿ka "$1" nie istnieje
% The specified path does not exist.
general_e_compilation_aborted=01018_E_Kompilacja zatrzymana
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
scan_f_end_of_file=02000_F_Nieoczekiwany koniec pliku
% this typically happens in one of the following cases :
% \begin{itemize}
% \item The source file ends before the final \var{end.} statement. This
% happens mostly when the \var{begin} and \var{end} statements aren't
% balanced;
% \item An include file ends in the middle of a statement.
% \item A comment wasn't closed.
% \end{itemize}
scan_f_string_exceeds_line=02001_F_String przekracza jedn± liniê
% You forgot probably to include the closing ' in a string, so it occupies
% multiple lines.
scan_f_illegal_char=02002_F_Nieprawid³owy znak $1 ($2)
% An illegal character was encountered in the input file.
scan_f_syn_expected=02003_F_B³±d sk³adni, oczekiwano $1 ale napotkano $2
% This indicates that the compiler expected a different token than
% the one you typed. It can occur almost everywhere where you make a
% mistake against the pascal language.
scan_t_start_include_file=02004_T_Rozpoczêcie czytania pliku include $1
% When you provide the \var{-vt} switch, the compiler tells you
% when it starts reading an included file.
scan_w_comment_level=02005_W_Znaleziono komentarz $1 stopnia
% When the \var{-vw} switch is used, then the compiler warns you if
% it finds nested comments. Nested comments are not allowed in Turbo Pascal
% and can be a possible source of errors.
scan_n_ignored_switch=02008_N_Zignorowany prze³±cznik kompilatora $1
% With \var{-vn} on, the compiler warns if it ignores a switch
scan_w_illegal_switch=02009_W_Nieprawid³owy prze³±cznik kompilatora $1
% You included a compiler switch (i.e. \var{\{\$... \}}) which the compiler
% doesn't know.
scan_w_switch_is_global=02010_W_Prze³±cznik globalny w nieprawid³owym miejscu
% The compiler switch is misplaced, and should be located at
% the start of the unit or program.
scan_e_illegal_char_const=02011_E_Nieprawid³owa sta³a znakowa
% This happens when you specify a character with its ASCII code, as in
% \var{\#96}, but the number is either illegal, or out of range.
scan_f_cannot_open_input=02012_F_Nie mo¿na otworzyæ pliku $1
% \fpc cannot find the program or unit source file you specified on the
% command line.
scan_f_cannot_open_includefile=02013_F_Nie mo¿na otworzyæ pliku include $1
% \fpc cannot find the source file you specified in a \var{\{\$include ..\}}
% statement.
scan_w_only_pack_records=02015_W_Pola rekordów mog± byæ wyrównane tylko do 1,2,4,8,16 lub 32 bajtów
% You are specifying the \var{\{\$PACKRECORDS n\} } or \var{\{\$ALIGN n\} }
% with an illegal value for \var{n}. For $PACKRECORDS valid alignments are 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, and for $ALIGN valid alignment are 1, 2, 4, 8, 16, 32, ON,
% OFF. Under mode MacPas $ALIGN also supports MAC68K, POWER and RESET.
scan_w_only_pack_enum=02016_W_Wyliczenia mog± byæ zapisane tylko na 1, 2 lub 4 bajtach
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2 or 4 are valid in this case.
scan_e_endif_expected=02017_E_Oczekiwano $ENDIF dla $1 $2 zdefiniowanych w linii $3
% Your conditional compilation statements are unbalanced.
scan_e_preproc_syntax_error=02018_E_B³±d sk³adni podczas parsowania dyrektyw kompilacji warunkowej
% There is an error in the expression following the \var{\{\$if ..\}} compiler
% directive.
scan_e_error_in_preproc_expr=02019_E_B³±d obliczania wyra¿enia kompilacji warunkowej
% There is an error in the expression following the \var{\{\$if ..\}} compiler
% directive.
scan_w_macro_cut_after_255_chars=02020_W_Tre¶æ makra zosta³a obciêta po 255 znaku
% The contents of macros canno be longer than 255 characters.
scan_e_endif_without_if=02021_E_ENDIF bez IF(N)DEF
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements aren't balanced.
scan_f_user_defined=02022_F_Zdefiniowany przez u¿ytkownika: $1
% A user defined fatal error occurred. see also the \progref
scan_e_user_defined=02023_E_Zdefiniowany przez u¿ytkownika: $1
% A user defined error occurred. see also the \progref
scan_w_user_defined=02024_W_Zdefiniowane przez u¿ytkownika: $1
% A user defined warning occurred. see also the \progref
scan_n_user_defined=02025_N_Zdefiniowana przez u¿ytkownika: $1
% A user defined note was encountered. see also the \progref
scan_h_user_defined=02026_H_Zdefiniowana przez u¿ytkownika: $1
% A user defined hint was encountered. see also the \progref
scan_i_user_defined=02027_I_Zdefiniowana przez u¿ytkownika: $1
% User defined information was encountered. see also the \progref
scan_e_keyword_cant_be_a_macro=02028_E_S³owo kluczowe nie mo¿e zostaæ przedefiniowane przez makro
% You cannot redefine keywords with macros.
scan_f_macro_buffer_overflow=02029_F_Przepe³nienie bufora podczas czytania lub rozwijania makra
% Your macro or its result was too long for the compiler.
scan_e_macro_deep_ten=02030_E_Rozwijanie makra przerwane po 16 rozwiniêciach.
% When expanding a macro macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_wrong_styled_switch=02031_E_Prze³±czniki kompilatora nie s± dozwolone w komentarzach typu (* ... *)
% Compiler switches should always be between \var{\{ \}} comment delimiters.
scan_d_handling_switch=02032_D_Obs³ugiwanie prze³±cznika "$1"
% When you set debugging info on (\var{-vd}) the compiler tells you when it
% is evaluating conditional compile statements.
scan_c_endif_found=02033_C_Znaleziono ENDIF $1
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifdef_found=02034_C_Znaleziono IFDEF $1 , $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifopt_found=02035_C_Znalezino IFOPT $1 , $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_if_found=02036_C_Znaleziono IF $1 , $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifndef_found=02037_C_Znaleziono IFNDEF $1 , $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_else_found=02038_C_Znaleziono ELSE $1 , $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_skipping_until=02039_C_Omijanie...
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements, and whether it is skipping or
% compiling parts.
scan_i_press_enter=02040_I_Naci¶nij <enter> aby kontynuowaæ
% When the \var{-vi} switch is used, the compiler stops compilation
% and waits for the \var{Enter} key to be pressed when it encounters
% a \var{\{\$STOP\}} directive.
scan_w_unsupported_switch=02041_W_Nieobs³ugiwany prze³±cznik $1
% When warnings are turned on (\var{-vw}) the compiler warns you about
% unsupported switches. This means that the switch is used in Delphi or
% Turbo Pascal, but not in \fpc
scan_w_illegal_directive=02042_W_Nieprawid³owa dyrektywa $1
% When warings are turned on (\var{-vw}) the compiler warns you about
% unrecognised switches. For a list of recognised switches, \progref
scan_t_back_in=02043_TL_Z powrotem w $1
% When you use (\var{-vt}) the compiler tells you when it has finished
% reading an include file.
scan_w_unsupported_app_type=02044_W_Nieprawid³owy typ aplikacji: $1
% You get this warning, if you specify an unknown application type
% with the directive \var{\{\$APPTYPE\}}
scan_w_app_type_not_support=02045_W_Typ aplikacji nieobs³ugiwany przez docelowy OS
% The \var{\{\$APPTYPE\}} directive is supported by win32 applications only.
scan_w_description_not_support=02046_W_Dyrektywa DESCRIPTION jest obs³ugiwany tylko przez OS2 i Win32
% The \var{\{\$DESCRIPTION\}} directive is only supported for OS2 and Win32 targets.
scan_n_version_not_support=02047_N_Dyrektywa VERSION nie jest obs³ugiwana przez OS
% The \var{\{\$VERSION\}} directive is only supported by win32 target.
scan_n_only_exe_version=02048_N_Dyrektywa VERSION jest dostêna tylko dla programów i DLLi
% The \var{\{\$VERSION\}} directive is only used for executable or DLL sources.
scan_w_wrong_version_ignored=02049_W_Nieprawid³owy format dyrektywy VERSION $1
% The \var{\{\$VERSION\}} directive format is majorversion.minorversion
% where majorversion and minorversion are words.
scan_w_unsupported_asmmode_specifier=02050_W_Nieprawid³owy styl asemblera $1
% When you specify an assembler mode with the \var{\{\$ASMMODE xxx\}}
% the compiler didn't recognize the mode you specified.
scan_w_no_asm_reader_switch_inside_asm=02051_W_Prze³±cznik stylu asemblera niedozwolony w bloku asemblera, $1 bêdzie dzia³aæ dopiero w nastêpnym bloku
% It is not possible to switch from one assembler reader to another
% inside an assembler block. The new reader will be used for next
% assembler statement only.
scan_e_wrong_switch_toggle=02052_E_Z³a warto¶æ prze³±cznika, u¿yj ON/OFF lub +/-
% You need to use ON or OFF or a + or - to toggle the switch
scan_e_resourcefiles_not_supported=02053_E_Pliki zasobów nie s± obs³ugiwane przez OS
% The target you are compiling for doesn't support resource files.
scan_w_include_env_not_found=02054_W_Zmienna ¶rodowiskowa $1 nie odnaleziona
% The included environment variable can't be found in the environment, it'll
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_Nieprawid³owa warto¶æ limitu rejestru FPU
% Valid values for this directive are 0..8 and NORMAL/DEFAULT
scan_w_only_one_resourcefile_supported=02056_W_Tylko jeden plik zasobów jest obs³ugiwany przez docelowy OS
% The target you are compiling for supports only one resource file.
% The first resource file found is used, the others are discarded.
scan_w_macro_support_turned_off=02057_W_Obs³uga makr jest wy³±czona
% A macro declaration has been found, but macro support is currently off,
% so the declaration will be ignored. To turn macro support on compile with
% -Sm on the commandline or add {$MACRO ON} in the source
scan_e_invalid_interface_type=02058_E_Nieprawid³owy typ interfejsu. Prawid³owe s± COM, CORBA i DEFAULT.
% The interface type that was specified is not supported
scan_w_appid_not_support=02060_W_APPNAME jest obs³ugiwane tylko przez PalmOS
% The \var{\{\$APPID\}} directive is only supported for the PalmOS target.
scan_w_appname_not_support=02061_W_Dyrektywa APPNAME jest obs³ugiwana tylko przez PalmOS
% The \var{\{\$APPNAME\}} directive is only supported for the PalmOS target.
scan_e_string_exceeds_255_chars=02061_E_Sta³e ³añcuchowe nie mog± byæ d³u¿sze ni¿ 255 znaków
% A single string constant can contain at most 255 chars. Try splitting up the
% string in multiple smaller parts and concatenate them with a + operator.
scan_f_include_deep_ten=02062_F_Przekroczono maksymaln± ilo¶æ zagnie¿d¿onych include (16)
% When including include files the files have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_too_many_push=02063_F_Zbyt du¿o poziomów PUSH
% A maximum of 20 levels is allowed. This error occurs only in mode MacPas.
scan_e_too_many_pop=02064_E_Napotkano POP bez poprzedzaj±cego go PUSH
% This error occurs only in mode MacPas.
scan_e_error_macro_lacks_value=02065_E_Makro lub zmienna "$1" nie ma przypisanej warto¶ci
% Thus the conditional compile time expression cannot be evaluated.
scan_e_wrong_switch_toggle_default=02066_E_Nieprawid³owy prze³±cznik, u¿yj ON/OFF/DEFAULT lub +/-/*
% You need to use ON or OFF or DEFAULT or a + or - or * to toggle the switch
scan_e_mode_switch_not_allowed=02067_E_Prze³±cznik trybu "$1" jest niedozwolony w tym miejscu
% A mode switch has already been encountered, or, in case of option -Mmacpas,
% a mode switch occur after UNIT.
scan_e_error_macro_undefined=02068_E_Zmienna "$1" jest niezdefiniowana
% Thus the conditional compile time expression cannot be evaluated.
scan_e_utf8_bigger_than_65535=02069_E_Napotkano kod UTF-8 wiêkszy od 65535
% \fpc handles utf-8 strings internally as widestrings e.g. the char codes are limited to 65535
scan_e_utf8_malformed=02070_E_Zniekszta³cony string UTF-8
% The given string isn't a valid UTF-8 string
scan_c_switching_to_utf8=02071_C_Napotkano sygnaturê UTF-8, u¿ycie kodowania UTF-8
% The compiler found an UTF-8 encoding signature ($ef, $bb, $bf) at the beginning of a file,
% so it interprets it as an UTF-8 file
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
parser_e_syntax_error=03000_E_Parser - B³±d sk³adni
% An error against the Turbo Pascal language was encountered. This happens
% typically when an illegal character is found in the sources file.
parser_e_dont_nest_interrupt=03004_E_Procedura typu INTERRUPT nie mo¿e byæ zagnie¿d¿ona
% An \var{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_Dyrektywa $1 zignorowana
% The specified is ignored by FPC programs.
parser_e_no_overload_for_all_procs=03006_E_Nie wszystkie deklaracje $1 s± przeci±¿one
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_export_name_double=03008_E_Zduplikowana nazwa eksportu $1
% Exported function names inside a specific DLL must all be different
parser_e_export_ordinal_double=03009_E_Zduplikowany indeks eksportowanej funkcji $1
% Exported function names inside a specific DLL must all be different
parser_e_export_invalid_index=03010_E_Nieprawid³owa warto¶æ indeksu funkcji
% DLL function index must be in the range \var{1..\$FFFF}
parser_w_parser_reloc_no_debug=03011_W_Informacje debugera w DLLu lub programie $1 nie dzia³aj±, wy³±czone.
parser_w_parser_win32_debug_needs_WN=03012_W_Aby móc debugowaæ kod win32 musisz wy³±czyæ generowanie kodu relokacji prze³±cznikiem -WN
% Stabs info is wrong for relocatable DLL or EXES use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_Konstruktor musi nazywaæ siê INIT
% You are declaring a constructor with a name which isn't \var{init}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_destructorname_must_be_done=03014_E_Destruktor musi nazywaæ siê DONE
% You are declaring an object destructor with a name which is not \var{done}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_proc_inline_not_supported=03016_E_Typ INLINE nieobs³ugiwany (u¿yj opcji -Si)
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler doesn't support C++
% styled inlining by default.
parser_w_constructor_should_be_public=03018_W_Konstruktor powinien byæ publiczny
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_Destruktor powinien byæ publiczny
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_Klasa powinna mieæ tylko jeden destruktor
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_Definicje lokalne w klasie s± niedozwolone
% Classes must be defined globally. They cannot be defined inside a
% procedure or function
parser_f_no_anonym_objects=03022_F_Definicje anonimowych klas s± niedozwolone
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_object_has_no_vmt=03023_E_Obiekt $1 nie posiada VMT
% This is a note indicating that the declared object has no
% virtual method table.
parser_e_illegal_parameter_list=03024_E_Nieprawid³owa lista parametrów
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_size=03026_E_Nieprawid³owa ilo¶æ parametrów
% There is an error in the parameter list of the function or procedure,
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_Przeci±¿ony identyfikator $1 nie jest funkcj±
% The compiler encountered a symbol with the same name as an overloaded
% function, but it is not a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_Przeci±¿ona funkcja musi mieæ chocia¿ jeden parametr wyró¿niaj±cy
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_Nag³ówek funkcji nie pasuje do deklaracji $1
% You declared a function with same parameters but
% different result type or function modifiers.
parser_e_header_different_var_names=03030_E_Nag³ówek funkcji $1 nie pasuje : nazwy parametrów zmienione $2 => $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but define it with a different parameter list.
parser_n_duplicate_enum=03031_N_Ci±g warto¶ci typu wyliczeniowego musi byæ rosn±cy
% \fpc allows enumeration constructions as in C. Given the following
% declaration two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_e_no_with_for_variable_in_other_segments=03033_E_With nie mo¿e byæ u¿yte ze zmiennymi w innym segmencie
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_Przekroczono limit zagnie¿d¿enia funkcji
% You can nest function definitions only 31 times.
parser_e_range_check_error=03035_E_B³±d zakresu podczas obliczania sta³ej
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_B³±d zakresu podczas obliczania sta³ej
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_Zduplikowana etykieta w instrukcji case
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_Górna granica zakresu jest mniejsza od dolnej
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless
parser_e_type_const_not_possible=03039_E_Sta³a typowana nie mo¿e byæ klas± ani obiektem
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_Do zmiennych proceduralnych nie mo¿na przypisaæ przeci±¿onych funkcji
% You are trying to assign an overloaded function to a procedural variable.
% This isn't allowed.
parser_e_invalid_string_size=03041_E_D³ugo¶æ stringa musi zawieraæ siê miêdzy 1 a 255
% The length of a shortstring in Pascal is limited to 255 characters. You are
% trying to declare a string with length lower than 1 or greater than 255
parser_w_use_extended_syntax_for_objects=03042_W_U¿yj rozszerzonej sk³adni NEW i DISPOSE dla obiektów
% If you have a pointer \var{a} to a class type, then the statement
% \var{new(a)} will not initialize the class (i.e. the constructor isn't
% called), although space will be allocated. you should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the class.
parser_w_no_new_dispose_on_void_pointers=03043_W_U¿ycie NEW lub DISPOSE nie ma sensu dla nietypowanych wska¼ników
parser_e_no_new_dispose_on_void_pointers=03044_E_U¿ycie NEW lub DISPOSE jest niemo¿liwe dla nietypowanych wska¼ników
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% Accepted for compatibility in \var{tp} and \var{delphi} modes.
parser_e_class_id_expected=03045_E_Oczekiwano identyfikatora klasy
% This happens when the compiler scans a procedure declaration that contains
% a dot,
% i.e., a object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_Identyfikator typu jest niedozwolony w tym miejscu
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_Oczekiwano identyfikatora metody
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_Nag³ówek funkcji nie pasuje do ¿adnej metody tej klasy $1
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_d_procedure_start=03049_DL_Procedura/Funkcja $1
% When using the \var{-vp} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_Nieprawid³owa warto¶æ sta³ej zmiennoprzecinkowej
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_FAIL mo¿e byæ u¿yte tylko w konstruktorach
% You are using the \var{FAIL} instruction outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_Destruktory nie mog± mieæ parametrów
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_methods_via_class_ref=03053_E_Tylko metody klasy mog± byæ wywo³ywane w ten sposób
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
parser_e_only_class_methods=03054_E_W metodach klasy mo¿esz u¿ywaæ tylko metod klasy
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
parser_e_case_mismatch=03055_E_Sta³a i zmienna u¿yta w CASE nie s± tego samego typu
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_Symbol nie mo¿e byæ wyeksportowany
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_Dziedziczona metoda jest ukryta przez $1
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendent class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_Nie ma takiej metody w dziedziczonej klasie: $1
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_no_procedure_to_access_property=03059_E_Brak metody do odczytania w³asno¶ci
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_Dyrektywa stored nie jest jeszcze zaimplementowana
% The \var{stored} directive is not yet implemented
parser_e_ill_property_access_sym=03061_E_Nieprawid³owy symbol dla dostêpu do w³asno¶ci
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code woud cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Nie mo¿na siê odwo³ywaæ do pola protected obiektu w tym miejscu
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module wher the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_Nie mo¿na siê odwo³ywaæ do pola private obiektu w tym miejscu
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_e_overridden_methods_not_same_ret=03066_E_Przeci±¿one metody wirtualne powinny mieæ taki sam typ zwracanych warto¶ci: "$2" jest przeci±¿one przez "$" o innym zwracanym typie
% If you declare overridden methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_Eksportowane funkcje nie mog± byæ zagnie¿d¿one
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_Metody nie mog± byæ eksportowane
% You cannot declare a procedure that is a method for an object as
% \var{export}ed. That is, your methods cannot be called from a C program.
parser_e_call_by_ref_without_typeconv=03069_E_Przy przekazywaniu przez referencjê parametry musz± byæ tego samego typu: Otrzymano $1, a oczekiwano $2
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_Ta klasa nie jest przodkiem bie¿±cej klasy
% When calling inherited methods, you are trying to call a method of a strange
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_SELF jest dostêpne tylko w metodach
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_Metody mog± byæ wywo³ane bezpo¶rednio identyfikatorem typu klasy tylko w innych metodach
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_Nieprawid³owe u¿ycie ':'
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_B³±d zakresu lub zduplikowany element w deklaracji zbioru
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, either two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_Oczekiwano wska¼nika do obiektu
% You specified an illegal type in a \var{New} statement.
% The extended syntax of \var{new} needs an object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_Wyra¿enie musi byæ wywo³aniem konstruktora
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_Wyra¿enie musi byæ wywo³aniem destruktora
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_Nieprawid³owa kolejno¶æ elementów rekordu
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_Wyra¿enie musi byæ rekordem lub klas±
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_Procedury nie mog± zwracaæ warto¶ci
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures cannot return a value.
parser_e_only_methods_allowed=03081_E_Konstruktory, destructory i class operators musz± byæ metodami
% You're declaring a procedure as destructor, constructor or class operator, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_Operator nie jest przeci±¿ony
% You're trying to use an overloaded operator when it isn't overloaded for
% this type.
parser_e_no_such_assignment=03083_E_Nie mo¿a przeci±¿yæ przypisania dla takich samych typów
% You can not overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_Niemo¿liwe przeci±¿enie operatora
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_Ponowne wywo³anie wyj±tku niemo¿liwe w tym miejscu
% You are trying to raise an exception where it isn't allowed. You can only
% raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_Rozszerzona sk³adnia new i dispose jest niedozwolona dla klas
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{Dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_procedure_overloading_is_off=03088_E_Przeci±¿anie procedur jest wy³±czone
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_Przeci±¿enie tego operatora jest niemo¿liwe
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_Operator porównania musi zwracaæ warto¶æ typu Boolean
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_Tylko metody wirtualne mog± byæ abstrakcyjne
% You are declaring a method as abstract, when it isn't declared to be
% virtual.
parser_f_unsupported_feature=03092_F_U¿ycie nieobs³ugiwanej funkcji kompilatora!
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_Mieszanie ró¿nych typów obiektów (obiektów, klas, interfejsów) jest niedozwolone
% You cannot derive \var{objects}, \var{classes}, \var{cppclasses} and \var{interfaces} interttwined . E.g.
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_Nieznana dyrektywa procedury zosta³a zignorowana: $1
% The procedure directive you specified is unknown.
parser_e_absolute_only_one_var=03095_E_Absolute mo¿e byæ powi±zane tylko z jedn± zmienn±
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
parser_e_absolute_only_to_var_or_const=03096_E_Absolute mo¿e byæ u¿yte tylko w stosunku do zmiennej lub sta³ej
% The address of a \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_Tylko jedna zmienna mo¿e byæ zainicjowana
% You cannot specify more than one variable with a initial value
% in Delphi mode.
parser_e_abstract_no_definition=03098_E_Metody abstrakcyjne nie mog± byæ zaimplementowane
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_Ta przeci±¿ona funkcja nie mo¿e byæ lokalna (musi byæ wyeksportowana)
% You are defining a overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_Metody wirtualne u¿yte bez konstruktora $1
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_c_macro_defined=03101_CL_Makro zdefiniowane: $1
% When \var{-vm} is used, the compiler tells you when it defines macros.
parser_c_macro_undefined=03102_CL_Makro wymazane: $1
% When \var{-vm} is used, the compiler tells you when it undefines macros.
parser_c_macro_set_to=03103_CL_Makro $1 ustawione na $2
% When \var{-vm} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_Kompilowanie $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_UL_Parsowanie czê¶ci interface modu³u $1
% This tells you that the reading of the interface
% of the current unit starts
parser_u_parsing_implementation=03106_UL_Parsowanie czê¶ci implementation modu³u $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_DL_Kompilowanie $1 po raz drugi
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_property_found_to_override=03109_E_Nie znaleziono w³asno¶ci do przeci±¿enia
% You want to overrride a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_Dozwolona jest tylko jedna w³asno¶æ domy¶lna, a znaleziono odziedziczon± w³asno¶æ domy¶ln± $1
% You specified a property as \var{Default}, but a parent class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_W³asno¶ci± domy¶ln± musi byæ tablica
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_Wirtualne konstruktory s± dostêpne tylko w klasach
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_Brak domy¶lnej w³asno¶ci w tej klasie
% You are trying to access a default property of a class, but this class (or one of
% its ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_Klasa nie mo¿e mieæ sekcji published, u¿yj prze³±cznika {$M+}
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, whch turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_Deklaracja naprzód klasy $1 musi byæ rozwi±zana, aby u¿yæ klasy jako przodka innej klasy
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% Where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_Lokalne przeci±¿enia operatorów niedostêpne
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_Dyrektywa $1 niedozwolona w sekcji interface
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_Dyrektywa $1 niedozwolona w sekcji implementation
% This procedure directive is not defined in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_Dyrektywa $1 niedozwolona w deklaracji typu proceduralnego
% This procedure directive cannot be part of a procedural or function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_Funkcja jest ju¿ zadeklarowana jako Public/Forward $1
% You will get this error if a function is defined as \var{forward} twice.
% Or it is once in the \var{interface} section, and once as a \var{forward}
% declaration in the \var{implmentation} section.
parser_e_not_external_and_export=03121_E_Nie mo¿na u¿yæ jednocze¶nie EXPORT i EXTERNAL
% These two procedure directives are mutually exclusive
parser_n_not_supported_for_inline=03123_N_$1 nieobs³ugiwane w procedurze/funkcji inline
% Inline procedures don't support this declaration.
parser_h_inlining_disabled=03124_H_Inline wy³±czone
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_Zapisywanie logu przegl±darki $1
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+ \}} switch).
parser_h_maybe_deref_caret_missing=03126_H_Mo¿liwe, ¿e brakuje dereferencji wska¼nika
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_Wybrany tryb asemblera nie jest obs³ugiwany
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_Dyrektywa $1 koliduje z innymi dyrektywami
% You specified a procedure directive that conflicts with other directives.
% for instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_Konwencja wywo³ywania musi byæ taka sama w czêsci interface i implementation
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_property_cant_have_a_default_value=03131_E_Ta w³asno¶æ nie mo¿e mieæ domy¶lnej warto¶ci
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_Domy¶lna warto¶æ w³asno¶ci musi byæ sta³±
% The value of a \var{default} declared property must be known at compile
% time. The value you specified is only known at run time. This happens
% .e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_Symbol nie mo¿e byæ w sekcji published
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_Ten typ warto¶ci nie mo¿e byæ w sekcji published
% Properties in a \var{published} section cannot be array properties.
% they must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_e_empty_import_name=03136_W_Wymagana jest nazwa importu
% Some targets need a name for the imported procedure or a \var{cdecl} specifier
parser_e_division_by_zero=03138_E_Dzielenie przez zero
% There is a divsion by zero encounted
parser_e_invalid_float_operation=03139_E_Nieprawid³owa operacja zmiennoprzecinkowa
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_Górna granica zakresu jest mniejsza od dolnej
% The upper bound of a an array declaration is less than the lower bound and this
% is not possible
parser_w_string_too_long=03141_W_String "$1" jest d³u¿szy ni¿ $2
% The size of the constant string is larger than the size you specified in
% string type definition
parser_e_string_larger_array=03142_E_D³ugo¶æ stringa jest wiêksza ni¿ d³ugo¶æ tablicy znaków
% The size of the constant string is larger than the size you specified in
% the array[x..y] of char definition
parser_e_ill_msg_expr=03143_E_Nieprawid³owe wyra¿enie przy dyrektywie message
% \fpc supports only integer or string values as message constants
parser_e_ill_msg_param=03144_E_Funkcje obs³uguj±ce komunikaty (message) mog± pobieraæ tylko jeden parametr przez referencjê
% A method declared with the \var{message}-directive as message handler
% can take only one parameter which must be declared as call by reference
% Parameters are declared as call by reference using the \var{var}-directive
parser_e_duplicate_message_label=03145_E_Zduplikowana etykieta komunikatu: $1
% A label for a message is used twice in one object/class
parser_e_self_in_non_message_handler=03146_E_Self mo¿e byæ parametrem tylko metod obs³uguj±cych komunikaty
% The self parameter can only be passed explicitly to a method which
% is declared as message handler.
parser_e_threadvars_only_sg=03147_E_Zmienne threadvar mog± byæ tylko statyczne lub globalne
% Threadvars must be static or global, you can't declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has its own stack and local variables
% are stored on the stack
parser_f_direct_assembler_not_allowed=03148_F_Wstawki asemblera niedostêpne gdy wyj¶ciowym formatem jest wbudowane zapisywanie binarne
% You can't use direct assembler when using a binary writer, choose an
% other outputformat or use another assembler reader
parser_w_no_objpas_use_mode=03149_W_Nie ³aduj modu³u OBJPAS bezpo¶rednio - u¿yj trybu obiektowego (objfpc, delphi)
% You are trying to load the ObjPas unit manually from a uses clause. This is
% not a good idea. Use the \var{\{\$mode objfpc\}} or
% \var{\{\$mode delphi\}}
% directives which load the unit automatically
parser_e_no_object_override=03150_E_OVERRIDE nie mo¿e byæ u¿yte dla obiektów
% Override is not supported for objects, use \var{virtual} instead to override
% a method of a parent object
parser_e_cant_use_inittable_here=03151_E_Typy danych wymagaj±ce inicjalizacji/finalizcji nie mog± byæ u¿yte w rekordach z wariantami
% Some data type (e.g. \var{ansistring}) needs initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% can't be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_Resourcestring mo¿e byæ tylko statyczny lub globalny
% Resourcestring can not be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_Exit z argumentem nie mo¿e byæ tu u¿yte
% an exit statement with an argument for the return value can't be used here, this
% can happen e.g. in \var{try..except} or \var{try..finally} blocks
parser_e_stored_property_must_be_boolean=03154_E_Symbol przechowywania w³asno¶ci musi byæ typu boolean
% If you specify a storage symbol in a property declaration, it must be of
% the type boolean
parser_e_ill_property_storage_sym=03155_E_Ten symbol nie mo¿e byæ symbolem przechowywania w³asno¶ci
% You can't use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants
parser_e_only_publishable_classes_can__be_published=03156_E_Tylko klasy skompilowane w trybie $M+ mog± byæ w sekcji published
% In the published section of a class can be only class as fields used which
% are compiled in \var{\{\$M+\}} or which are derived from such a class. Normally
% such a class should be derived from TPersitent
parser_e_proc_directive_expected=03157_E_Oczekiwano dyrektywy procedury
% When declaring a procedure in a const block you used a ; after the
% procedure declaration after which a procedure directive must follow.
% Correct declarations are:
% \begin{verbatim}
% const
%   p : procedure;stdcall=nil;
%   p : procedure stdcall=nil;
% \end{verbatim}
parser_e_invalid_property_index_value=03158_E_Warto¶æ indeksu w³asno¶ci musi byæ typu porz±dkowego
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_Zbyt krótka nazwa procedury, aby eksportowaæ
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correct with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_Nie mo¿na wygenerowaæ pozycji DEFFILE dla zmiennych globalnych modu³u
parser_e_dlltool_unit_var_problem2=03161_E_Skompiluj bez opcji -WD
% You need to compile this file without the -WD switch on the
% commandline
parser_f_need_objfpc_or_delphi_mode=03162_F_Aby skompilowaæ ten modu³ wymagany jest tryb ObjFpc (-S2) lub Delphi (-Sd)
% You need to use {$mode objfpc} or {$mode delphi} to compile this file.
% Or use the equivalent commandline switches -S2 or -Sd.
parser_e_no_export_with_index_for_target=03163_E_Nie mo¿na wyeksportowaæ z indeksem pod $1
% Exporting of functions or procedures with a specified index is not
% supported on this target.
parser_e_no_export_of_variables_for_target=03164_E_Nie mo¿na wyeksportowaæ zmiennych pod $1
% Exporting of variables is not supported on this target.
parser_e_improper_guid_syntax=03165_E_Nieprawid³owa sk³adnia identyfikatora GUID
parser_w_interface_mapping_notfound=03168_W_Nie znaleziono procedury "$1", która mog³aby implementowaæ $2.$3
parser_e_interface_id_expected=03169_E_Oczekiwano interfejsu
% This happens when the compiler scans a \var{class} declaration that contains
% \var{interface} function name mapping code like this:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% and the \var{interface} before the dot not listed in the inheritance list.
parser_e_type_cant_be_used_in_array_index=03170_E_Typ "$1" nie mo¿e byæ u¿yty jako indeks tablicy
% Types like \var{qword} or \var{int64} aren't allowed as array index type
parser_e_no_con_des_in_interfaces=03171_E_Konstruktory i destruktory s± niedozwolone w interfejsach
% Constructor and destructor declarations aren't allowed in interface
% In the most cases the method \var{QueryInterface} of \var{IUnknown} can
% be used to create a new interface.
parser_e_no_access_specifier_in_interfaces=03172_E_Okre¶lenia dostêpu nie mog± byæ u¿yte w interfejsach
% The access specifiers \var{public}, \var{private}, \var{protected} and
% \var{pusblished} can't be used in interfaces because all methods
% of an interfaces must be public.
parser_e_no_vars_in_interfaces=03173_E_Interfejs nie mo¿e posiadaæ pól
% Declarations of fields aren't allowed in interfaces. An interface
% can contain only methods
parser_e_no_local_external=03174_E_Lokalne procedury nie mog± byæ zadeklarowane jako EXTERNAL
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance of errors very high
parser_w_skipped_fields_before=03175_W_Niektóre pola przed "$1" nie zosta³y zainicjalizowane
% In Delphi mode, not all fields of a typed constant record have to be
% initialized, but the compiler warns you when it detects such situations.
parser_e_skipped_fields_before=03176_E_Niektóre pola przed "$1" nie zosta³y zainicjalizowane
% In all syntax modes but Delphi mode, you can't leave some fields uninitialized
% in the middle of a typed constant record
parser_w_skipped_fields_after=03177_W_Niektóre pola po "$1" nie zosta³y zainicjalizowane
% You can leave some fields at the end of a type constant record uninitialized
% (the compiler will initialize them to zero automatically). This may be the cause
% of subtle problems.
parser_e_varargs_need_cdecl_and_external=03178_E_Dyrektywa VarArgs bez CDecl i External
% The varargs directive can only be used with procedures or functions
% that are declared with \var{cdecl} and \var{external} directives. The varargs directive
% is only meant to provide a compatible interface to C functions like printf.
parser_e_self_call_by_value=03179_E_Self musi byæ normalnym parametrem (przekazywanym przez warto¶æ)
% You can't declare self as a const or var parameter, it must always be
% a call-by-value parameter
parser_e_interface_has_no_guid=03180_E_Interfejs "$1" nie posiada identyfikatora GUID
% When you want to assign an interface to a constant, then the interface
% must have a GUID value set.
parser_e_illegal_field_or_method=03181_E_Nieznana metoda albo pole klasy "$1"
% Properties must refer to a field or method in the same class.
parser_w_proc_overriding_calling=03182_W_Zmiana sposobu wywo³ywania z "$1" na "$2"
% There are two directives in the procedure declaration that specify a calling
% convention. Only the last directive will be used
parser_e_no_procvarobj_const=03183_E_Typowane sta³e typu "procedure of object" mog± byæ inicjowane tylko NILem
% You can't assign the address of a method to a typed constant which has a
% 'procedure of object' type, because such a constant requires two addresses:
% that of the method (which is known at compile time) and that of the object or
% class instance it operates on (which can not be known at compile time).
parser_e_default_value_only_one_para=03184_E_Domy¶lna warto¶æ mo¿e byæ przypisana tylko jednemu parametrowi
parser_e_default_value_expected_for_para=03185_E_Domy¶lna warto¶æ wymagana dla "$1"
parser_w_unsupported_feature=03186_W_U¿ycie (jeszcze) nieobs³ugiwanej funkcji kompilatora!
% You're trying to force the compiler into doing something it cannot do yet.
parser_h_c_arrays_are_references=03187_H_Tablice C s± przekazywane przez referencjê
% Any array passed to a C functions is passed
% by a pointer (i.e. by reference).
parser_e_C_array_of_const_must_be_last=03188_E_Tablica sta³ych musi byæ ostatnim argumentem
% You can not add any other argument after an \var{array of const} for
% \var{cdecl} functions, as the size pushed on stack for this argument is
% not known.
parser_h_type_redef=03189_H_Powtórna definicja typu "$1"
% This is an indicator that a previously declared type is
% being redefined as something else. This may, or may not
% be, a cause for errors.
parser_w_cdecl_has_no_high=03190_W_Funkcje cdecl nie maj± parametru high
% Functions declared with cdecl modifier do not pass an extra implicit parameter.
parser_w_cdecl_no_openstring=03191_W_Funkcje cdecl nie obs³uguj± open strings
%Openstring is not supported for cdecl'ared functions.
parser_e_initialized_not_for_threadvar=03192_E_Nie mo¿na zainicjalizowaæ zmiennych zadeklarowanych jako threadvar
% Variables declared as threadvar can not be initialized with a default value.
% The variables will always be filled with zero at the start of a new thread.
parser_e_msg_only_for_classes=03193_E_Dyrektywa Message jest dozwolona tylko w klasach
% The message directive is only supported for Class types.
parser_e_procedure_or_function_expected=03194_E_Oczekiwano procedury lub funkcji
% A class method can only be specified for procedures and functions.
parser_e_illegal_calling_convention=03195_W_Dyrektywa "$1" zignorowana
% Some calling conventions are supported only by certain CPUs. I.e. most non-i386 ports support
% only the standard ABI calling convention of the CPU.
parser_e_no_object_reintroduce=03196_E_Nie mo¿na u¿yæ REINTRODUCE dla obiektów
% \var{reintroduce} is not supported for objects.
parser_e_paraloc_only_one_para=03197_E_Ka¿dy argument musi posiadaæ swoj± pozycjê
% If locations for arguments are specified explicitly as it is required by
% some syscall conventions, each argument must have it's only location, things
% like \var{procedure p(i,j : longint 'r1');} aren't allowed
parser_e_paraloc_all_paras=03198_E_Ka¿dy argument musi posiadaæ swoj± pozycjê
% If one argument has an explicit argument location, all arguments of a procedure
% must have one.
parser_e_illegal_explicit_paraloc=03199_E_Nieznana pozycja argumentu
% The location specified for an argument isn't recognized by the compiler
parser_e_32bitint_or_pointer_variable_expected=03200_E_Oczekiwano 32 bitowej liczby typu Integer albo wska¼nika
% The libbase for MorphOS/AmigaOS can be give only as \var{longint}, \var{dword} or any pointer variable.
parser_e_goto_outside_proc=03201_E_Nie mo¿na u¿yæ instrukcji GOTO pomiêdzy procedurami
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
parser_f_too_complex_proc=03202_F_Zbyt rozbudowana procedura, spróbuj rozbiæ j± na mniejsze
% Your procedure body is too long for the compiler. You should split the
% procedure into multiple smaller procedures.
parser_e_illegal_expression=03203_E_Nieprawid³owe wyra¿enie
% This can occur under many circumstances. Mostly when trying to evaluate
% constant expressions.
parser_e_invalid_integer=03204_E_Nieprawid³owe wyra¿enie (powinno zwóciæ liczbê ca³kowit±)
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
parser_e_invalid_qualifier=03205_E_Nieprawid³owy kwalifikator
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
parser_e_upper_lower_than_lower=03206_E_Górna granica zakresu jest mniejsza od dolnej
% You are declaring a subrange, and the lower limit is higher than the high
% limit of the range.
parser_e_macpas_exit_wrong_param=03207_E_Parametr funkcji EXIT musi byæ nazw± procedury, w której jej u¿yto
% Non local exit is not allowed. This error occurs only in mode MacPas.
parser_e_illegal_assignment_to_count_var=03208_E_Nieprawid³owe przypisanie do licznika pêtli "$1"
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
parser_e_no_local_var_external=03209_E_Lokalne zmienne nie mog± byæ zadeklarowane jako EXTERNAL
% Declaring local variables as external is not allowed. Only global variables can reference
% to external variables.
parser_e_proc_already_external=03210_E_Procedura ju¿ zadeklarowana jako EXTERNAL
% The procedure is already declared with the EXTERNAL directive in an interface or
% forward declaration.
parser_w_implicit_uses_of_variants_unit=03211_W_Dopisanie modu³u Variants do sekcji uses
% The Variant type is used in the unit without any used unit using the Variants unit. The
% compiler has implicitly added the Variants unit to the uses list. To remove this warning
% the Variants unit needs to be added to the uses statement.
parser_e_no_static_method_in_interfaces=03212_E_Metody statyczne nie mog± byæ u¿yte w interfejsach
% The specifier \var{class} and directive \var{static} can't be used in interfaces
% because all methods of an interfaces must be public.
parser_e_arithmetic_operation_overflow=03213_E_Wyst±pi³o przepe³nienie w operacji arytmetycznej
% An operation on two integers values produced an overflow
parser_e_protected_or_private_expected=03214_E_Oczekiwano "protected" albo "private"
% \var{strict} can be only used together with \var{protected} or \var{private}.
% \end{description}
#
# Type Checking
#
# 04049 is the last one used
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_Nieprawid³owy typ
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_Niezgodno¶æ typów: otrzymano "$1" oczekiwano "$2"
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
type_e_not_equal_types=04002_E_Niezgodno¶æ typów pomiêdzy $1 i $2
% The types are not equal
type_e_type_id_expected=04003_E_Oczekiwano identyfikatora typu
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_Oczekiwano identyfikatora zmiennej
% This happens when you pass a constant to a routine (such as \var{Inc} var or \var{Dec})
% when it expects a variable. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_Oczekiwano wyra¿enia ca³kowitego, ale otrzymano "$1"
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_Oczekiwano wyra¿enia logicznego, ale otrzymano "$1"
% The expression must be a boolean type, it should be return true or
% false.
type_e_ordinal_expr_expected=04007_E_Oczekiwano wyra¿enia porz±dkowego
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_Oczekiwano wska¼nika, ale otrzymano "$1"
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_Oczekiwano klasy, ale otrzymano "$1"
% The variable of expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_cant_eval_constant_expr=04011_E_Nie mo¿na obliczyæ warto¶ci wyra¿enia
% This error can occur when the bounds of an array you declared does
% not evaluate to ordinal constants
type_e_set_element_are_not_comp=04012_E_Niekompatybilne elementy zbioru
% You are trying to make an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union
type_e_set_operation_unknown=04013_E_Operacja niezaimplementowana dla zbioru
% several binary operations are not defined for sets
% like div mod ** (also >= <= for now)
type_w_convert_real_2_comp=04014_W_Konwersja typu zmiennoprzecinkowego do ca³kowitego typu COMP
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{Comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_U¿yj DIV zamiast '/' aby wynik by³ ca³kowity
% When hints are on, then an integer division with the '/' operator will
% procuce this message, because the result will then be of type real
type_e_strict_var_string_violation=04016_E_Typy stringów nie pasuj± z powodu trybu $V+
% When compiling in \var{\{\$V+\}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_Succ i Pred nie dzia³aj± dla wyliczeñ z przypisaniami
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use the \var{Succ} or \var{Pred} functions on them.
type_e_cant_read_write_type=04018_E_Nie mo¿na zapisaæ lub odczytaæ zmiennych tego typu
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that. Only integer types,
% reals, pchars and strings can be read from/written to a text file.
% Booleans can only be written to text files.
type_e_no_readln_writeln_for_typed_file=04019_E_Nie mo¿na u¿yæ readln ani writeln na pliku tego typu
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_Nie mo¿na u¿yæ readln ani writeln na nietypowanym pliku
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_Konflikt typów pomiêdzy elementami zbioru
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(dword/qword) zwraca starsze/m³odsze s³owo/podwójne s³owo
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword/int64/qword}
% which returns the lower/upper word/dword of the argument. TP always uses
% a 16 bit \var{lo/hi} which returns always bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the TP behavior you have
% to type cast the argument to \var{word/integer}
type_e_integer_or_real_expr_expected=04023_E_Oczekiwano wyra¿enia ca³kowitego lub zmiennoprzecinkowego
% The first argument to \var{str} must a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_Nieprawid³owy typ $1 w konstruktorze tablicy
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_Nieprawid³owy typ parametru nr $1: Otrzymano $2, a oczekiwano $3
% You are trying to pass an invalid type for the specified parameter.
type_e_no_method_and_procedure_not_compatible=04026_E_Metoda i Procedura nie s± zgodne (jako zmienne)
% You can't assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_Nieprawid³owa sta³a przekazana do wbudowanej funkcji matematycznej
% The constant argument passed to a ln or sqrt function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_Nie mo¿na pobraæ adresu sta³ej
% It's not possible to get the address of a constant, because they
% aren't stored in memory, you can try making it a typed constant.
type_e_argument_cant_be_assigned=04029_E_Ten argument nie mo¿e byæ przekazany przez referencjê
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference argument
% Remark: Properties can be only
% used on the left side of an assignment, but they can't be used as arguments
type_e_cannot_local_proc_to_procvar=04030_E_Nie mo¿na przypisaæ lokalnej procedury/funkcji do zmiennej proceduralnej
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling of local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_Nie mo¿na przypisaæ warto¶ci adresowi
% It is not allowed to assign a value to an address of a variable, constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_Nie mo¿na sta³ej przypisaæ warto¶ci
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const, to allow
% changing the value make the parameter as a value parameter or a var.
type_e_array_required=04033_E_Dozwolone tylko dla tablic
% If you are accessing a variable using an index '[<x>]' then
% the type must be an array. In FPC mode also a pointer is allowed.
type_e_interface_type_expected=04034_E_Oczekiwano interfejsu ale otrzymano "$1"
type_w_mixed_signed_unsigned=04035_W_U¿ywanie zmiennych typu Cardinal w wyra¿eniach ze znakiem daje 64bitowy wynik
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetical
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64bit which is slower than normal
% 32bit arithmetic. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_U¿ywanie zmiennych typu Cardinal w wyra¿eniach ze znakiem mo¿e spowodowaæ b³±d zakresu w tym miejscu
% If you use a binary operator (and, or, xor) and one of
% the operands is a cardinal while the other one is a signed expression, then,
% if range checking is turned on, you may get a range check error because in
% such a case both operands are converted to cardinal before the operation is
% carried out. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_e_typecast_wrong_size_for_assignment=04037_E_Rzutowanie typów ró¿nych rozmiarów ($1 -> $2) w przypisaniu
% Type casting to a type with a different size is not allowed when the variable is
% used for assigning.
type_e_array_index_enums_with_assign_not_possible=04038_E_Typ wyliczeniowy z przypisanymi warto¶ciami nie moze byæ u¿yty do ideksowania tablicy
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use it as index of an array.
type_e_classes_not_related=04039_E_Typy obiektowe "$1" i "$2" nie s± pokrewne
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors
type_w_classes_not_related=04040_W_Typy obiektowe "$1" i "$2" nie s± pokrewne
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors
type_e_class_or_interface_type_expected=04041_E_Oczekiwano klasy lub interfejsu ale otrzymano "$1"
type_e_type_is_not_completly_defined=04042_E_Typ "$1" nie jest kompletnie zdefiniowany
type_w_string_too_long=04043_W_Przekroczona maksymalna d³ugo¶æ ³añcucha
% The size of the constant string, which is assigned to a shortstring,
% is longer than the maximum size of the shortstring
type_w_signed_unsigned_always_false=04044_W_To porównanie zawsze zwraca fa³sz z powodu zakresów warto¶ci zmiennych
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% false. Exlicitly typecast the constant to the correct range to avoid this problem.
type_w_signed_unsigned_always_true=04045_W_To porównanie zawsze zwraca fa³sz z powodu zakresów warto¶ci zmiennych
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% true. Exlicitly typecast the constant to the correct range to avoid this problem.
type_w_instance_with_abstract=04046_W_Tworzenie klasy "$1" posiadaj±cej metody abstrakcyjne
% An instance of a class is created which contains non-implemented abstract
% methods. This will probably lead to a runtime error 211 in the code if that
% routine is ever called. All abstract methods should be overridden.
type_h_in_range_check=04047_H_Lewy operand operatora IN powinien byæ wielko¶ci jednego bajta
% The left operand of the \var{in} operator is not an ordinal or enumeration which fits
% within 8-bits, this may lead to range check errors. The \var{in} operator
% currently only supports a left operand which fits within a byte. In the case of
% enumerations, the size of an element of an enumeration can be controlled with
% the \var{\{\$PACKENUM\}} or \var{\{\$Zn\}} switches.
type_w_smaller_possible_range_check=04048_W_Typy ró¿nej wielko¶ci - mo¿liwa utrata danych lub b³±d zakresu
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_h_smaller_possible_range_check=04049_H_Typy ró¿nej wielko¶ci - mo¿liwa utrata danych lub b³±d zakresu
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_e_cant_take_address_of_abstract_method=04050_E_Nie mo¿na uzyskaæ adresu metody abstrakcyjnej
% An abstract method has no body, so the address of an abstract method can't be taken.
type_e_operator_not_allowed=04051_E_Operator nie pasuje do operandu
% You are trying an operator that is not available for the type of the
% operands
type_e_constant_expr_expected=04052_E_Oczekiwano sta³ego wyra¿enia
% The compiler expects an constant expression, but gets a variable expression.
type_e_operator_not_supported_for_types=04053_E_Operacja "$1" niedozwolona dla typów "$2" i "$3"
% The operation is not allowed for the supplied types
type_e_illegal_type_conversion=04054_E_Nieprawid³owa konwersja typów: "$1" do "$2"
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
type_h_pointer_to_longint_conv_not_portable=04055_H_Konwersja pomiêdzy typami ca³kowitymi i wska¼nikami jest nieprzeno¶na
% If you typecast a pointer to a longint (or vice-versa), this code will not compile
% on a machine using 64-bit for pointer storage.
type_w_pointer_to_longint_conv_not_portable=04056_W_Konwersja pomiêdzy typami ca³kowitymi i wska¼nikami jest nieprzeno¶na
% If you typecast a pointer to a ordinal type of a different size (or vice-versa), this can
% cause problems. This is a warning to help finding the 32bit specific code where cardinal/longint is used
% to typecast pointers to ordinals. A solution is to use the ptrint/ptruint types instead.
type_e_cant_choose_overload_function=04057_E_Nie mo¿na okre¶liæ, któr± z przeci±¿onych funkcji wywo³aæ
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
type_e_illegal_count_var=04058_E_Nieprawid³owy typ licznika pêtli
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
% \end{description}
#
# Symtable
#
# 05055 is the last one used
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Nie odnaleziono identyfikatora $1
% The compiler doesn't know this symbol. Usually happens when you misspel
% the name of a variable or procedure, or when you forgot to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_B³±d wewnêtrzny w SymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_Zduplikowany identyfikator $1
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_Identyfikator ju¿ zdefiniowany w $1 w lini $2
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_Nieznany identyfikator $1
% The identifier encountered has not been declared, or is used outside the
% scope where it is defined.
sym_e_forward_not_resolved=05005_E_Deklaracaja naprzód nie rozwi±zana $1
% This can happen in two cases:
% \begin{itemize}
% \item This happens when you declare a function (in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_e_error_in_type_def=05007_E_B³±d w definicji typu
% There is an error in your definition of a new array type:
% \item One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_forward_type_not_resolved=05009_E_Brak definicji typu $1
% A symbol was forward defined, but no declaration was encountered.
sym_e_only_static_in_static=05010_E_Tylko zmienne statyczne mog± byæ u¿ywane w metodach statycznych
% A static method of an object can only access static variables.
sym_e_type_must_be_rec_or_object_or_class=05012_E_Record or object or class type expected
% The variable or expression isn't of the type \var{record} or \var{object} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Instancje klas i obiektów z metodami abstrakcyjnymi niedozwolone
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_Etykieta nie zosta³a zdefiniowana $1
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_Etykieta niezdefiniowana przed u¿yciem $1
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_Nieprawid³owa deklaracja etykiety
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO i LABEL nieobs³ugiwane (u¿yj opcji -Sg)
% You must compile a program which has \var{label}s and \var{goto} statements
% with the \var{-Sg} switch. By default, \var{label} and \var{goto} aren't
% supported.
sym_e_label_not_found=05018_E_Nie odnaleziono etykiety
% A \var{goto label} was encountered, but the label isn't declared.
sym_e_id_is_no_label_id=05019_E_Identyfikator nie jest etykiet±
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_Powtórna definicja etykiety
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_Nieprawid³owa deklaracja typu zbiorowego
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_Brak implementacji klasy $1
% You declared a class, but you didn't implement it.
sym_n_unit_not_used=05023_H_Modu³ $1 nieu¿ywany w $2
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_Parametr $1 nieu¿ywany
% The identifier was declared (locally or globally) but
% was not used (locally or globally).
sym_n_local_identifier_not_used=05025_N_Lokalna zmienna $1 nieu¿ywana
% You have declared, but not used a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_Warto¶c przypisana do $1 nie jest nigdzie u¿yta
% The identifier was declared (locally or globally)
% set but not used (locally or globally).
sym_n_local_identifier_only_set=05027_N_Warto¶c przypisana do $1 nie jest nigdzie u¿yta
% The variable in a procedure or function
% implementation is declared, set but never used.
sym_h_local_symbol_not_used=05028_H_Lokalna(y) $1 $2 nie zosta³(a) u¿yta(y)
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_Prywatne pole $1.$2 nieu¿ywane
sym_n_private_identifier_only_set=05030_N_Warto¶c przypisana do lokalnego pola $1.$2 nie jest nigdzie u¿yta
sym_n_private_method_not_used=05031_N_Metoda prywatna $1.$2 nieu¿ywana
sym_e_set_expected=05032_E_Oczekiwano typu zbiorowego
% The variable or expression is not of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_Nie wygl±da na to, ¿eby warto¶æ zwracana przez funkcjê by³a ustawiona
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_Typ $1 jest ¼le wyrównany dla C w bie¿±cym rekordzie
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_Nieznany identyfikator pola rekordu $1
% The field doesn't exist in the record definition.
sym_n_uninitialized_local_variable=05036_W_Nie wygl±da na to, ¿eby lokalna zmienna $1 by³a zainicjowana
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_n_uninitialized_variable=05037_W_Nie wygl±da na to, ¿eby zmienna $1 by³a zainicjowana
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_e_id_no_member=05038_E_Identyfikator nie wskazuje ¿adnej metody ani pola $1
% This error is generated when an identifier of a record,
% field, or method is accessed while it is not defined.
sym_h_param_list=05039_H_Znaleziono deklaracjê: $1
% You get this when you use the \var{-vb} switch. In case an overloaded
% procedure is not found, then all candidate overloaded procedures are
% listed, with their parameter lists.
sym_e_segment_too_large=05040_E_Element jest zbyt du¿y
% You get this when you declare a data element whose size exceeds the
% prescribed limit (2 Gb on 80386+/68020+ processors)
sym_e_no_matching_implementation_found=05042_E_Nie znaleziono implementacji metody interfejsu "$1"
% There was no matching method found which could implement the interface
% method. Check argument types and result type of the methods.
sym_w_deprecated_symbol=05043_W_Symbol "$1" jest przestarza³y
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Usage of this symbol
% should be avoided as much as possible.
sym_w_non_portable_symbol=05044_W_Symbolu "$1" nie da siê przenie¶æ na inne platformy
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{platform} is used. This symbol's value, usage
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_non_implemented_symbol=05055_W_Symbol "$1" nie jest zaimplementowany
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{unimplemented} is used. This symbol is defined,
% but is not yet implemented on this specific platform.
sym_e_cant_create_unique_type=05056_E_Nie mo¿na utworzyæ unikalnego typu z tego typu
% Only simple types like ordinal, float and string types are supported when
% redefining a type with \var{type newtype = type oldtype;}.
sym_h_uninitialized_local_variable=05057_H_Zmienna lokalna "$1" mo¿e nie byæ zainicjalizowana
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_h_uninitialized_variable=05058_H_Zmienna "$1" mo¿e nie byæ zainicjalizowana
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
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
cg_e_parasize_too_big=06009_E_Rozmiar listy parametrów przekracza 65535 bajtów
% The I386 processor limits the parameter list to 65535 bytes (the \var{RET}
% instruction causes this)
cg_e_file_must_call_by_reference=06012_E_Zmienne plikowe musz± byæ przekazywane przez referencje
% You cannot specify files as value parameters, i.e. they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_U¿ycie wska¼nika far niedozwolone w tym miejscu
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_dont_call_exported_direct=06015_E_EXPORT declared functions can't be called
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_Mo¿liwe nieprawid³owe wywo³anie konstruktora/destruktora
% The compiler detected that a constructor or destructor is called within a
% a method. This will probably lead to problems, since constructors / destructors
% require parameters on entry.
cg_n_inefficient_code=06017_N_Niewydajny kod
% Your statement seems dubious to the compiler.
cg_w_unreachable_code=06018_W_Kod nie zostanie nigdy wykonany
% You specified a construct which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_cant_call_abstract_method=06020_E_Nie mo¿na bezpo¶rednio wywo³aæ metod abstrakcyjnych
% You cannot call an abstract method directy, instead you must call a
% overriding child method, because an abstract method isn't implemented.
cg_d_register_weight=06027_DL_Rejestr $1 waga $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_d_stackframe_omited=06029_DL_Pominiêto ramkê stosu
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_e_unable_inline_object_methods=06031_E_Metody obiektów i klas nie mog± byæ inline
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_Wywo³ania procedur ze zmiennych proceduralnych nie mog± byæ inline
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_Brak kodu dla procedury inline
% The compiler couldn't store code for the inline procedure.
cg_e_can_access_element_zero=06035_E_Brak dostêpu do zerowego elementu ansi/wide- lub longstringa, u¿yj (set)length
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such kinf of string
cg_e_cannot_call_cons_dest_inside_with=06037_E_Konstruktory i destruktory nie mog± byæ wywo³ywane w klauzuli 'with'
% Inside a \var{with} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_Nie mo¿na wywo³aæ bezpo¶rednio metody obs³uguj±cej komunikaty
% A message method handler method cannot be called directly if it contains an
% explicit self argument
cg_e_goto_inout_of_exception_block=06039_E_Skok do bloku lub poza blok exception
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
cg_e_control_flow_outside_finally=06040_E_Break, Continue i Exit nie s± dozwolone w bloku finally
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
cg_w_parasize_too_big=06041_W_Rozmiar parametrów przekroczy³ limit dla niektórych procesorów
% This indicates that you are declaring more than 64K of parameters, which
% might not be supported on other processor targets.
cg_w_localsize_too_big=06042_W_Rozmiar zmiennych lokalnych przekroczy³ limit dla dla niektórych procesorów
% This indicates that you are declaring more than 32K of lcoal variables, which
% might not be supported on other processor targets.
cg_e_localsize_too_big=06043_E_Rozmiar zmiennych lokalnych przekroczy³ dopuszczalny limit
% This indicates that you are declaring more than 32K of lcoal variables, which
% is not supported by this processor.
cg_e_break_not_allowed=06044_E_BREAK mo¿na u¿yæ tylko wewn±trz pêtli
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06045_E_CONTINUE mo¿na u¿yæ tylko wewn±trz pêtli
% You're trying to use \var{continue} outside a loop construction.
% \end{description}
# EndOfTeX

#
# Assembler reader
#
# 07097 is the last used one
#
asmr_d_start_reading=07000_DL_Rozpoczêcie parsowania bloku asemblera w stylu $1
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_DL_Zakoñczenie parsowania bloku asemblera w stylu $1
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_¯aden identyfikator (poza etykietami) nie mo¿e zawieraæ @
% A identifier which isn't a label can't contain a @.
asmr_e_building_record_offset=07004_E_B³±d tworzenia offsetu rekordu
% There has an error occured while building the offset of a record/object
% structure, this can happend when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_OFFSET u¿yty bez identyfikatora
% You can only use OFFSET with an identifier. Other syntaxes aren't
% supported
asmr_e_type_without_identifier=07006_E_TYPE u¿yte bez identyfikatora
% You can only use TYPE with an identifier. Other syntaxes aren't
% supported
asmr_e_no_local_or_para_allowed=07007_E_Nie mo¿na u¿ywaæ lokalnych zmiennych i parametrów w tym miejscu
% You can't use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the frame pointer register so the
% address can't be obtained directly.
asmr_e_need_offset=07008_E_Nale¿y tu u¿yæ OFFSET
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_Nale¿y u¿yæ tu $
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Mo¿na u¿yæ tylko jednego symbolu relokacyjnego
% You can't have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_Symbole relokacyjne mog± byæ tylko dodane
% Relocatable symbols (variable/typed constant) can't be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_Nieprawid³owe sta³e wyra¿enie
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_Nie mo¿na u¿yæ symbolu relokacyjnego w tym miejscu
% You can't use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_Nieprawid³owa sk³adnia referencji
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_Nie mo¿na osi±gn±æ $1 z tego kodu
% You can not read directly the value of local or para
% of a higher level in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_Lokalne symbole/etykiety nie s± dozwolone jako referencje
% You can't use local symbols/labels as references
asmr_e_wrong_base_index=07017_E_Nieprawid³owe u¿ycie rejestru indeksu i bazy
% There is an error with the base and index register, they are
% probably incorrect
asmr_w_possible_object_field_bug=07018_W_Mo¿liwy b³±d w obs³ugiwaniu pola obiektu
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_Nieprawid³owa skala
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_U¿ycie wielu rejestrów indeksu
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_Nieprawid³owy typ operandu
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_Nieprawid³owy string jako operand: $1
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE i @DATA nieobs³ugiwane
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_Puste etykiety referencji s± niedozwolone
asmr_e_expr_zero_divide=07025_E_Dzielenie przez zero
% There is a division by zero in a constant expression
asmr_e_expr_illegal=07026_E_Nieprawid³owe wyra¿enie
% There is an illegal expression in a constant expression
asmr_e_escape_seq_ignored=07027_E_Sekwencja zignorowana: $1
% There is a C-styled string, but the escape sequence in the string
% is unknown, and is therefore ignored
asmr_e_invalid_symbol_ref=07028_E_Nieprawid³owa referencja symbolu
asmr_w_fwait_emu_prob=07029_W_Fwait mo¿e powodowaæ problemy emulacji z emu387
asmr_w_fadd_to_faddp=07030_W_$1 bez operandu przet³umaczone na $1P
asmr_w_enter_not_supported_by_linux=07031_W_Instrukcja ENTER nie jest obs³ugiwana przez j±dro Linuksa
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_Wywo³anie przeci±¿onej funkcji z poziomu asemblera
% There is a call to an overloaded method in the assembler block,
% this might be the sign there is a problem
asmr_e_unsupported_symbol_type=07033_E_Nieobs³ugiwany typ symbolu dla operandu
asmr_e_constant_out_of_bounds=07034_E_Warto¶æ sta³a przekracza zakres
asmr_e_error_converting_decimal=07035_E_B³±d konwersji liczby dziesiêtnej $1
% A constant decimal value does not have the correct syntax
asmr_e_error_converting_octal=07036_E_B³±d konwersji liczby ósemkowej $1
% A constant octal value does not have the correct syntax
asmr_e_error_converting_binary=07037_E_B³±d konwersji liczby dwójkowej $1
% A constant binary value does not have the correct syntax
asmr_e_error_converting_hexadecimal=07038_E_B³±d konwersji liczby szesnastkowej $1
% A constant hexadecimal value does not have the correct syntax
asmr_h_direct_global_to_mangled=07039_H_$1 przet³umaczone na $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 jest przypisane do przeci±¿onej funkcji
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Nie mo¿na u¿yæ SELF poza metod±
% There is a reference to the \var{self} symbol while it is not
% allowed. \var{self} can only be referenced inside methods
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Nie mo¿na u¿yæ OLDEBP poza zagnie¿d¿on± procedur±
% There is a reference to the \var{oldebp} symbol while it is not
% allowed. \var{oldebp} can only be referenced inside nested routines
asmr_e_void_function=07043_W_Procedury nie mog± zwracaæ warto¶ci w kodzie asemblera
% Trying to return a value while in a procedure. A procedure
% does not have any return value
asmr_e_SEG_not_supported=07044_E_SEG nie obs³ugiwane
asmr_e_size_suffix_and_dest_dont_match=07045_E_Przyrostek okre¶laj±cy rozmiar nie pasuje do ¼ród³a lub celu
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_w_size_suffix_and_dest_dont_match=07046_W_Przyrostek okre¶laj±cy rozmiar nie pasuje do ¼ród³a lub celu
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_e_syntax_error=07047_E_B³±d sk³adni asemblera
% There is an assembler syntax error
asmr_e_invalid_opcode_and_operand=07048_E_Nieprawid³owa kombinacja opkodu i operandu
% The opcode cannot be used with this type of operand
asmr_e_syn_operand=07049_E_B³±d sk³adni asemblera w operandzie
asmr_e_syn_constant=07050_E_B³±d sk³adni asemblera w sta³ej
asmr_e_invalid_string_expression=07051_E_Nieprawid³owe wyra¿enie ze stringiem
asmr_w_const32bit_for_address=07052_W_Sta³a $1 prawdopodobnie nie powinna byæ wska¼nikiem
% A constant expression represents an address which does not fit
% into a pointer. The address is probably incorrect
asmr_e_unknown_opcode=07053_E_Nieznany opkod $1
% This opcode is not known
asmr_e_invalid_or_missing_opcode=07054_E_Nieprawid³owy opkod lub brak opkodu
asmr_e_invalid_prefix_and_opcode=07055_E_Nieprawid³owa kombinacja prefiksu i opkodu: $1
asmr_e_invalid_override_and_opcode=07056_E_Nieprawid³owa kombinacja override i opkodu: $1
asmr_e_too_many_operands=07057_E_Zbyt du¿o operandów
% There are too many operands for this opcode. Check your
% assembler syntax
asmr_w_near_ignored=07058_W_NEAR zignorowane
asmr_w_far_ignored=07059_W_FAR zignorowane
asmr_e_dup_local_sym=07060_E_Zduplikowany symbol lokalny $1
asmr_e_unknown_local_sym=07061_E_Niezdefiniowany symbol lokalny $1
asmr_e_unknown_label_identifier=07062_E_Nieznana etykieta $1
asmr_e_invalid_register=07063_E_Nieprawid³owa nazwa rejestru
% There is an unknown register name used as operand.
asmr_e_invalid_fpu_register=07064_E_Nieprawid³owa nazwa rejestru zmiennoprzecinkowego
% There is an unknown register name used as operand.
asmr_w_modulo_not_supported=07066_W_Modulo nieobs³ugiwane
asmr_e_invalid_float_const=07067_E_Nieprawid³owa sta³a zmiennoprzecinkowa $1
% The floating point constant declared in an assembler block is
% invalid.
asmr_e_invalid_float_expr=07068_E_Nieprawid³owe wyra¿enie zmiennoprzecinkowe
% The floating point expression declared in an assembler block is
% invalid.
asmr_e_wrong_sym_type=07069_E_Nieprawid³owy typ symbolu
asmr_e_cannot_index_relative_var=07070_E_Nie mo¿na ideksowaæ rejestrem zmiennej lokalnej/parametru
% Trying to index using a base register a symbol which is already relative
% to a register. This is not possible, and will probably lead to crashes.
asmr_e_invalid_seg_override=07071_E_Invalid segment override expression
asmr_w_id_supposed_external=07072_W_Identifier $1 supposed external
% There is a reference to an undefined symbol. This will not result
% in an error, since the symbol might be external, but may cause
% problems at link time if the symbol is not defined anywhere.
asmr_e_string_not_allowed_as_const=07073_E_Stringi nie mog± byæ sta³ymi
% Character strings are not allowed as constants.
asmr_e_no_var_type_specified=07074_Nie okre¶lono typu zmiennej
% The syntax expects a type idenfitifer after the dot, but
% none was found.
asmr_w_assembler_code_not_returned_to_text=07075_E_brak powrotu do sekcji text w kodzie asemblera
% There was a directive in the assembler block to change sections,
% but there is a missing return to the text section at the end
% of the assembler block. This might cause errors during link time.
asmr_e_not_directive_or_local_symbol=07076_E_To nie jest ani dyrektywa, ani symbol lokalny: $1
% This symbol is unknown.
asmr_w_using_defined_as_local=07077_E_U¿ycie zdefiniowanej nazwy jako lokalnej etykiety
asmr_e_dollar_without_identifier=07078_E_Brak identyfikatora przy znaku dolara
% A constant expression has an identifier which does not start with
% the $ symbol.
asmr_w_32bit_const_for_address=07079_W_32bitowa sta³a u¿yta jako adres
% A constant was used as an address. This is probably an error,
% since using absolute addresses will probably not work.
asmr_n_align_is_target_specific=07080_N_.align nie jest przeno¶ne miêdzy platformami, u¿yj .balign lub .p2align
% Using the .align directive is platform specific, and its meaning will vary
% from one platform to another.
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Nie mo¿na odwo³ywaæ siê bezpo¶rednio do pól parametrów
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_Nie mo¿na odwo³ywaæ siê bezpo¶rednio do pól obiektów/klas
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
asmr_e_unable_to_determine_reference_size=07083_E_Nie okre¶lono rozmiaru operandu
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference.
asmr_e_cannot_use_RESULT_here=07084_E_Nie mo¿na u¿yæ RESULT w tej funkcji
% Some functions which return complex types cannot use the \var{result}
% keyword.
asmr_w_adding_explicit_args_fXX=07086_W_"$1" bez operandu przet³umaczone na "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" przet³umaczone na "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" przet³umaczone na "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_Znak < niedozwolony w tym miejscu
% The shift operator requires the << characters. Only one
% of those characters was found.
asmr_e_invalid_char_greater=07090_E_Znak > niedozwolony w tym miejscu
% The shift operator requires the >> characters. Only one
% of those characters was found.
asmr_w_align_not_supported=07093_W_ALIGN nieobs³ugiwane
asmr_e_no_inc_and_dec_together=07094_E_Inc i Dec nie mog± wystêpowaæ razem
% Trying to use an increment and a decrement within the same
% opcode on the 680x0. This is impossible.
asmr_e_invalid_reg_list_in_movem=07095_E_Nieprawid³owa lista rejestrów dla movem
% Trying to use the \var{movem} opcode with invalid registers
% to save or restore.
asmr_e_invalid_reg_list_for_opcode=07096_E_Nieprawid³owa lista rejestrów dla opkodu
asmr_e_higher_cpu_mode_required=07097_E_Ta instrukcja wymaga wy¿szego typu procesora ($1)
% Trying to use an instruction which is not supported in the current
% cpu mode. Use a higher cpu generation to be able to use this
% opcode in your assembler block
asmr_w_unable_to_determine_reference_size_using_dword=07098_W_Nie okre¶lono rozmiaru operandów, domy¶lne u¿ycie DWORD
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use DWORD as default.
asmr_e_illegal_shifterop_syntax=07099_E_B³±d sk³adni przy operandzie shifter
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
asmw_f_too_many_asm_files=08000_F_Zbyt du¿o plików asemblera
% With smartlinking enabled, there are too many assembler
% files generated. Disable smartlinking.
asmw_f_assembler_output_not_supported=08001_F_Wybrany typ wyj¶ciowy asemblera nieobs³ugiwany
asmw_f_comp_not_supported=08002_F_Comp nieobs³ugiwany
asmw_f_direct_not_supported=08003_F_Direct nieobs³ugiwane przy zapisywaniu bezpo¶rednim
% Direct assembler mode is not supported for binary writers.
asmw_e_alloc_data_only_in_bss=08004_E_Alokacja danych jest dozwolona tylko w sekcji bss
asmw_f_no_binary_writer_selected=08005_F_Nie wybrano sposobu zapisywania binariów
asmw_e_opcode_not_in_table=08006_E_Asm: Brak opkodu $1 w tablicy
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 nieprawid³owa kombinacja opkodu i operandów
asmw_e_16bit_not_supported=08008_E_Asm: 16bitowe referencje nieobs³ugiwane
asmw_e_invalid_effective_address=08009_E_Asm: Nieprawid³owy adres efektywny
asmw_e_immediate_or_reference_expected=08010_E_Asm: Immediate or reference expected
asmw_e_value_exceeds_bounds=08011_E_Asm: $1 warto¶æ przekracza zakres $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: Krótki skok poza zasiêg $1
asmw_e_undefined_label=08013_E_Asm: Nieznana etykieta $1
asmw_e_comp_not_supported=08014_E_Asm: Typ Comp nieobs³ugiwany dla tego celu
asmw_e_extended_not_supported=08015_E_Asm: Typ Extended nieobs³ugiwany dla tego celu
asmw_e_duplicate_label=08016_E_Asm: Zduplikowana etykieta $1
asmw_e_redefined_label=08017_E_Asm: Etykieta powtórnie zdefiniowana $1
asmw_e_first_defined_label=08018_E_Asm: Najpierw zdefiniowana tutaj
asmw_e_invalid_register=08019_E_Asm: Nieprawid³owy rejestr $1

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
exec_w_source_os_redefined=09000_W_Zmieniono ¼ród³owy system operacyjny
exec_i_assembling_pipe=09001_I_Asemblowanie (potok) $1
exec_d_cant_create_asmfile=09002_E_Nie mo¿na utworzyæ plików asemblera: $1
% The mentioned file can't be created. Check if you have got
% access permissions to create this file
exec_e_cant_create_objectfile=09003_E_Nie mo¿na utworzyæ pliku: $1
% The mentioned file can't be created. Check if you've
% got access permissions to create this file
exec_e_cant_create_archivefile=09004_E_Nie mo¿na utworzyæ pliku: $1
% The mentioned file can't be created. Check if you've
% access permissions to create this file
exec_e_assembler_not_found=09005_E_Assembler $1 nie znaleziony, prze³±czenie na zewnêtrzn± asemblacjê
exec_t_using_assembler=09006_T_U¿ywanie assemblera: $1
exec_e_error_while_assembling=09007_E_B³±d asemblera, kod wyj¶cia $1
% There was an error while assembling the file using an external assembler.
% Consult the documentation of the assembler tool to find out more information
% on this error.
exec_e_cant_call_assembler=09008_E_Nie mo¿na wywo³aæ asemblera, b³±d $1 prze³±czenie na zewnêtrzn± asemblacjê
exec_i_assembling=09009_I_Asemblowanie $1
exec_i_assembling_smart=09010_I_Asemblowanie do linkowania smart $1
exec_w_objfile_not_found=09011_W_Obiekt $1 nieodnaleziony, linkowanie mo¿e siê nie powie¶æ!
% One of the object file is missing, and linking will probably fail.
% Check your paths.
exec_w_libfile_not_found=09012_W_Biblioteka $1 nieodnaleziona, Linkowanie mo¿e siê nie powie¶æ!
% One of the library file is missing, and linking will probably fail.
% Check your paths.
exec_e_error_while_linking=09013_E_B³±d podczas linkowania
% Generic error while linking.
exec_e_cant_call_linker=09014_E_Nie mo¿na wywo³aæ linkera, prze³±czenie na zewnêtrzne linkowanie
exec_i_linking=09015_I_Linkowanie $1
exec_e_util_not_found=09016_E_Narzêdzie $1 nieodnalezione, prze³±czenie na zewnêtrzne linkowanie
exec_t_using_util=09017_T_Uruchamianie $1
exec_e_exe_not_supported=09018_E_Tworzenie plików wykonywalnych nieobs³ugiwane
exec_e_dll_not_supported=09019_E_Tworzenie bibliotek dynamicznych/wspó³dzielonych nieobs³ugiwane
exec_i_closing_script=09020_I_Zamykanie skryptu $1
exec_e_res_not_found=09021_E_Kompilator zasobów nieodnaleziony, prze³±czenie na tryb zewnêtrzny
exec_i_compilingresource=09022_I_Kompilacja zasobu $1
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_Modu³ $1 nie mo¿e byæ linkowany statycznie, prze³±czenie na linkowanie smart
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_Modu³ $1 nie moze byæ linkowany smart, prze³±czenie na linkowanie statyczne
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_Modu³ $1 nie mo¿e byæ linkowany dynamicznie, prze³±czenie na linkowanie statyczne
exec_e_unit_not_smart_or_static_linkable=09026_E_Modu³ $1 nie mo¿e byæ linkowany statycznie ani smart
exec_e_unit_not_shared_or_static_linkable=09027_E_Modu³ $1 nie mo¿e byæ linkowany statycznie ani dynamicznie
%\end{description}
# EndOfTeX

#
# Executable information
#
execinfo_f_cant_process_executable=09128_F_Nie mo¿na przetwarzaæ pliku $1
execinfo_f_cant_open_executable=09129_F_Nie mo¿na otworzyæ pliku wykonywalnego $1
execinfo_x_codesize=09130_X_Rozmiar kodu: $1 bajtów
execinfo_x_initdatasize=09131_X_Rozmiar zainicjowanych danych: $1 bajtów
execinfo_x_uninitdatasize=09132_X_Rozmiar niezainicjowanych danych: $1 bajtów
execinfo_x_stackreserve=09133_X_Zarezerwowany rozmiar stosu: $1 bajtów
execinfo_x_stackcommit=09134_X_Aktywny rozmiar stosu: $1 bajtów

#
# Unit loading
#
# 10041 is the last used one
#
# BeginOfTeX
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these mesages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_Poszukiwanie modu³u: $1
% When you use the \var{-vt}, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_Otwieranie PPU $1
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_Nazwa PPU: $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_Flagi PPU: $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_Crc PPU: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_Czas PPU: $1
% When you use the \var{-vu} flag, the time the unit was compiled is shown.
unit_u_ppu_file_too_short=10006_U_Plik PPU zbyt krótki
% The ppufile is too short, not all declarations are present.
unit_u_ppu_invalid_header=10007_U_Nieprawid³owy nag³ówek PPU (brak PPU na pocz±tku)
% A unit file contains as the first three bytes the ascii codes of \var{PPU}
unit_u_ppu_invalid_version=10008_U_Nieprawid³owa wersja PPU $1
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_PPU jest skompilowany dla innego procesora
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_invalid_target=10010_U_PPU skompilowany dla innej platformy docelowej
% This unit file was compiled for a different target, and
% cannot be read
unit_u_ppu_source=10011_U_¬ród³o PPU: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_write=10012_U_Zapisywanie $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_Nie mo¿na zapisaæ pliku PPU
% An error occurred when writing the unit file.
unit_f_ppu_read_error=10014_F_B³±d podczas czytania pliku PPU
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_read_unexpected_end=10015_F_Nieoczekiwane zakoñczenie pliku PPU
% Unexpected end of file. This may mean that the PPU file is
% corrupted.
unit_f_ppu_invalid_entry=10016_F_Nieprawid³owy plik PPU: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_B³±d podczas liczenia PPU Dbx
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_Nieprawid³owa nazwa modu³u: $1
% The name of the unit does not match the file name.
unit_f_too_much_units=10019_F_Zbyt du¿o modu³ów
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{files.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_Zapêtlone nawi±zanie pomiêdzy modu³ami $1 i $2
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_Nie mo¿na skompilowaæ modu³u $1, brak ¼róde³
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_Nie mo¿na znale¼æ modu³u $1
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your config files for the unit pathes
unit_w_unit_name_error=10023_W_Nie znaleziono modu³u $1 ale istnieje $2
unit_f_unit_name_error=10024_F_Szukano modu³u $1 ale znaleziono tylko $2
% Dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_Kompilowanie modu³u System wymaga u¿ycia prze³±cznika -Us
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} must be specified.
unit_f_errors_in_unit=10026_F_Napotkano $1 b³êdów podczas kompilacji modu³u, zatrzymanie
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_£adowanie z $1 ($2) modu³u $3
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_Rekompilacja $1, suma kontrolna zmieniona dla $2
unit_u_recompile_source_found_alone=10029_U_Rekompilacja $1, znaleziono tylko ¼ród³o
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_Rekompilacja modu³u, biblioteka statyczna jest starsza ni¿ plik ppu
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit are older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_Rekompilacja modu³u, biblioteka wspó³dzielona jest starsza ni¿ plik ppu
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit are older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_Rekompilacja modu³u, obj i asm s± starsze ni¿ plik ppu
% When you use the \var{-vu} flag, the compiler warns if the assembler or
% object file of the unit are older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_Rekompilacja modu³u, obj jest starszy ni¿ asm
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_parsing_interface=10034_U_Parsowanie czê¶ci interface modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit
unit_u_parsing_implementation=10035_U_Parsowanie czê¶ci implementation modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit
unit_u_second_load_unit=10036_U_Drugie za³adowanie pliku $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_check_time=10037_U_Sprawdzanie pliku PPU: $1 czas: $2
% When you use the \var{-vu} flag, the compiler show the filename and
% date and time of the file which a recompile depends on
### The following two error msgs is currently disabled.
#unit_h_cond_not_set_in_last_compile=10038_H_Warunek $1 nie by³ ustawiony na pocz±tku ostatniej kompilacji modu³u $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that currently is defined, but was not used the last
#% time the unit was compiled.
#unit_h_cond_set_in_last_compile=10039_H_Warunek $1 nie by³ ustawiony na pocz±tku ostatniej kompilacji modu³u $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that was used the last time the unit was compiled, but
#% the conditional is currently not defined.
unit_w_cant_compile_unit_with_changed_incfile=10040_W_Nie mo¿na skompilowaæ modu³u $1, znaleziono zmodyfikowane pliki include
% A unit was found to have modified include files, but
% some source files were not found, so recompilation is impossible.
unit_h_source_modified=10041_H_Plik $1 jest nowszy ni¿ plik $2
% A modified source file for a unit was found that was compiled with the
% release flag (-Ur). The unit will not implicitly be recompiled because
% this release flag is set.
unit_u_ppu_invalid_fpumode=10042_U_U¿ycie modu³u skompilowanego z innym formatem zapisu liczb zmiennoprzecinkowych
% Trying to compile code while using units which were not compiled with
% the same floating point format mode. Either all code should be compiled
% with FPU emulation on, or with FPU emulation off.
unit_u_loading_interface_units=10043_U_£adowanie modu³ów z czê¼ci interface modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the interface part of the unit.
unit_u_loading_implementation_units=10044_U_£adowanie modu³ów z czê¼ci implementation modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the implementation part of the unit.
unit_u_interface_crc_changed=10045_U_Zmieniona suma CRC sekcji interface modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated for the interface has been changed after the implementation
% has been parsed.
unit_u_implementation_crc_changed=10046_U_Zmieniona suma CRC sekcji implementation modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated has been changed after the implementation
% has been parsed.
unit_u_finished_compiling=10047_U_Zakoñczono kompilacjê modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it
% has finished compiling the unit.
unit_u_add_depend_to=10048_U_Dodanie zale¿no¶ci $1 do $2
% When you use the \var{-vu} flag, the compiler warns that it
% has added a dependency between the two units.
unit_u_no_reload_is_caller=10049_U_Ominiêcie prze³adowania modu³u, to ten sam modu³: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is the unit that wants
% to load this unit
unit_u_no_reload_in_second_compile=10050_U_Ominiêcie prze³adowania modu³u, trwa druga kompilacaja: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is already in a second recompile
unit_u_flag_for_reload=10051_U_Flaga do prze³adowania: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to reload the unit
unit_u_forced_reload=10052_U_Wymuszone prze³adowanie modu³u
% When you use the \var{-vu} flag, the compiler warns that it
% has is reloading the unit because it was required
unit_u_previous_state=10053_U_Poprzedni stan modu³u $1: $2
% When you use the \var{-vu} flag, the compiler shows the
% previous state of the unit
unit_u_second_compile_unit=10054_U_Ju¿ kompilowane $1, ustawienie drugiej kompilacji
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_loading_unit=10055_U_£adowanie modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the unit.
unit_u_finished_loading_unit=10056_U_Zakoñczenie ³adowania modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it finished
% loading the unit.
unit_u_registering_new_unit=10057_U_Rejestrowanie nowego modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it has
% found a new unit and registers it in the internal lists.
unit_u_reresolving_unit=10058_U_Powtórne odnajdowanie modu³u $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to recalculate the internal data of the unit
unit_u_skipping_reresolving_unit=10059_U_Ominiêcie powtórnego odnajdowania modu³u $1, ci±gle ³aduje u¿ywane modu³y
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
option_usage=11000_O_$1 [opcje] <nazwapliku> [opcje]
# BeginOfTeX
%
% \section{Command-line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_only_one_source_support=11001_W_Obs³ugiwany tylko jeden plik ¼ród³owy
% You can specify only one source file on the command line. The first
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_Plik DEF mo¿e byæ stworzony tylko pod OS/2
% This option can only be specified when you're compiling for OS/2
option_no_nested_response_file=11003_E_Zagnie¿d¿enia plików nieobs³ugiwane
% you cannot nest response files with the \var{@file} command-line option.
option_no_source_found=11004_F_Brak nazwy pliku w lini poleceñ
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_Brak opcji w pliku konfiguracyjnym $1
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_Nieprawid³owy parametr: $1
% You specified an unknown option.
option_help_pages_para=11007_H_-? wy¶wietla pomoc
% When an unknown option is given, this message is diplayed.
option_too_many_cfg_files=11008_F_Za du¿o zagnie¿d¿onych plików konfiguracyjnych
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_Nie mo¿na otworzyæ pliku konfiguracyjnego $1
% The option file cannot be found.
option_reading_further_from=11010_D_Czytanie dalszych opcji z $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_Cel jest ju¿ ustawiony na: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_Biblioteki wspó³dzielone nie s± obs³ugiwane w DOSie, reverting to static
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos
option_too_many_ifdef=11013_F_Za du¿o IF(N)DEF
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_Za du¿o ENDIF
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_Warunek wci±¿ otwarty na koñcu pliku
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_Generowanie informacji do debugowania nieobs³ugiwane przez t± wersjê kompilatora
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_Spróbuj przekompilowaæ z -dGDB
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_E_U¿ycie przedawnionego prze³±cznika $1
% this warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the switch meaning may change.
option_obsolete_switch_use_new=11019_E_U¿ycie przedawnionego prze³±cznika $1, proszê u¿yæ $2
% this warns you when you use a switch that is not supported anymore. You
% must now use the second switch instead.
% It is recommended that you change the switch to overcome problems in the
% future, when the switch meaning may change.
option_switch_bin_to_src_assembler=11020_N_Prze³±czenie na domy¶lny zapis asemblera
% this notifies you that the assembler has been changed because you used the
% -a switch which can't be used with a binary assembler writer.
option_incompatible_asm=11021_W_Wybrane wyj¶cie asemblera "$1" nie jest zgodne z "$2"
option_asm_forced=11022_W_Wymuszenie u¿ycia asemblera "$1"
% The assembler output selected can not generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
option_using_file=11026_T_Czytanie opcji z pliku $1
% Options are also read from this file
option_using_env=11027_T_Czytanie opcji ze zmiennej ¶rodowiskowej $1
% Options are also read from this environment string
option_handling_option=11028_D_Obs³ugiwanie opcji "$1"
% Debug info that an option is found and will be handled
option_help_press_enter=11029__*** naci¶nij enter ***
option_start_reading_configfile=11030_H_Rozpoczêto czytanie pliku konfiguracyjnego $1
% Starting of config file parsing.
option_end_reading_configfile=11031_H_Zakoñczono czytanie pliku konfiguracyjnego $1
% End of config file parsing.
option_interpreting_option=11032_D_Interpretowanie opcji "$1"
option_interpreting_firstpass_option=11036_D_Interpretowanie opcji "$1"
option_interpreting_file_option=11033_D_Interpretowanie opcji plikowej "$1"
option_read_config_file=11034_D_Czytanie pliku konfiguracyjnego "$1"
option_found_file=11035_D_Znaleziono nazwê pliku ¼ród³owego "$1"
% Additional infos about options, displayed
% when you have debug option turned on.
option_code_page_not_available=11039_E_Nieznana strona kodowa
%\end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_logo=11023_[
Free Pascal Compiler wersja $FPCFULLVERSION [$FPCDATE] dla $FPCCPU
Copyright (c) 1993-2020 by Florian Klaempfl and others
]

#
# Info (option -i)
#
option_info=11024_[
Free Pascal Compiler wersja $FPCVERSION

Data kompilatora  : $FPCDATE
Platforma docelowa: $FPCCPU

Wspierane platformy:
  $OSTARGETS
  
Wspierane instrukcje CPU:
  $INSTRUCTIONSETS

Wspierane instrukcje FPU:
  $FPUINSTRUCTIONSETS
  
Ten program jest oparty na GNU General Public Licence
Przeczytaj COPYING.v2 aby dowiedzieæ siê wiêcej

Zg³aszanie b³êdów, sugestii itp.
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
**0*_wpisz + po prze³±czniku aby go aktywowaæ, - aby dezaktywowaæ
**1a_kompilator nie usuwa wygenerowanego pliku asemblera
**2al_wypisuje numery linii do pliku asemblera
**2an_wypisuje informacje o wêz³ach do pliku asemblera
*L2ap_u¿ycie potoków zamiast tymczasowych plików asemblera
**2ar_wypisuje alokacje/zwalnianie rejestrów do pliku asemblera
**2at_wypisuje tymczasowe alokacje/zwalnianie do pliku asemblera
**1A<x>_format wyj¶ciowy:
**2Adefault_u¿ycie domy¶lnego asemblera
3*2Aas_asemblacja przy u¿yciu GNU AS
3*2Anasmcoff_plik coff (Go32v2) przy u¿yciu Nasm
3*2Anasmelf_plik elf32 (Linux) przy u¿yciu Nasm
3*2Anasmwin32_plik obj (Win32) przy u¿yciu Nasm
3*2Anasmwdosx_plik obj (Win32/WDOSX) przy u¿yciu Nasm
3*2Awasm_plik obj przy u¿yciu Wasm (Watcom)
3*2Anasmobj_plik obj przy u¿yciu Nasm
3*2Amasm_plik obj przy u¿yciu Masm (Microsoft)
3*2Atasm_plik obj przy u¿yciu Tasm (Borland)
3*2Aelf_bezpo¶redni zapis do pliku elf32 (Linux)
3*2Acoff_bezpo¶redni zapis do pliku coff (Go32v2)
3*2Apecoff_bezpo¶redni zapis do pliku pecoff (Win32)
4*2Aas_asemblacja przy u¿yciu GNU AS
6*2Aas_o-file (Unix) przy u¿yciu GNU AS
6*2Agas_asembler GNU Motorola
6*2Amit_sk³adnia MIT (dawniej GAS)
6*2Amot_standardowy asembler Motorola
A*2Aas_asemblacja przy u¿yciu GNU AS
P*2Aas_asemblacja przy u¿yciu GNU AS
S*2Aas_asemblacja przy u¿yciu GNU AS
**1b_generuje informacje przegl±darki
**2bl_generuje informacje o lokalnych symbolach
**1B_budowanie wszystkich modu³ów
**1C<x>_opcje generacji kodu:
**2Cc<x>_ustawia domy¶lny sposób wywo³ania funkcji/procedur na <x>
**2CD_tworzenie tak¿e bibliotek dynamicznych (niedostêpne)
**2Ce_kompilacja z emulowanymi opkodami zmiennoprzecinkowymi
**2Cf<x>_ustawia u¿ywany zbiór instrukcji FPU na <x> (u¿yj "fpc -i", aby zobaczyæ mo¿liwe warto¶ci)
**2Cg_generowanie kodu PIC
**2Ch<n>_rozmiar sterty w bajtach (pomiêdzy 1023 i 67107840)
**2Ci_sprawdzanie operacji wej¶cia/wyj¶cia
**2Cn_pominiêcie etapu linkowania
**2Co_sprawdzanie przepe³nienia liczb ca³kowitych
**2Cp<x>_ustawia u¿ywany zbiór instrukcji na <x> (u¿yj "fpc -i", aby zobaczyæ mo¿liwe warto¶ci)
**2Cr_sprawdzanie zakresu
**2CR_sprawdzanie poprawno¶ci wywo³ywania metod obiektów
**2Cs<n>_ustawienie rozmiar stosu na <n>
**2Ct_sprawdzanie stosu
**2CX_tworzenie bibliotek smartlinked
**1d<x>_zdefiniowanie symbolu <x>
*O1D_generacja pliku DEF
*O2Dd<x>_ustawienie opisu na <x>
**2Dv<x>_ustawienie wersji DLL na <x>
*O2Dw_aplikacja PM
**1e<x>_ustawienie ¶cie¿ki do asemblera i linkera
**1E_to samo co -Cn
**1F<x>_ustawianie nazw i ¶cie¿ek:
**2Fa<x>[,y]_wczytuje <x> (i [y]) zanim zostani± wczytane modu³y z sekcji USES
**2Fc<x>_ustawienie strony kodowej ¼róde³ na <x>
**2FD<x>_ustawienie katalogu z narzêdziami kompilatora
**2Fe<x>_przekierowanie komunikatów o b³êdach do pliku <x>
**2FE<x>_ustawienie katalogu dla skompilowanych programów/modu³ów na <x>
**2Fi<x>_dodanie <x> do ¶cie¿ki include'ów
**2Fl<x>_dodanie <x> do ¶cie¿ki bibliotek
**2FL<x>_u¿ycie <x> jako dynamicznego linkera
**2Fo<x>_dodanie <x> do ¶cie¿ki obiektów
**2Fr<x>_³adowanie pliku <x> z komunikatami b³êdów
**2Fu<x>_dodanie <x> do ¶cie¿ki modu³ów
**2FU<x>_ustawienie katalogu dla skompilowanych modu³ów, uniewa¿nia -FE
*g1g_generacja informacji debuggera:
*g2gc_generacja kodu sprawdzaj±cego wska¼niki
*g2gd_u¿ycie dbx
*g2gg_u¿ycie gsym
*g2gh_u¿ycie modu³u ¶ledzenia sterty(do wykrywania wycieków pamiêci)
*g2gl_u¿ycie modu³u z informacjami o numerach linii programu
*g2gv_generacja kodu mo¿liwego do ¶ledzenia przy pomocy valgrind
*g2gw_generacja informacji debuggera dwarf
**1i_informacje
**2iD_zwraca datê kompilatora
**2iV_zwraca wersjê kompilatora
**2iSO_zwraca OS kompilatora
**2iSP_zwraca typ procesora kompilatora
**2iTO_zwraca docelowy OS
**2iTP_zwraca docelowy typ kompilatora
**1I<x>_dodanie <x> do ¶cie¿ki include'ów
**1k<x>_podanie <x> do linkera
**1l_wypisanie logo
**1M<x>_ustawia tryb jêzyka na <x>
**2Mfpc_dialekt free pascala (domy¶lny)
**2Mobjfpc_w³±czenie niektórych rozszerzeñ Delphi 2
**2Mdelphi_kompatybilno¶æ z Delphi
**2Mtp_kompatybilno¶æ z TP/BP 7.0
**2Mgpc_kompatybilno¶æ z gpc
**2Mmac_kompatybilno¶æ z dialektami pascala na Macintosha
**1n_zignorowanie standardowego pliku konfiguracyjnego
**1o<x>_zmiana nazwy skompilowanego programu na <x>
3*1O<x>_optymalizacje:
3*2Og_generacja mniejszego kodu
3*2OG_generacja szybszego kodu (domy¶lne)
3*2Or_trzymanie niektórych zmiennych w rejestrach
3*2Ou_w³±czenie niepewnych optymalizacji (zobacz w dokumentacji)
3*2O1_optymalizacje pierwszego stopnia (szybkie)
3*2O2_optymalizacje drugiego stopnia (-O1 + wolniejsze)
3*2O3_optymalizacje trzeciego stopnia (powtarzane maksymalnie 5 razy -02)
3*2Op<x>_procesor docelowy:
3*3Op1_ustawienie procesora docelowego na 386/486
3*3Op2_ustawienie procesora docelowego na Pentium/PentiumMMX (tm)
3*3Op3_ustawienie procesora docelowego na PPro/PII/c6x86/K6 (tm)
6*2Og_generacja mniejszego kodu
6*2OG_generacja szybszego kodu (domy¶lne)
6*2Ox_maksymalne optymalizacje (ci±gle zawieraj± B£ÊDY!!!)
6*2O0_ustawia docelowy procesor na MC68000
6*2O2_ustawia docelowy procesor na MC68020+ (domy¶lne)
**1pg_generacja kodu do profilowania przy pomocy gprof (definiuje FPC_PROFILE)
**1R<x>_styl asemblera u¿ywanego w ¼ród³ach:
**2Rdefault_u¿ycie domy¶lnego asemblera
3*2Ratt_styl AT&T
3*2Rintel_styl Intel
4*2Ratt_styl AT&T
4*2Rintel_styl Intel
8*2Ratt_styl AT&T
8*2Rintel_styl Intel
6*2RMOT_styl Motorola
**1S<x>_opcje sk³adni:
**2S2_to samo co -Mobjfpc
**2Sc_wspieranie operatorów C (*=,+=,/= oraz -=)
**2Sa_do³±czanie kodu asercji
**2Sd_to samo co -Mdelphi
**2Se<x>_opcje b³êdów. <x> jest kombinacj±:
**3*_<n> : kompilator zatrzymuje siê po <x> b³êdach (domy¶lnie 1)
**3*_w : kompilator zatrzymuje siê tak¿e na ostrze¿eniach
**3*_n : kompilator zatrzymuje siê tak¿e na notkach
**3*_h : kompilator zatrzymuje siê tak¿e na podpowiedziach
**2Sg_zezwolenie na LABEL i GOTO
**2Sh_u¿ycie ansistringów
**2Si_wspieranie INLINE w stylu C++
**2SI<x>_ustawia styl interfejsów na <x>
**3SIcom_interfejsy kompatybilne z COM (domy¶lne)
**3SIcorba_interfejsy kompatybilne z CORBA
**2Sm_wspieranie makr jak w C (opcja globalna)
**2So_to samo co -Mtp
**2Sp_to samo co -Mgpc
**2Ss_konstruktory musz± mieæ nazwê init (destruktory - done)
**1s_pominiêcie wywo³ania asemblera i linkera
**2sh_generacja skryptu do linkowania na ho¶cie
**2st_generacja skryptu do linkowania na celu
**2sr_ominiêcie fazy alokowania rejestrów (u¿ywaj z -alr)
**1T<x>_docelowy system operacyjny:
3*2Temx_OS/2 na EMX (wliczaj±c EMX/RSX extender)
3*2Tfreebsd_FreeBSD
3*2Tgo32v2_DJ Delorie DOS extender - wersja 2
3*2Tlinux_Linux
3*2Tnetbsd_NetBSD
3*2Tnetware_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2Topenbsd_OpenBSD
3*2Tos2_OS/2 / eComStation
3*2Twatcom_DOS extendery zgodne z Watcomem
3*2Twdosx_WDOSX DOS extender
3*2Twin32_32-bitowy Windows
4*2Tlinux_Linux
6*2Tamiga_Commodore Amiga
6*2Tatari_Atari ST/STe/TT
6*2Tlinux_Linux-68k
6*2Tmacos_Macintosh m68k (nie wspierane)
6*2Tpalmos_PalmOS
A*2Tlinux_Linux
P*2Tdarwin_Darwin i MacOS X na PowerPC
P*2Tlinux_Linux na PowerPC
P*2Tmacos_MacOS (klasyczny) na PowerPC
P*2Tmorphos_MorphOS
S*2Tlinux_Linux
**1u<x>_usuniêcie symbolu <x>
**1U_opcje modu³ów:
**2Un_pominiêcie sprawdzania nazwy modu³u
**2Ur_generacja modu³u do wydania (release)
**2Us_kompilacja modu³u system
**1v<x>_obszerne opisy. <x> jest kombinacj± nastêpuj±cych liter i cyfr:
**2*_e : pokazuje b³êdy (domy¶lne)   0 : nic nie pokazuje (oprócz b³êdów)
**2*_w : pokazuje ostrze¿enia        u : pokazuje informacje o modu³ach
**2*_n : pokazuje notki              t : pokazuje próbowane/u¿yte pliki
**2*_h : pokazuje podpowiedzi        c : pokazuje warunki
**2*_i : pokazuje ogólne informacje  d : pokazuje informacje debugowania
**2*_l : pokazuje numery lini        r : tryb kompatybilno¶ci z Rhide/GCC
**2*_a : pokazuje wszystko           x : informacje o pliku exe (tylko Win32)
**2*_v : zapisuje plik fpcdebug.txt  p : zapisuje tree.log z drzewem
**2*_    z du¿± ilo¶ci± informacji       parsowania
3*1W<x>_opcje dla Win32 i podobnych
3*2WB_tworzenie relokacyjnego obrazu
3*2WB<x>_ustawienie bazy obrazu na szesnastkow± warto¶æ <x>
3*2WC_aplikacja konsolowa
3*2WD_u¿ycie DEFFILE do eksportu funkcji z DLLa lub EXE
3*2WF_aplikacja pe³noekranowa (tylko OS/2)
3*2WG_aplikacja graficzna
3*2WN_nie generuje kodu relokacji (potrzebne do debugowania)
3*2WR_generuje kod relokacji
P*2WC_aplikacja konsolowa (tylko MacOS)
P*2WG_aplikacja graficzna (tylko MacOS)
P*2WT_aplikacja narzêdziowa (narzêdzie MPW, tylko MacOS)
**1X_opcje plików wykonywalnych:
**2Xc_podaje --shared do linkera (tylko Unix)
**2Xd_nie u¿ywa standardowej ¶cie¿ki bibliotek (potrzebne do cross-kompilacji)
**2XD_linkowanie dynamiczne           (definiuje FPC_LINK_DYNAMIC)
**2Xm_generuje link map
**2XM<x>_ustawia nazwê g³ównego bloku programu (domy¶lnie 'main')
**2XP<x>_poprzedza nazwê binutils tekstem <x>
**2Xr<x>_ustawia ¶cie¿kê bibliotek na <x> (potrzebne do cross-kompilacji)
**2Xs_wyciêcie wszystkich symboli z pliku
**2XS_linkowanie statycznie (domy¶lne)(definiuje FPC_LINK_STATIC)
**2Xt_linkuje z bibliotekami statycznymi (podaje -static do linkera)
**2XX_linkowanie smart                (definiuje FPC_LINK_SMART)
**1*_
**1?_pokazanie pomocy
**1h_pokazanie pomocy bez zatrzymywania
]

#
# The End...
