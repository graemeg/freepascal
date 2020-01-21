#
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1998-2018 by the Free Pascal Development team
#
#   French (cp850) Language File for Free Pascal
#
#   See the file COPYING.v2, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#

# CodePage 850

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
#   m_   macro
#   p_   procedure
#   c_   conditional
#   d_   debug message
#   b_   display overloaded procedures
#   x_   executable informations
#

#
# General
#
# BeginOfTeX
% \section{General compiler messages}
% This section gives the compiler messages which are not fatal, but which
% display useful information. The number of such messages can be
% controlled with the various verbosity level \var{-v} switches.
% \begin{description}
general_t_compilername=01000_T_Compilateur : $1
% When the \var{-vt} switch is used, this line tells you what compiler
% is used.
general_d_sourceos=01001_D_SystŠme d'exploitation source : $1
% When the \var{-vd} switch is used, this line tells you what the source
% operating system is.
general_i_targetos=01002_I_SystŠme d'exploitation cible : $1
% When the \var{-vd} switch is used, this line tells you what the target
% operating system is.
general_t_exepath=01003_T_R‚pertoire pour fichiers executables : $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its binaries.
general_t_unitpath=01004_T_R‚pertoire de recherche d'unit‚s : $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for compiled units. You can set this path with the \var{-Fu}
general_t_includepath=01005_T_R‚pertoire pour recherche de fichiers inclus : $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its include files (files used in \var{\{\$I xxx\}} statements).
% You can set this path with the \var{-I} option.
general_t_librarypath=01006_T_R‚pertoire pour librairies : $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for the libraries. You can set this path with the \var{-Fl} option.
general_t_objectpath=01007_T_R‚pertoire pour fichiers objets : $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for object files you link in (files used in \var{\{\$L xxx\}} statements).
% You can set this path with the \var{-Fo} option.
general_i_abslines_compiled=01008_I_$1 lignes compil‚es, $2 secondes
% When the \var{-vi} switch is used, the compiler reports the number
% of lines compiled, and the time it took to compile them (real time,
% not program time).
general_f_no_memory_left=01009_F_Plus de m‚moire disponible
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
% \end{description}
#
# Scanner
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compiling handling.
% \begin{description}
general_i_writingresourcefile=01010_I_Ecriture du fichier Resource : $1
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_Ecriture du fichier Resource String Table : $1
% This message is shown when the compiler encountered an error when writing
% the Resource String Table file
% \end{description}
#
# Scanner
#
% \section{Scanner messages.}
% This section lists the messages that the scanner emits. The scanner takes
% care of the lexical structure of the pascal file, i.e. it tries to find
% reserved words, strings, etc. It also takes care of directives and
% conditional compiling handling.
% \begin{description}
scan_f_end_of_file=02000_F_Fin de fichier inattendue
% this typically happens in on of the following cases :
% \begin{itemize}
% \item The source file ends befor then final \var{end.} statement. This
% happens mostly when the \var{begin} and \var{end} statements aren't
% balanced;
% \item An include file ends in the middle of a statement.
% \item A comment wasn't closed.
% \end{itemize}
scan_f_string_exceeds_line=02001_F_String au-del… de la fin de ligne
% You forgot probably to include the closing ' in a string, so it occupies
% multiple lines.
scan_f_illegal_char=02002_F_caractŠre ill‚gal
% An illegal character was encountered in the input file.
scan_f_syn_expected=02003_F_Erreur de syntaxe, $1 attendu mais $2 trouv‚
% This indicates that the compiler expected a different token than
% the one you typed. It can occur almost everywhere where you make a
% mistake against the pascal language.
scan_t_start_include_file=02004_T_D‚but de lecture du fichier inclus $1
% When you provide the \var{-vt} switch, the compiler tells you
% when it starts reading an included file.
scan_w_comment_level=02005_W_Commentaire de niveau $1
% When the \var{-vw} switch is used, then the compiler warns you if
% it finds nested comments. Nested comments are not allowed in Turbo Pascal
% and can be a possible source of errors.
scan_n_far_directive_ignored=02006_N_$F directive (FAR) ignor‚e
% The \var{FAR} directive is a 16-bit construction which is recorgnised
% but ignored by the compiler, since it produces 32 bit code.
scan_n_stack_check_global_under_linux=02007_N_Le controle de la gestion de la pile est global sous Linux
% Stack checking with the \var{-Cs} switch is ignored under \linux, since
% \linux does this for you. Only displayed when \var{-vn} is used.
scan_n_ignored_switch=02008_N_Switch "$1" ignor‚
% With \var{-vn} on, the compiler warns if it ignores a switch
scan_w_illegal_switch=02009_W_Switch "$1" ill‚gal
% You included a compiler switch (i.e. \var{\{\$... \}}) which the compiler
% doesn't know.
scan_w_switch_is_global=02010_W_Ce switch a un effet global
% When \var{-vw} is used, the compiler warns if a switch is global.
scan_e_illegal_char_const=02011_E_Constante de type char ill‚gale
% This happens when you specify a character with its ASCII code, as in
% \var{\#96}, but the number is either illegal, or out of range. The range
% is 1-255.
scan_f_cannot_open_input=02012_F_Impossible d'ouvrir le fichier $1
% \fpc cannot find the program or unit source file you specified on the
% command line.
scan_f_cannot_open_includefile=02013_F_Impossible d'ouvrir le fichier inclus $1
% \fpc cannot find the source file you specified in a \var{\{\$include \}}
% stateent.
scan_e_too_much_endifs=02014_E_Trop de $ENDIFs ou de $ELSEs
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements aren't balanced.
scan_w_only_pack_records=02015_W_L'alignement des Records peut uniquement ˆtre de 1,2,4,8 ou 16 bytes
% You are specifying the \var{\{\$PACKRECORDS n\} } with an illegal value for
% \var{n}. Only 1,2,4 or 16 are valid in this case.
scan_w_only_pack_enum=02016_W_Les ‚numerations peuvent seulement ˆtre sauvegard‚s en 1,2 ou 4 bytes
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2 or 4 are valid in this case.
scan_e_endif_expected=02017_E_$1 attendu pour $2 d‚fini … la ligne $3
% Your conditional compilation statements are unbalanced.
scan_e_preproc_syntax_error=02018_E_Erreur de syntaxe pendant l'interpr‚tation d'une directive de compilation
% There is an error in the expression following the \var{\{\$if \}} compiler
% directive.
scan_e_error_in_preproc_expr=02019_E_Erreur d'‚valuation d'une directive de compilation
% There is an error in the expression following the \var{\{\$if \}} compiler
% directive.
scan_w_macro_cut_after_255_chars=02020_W_Le contenu d'une macro est tronqu‚ … 255 caractŠres
% The contents of macros cannot be longer than 255 characters. This is a
% safety in the compiler, to prevent buffer overflows. This is shown as a
% warning, i.e. when the \var{-vw} switch is used.
scan_e_endif_without_if=02021_E_ENDIF sans IF(N)DEF
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements aren't balanced.
scan_f_user_defined=02022_F_D‚fini par l'utilisateur : $1
% A user defined fatal error occurred. see also the \progref
scan_e_user_defined=02023_E_D‚fini par l'utilisateur : $1
% A user defined error occurred. see also the \progref
scan_w_user_defined=02024_W_D‚fini par l'utilisateur : $1
% A user defined warning occurred. see also the \progref
scan_n_user_defined=02025_N_D‚fini par l'utilisateur : $1
% A user defined note was encountered. see also the \progref
scan_h_user_defined=02026_H_D‚fini par l'utilisateur : $1
% A user defined hint was encountered. see also the \progref
scan_i_user_defined=02027_I_D‚fini par l'utilisateur : $1
% User defined information was encountered. see also the \progref
scan_e_keyword_cant_be_a_macro=02028_E_Impossible de redefinir un mot r‚serv‚
% You cannot redefine keywords with macros.
scan_f_macro_buffer_overflow=02029_F_D‚bordement du buffer de la macro en lecture ou expansion
% Your macro or its result was too long for the compiler.
scan_w_macro_deep_ten=02030_W_L'expansion des macros d‚passe un niveau de 16.
% When expanding a macro macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_wrong_styled_switch=02031_E_Les directives de compilation entre (* ... *) ne sont pas accept‚es
% Compiler switches should always be between \var{\{ \}} comment delimiters.
scan_d_handling_switch=02032_D_Interpr‚tation switch "$1"
% When you set debugging info on (\var{-vd}) the compiler tells you when it
% is evaluating conditional compile statements.
scan_c_endif_found=02033_C_ENDIF $1 trouv‚
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifdef_found=02034_C_IFDEF $1 trouv‚, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifopt_found=02035_C_IFOPT $1 trouv‚, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_if_found=02036_C_IF $1 trouv‚, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifndef_found=02037_C_IFNDEF $1 trouv‚, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_else_found=02038_C_ELSE $1 trouv‚, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_skipping_until=02039_C_Passant jusqu'…...
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements, and whether it is skipping or
% compiling parts.
scan_i_press_enter=02040_I_Appuyez sur <Entr‚e> pour continuer
% When the \var{-vi} switch is used, the compiler stops compilation
% and waits for the \var{Enter} key to be pressed when it encounters
% a \var{\{\$STOP\}} directive.
scan_w_unsupported_switch=02041_W_Switch non support‚ $1
% When warings are turned on (\var{-vw}) the compiler warns you about
% unsupported switches. This means that the switch is used in Delphi or
% Turbo Pascal, but not in \fpc
scan_w_illegal_directive=02042_W_directive de compilation ill‚gale $1
% When warings are turned on (\var{-vw}) the compiler warns you about
% unrecognised switches. For a list of recognised switches, \progref
scan_t_back_in=02043_T_De retour dans $1
% When you use (\var{-vt}) the compiler tells you when it has finished
% reading an include file.
scan_w_unsupported_app_type=02044_W_Type d'application non support‚e : $1
% You get this warning, if you specify an unknown application type
% with the directive $APPTYPE
scan_w_app_type_not_support=02045_W_$APPTYPE non support‚ par OS cible
% The $APPTYPE directive is supported by win32 applications only
scan_w_decription_not_support=02046_W_DESCRIPTION est support‚ seulement pour OS2 et Win32
% The \var{\{\$DESCRIPTION\}} directive is only supported for OS2 and Win32 targets.
scan_n_version_not_support=02047_N_VERSION non support‚ par OS cible
% The \var{\{\$VERSION\}} directive is only supported by win32 target.
scan_n_only_exe_version=02048_N_VERSION seulement pour exes ou DLLs
% The \var{\{\$VERSION\}} directive is only used for executable or DLL sources.
scan_w_wrong_version_ignored=02049_W_Mauvais format de la directive VERSION $1
% The \var{\{\$VERSION\}} directive format is major_version.minor_version
% where major_version and minor_version are words.
scan_w_unsupported_asmmode_specifier=02050_W_Style assembleur non support‚ $1
% When you specify an assembler mode with the \var{\{\$ASMMODE xxx\}}
% the compiler didn't recognize the mode you specified.
scan_w_no_asm_reader_switch_inside_asm=02051_W_Changement de type d'interpr‚teur ASM … l'int‚rieur de code assembleur, $1 sera effectif seulement pour le prochain
% It is not possible to switch from one assembler reader to another
% inside an assembler block. The new reader will be used for next
% assembler statement only.
scan_e_wrong_switch_toggle=02052_E_Mauvais argument de switch, utilisez ON/OFF ou +/-
% You need to use ON or OFF or a + or - to toggle the switch
scan_e_resourcefiles_not_supported=02053_E_Fichiers ressource non support‚ pour cette cible
% The target you are compiling for doesn't support Resource files. The
% only target which can use resource files is Win32
scan_w_include_env_not_found=02054_W_Include pour variable d'environment $1 non trouv‚
% The included environment variable can't be found in the environment, it'll
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_Valeur invalide pour nombre limite de variables en registre FPU
% Valid values for this directive are 0..8 and NORMAL/DEFAULT
% \end{description}
#
# Parser
#
% \section{Parser messages}
% This section lists all parser messages. The parser takes care of the
% semantics of you language, i.e. it determines if your pascal constructs
% are correct.
% \begin{description}
scan_w_only_one_resourcefile_supported=02056_W_Un fichier ressource est seulement support‚ pour cette cible
% Only one resource file can be supported for this target - this is the case of
% OS/2 (EMX) currently. The first one found is used, the others are discarded.
%
% \end{description}
#
# Parser
#
% \section{Parser messages}
% This section lists all parser messages. The parser takes care of the
% semantics of you language, i.e. it determines if your pascal constructs
% are correct.
% \begin{description}
parser_e_syntax_error=03000_E_Parseur - Erreur de syntaxe
% An error against the Turbo Pascal language was encountered. This happens
% typically when an illegal character is found in the sources file.
parser_w_proc_far_ignored=03001_W_Directive pour proc‚dure FAR ignor‚
% This is a warning. \var{FAR} is a construct for 8 or 16 bit programs. Since
% the compile generates 32 bit programs, it ignores this directive.
parser_w_proc_near_ignored=03002_W_Directive pour proc‚dure NEAR ignor‚
% This is a warning. \var{NEAR} is a construct for 8 or 16 bit programs. Since
% the compile generates 32 bit programs, it ignores this directive.
parser_w_proc_interrupt_ignored=03003_W_Directive pour proc‚dure INTERRUPT ignor‚
% This is a warning. \var{INTERRUPT} is a i386 specific construct
% and is igonred for other processors.
parser_e_dont_nest_interrupt=03004_E_Une proc‚dure de type INTERRUPT ne peut ˆtre locale
% An \VAR{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_Procedure type $1 ignored
% This is a warning. \var{REGISTER},\var{REINTRODUCE} is ignored by FPC programs for now.
% This is introduced first for Delphi compatibility.
parser_e_no_overload_for_all_procs=03006_E_Not all declarations of $1 are declared with OVERLOAD
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_no_dll_file_specified=03007_E_Pas de fichier DLL sp‚cifi‚
% No longer in use.
parser_e_export_name_double=03008_E_Nom de fonction export‚e doubl‚ $1
% Exported function names inside a specific DLL must all be different
parser_e_export_ordinal_double=03009_E_Index de fonction export‚e doubl‚ $1
% Exported function names inside a specific DLL must all be different
parser_e_export_invalid_index=03010_E_Index non valide pour fonction export‚e
% DLL function index must be in the range 1..$FFFF
parser_w_parser_reloc_no_debug=03011_W_Les DLL ou EXE relogeables sont incompatibles avec les informations de d‚bogage, d‚bogage d‚sactiv‚.
parser_w_parser_win32_debug_needs_WN=03012_W_Pour permettre le d‚bogage de code win32, utilisez l'option -WN
% Stabs info is wrong for relocatable DLL or EXES use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_Le nom du constructeur doit ˆtre INIT
% You are declaring a constructor with a name which isn't \var{init}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_destructorname_must_be_done=03014_E_Le nom du destructeur doit ˆtre DONE
% You are declaring a constructor with a name which isn't \var{done}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_illegal_open_parameter=03015_E_ParamŠtre ouvert non valide
% You are trying to use the wrong type for an open parameter.
parser_e_proc_inline_not_supported=03016_E_Directive de fonction INLINE non support‚e
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler doesn't support C++
% styled inlining by default.
parser_w_priv_meth_not_virtual=03017_W_Les m‚thodes priv‚es ne devraient pas ˆtre virutelles
% You declared a method in the private part of a object (class) as
% \var{virtual}. This is not allowed. Private methods cannot be overridden
% anyway.
parser_w_constructor_should_be_public=03018_W_Le constructeur devrait ˆtre public
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_Le destructeur devrait ˆtre public
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_Les classes ne devraient avoir qu'un selu destructeur
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_Les D‚finitions locales de classes ne sont pas autoris‚es
% Classes must be defined globally. They cannot be defined inside a
% procedure or function
parser_f_no_anonym_objects=03022_F_Les d‚finitions de classes anonymes sont interdites
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_object_has_no_vmt=03023_E_L'object $1 n'a pas de table de m‚thodes virtuelles (VMT)
parser_e_illegal_parameter_list=03024_E_Liste de paramŠtres ill‚gale
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_type=03025_E_Mauvais type pour paramŠtre #$1
% There is an error in the parameter list of the function or procedure.
% The compiler cannot determine the error more accurate than this.
parser_e_wrong_parameter_size=03026_E_Mauvais nombre de paramŠtres sp‚cifi‚
% There is an error in the parameter list of the function or procedure,
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_L'identificateur surcharg‚ $1 n'est pas une fonction
% The compiler encountered a symbol with the same name as an overloaded
% function, but it isn't a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_Les fonctions surcharg‚es ont les mˆmes paramŠtres
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_L'entˆte de la fonction ne correspond pas … la d‚claration $1
% You declared a function with same parameters but
% different result type or function specifiers.
parser_e_header_different_var_names=03030_E_L'entˆte de la fonction $1 ne correspond pas … la d‚claration : le nom de variable change $2 => $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but define it with a different parameter list.
parser_n_duplicate_enum=03031_N_Les valeurs d'une ‚num‚ration doivent ˆtre en ordre ascendant
% \fpc allows enumeration constructions as in C. Given the following
% declaration two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_n_interface_name_diff_implementation_name=03032_N_Les noms d'interface et d'implementation sont diff‚rents $1 => $2
% This note warns you if the implementation and interface names of a
% functions are different, but they have the same mangled name. This
% is important when using overloaded functions (but should produce no error).
parser_e_no_with_for_variable_in_other_segments=03033_E_With ne peut ˆtre utilis‚ pour des variables dans un autre segment
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_Niveau d'inbrication de fonctions > 31
% You can nest function definitions only 31 times.
parser_e_range_check_error=03035_E_Erreur d'intervalle dans l'‚valuation d'une constante
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_Erreur d'intervalle dans l'‚valuation d'une constante
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_Valeur en double dans une instruction CASE
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_Valeur max inf‚rieure … la valeur min dans CASE
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless
parser_e_type_const_not_possible=03039_E_Les constantes typ‚es de classes sont interdites
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_Impossible d'assigner … une variable fonctionelle une fonction surcharg‚e
% You are trying to assign an overloaded function to a procedural variable.
% This isn't allowed.
parser_e_invalid_string_size=03041_E_La longueur d'une STRING doit ˆtre situ‚e entre 1 et 255
% The length of a string in Pascal is limited to 255 characters. You are
% trying to declare a string with length lower than 1 or greater than 255
% (This is not true for \var{Longstrings} and \var{AnsiStrings}.
parser_w_use_extended_syntax_for_objects=03042_W_Utilisez la syntaxe ‚tendue de NEW et DISPOSE pour les objets
% If you have a pointer \var{a} to a class type, then the statement
% \var{new(a)} will not initialize the class (i.e. the constructor isn't
% called), although space will be allocated. you should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the class.
parser_w_no_new_dispose_on_void_pointers=03043_W_L'usage de NEW ou DISPOSE pour des pointeurs non typ‚s est sans signification meaningless
parser_e_no_new_dispose_on_void_pointers=03044_E_L'usage de NEW ou DISPOSE pour des pointeurs non typ‚s est sans signification meaningless
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% Accepted for compatibility in \var{tp} and \vat{delphi} modes.
parser_e_class_id_expected=03045_E_Identificateur de classe attendu
% This happens when the compiler scans a procedure declaration that contains
% a dot,
% i.e., a object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_Identificateur de type interdit ici
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_Identificateur de m‚thode attendu
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_La fonction ne correspond … aucune m‚thode de cette classe $1
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_p_procedure_start=03049_P_proc‚dure/fonction $1
% When using the \var{-vp} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_Constante r‚elle ill‚gale
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_FAIL peut uniquement ˆtre utilis‚ dans un constructeur
% You are using the \var{FAIL} instruction outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_Les destructeurs ne peuvent avoir de paramŠtres
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_methods_via_class_ref=03053_E_Seules des m‚thodes de classes peuvent ˆtre r‚f‚r‚ avec des r‚f‚rences de classes
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
parser_e_only_class_methods=03054_E_Seuls des m‚thodes de classes peuvent ˆtre appel‚s dans une m‚thode de classe
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
parser_e_case_mismatch=03055_E_La constante et le type de CASE ne conserpodnet pas
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_Le symbole ne peut ˆtre export‚ d'une librairie
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_Une m‚thode h‚rit‚e est cach‚e par $1
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendent class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_Il n'y a pas de m‚thode dans l'ancˆtre que l'on peut surcharger : $1
% You try to \var{override} a virtual method of a parent class that doesn't
% exist.
parser_e_no_procedure_to_access_property=03059_E_Il manque un champ pour sp‚cifier l'accŠss … la property
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_Directive de property Stored non implement‚e
% The \var{stored} directive is not yet implemented
parser_e_ill_property_access_sym=03061_E_Symbole ill‚gal pour accŠs … la property
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code woud cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Impossible d'acc‚der … un champ "protected" d'un object ici
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module wher the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_Impossible d'acc‚der … un champ "private" d'un object ici
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_w_overloaded_are_not_both_virtual=03064_W_M‚thode surchargeant une m‚thode virtuelle devrait ˆtre virtuelle : $1
% If you declare overloaded methods in a class, then they should either all be
% virtual, or none. You shouldn't mix them.
parser_w_overloaded_are_not_both_non_virtual=03065_W_M‚thode surchargeant une m‚thode non virtuelle devrait ˆtre non virtuelle : $1
% If you declare overloaded methods in a class, then they should either all be
% virtual, or none. You shouldn't mix them.
parser_e_overloaded_methodes_not_same_ret=03066_E_M‚thodes virtuelles surcharg‚es doivent avoir le mˆme type r‚sultat : $1
% If you declare virtual overloaded methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_Des fonctions d‚clar‚es comme EXPORT ne peuvent ˆtre locales
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_Les m‚thodes ne peuvent pas ˆtre EXPORT‚es
% You cannot declare a procedure that is a method for an object as
% \var{export}ed. That is, your methods cannot be called from a C program.
parser_e_call_by_ref_without_typeconv=03069_E_Un appel avec paramŠtre par variable doivent ˆtre du type exact de la d‚claration
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_Cette classe n'est pas un paarent de la classe actuelle
% When calling inherited methods, you are trying to call a method of a strange
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_SELF est seulement possible dans les m‚thodes
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_Les m‚thodes ne peuvent ˆtre appel‚s avec un type qu'… l'int‚rieur d'une m‚thode
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_Utilisation non valide de ':'
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_Erreur d'intervalle ou ‚l‚ment dupliqu‚ dans un constructeur d'ensemble
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, either two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_Pointeur d'object attendu
% You specified an illegal type in a \var{New} statement.
% The extended synax of \var{New} needs an object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_Expression doit ˆtre un appel … un constructeur
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_Expression doit ˆtre un appel … un destructeur
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_Mauvais ordre des ‚l‚ments d'un record
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_L'expression doit ˆtre de type record ou objet
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_Les proc‚dures ne peuvent retourner une valeur
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures cannot return a value.
parser_e_only_methods_allowed=03081_E_Les constructeurs, destructeurs et class operators doivent ˆtre des m‚thodes
% You're declaring a procedure as destructor, constructor or class operator, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_Operator n'est pas surcharg‚
% You're trying to use an overloaded operator when it isn't overloaded for
% this type.
parser_e_no_such_assignment=03083_E_Impossible de surcharger l'assignement pour des types ‚gaux
% You can not overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_Impossible operator overload
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_Re-raise impossible ici
% You are trying to raise an exception where it isn't allowed. You can only
% raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_La syntaxe ‚tendue de new ou dispose n'est pas valide pour une classe
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{Dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_asm_incomp_with_function_return=03087_E_Directive "Assembler" incompatible avec le type de la fonction
% You're trying to implement a \var{assembler} function, but the return type
% of the function doesn't allow that.
parser_e_procedure_overloading_is_off=03088_E_L'overloading de proc‚dure est d‚sactiv‚
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_Impossible de red‚finir cet op‚rateur
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_Un op‚ratuer de comparaison doit retourner une valeur bool‚enne
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_Seule une m‚thode virtuelle peut ˆtre abstraite
% You are declaring a method as abstract, when it isn't declared to be
% virtual.
parser_f_unsupported_feature=03092_F_Utilisation d'une caract‚ristique non support‚e !
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_Il est interdit de m‚langer des objets et des classes
% You cannot derive \var{objects} and \var{classes} intertwined . That is,
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_Directive de proc‚dure non reconnue a due ˆtre ignor‚e : $1
% The procedure direcive you secified is unknown. Recognised procedure
% directives are \var{cdecl}, \var{stdcall}, \var{popstack}, \var{pascal}
% \var{register}, \var{export}.
parser_e_absolute_only_one_var=03095_E_absolute put seulement ˆtre associ‚ … une seule variable
% You cannot specify more than one variable before the \var{absolute} directive.
% Thus, the following construct will provide this error:
% \begin{verbatim}
% Var Z : Longint;
%     X,Y : Longint absolute Z;
% \end{verbatim}
% \item [ absolute can only be associated a var or const ]
% The address of a \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_absolute_only_to_var_or_const=03096_E_absolute peut seulement ˆtre associ‚ … une variable ou une constante
% The address of a \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_Seule UNE variable peut ˆtre initialis‚e
% You cannot specify more than one variable with a initial value
% in Delphi syntax.
parser_e_abstract_no_definition=03098_E_Les m‚thodes "Abstract" ne peuvent pas avoir d'implementation
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_Cette fonction surcharg‚e ne peut ˆtre locale (doit ˆtre export‚e)
% You are defining a overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_Des m‚thodes virtuelles sont utilis‚es sans constructeur pour $1
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_m_macro_defined=03101_M_Macro d‚finie : $1
% When \var{-vm} is used, the compiler tells you when it defines macros.
parser_m_macro_undefined=03102_M_Macro non d‚finie : $1
% When \var{-vm} is used, the compiler tells you when it undefines macros.
parser_m_macro_set_to=03103_M_Macro $1 … pour valeur $2
% When \var{-vm} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_Compilation de $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_U_Lecture de l'interface de l'unit‚ $1
% This tells you that the reading of the interface
% of the current unit starts
parser_u_parsing_implementation=03106_U_Lecture de l'implementation de $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_D_Compilation de $1 pour la seconde fois
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_paras_allowed=03108_E_Les propri‚t‚s vecteurs ne sont pas accept‚es ici
% You cannot use array properties at that point.
parser_e_no_property_found_to_override=03109_E_Aucune propri‚t‚ trouv‚e pour override
% You want to overrride a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_Seule une propri‚t‚ pard‚faut autoris‚e, il y en a une dans la classe $1
% You specified a property as \var{Default}, but a parent class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_La propri‚t‚ par d‚faut doit ˆtre une propri‚t‚ vecteur
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_Les constructeurs virtuels sont seulement possible pour les classes
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_Pas de propri‚t‚ par d‚faut
% You try to access a default property of a class, but this class (or one of
% its ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_PUBLISHED non autoris‚ ici pour des classes, utilisez {$M+}
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, whch turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_La d‚claration anticip‚e de la classe $1 doit ˆtre r‚solue ici pour pouvoir ˆtreparent
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% Where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_Les op‚rateurs ne peuvent ˆtre locaux
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_La directive de proc‚dure $1 n'est pas autoris‚e en interface
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_La directive de proc‚dure $1 n'est pas autoris‚e en impl‚mentation
% This procedure directive is not defined in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_La directive de proc‚dure n'est pas valide pour une variable
% This procedure directive cannot be part of a procedural of function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_La fonction $1 est d‚j… d‚clar‚e comme publique ou forward
% You will get this error if a function is defined as \var{forward} twice.
% Or it is once in the \var{interface} section, and once as a \var{forward}
% declaration in the \var{implmentation} section.
parser_e_not_external_and_export=03121_E_EXPORT et EXTERNAL sont incompatibles
% These two procedure directives are mutually exclusive
parser_e_name_keyword_expected=03122_E_Le mot r‚serv‚ NAME est requis ici
% The definition of an external variable needs a \var{name} clause.
parser_n_not_supported_for_inline=03123_N_$1 n'est pas support‚ pour des fonctions INLINE
% Inline procedures don't support this declaration.
parser_h_inlining_disabled=03124_H_Inlining d‚sactiv‚
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_Ecriture du Browser log $1
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+ \}} switch).
parser_h_maybe_deref_caret_missing=03126_H_le d‚r‚f‚rencement du pointeur semble manquer
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_Lecteur de code assembleur non support‚
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_La directive de proc‚dure $1 est en conflit avec d'autres directives
% You specified a procedure directive that conflicts with other directives.
% for instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_La convention d'appel ne correspond pas … la fonction pr‚d‚finie
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_register_calling_not_supported=03130_E_Appel avec registres (fastcall) non support‚
% The \var{register} calling convention, i.e., arguments are passed in
% registers instead of on the stack is not supported. Arguments are always
% passed on the stack.
parser_e_property_cant_have_a_default_value=03131_E_Cette propri‚t‚ ne peut avoir de valeur par d‚faut
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_La valeur par d‚faut d'une propri‚t‚ doit ˆtre une constante
% The value of a \var{default} declared property must be knwon at compile
% time. The value you specified is only known at run time. This happens
% .e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_Ce symbole n'est pas une classe et ne peut donc pas ˆtre PUBLISHED
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_Ce genre de propri‚t‚ ne peut ˆtre PUBLISHED
% Properties in a \var{published} section cannot be array properties.
% they must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_w_empty_import_name=03135_W_Nom d'importation vide
% Both index and name for the import are 0 or empty
parser_e_empty_import_name=03136_W_Empty import name specified
% Some targets need a name for the imported procedure or a cdecl specifier
parser_e_used_proc_name_changed=03137_E_Le nom interne de la fonction … chang‚ aprŠs son usage
% This is an internal error; please report any occurrences of this error
% to the \fpc team.
parser_e_division_by_zero=03138_E_Division par z‚ro
% There is a divsion by zero encounted
parser_e_invalid_float_operation=03139_E_Op‚ration en virgule flottatne invalide
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_Limite sup‚rieure inf‚rieure … la limite inf‚rieure d'un intervalle
% The upper bound of a \var{case} label is less than the lower bound and this
% is not possible
parser_w_string_too_long=03141_W_String "$1" est plus long que $2
% The size of the constant string is larger than the size you specified in
% string type definition
parser_e_string_larger_array=03142_E_longueur du String sup‚rieure … la longueur du CHAR ARRAY
% The size of the constant string is larger than the size you specified in
% the array[x..y] of char definition
parser_e_ill_msg_expr=03143_E_Expression invalide aprŠs directive MESSAGE
% \fpc supports only integer or string values as message constants
parser_e_ill_msg_param=03144_E_MESSAGE handler peuvent seulement accepter un paramŠtre par r‚f‚rence
% A method declared with the \var{message}-directive as message handler
% can take only one parameter which must be declared as call by reference
% Parameters are declared as call by reference using the \var{var}-directive
parser_e_duplicate_message_label=03145_E_Duplicate message label: %1
% A label for a message is used twice in one object/class
parser_e_self_in_non_message_handler=03146_E_SELF ne peut ˆtre un paramŠtre explicite que dans les MESSAGE handlers
% The self parameter can be passed only explicit if it is a method which
% is declared as message method handler
parser_e_threadvars_only_sg=03147_E_THREADVARS peuvent seulement ˆtre statiques ou globaux
% Threadvars must be static or global, you can't declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has its own stack and local variables
% are stored on the stack
parser_f_direct_assembler_not_allowed=03148_F_Assembleur direct non support‚ pour la sortie binaire
% You can't use direct assembler when using a binary writer, choose an
% other outputformat or use another assembler reader
parser_w_no_objpas_use_mode=03149_W_Ne chargez pas l'unit‚ OBJPAS manuellement, utilisez {$mode objfpc} ou {$mode delphi}
% You're trying to load the ObjPas unit manual from a uses clause. This is
% not a good idea to do, you can better use the {$mode objfpc} or {$mode delphi}
% directives which load the unit automatically
parser_e_no_object_override=03150_E_OVERRIDE nepeut ˆtre utilis‚ pour des objets
% Override isn't support for objects, use VIRTUAL instead to override
% a method of an anchestor object
% \end{description}
#
# Type Checking
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
parser_e_cant_use_inittable_here=03151_E_Les types de donn‚es n‚cessitant des initialisations ne peuvent faire partie de RECORD variables
% Some data type (e.g. \var{ansistring}) needs initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% can't be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_RESOURCESTRINGS doivent ˆtre statiques ou globaux
% Resourcestring can not be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_Exit avec un argument ne peut ˆtre utilis‚ ici
% an exit statement with an argument for the return value can't be used here, this
% can happen e.g. in \var{try..except} or \var{try..finally} blocks
parser_e_stored_property_must_be_boolean=03154_E_Le type du symbole STORED doit ˆtre un bool‚en
% If you specify a storage symbol in a property declaration, it must be of
% the type boolean
parser_e_ill_property_storage_sym=03155_E_Ce symbole n'est pas admis comme symbole STORED
% You can't use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants
parser_e_only_publishable_classes_can__be_published=03156_E_Selues les classes compil‚es en mode $M+ peuvent ˆtre PUBLISHED
% In the published section of a class can be only class as fields used which
% are compiled in $M+ or which are derived from such a class. Normally
% such a class should be derived from TPersitent
parser_e_proc_directive_expected=03157_E_Directive de proc‚dure attendue
% When declaring a procedure in a const block you used a ; after the
% procedure declaration after which a procedure directive must follow.
% Correct declarations are:
% \begin{verbatim}
% const
%   p : procedure;stdcall=nil;
%   p : procedure stdcall=nil;
% \end{verbatim}
parser_e_invalid_property_index_value=03158_E_Le type d'un index de propri‚t‚ doit ˆtre un type ordinal
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_Nom de proc‚dure trop court pour ˆtre exprot‚
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correct with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_Impossible de cr‚er une entr‚e DEFFILE pour des variables globales d'unit‚s
parser_e_dlltool_unit_var_problem2=03161_E_Compilez sans l'option -WD
% \end{description}
#
# Type Checking
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_Incompatibilit‚ de types
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_Incompatible types: got $1 expected $2
% There is no conversion possible between the two types
type_e_not_equal_types=04002_E_Incompatibilit‚ de types entre $1 et $2
% The types are not equal
type_e_type_id_expected=04003_E_Identificateur de type attendu
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_Identificateur de variable attendu
% This happens when you pass a constant to a \var{Inc} var or \var{Dec}
% procedure. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_Integer expression expected
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_Expression bool‚enne attendue, mais "$1" obtenu
% The expression must be a boolean type, it should be return true or
% false.
type_e_ordinal_expr_expected=04007_E_Expression ordinale attendue
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_Type pointeur attendu
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_Type classe attendu
% The variable of expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_varid_or_typeid_expected=04010_E_Identificateur de variable ou de type attendu
% The argument to the \var{High} or \var{Low} function is not a variable
% nor a type identifier.
type_e_cant_eval_constant_expr=04011_E_Impossible d'‚valuer l'expression constante
% No longer in use.
type_e_set_element_are_not_comp=04012_E_Elements d'ensembles non compatibles
% You are trying to make an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union
type_e_set_operation_unknown=04013_E_Op‚ration non permise pour des ensembles
% several binary operations are not defined for sets
% like div mod ** (also >= <= for now)
type_w_convert_real_2_comp=04014_W_Conversion automatique de r‚el vers COMP qui est un type entier
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{Comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_Utilisez DIV pour obtenir un r‚sultat entier
% When hints are on, then an integer division with the '/' operator will
% procuce this message, because the result will then be of type real
type_e_strict_var_string_violation=04016_E_Types string incompatibles, … cause du mode $V+
% When compiling in \var{\{\$V+ \}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_succ ou pred impossible pour des ‚num‚rations avec valeurs fix‚es
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use the \var{Succ} or \var{Pred} functions on them.
type_e_cant_read_write_type=04018_E_Impossible de lire ou d'‚crire des variables de ce type
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that. Only integer types,
% booleans, reals, pchars and strings can be read from/written to a text file.
type_e_no_readln_writeln_for_typed_file=04019_E_Impossible d'utiliser READLN ou WRITELN pour un ficher typ‚
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_Impossible d'utiliser READ ou WRITE pour un FILE non typ‚
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_Confit de type pour des ‚l‚ments d'un ensemble
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(longint/dword) retourne le word bas/haut
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword}
% which returns the lower/upper word of the argument. TP always uses
% a 16 bit \var{lo/hi} which returns always bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the TP behavior you have
% to type case the argument to \var{word/integer}
type_e_integer_or_real_expr_expected=04023_E_Expression entiŠre ou r‚elle attendue
% The first argument to \var{str} must a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_Mauvais type dans array constructor
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_Type incompatible type pour arg #$1: $2 au lieu de $3
% You are trying to pass an invalid type for the specified parameter.
% \end{description}
type_e_no_method_and_procedure_not_compatible=04026_E_Method (variable) et Procedure (variable) sont incompatibles
% You can't assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_Constante invalide pass‚e … une fonction math‚matique interne
% The constant argument passed to a ln or sqrt function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_Impossible d'obtenir l'adresse d'une constante
% It's not possible to get the address of a constant, because they
% aren't stored in memory, you can try making it a typed constant.
% \end{description}
#
# Symtable
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
#
# Symtable
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
type_e_argument_cant_be_assigned=04029_E_Impossible d'assigner une valeur … l'argument
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference argument
% Remark: Properties can be only
% used on the left side of an assignment, but they can't be used as arguments
type_e_cannot_local_proc_to_procvar=04030_E_Impossible d'assigner une fonction ou proc‚dure locale … une variable de proc‚dure
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling of local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_Can't assign values to an address
% It's not allowed to assign a value to an address of a variable,constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_Can't assign values to const variable
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const, to allow
% changing make the parameter value or var.
% \end{description}
#
# Symtable
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Identificateur non trouv‚ $1
% The compiler doesn't know this symbol. Usually happens when you misspel
% the name of a variable or procedure, or when you forgot to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_Internal Error in SymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_Duplicate identifier $1
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_Identifier already defined in $1 at line $2
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_Unknown identifier $1
% The identifier encountered hasn't been declared, or is used outside the
% scope where it's defined.
sym_e_forward_not_resolved=05005_E_Forward declaration not solved $1
% This can happen in two cases:
% \begin{itemize}
% \item This happens when you declare a function (in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_f_id_already_typed=05006_F_Identifier type already defined as type
% You are trying to redefine a type.
sym_e_error_in_type_def=05007_E_Error in type definition
% There is an error in your definition of a new array type:
% \item One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_type_id_not_defined=05008_E_Type identifier not defined
% The type identifier has not been defined yet.
sym_e_forward_type_not_resolved=05009_E_Forward type not resolved $1
% The compiler encountered an unknown type.
sym_e_only_static_in_static=05010_E_Only static variables can be used in static methods or outside methods
% A static method of an object can only access static variables.
sym_e_invalid_call_tvarsymmangledname=05011_E_Invalid call to tvarsym.mangledname()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_type_must_be_rec_or_object_or_class=05012_E_Record or object or class type expected
% The variable or expression isn't of the type \var{record} or \var{object} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Instances of classes or objects with an abstract method are not allowed
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_Label not defined $1
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_Label used but not defined $1
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_Illegal label declaration
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO und LABEL are not supported (use switch -Sg)
% You must compile a program which has \var{label}s and \var{goto} statements
% with the \var{-Sg} switch. By default, \var{label} and \var{goto} aren't
% supported.
sym_e_label_not_found=05018_E_Label not found
% A \var{goto label} was encountered, but the label isn't declared.
sym_e_id_is_no_label_id=05019_E_identifier isn't a label
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_label already defined
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_illegal type declaration of set elements
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_Forward class definition not resolved $1
% You declared a class, but you didn't implement it.
sym_n_unit_not_used=05023_H_Unit $1 not used in $2
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_Parameter not used $1
% This is a warning. The identifier was declared (locally or globally) but
% wasn't used (locally or globally).
sym_n_local_identifier_not_used=05025_N_Local variable not used $1
% You have declared, but not used a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_Value parameter $1 is assigned but never used
% This is a warning. The identifier was declared (locally or globally)
% set but not used (locally or globally).
sym_n_local_identifier_only_set=05027_N_Local variable $1 is assigned but never used
% The variable in a procedure or function
% implementation is declared, set but never used.
sym_h_local_symbol_not_used=05028_H_Local $1 $2 is not used
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_Private field $1.$2 is never used
sym_n_private_identifier_only_set=05030_N_Private field $1.$2 is assigned but never used
sym_n_private_method_not_used=05031_N_Private method $1.$2 never used

sym_e_set_expected=05032_E_Set type expected
% The variable or expression isn't of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_Function result does not seem to be set
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_Type $1 is not aligned correctly in current record for C
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_Unknown record field identifier $1
% The field doesn't exist in the record definition.
sym_n_uninitialized_local_variable=05036_W_Local variable $1 does not seem to be initialized
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% wasn't initialized first (i.e. appeared in the right-hand side of an
% assigment)
sym_n_uninitialized_variable=05037_W_Variable $1 does not seem to be initialized
% These messages are displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% wasn't initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_e_id_no_member=05038_E_identifier idents no member $1
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the class you are trying to create. The procedure you specified
% does not exist.
sym_b_param_list=05039_B_Found declaration: $1
% You get this when you use the \var{-vb} switch. In case an overloaded
% procedure is not found, then all candidate overloaded procedures are
% listed, with their parameter lists.
% \end{description}
#
# Codegenerator
#
% \section{Code generator messages}
% This section lists all messages that can be displayed if the code
% generator encounters an error condition.
% \begin{description}
sym_e_segment_too_large=05040_E_Data segment too large (max. 2GB)
% You get this when you declare an array whose size exceeds the 2GB limit.
% \end{description}
#
# Codegenerator
#
% \section{Code generator messages}
% This section lists all messages that can be displayed if the code
% generator encounters an error condition.
% \begin{description}
cg_e_break_not_allowed=06000_E_BREAK not allowed
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06001_E_CONTINUE not allowed
% You're trying to use \var{continue} outside a loop construction.
cg_e_too_complex_expr=06002_E_Expression too complicated - FPU stack overflow
% Your expression is too long for the compiler. You should try dividing the
% construct over multiple assignments.
cg_e_illegal_expression=06003_E_Illegal expression
% This can occur under many circumstances. Mostly when trying to evaluate
% constant expressions.
cg_e_invalid_integer=06004_E_Invalid integer expression
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
cg_e_invalid_qualifier=06005_E_Illegal qualifier
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
cg_e_upper_lower_than_lower=06006_E_High range limit < low range limit
% You are declaring a subrange, and the lower limit is higher than the high
% limit of the range.
cg_e_illegal_count_var=06007_E_Illegal counter variable
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
cg_e_cant_choose_overload_function=06008_E_Can't determine which overloaded function to call
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
cg_e_parasize_too_big=06009_E_Parameter list size exceeds 65535 bytes
% The I386 processor limits the parameter list to 65535 bytes (the \var{RET}
% instruction causes this)
cg_e_illegal_type_conversion=06010_E_Illegal type conversion
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
cg_d_pointer_to_longint_conv_not_portable=06011_D_Conversion between ordinals and pointers is not portable across platforms
% If you typecast a pointer to a longint, this code will not compile
% on a machine using 64bit for pointer storage.
cg_e_file_must_call_by_reference=06012_E_File types must be var parameters
% You cannot specify files as value parameters, i.e. they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_The use of a far pointer isn't allowed there
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_var_must_be_reference=06014_E_illegal call by reference parameters
% You are trying to pass a constant or an expression to a procedure that
% requires a \var{var} parameter. Only variables can be passed as a \var{var}
% parameter.
cg_e_dont_call_exported_direct=06015_E_EXPORT declared functions can't be called
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_Possible illegal call of constructor or destructor (doesn't match to this context)
% No longer in use.
cg_n_inefficient_code=06017_N_Inefficient code
% You construction seems dubious to the compiler.
cg_w_unreachable_code=06018_W_unreachable code
% You specified a loop which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_stackframe_with_esp=06019_E_procedure call with stackframe ESP/SP
% The compiler encountered a procedure or function call inside a
% procedure that uses a \var{ESP/SP} stackframe. Normally, when a call is
% done the procedure needs a \var{EBP} stackframe.
cg_e_cant_call_abstract_method=06020_E_Abstract methods can't be called directly
% You cannot call an abstract method directy, instead you must call a
% overriding child method, because an abstract method isn't implemented.
cg_f_internal_error_in_getfloatreg=06021_F_Internal Error in getfloatreg(), allocation failure
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
cg_f_unknown_float_type=06022_F_Unknown float type
% The compiler cannot determine the kind of float that occurs in an expression.
cg_f_secondvecn_base_defined_twice=06023_F_SecondVecn() base defined twice
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
cg_f_extended_cg68k_not_supported=06024_F_Extended cg68k not supported
% The var{extended} type is not supported on the m68k platform.
cg_f_32bit_not_supported_in_68000=06025_F_32-bit unsigned not supported in MC68000 mode
% The cardinal is not supported on the m68k platform.
cg_f_internal_error_in_secondinline=06026_F_Internal Error in secondinline()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
cg_d_register_weight=06027_D_Register $1 weight $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_e_stacklimit_in_local_routine=06028_E_Stack limit excedeed in local routine
% Your code requires a too big stack. Some operating systems pose limits
% on the stack size. You should use less variables or try ro put large
% variables on the heap.
cg_d_stackframe_omited=06029_D_Stack frame is omitted
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_w_64bit_range_check_not_supported=06030_W_Range check for 64 bit integers is not supported on this target
% 64 bit range check is not yet implemented for 32 bit processors.
cg_e_unable_inline_object_methods=06031_E_Object or class methods can't be inline.
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_Procvar calls can't be inline.
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_No code for inline procedure stored
% The compiler couldn't store code for the inline procedure.
cg_e_no_call_to_interrupt=06034_E_Direct call of interrupt procedure $1 is not possible
% You can not call an interrupt procedure directly from FPC code
cg_e_can_access_element_zero=06035_E_Element zero of an ansi/wide- or longstring can't be accessed, use (set)length instead
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such kinf of string
cg_e_include_not_implemented=06036_E_Include and exclude not implemented in this case
% \var{include} and \var{exclude} are only partially
% implemented for \var{i386} processors
% and not at all for \var{m68k} processors.
cg_e_cannot_call_cons_dest_inside_with=06037_E_Constructors or destructors can not be called inside a 'with' clause
% Inside a \var{With} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_Cannot call message handler method directly
% A message method handler method can't be called directly if it contains an
% explicit self argument
% \end{description}
# EndOfTeX
#
# Assembler reader
#
cg_e_goto_inout_of_exception_block=06039_E_Jump in or outside of an exception block
% It isn't allowed to jump in or outside of an exception block like \var{try..finally..end;}:
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
% \end{description}
cg_e_control_flow_outside_finally=06040_E_Control flow statements aren't allowed in a finally block
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
# EndOfTeX
asmr_d_start_reading=07000_D_Starting $1 styled assembler parsing
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_D_Finished $1 styled assembler parsing
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_Non-label pattern contains @
% A identifier which isn't a label can't contain a @.
asmr_w_override_op_not_supported=07003_W_Override operator not supported
% The Override operator is not supported
asmr_e_building_record_offset=07004_E_Error building record offset
% There has an error occured while building the offset of a record/object
% structure, this can happend when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_OFFSET used without identifier
% You can only use OFFSET with an identifier. Other syntaxes aren't
% supported
asmr_e_type_without_identifier=07006_E_TYPE utilis‚ sans identificateur
% You can only use TYPE with an identifier. Other syntaxes aren't
% supported
asmr_e_no_local_or_para_allowed=07007_E_Cannot use local variable or parameters here
% You can't use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the %ebp register so the
% address can't be get directly.
asmr_e_need_offset=07008_E_need to use OFFSET here
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_need to use $ here
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Cannot use multiple relocatable symbols
% You can't have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_Relocatable symbol can only be added
% Relocatable symbols (variable/typed constant) can't be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_Invalid constant expression
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_Relocatable symbol is not allowed
% You can't use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_Invalid reference syntax
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_Impossible d'atteindre $1 depuis ce code
% You can not read directly the value of local or para
% of a higher level in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_Local symboles/labels ne sont pas autoris‚s
% You can't use local symbols/labels as references
asmr_e_wrong_base_index=07017_E_Invalid base and index register usage
% There is an error with the base and index register
asmr_w_possible_object_field_bug=07018_W_Erreur possible dans l'utilisation d'un champ d'un object
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_Wrong scale factor specified
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_Multiple index register usage
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_Invalid operand type
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_Invalid string as opcode operand: $1
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE and @DATA not supported
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_Null label references are not allowed
asmr_e_expr_zero_divide=07025_E_Divide by zero in asm evaluator
asmr_e_expr_illegal=07026_E_Illegal expression
asmr_e_escape_seq_ignored=07027_E_escape sequence ignored: $1
asmr_e_invalid_symbol_ref=07028_E_Invalid symbol reference
asmr_w_fwait_emu_prob=07029_W_Fwait can cause emulation problems with emu387
asmr_w_fadd_to_faddp=07030_W_FADD without operand translated into FADDP
asmr_w_enter_not_supported_by_linux=07031_W_ENTER instruction is not supported by Linux kernel
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_Calling an overload function in assembler
asmr_e_unsupported_symbol_type=07033_E_Unsupported symbol type for operand
asmr_e_constant_out_of_bounds=07034_E_Constant value out of bounds
asmr_e_error_converting_decimal=07035_E_Error converting decimal $1
asmr_e_error_converting_octal=07036_E_Error converting octal $1
asmr_e_error_converting_binary=07037_E_Error converting binary $1
asmr_e_error_converting_hexadecimal=07038_E_Error converting hexadecimal $1
asmr_h_direct_global_to_mangled=07039_H_$1 translated to $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 is associated to an overloaded function
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Cannot use SELF outside a method
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Cannot use OLDEBP outside a nested procedure
asmr_e_void_function=07043_W_Functions with void return value can't return any value in asm code
asmr_e_SEG_not_supported=07044_E_SEG not supported
asmr_e_size_suffix_and_dest_dont_match=07045_E_Size suffix and destination or source size do not match
asmr_w_size_suffix_and_dest_dont_match=07046_W_Size suffix and destination or source size do not match
asmr_e_syntax_error=07047_E_Assembler syntax error
asmr_e_invalid_opcode_and_operand=07048_E_Invalid combination of opcode and operands
asmr_e_syn_operand=07049_E_Assembler syntax error in operand
asmr_e_syn_constant=07050_E_Assembler syntax error in constant
asmr_e_invalid_string_expression=07051_E_Invalid String expression
asmr_w_const32bit_for_address=07052_bit constant created for address
asmr_e_unknown_opcode=07053_E_Instruction non reconnue $1
asmr_e_invalid_or_missing_opcode=07054_E_Invalid or missing opcode
asmr_e_invalid_prefix_and_opcode=07055_E_Invalid combination of prefix and opcode: $1
asmr_e_invalid_override_and_opcode=07056_E_Invalid combination of override and opcode: $1
asmr_e_too_many_operands=07057_E_Too many operands on line
asmr_w_near_ignored=07058_W_NEAR ignored
asmr_w_far_ignored=07059_W_FAR ignored
asmr_e_dup_local_sym=07060_E_Duplicate local symbol $1
asmr_e_unknown_local_sym=07061_E_Undefined local symbol $1
asmr_e_unknown_label_identifier=07062_E_Unknown label identifier $1
asmr_e_invalid_register=07063_E_Invalid register name
asmr_e_invalid_fpu_register=07064_E_Invalid floating point register name
asmr_e_nor_not_supported=07065_E_NOR not supported
asmr_w_modulo_not_supported=07066_W_Modulo not supported
asmr_e_invalid_float_const=07067_E_Invalid floating point constant $1
asmr_e_invalid_float_expr=07068_E_Invalid floating point expression
asmr_e_wrong_sym_type=07069_E_Wrong symbol type
asmr_e_cannot_index_relative_var=07070_E_Cannot index a local var or parameter with a register
asmr_e_invalid_seg_override=07071_E_Invalid segment override expression
asmr_w_id_supposed_external=07072_W_Identifier $1 supposed external
asmr_e_string_not_allowed_as_const=07073_E_Strings not allowed as constants
asmr_e_no_var_type_specified=07074_No type of variable specified
asmr_w_assembler_code_not_returned_to_text=07075_E_assembler code not returned to text section
asmr_e_not_directive_or_local_symbol=07076_E_Not a directive or local symbol $1
asmr_w_using_defined_as_local=07077_E_Using a defined name as a local label
#
# Assembler/binary writers
#
asmr_e_dollar_without_identifier=07078_E_Dollar utilis‚ sans identificateur
asmr_w_32bit_const_for_address=07079_W_32bit constante cr‚‚e pour une addresse
asmr_n_align_is_target_specific=07080_N_.align d‚pend de la cible, utilisez .balign ou .p2align
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Impossible d'acc‚der directement aux champs pour des paramŠtres
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_Impossible d'acc‚der aux champs d'objectsou de classes directement
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
#
# Assembler/binary writers
#
asmw_f_too_many_asm_files=08000_F_Too many assembler files
asmw_f_assembler_output_not_supported=08001_F_Selected assembler output not supported
asmw_f_comp_not_supported=08002_F_Comp not supported
asmw_f_direct_not_supported=08003_F_Direct not support for binary writers
asmw_e_alloc_data_only_in_bss=08004_E_Allocating of data is only allowed in bss section
asmw_f_no_binary_writer_selected=08005_F_No binary writer selected
asmw_e_opcode_not_in_table=08006_E_Asm: Opcode $1 not in table
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 invalid combination of opcode and operands
asmw_e_16bit_not_supported=08008_E_Asm: 16 Bit references not supported
asmw_e_invalid_effective_address=08009_E_Asm: Invalid effective address
asmw_e_immediate_or_reference_expected=08010_E_Asm: Immediate or reference expected
asmw_e_value_exceeds_bounds=08011_E_Asm: $1 value exceeds bounds $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: Short jump is out of range $1
#
# Executing linker/assembler
#
asmw_e_undefined_label=08013_E_Asm: Label $1 non d‚fini

#
# Executing linker/assembler
#
# BeginOfTeX
%
% \section{Errors of assembling/linking stage}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
exec_w_source_os_redefined=09000_W_Source operating system redefined
exec_i_assembling_pipe=09001_I_Assembling (pipe) $1
exec_d_cant_create_asmfile=09002_E_Can't create assember file $1
% The mentioned file can't be create. Check if you've
% permission to create this file
exec_e_cant_create_objectfile=09003_E_Impossible de cr‚er le fichier object : $1
% The mentioned file can't be create. Check if you've
% permission to create this file
exec_e_cant_create_archivefile=09004_E_Impossible de cr‚er le fichier archive : $1
% The mentioned file can't be create. Check if you've
% permission to create this file
exec_w_assembler_not_found=09005_W_Assembler $1 not found, switching to external assembling
exec_t_using_assembler=09006_T_Using assembler: $1
exec_w_error_while_assembling=09007_W_Error while assembling exitcode $1
exec_w_cant_call_assembler=09008_W_Can't call the assembler, error $1 switching to external assembling
exec_i_assembling=09009_I_Assembling $1
exec_i_assembling_smart=09010_I_Assemblage smartlink $1
exec_w_objfile_not_found=09011_W_Object $1 not found, Linking may fail !
exec_w_libfile_not_found=09012_W_Library $1 not found, Linking may fail !
exec_w_error_while_linking=09013_W_Error while linking
exec_w_cant_call_linker=09014_W_Can't call the linker, switching to external linking
exec_i_linking=09015_I_Linking $1
exec_w_util_not_found=09016_W_Utilitaire $1 non trouv‚, force la liaison externe
exec_t_using_util=09017_T_Utilitaire $1 trouv‚
exec_e_exe_not_supported=09018_E_Cr‚ation d'executables impossible
exec_e_dll_not_supported=09019_E_Dynamic Libraries not supported
exec_i_closing_script=09020_I_Closing script $1
exec_w_res_not_found=09021_W_resource compiler not found, switching to external mode
exec_i_compilingresource=09022_I_Compiling resource $1
#
# Executable information
#
execinfo_f_cant_process_executable=09128_F_Can't post process executable $1
execinfo_f_cant_open_executable=09129_F_Can't open executable $1
execinfo_x_codesize=09130_X_Size of Code: $1 bytes
execinfo_x_initdatasize=09131_X_Size of initialized data: $1 bytes
execinfo_x_uninitdatasize=09132_X_Size of uninitialized data: $1 bytes
execinfo_x_stackreserve=09133_X_Stack space reserved: $1 bytes
execinfo_x_stackcommit=09134_X_Stack space commited: $1 bytes
# Unit loading
#
# BeginOfTeX
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these mesages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_Unitsearch: $1
% When you use the \var{-vt}, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_PPU Loading $1
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_PPU Name: $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_PPU Flags: $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_PPU Crc: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_PPU Time: $1
% When you use the \var{-vu} flag, the unit time is shown.
unit_u_ppu_file_too_short=10006_U_PPU File too short
% When you use the \var{-vu} flag, the unit time is shown.
unit_u_ppu_invalid_header=10007_U_PPU Invalid Header (no PPU at the begin)
% A unit file contains as the first three bytes the ascii codes of \var{PPU}
unit_u_ppu_invalid_version=10008_U_PPU Invalid Version $1
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_PPU is compiled for another processor
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_invalid_target=10010_U_PPU is compiled for another target
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_source=10011_U_PPU Source: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_write=10012_U_Writing $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_Can't Write PPU-File
% An err
unit_f_ppu_read_error=10014_F_reading PPU-File
% Unexpected end of file
unit_f_ppu_read_unexpected_end=10015_F_unexpected end of PPU-File
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_invalid_entry=10016_F_Invalid PPU-File entry: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_PPU Dbx count problem
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_Illegal unit name: $1
% The name of the unit doesn't match the file name.
unit_f_too_much_units=10019_F_Too much units
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{files.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_Circular unit reference between $1 and $2
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_Can't compile unit $1, no sources available
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_Impossible de trouver l'unit‚ $1
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your config files for the unit pathes
unit_w_unit_name_error=10023_W_Unit‚ $1 non trouv‚ mais $2 existe
unit_f_unit_name_error=10024_F_Unit‚ $1 cherch‚ mais $2 trouv‚
% Dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_Compiling the system unit requires the -Us switch
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} must be specified.
unit_f_errors_in_unit=10026_F_There were $1 errors compiling module, stopping
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_Load from $1 ($2) unit $3
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_Recompiling $1, checksum changed for $2
unit_u_recompile_source_found_alone=10029_U_Recompiling $1, source found only
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_Recompiling unit, static lib is older than ppufile
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit are older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_Recompiling unit, shared lib is older than ppufile
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit are older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_Recompiling unit, obj and asm are older than ppufile
% When you use the \var{-vu} flag, the compiler warns if the assembler of
% object file of the unit are older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_Recompiling unit, obj is older than asm
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_start_parse_interface=10034_U_Parsing interface of $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit
unit_u_start_parse_implementation=10035_U_Parsing implementation of $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit
unit_u_second_load_unit=10036_U_Second load for unit $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_check_time=10037_U_PPU Check file $1 time $2
% When you use the \var{-vu} flag, the compiler show the filename and
% date and time of the file which a recompile depends on
% \end{description}
# EndOfTeX

#
#  Options
#
option_usage=11000_O_$1 [options] <inputfile> [options]
# BeginOfTeX
%
% \section{Command-line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_only_one_source_support=11001_W_Only one source file supported
% You can specify only one source file on the command line. The first
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_DEF file can be created only for OS/2
% This option can only be specified when you're compiling for OS/2
option_no_nested_response_file=11003_E_nested response files are not supported
% you cannot nest response files with the \var {@file} command-line option.
option_no_source_found=11004_F_No source file name in command line
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_Aucune option trouv‚e dans le fichier de configuration $1
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_Illegal parameter: $1
% You specified an unknown option.
option_help_pages_para=11007_H_-? writes help pages
% When an unknown option is given, this message is diplayed.
option_too_many_cfg_files=11008_F_Too many config files nested
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_Unable to open file $1
% The option file cannot be found.
option_reading_further_from=11010_N_Reading further options from $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_Target is already set to: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_Shared libs not supported on DOS platform, reverting to static
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos
option_too_many_ifdef=11013_F_too many IF(N)DEFs
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_too many ENDIFs
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_open conditional at the end of the file
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_Debug information generation is not supported by this executable
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_Try recompiling with -dGDB
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_W_You are using the obsolete switch $1
% this warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the switch meaning may change.
option_obsolete_switch_use_new=11019_W_You are using the obsolete switch $1, please use $2
% this warns you when you use a switch that is not supported anymore. You
% must now use the second switch instead.
% It is recommended that you change the switch to overcome problems in the
% future, when the switch meaning may change.
option_switch_bin_to_src_assembler=11020_N_Switching assembler to default source writing assembler
% this notifies you that the assembler has been changed because you used the
% -a switch which can't be used with a binary assembler writer.
option_incompatible_asm=11021_W_Assembler output selected "$1" is not compatible with "$2"
option_asm_forced=11022_W_"$1" assembler use forced
% The assembler output selected can not generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
%\end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_logo=11023_[
Compilateur Free Pascal version $FPCFULLVERSION [$FPCDATE] pour $FPCTARGET
Copyright (c) 1993-2020 by Florian Klaempfl and others
]

#
# Info (option -i)
#
option_info=11024_[
Compilateur Free Pascal version $FPCVERSION

Date du compilateur  : $FPCDATE
Cible du compilateur : $FPCTARGET
# the next lines are NOT translated on purpose !!!

This program comes under the GNU General Public Licence
For more information read COPYING.v2

Report bugs,suggestions etc to:
                 http://bugs.freepascal.org
]

#
# Help pages (option -? and -h)
#
option_help_pages=11025_[
**0*_mettre + aprŠs un bool‚en pour activer l'option, - pour la d‚sactiver
**1a_conserve les fichiers assembleurs cr‚‚s pendant la compilation
**2al_liste le code source dans les fichiers assembleur
**2ar_liste les allocations de registres dans les fichiers assembleur
**2at_liste les allocations de donn‚es temporaire dans la pile
**1b_g‚nŠre les informations pour le browser
**2bl_g‚nŠre les informations locales ‚galement
**1B_recompile toutes les unit‚s
**1C<x>_options de g‚n‚ration de code :
3*2CD_cr‚erune librairie dynamique
**2Ch<n>_<n> taille du tas en bytes (entre 1023 et 67107840)
**2Ci_IO-checking
**2Cn_pas de linking
**2Co_g‚nŠre des tests d'overflow pour les op‚rations sur les entiers
**2Cr_Controle d'intervalles
**2Cs<n>_sp‚cifie <n> comme taille de la pile
**2Ct_test de d‚bordement de pile
3*2CS_cr‚er une librairie statique
3*2Cx_utiliser le smartlinking
**1d<x>_d‚finit le symbole <x>
*O1D_g‚nŠre un fichier DEF
*O2Dd<x>_assigne la description … <x>
*O2Dw_application en mode prot‚g‚
**1e<x>_d‚finit le chemin vers l'ex‚cutable
**1E_comme -Cn
**1F<x>_d‚finit des chemins et noms de fichiers :
**2FD<x>_d‚finit le r‚pertoire o— chercher les utilitaires de compilation
**2Fe<x>_redirige les erreurs vers le fichier <x>
**2FE<x>_r‚pertoire pour les exe/unit‚s : <x>
**2Fi<x>_ajoute <x> … la liste des r‚pertoires pour fichiers inclus
**2Fl<x>_ajoute <x> … la liste des r‚pertoires pour librairies
*L2FL<x>_utilises <x> comme lieur dynamique
**2Fo<x>_ajoute <x> … la liste des r‚pertoires pour fichiers objets
**2Fr<x>_charge le fichier erreur <x>
**2Fu<x>_ajoute <x> … la liste des r‚pertoires pour unit‚s
**2FU<x>_d‚finit le r‚pertoire d'‚criture des unit‚s … <x>, en d‚pit de -FE
*g1g<x>_g‚nŠre des informations de d‚bogage :
*g2gg_utilise gsym
*g2gd_utilise dbx
*g2gh_use l'unit‚ de tracage du tas
*g2gc_generate checks for pointers
**1i_information
**2iD_donne la date du compilateur
**2iV_donne la version du compilateur
**2iSO_donne l'OS du compilateur
**2iSP_donne le processeur du compilateur
**2iTO_donne l'OS cible
**2iTP_donne le processeur cible
**1I<x>_ajoute <x> … la liste des r‚pertoires pour fichiers inclus
**1k<x>_transmet <x> au linker
**1l_‚crit le logo
**1n_ne pas lire le fichier de configuration par d‚faut
**1o<x>_change le nom de l'executable en <x>
**1pg_g‚nŠre du code pour profiler avec gprof
*L1P_utilise des pipes au lieu de cr‚er des fichiers temporaires
**1S<x>_options de syntaxe :
**2S2_autorise quelques extensions Delphi 2
**2Sc_autorise les operateurs type C (*=,+=,/= et -=)
**2Sd_essaye d'ˆtre compatible avec Delphi
**2Se_stoppe la compilation … la premiŠre erreur
**2Sg_autorise LABEL et GOTO
**2Sh_Utilise les ansistrings
**2Si_supporte les INLINE type C++
**2Sm_support des macros comme C (global)
**2So_essaye d'ˆtre compatible avec TP/BP 7.0
**2Sp_essaye d'ˆtre compatible avec GPC
**2Ss_les constructeurs doivent s'appeler init (et les destructeurs done)
**1s_n'appelle pas l'assembleur ni le linker
**1u<x>_rend le symbole <x> non d‚fini
**1U_options d'unit‚s :
**2Un_ne pas v‚rifier le nom de l'unit‚
**2Us_compiler en temps qu'unit‚ systŠme
**1v<x>_donne des informations. <x> est une combination des lettres suivantes :
**2*_e : montre les erreurs (d‚faut)   d : informations de d‚boggage
**2*_w : montre les avertissements     u : montre les info sur les unit‚s
**2*_n : montre les notes              t : montre les fichiers essay‚s/utilis‚s
**2*_h : montre les suggestions        m : montre les macros d‚finies
**2*_i : montre des infos g‚n‚rales    p : montre les proc‚dures compil‚es
**2*_l : montre les num‚ros de lignes  c : montre les conditionniels
**2*_a : montre tout                   0 : ne montre rien (sauf les erreurs)
**2*_b : montre toutes les proc‚dures  r : mode compatibilit‚ Rhide/GCC
**2*_    quand une erreur survient     x : infos Executable (Win32 seulement)
**2*_
**1X_options pour executable :
*L2Xc_lien avec librairie C
**2XD_lien avec la librarie dynamique (d‚finit FPC_LINK_DYNAMIC)
**2Xs_enlˆve tous les symboles de l'executable
**2XS_lien avec les librairies statiques (d‚finit FPC_LINK_STATIC)
**0*_options sp‚cifiques au processeur :
3*1A<x>_format de sortie :
3*2Aas_fichier objet g‚n‚r‚ par GNU AS
3*2Aasaout_fichier objet g‚n‚r‚ par GNU AS pour aout (Go32v1)
3*2Anasmcoff_fichier COFF (Go32v2) avec Nasm
3*2Anasmelf_fichier ELF32 (Linux) avec Nasm
3*2Anasmobj_fichier OBJ avec Nasm
3*2Amasm_fichier OBJ avec Masm (Microsoft)
3*2Atasm_fichier OBJ avec Tasm (Borland)
3*2Acoff_coff (Go32v2) using internal writer
3*2Apecoff_pecoff (Win32) using internal writer
3*1R<x>_type lecture assembleur :
3*2Ratt_lit l'assembleur AT&T
3*2Rintel_lit l'assembleur Intel
4*2Ratt_lit l'assembleur AT&T
4*2Rintel_lit l'assembleur Intel
8*2Ratt_lit l'assembleur AT&T
8*2Rintel_lit l'assembleur Intel
3*2Rdirect_copie le texte assembleur directement dans le fichier assembleur
3*1O<x>_optimisations :
3*2Og_g‚nŠre du code compact
3*2OG_g‚nŠre du code rapide (defaut)
3*2Or_garde certaines variables dans des registres (toujours BUGGY!!!)
3*2Ou_autorise les optimisations incertaines (voir docs)
3*2O1_optimisations niveau 1 (optimisations rapides)
3*2O2_optimisations niveau 2 (-O1 + optimisations plus lentes)
3*2O3_optimisations niveau 3 (comme -O2u)
3*2Op<x>_processeur cible :
3*3Op1_d‚finit 386/486 comme processeur cible
3*3Op2_d‚finit Pentium/PentiumMMX (tm) comme processeur cycle
3*3Op3_d‚finit PPro/PII/c6x86/K6 (tm) comme processeur cycle
3*1T<x>_systŠme d'expliotation cible:
3*2TEMX_OS/2 via EMX (et les extensions EMX/RSX)
3*2TGO32V2_version 2 de l'extension DOS de DJ Delorie
3*2TLINUX_Linux
3*2TNETWARE_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2TOS2_OS/2 / eComStation
3*2TSUNOS_SunOS/Solaris
3*2TWDOSX_WDOSX DOS extension
3*2TWIN32_Windows 32 Bits
6*1A<x>_output format
6*2Aas_Unix o-file using GNU AS
6*2Agas_GNU Motorola assembler
6*2Amit_MIT Syntax (old GAS)
6*2Amot_Standard Motorola assembler
6*1O_optimizations:
6*2Oa_turn on the optimizer
6*2Og_generate smaller code
6*2OG_generate faster code (default)
6*2Ox_optimize maximum (still BUGGY!!!)
6*2O2_set target processor to a MC68020+
6*1R<x>_styles d'assembleur :
6*2RMOT_lire l'assembleur type Motorola
6*1T<x>_SystŠme d'exploitation cible :
6*2TAMIGA_Commodore Amiga
6*2TATARI_Atari ST/STe/TT
6*2TMACOS_Macintosh m68k
6*2TLINUX_Linux-68k
**1*_
**1?_affiche cette aide
6*1R<x>_version de l'assembleur lu
6*2RMOT_read Motorola style assembler
6*1T<x>_Target operating system:
6*2TAMIGA_Commodore Amiga
6*2TATARI_Atari ST/STe/TT
6*2TMACOS_Macintosh m68k
6*2TLINUX_Linux-68k
**1*_
**1?_shows this help
**1h_affiche cette aide sans attente
]

#
# The End...
