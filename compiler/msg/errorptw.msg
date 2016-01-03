#
#   This file is part of the Free Pascal Compiler
#   Copyright (c) 1999-2008 by the Free Pascal Development team
#
#   Portuguese (Windows encoding) language file for Free Pascal Compiler
#   Contributed by Ari Ricardo Ody <ari.ody at japinfo.com.br>
#   Based on errore.msg of SVN revision 3935
#
#   See the file COPYING.FPC, included in this distribution,
#   for details about the copyright.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
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
#
# 01023 is the last used one
#
general_text_bytes_code=01019_bytes de código
general_text_bytes_data=01020_bytes de dados
# BeginOfTeX
% \section{General compiler messages}
% This section gives the compiler messages which are not fatal, but which
% display useful information. The number of such messages can be
% controlled with the various verbosity level \var{-v} switches.
% \begin{description}
general_t_compilername=01000_T_Compilador usado: $1
% When the \var{-vt} switch is used, this line tells you what compiler
% is used.
general_d_sourceos=01001_D_Sist.Op. da compilação: $1
% When the \var{-vd} switch is used, this line tells you what the source
% operating system is.
general_i_targetos=01002_I_Sist.Op. de execução: $1
% When the \var{-vd} switch is used, this line tells you what the target
% operating system is.
general_t_exepath=01003_T_Caminho para os executáveis: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its binaries.
general_t_unitpath=01004_T_Caminho para as units: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for compiled units. You can set this path with the \var{-Fu}
general_t_includepath=01005_T_Caminho para os includes: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for its include files (files used in \var{\{\$I xxx\}} statements).
% You can set this path with the \var{-I} option.
general_t_librarypath=01006_T_Caminho para as libraries: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for the libraries. You can set this path with the \var{-Fl} option.
general_t_objectpath=01007_T_Caminho para módulos objeto: $1
% When the \var{-vt} switch is used, this line tells you where the compiler
% looks for object files you link in (files used in \var{\{\$L xxx\}} statements).
% You can set this path with the \var{-Fo} option.
general_i_abslines_compiled=01008_I_$1 Linhas compiladas, $2 seg$3
% When the \var{-vi} switch is used, the compiler reports the number
% of lines compiled, and the time it took to compile them (real time,
% not program time).
general_f_no_memory_left=01009_F_Memória insuficiente para esta compilação
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
general_i_writingresourcefile=01010_I_Gravando arquivo com os Resource strings: $1
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_Gravando arquivo com os Resource strings: $1
% This message is shown when the compiler encountered an error when writing
% the Resource String Table file
general_i_fatal=01012_I_Erro fatal:
% Prefix for Fatal Errors
general_i_error=01013_I_Erro:
% Prefix for Errors
general_i_warning=01014_I_Aviso:
% Prefix for Warnings
general_i_note=01015_I_Nota:
% Prefix for Notes
general_i_hint=01016_I_Sugestão:
% Prefix for Hints
general_e_path_does_not_exist=01017_E_Caminho "$1" não existe
% The specified path does not exist.
general_f_compilation_aborted=01018_F_Compilação abortada
% Compilation was aborted.
general_i_number_of_warnings=01021_I_Ocorreram $1 warning(s)
% Total number of warnings issued during compilation.
general_i_number_of_hints=01022_I_Feitas $1 sugestões
% Total number of hints issued during compilation.
general_i_number_of_notes=01023_I_Feitos $1 comentários 
% Total number of notes issued during compilation.
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
scan_f_end_of_file=02000_F_Fim inesperado de arquivo
% this typically happens in one of the following cases :
% \begin{itemize}
% \item The source file ends before the final \var{end.} statement. This
% happens mostly when the \var{begin} and \var{end} statements aren't
% balanced;
% \item An include file ends in the middle of a statement.
% \item A comment was not closed
% \end{itemize}
scan_f_string_exceeds_line=02001_F_String não fechado na mesma linha
% There is a missing closing ' in a string, so it occupies
% multiple lines.
scan_f_illegal_char=02002_F_caracter inválido "$1" ($2)
% An illegal character was encountered in the input file.
scan_f_syn_expected=02003_F_Erro de sintaxe "$1" esperado mas "$2" encontrado
% This indicates that the compiler expected a different token than
% the one you typed. It can occur almost everywhere where you make a
% mistake against the pascal language.
scan_t_start_include_file=02004_TL_Começou a ler include file $1
% When you provide the \var{-vt} switch, the compiler tells you
% when it starts reading an included file.
scan_w_comment_level=02005_W_Encontrado comentário de nível $1
% When the \var{-vw} switch is used, then the compiler warns you if
% it finds nested comments. Nested comments are not allowed in Turbo Pascal
% and can be a possible source of errors.
scan_n_ignored_switch=02008_N_Ignorada diretiva "$1" do compilador
% With \var{-vn} on, the compiler warns if it ignores a switch
scan_w_illegal_switch=02009_W_Diretiva de compilador inválida "$1"
% You included a compiler switch (i.e. \var{\{\$... \}}) which the compiler
% does not recognise
scan_w_switch_is_global=02010_W_Diretiva global do compilador em local inválido
% The compiler switch is misplaced, and should be located at
% the start of the unit or program.
scan_e_illegal_char_const=02011_E_Constante caracter inválida
% This happens when you specify a character with its ASCII code, as in
% \var{\#96}, but the number is either illegal, or out of range.
scan_f_cannot_open_input=02012_F_Arquivo "$1" não encontrado
% \fpc cannot find the program or unit source file you specified on the
% command line.
scan_f_cannot_open_includefile=02013_F_Includefile "$1" não encontrado
% \fpc cannot find the source file you specified in a \var{\{\$include ..\}}
% statement.
scan_e_illegal_pack_records=02015_E_Especificador de alinhamento de registro inválido "$1"
% You are specifying the \var{\{\$PACKRECORDS n\} } or \var{\{\$ALIGN n\} }
% with an illegal value for \var{n}. For \$PACKRECORDS valid alignments are 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, and for \$ALIGN valid alignment are 1, 2, 4, 8, 16, 32, ON,
% OFF. Under mode MacPas \$ALIGN also supports MAC68K, POWER and RESET.
scan_e_illegal_pack_enum=02016_E_Especificador de tamanho mínimo de variável tipo enum inválido "$1"
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2,4, NORMAL or DEFAULT are valid here.
scan_e_endif_expected=02017_E_$ENDIF esperado para $1 $2 definidos em $3 na linha $4
% Your conditional compilation statements are unbalanced.
scan_e_preproc_syntax_error=02018_E_Erro de sintaxe na interpretação(parse) de uma expressão de compilação condicional
% There is an error in the expression following the \var{\{\$if ..\}}, $ifc or $setc compiler
% directives.
scan_e_error_in_preproc_expr=02019_E_Avaliação de uma expressão de compilação condicional
% There is an error in the expression following the \var{\{\$if ..\}}, $ifc or $setc compiler
% directives.
scan_w_macro_cut_after_255_chars=02020_W_O tamanho máximo de uma macro está limitado a 255 caracteres
% The contents of macros cannot be longer than 255 characters.
scan_e_endif_without_if=02021_E_ENDIF sem correspondente IF(N)DEF
% Your \var{\{\$IFDEF ..\}} and {\{\$ENDIF\}} statements aren't balanced.
scan_f_user_defined=02022_F_Definido pelo usuário: $1
% A user defined fatal error occurred. see also the \progref
scan_e_user_defined=02023_E_Definido pelo usuário: $1
% A user defined error occurred. see also the \progref
scan_w_user_defined=02024_W_Definido pelo usuário: $1
% A user defined warning occurred. see also the \progref
scan_n_user_defined=02025_N_Definido pelo usuário: $1
% A user defined note was encountered. see also the \progref
scan_h_user_defined=02026_H_Definido pelo usuário: $1
% A user defined hint was encountered. see also the \progref
scan_i_user_defined=02027_I_Definido pelo usuário: $1
% User defined information was encountered. see also the \progref
scan_e_keyword_cant_be_a_macro=02028_E_Macro que substitui palavra chave não tem efeito
% You cannot redefine keywords with macros.
scan_f_macro_buffer_overflow=02029_F_Overflow no buffer de macros durante leitura/expansão de macro
% Your macro or its result was too long for the compiler.
scan_w_macro_too_deep=02030_W_Mais de 16 níveis de aninhamento de macro.
% When expanding a macro, macros have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_w_wrong_styled_switch=02031_W_Diretivas de compilador não são aceitas em comentários tipo //
% Compiler switches should be in normal pascal style comments.
scan_d_handling_switch=02032_DL_Tratando diretiva "$1"
% When you set debugging info on (\var{-vd}) the compiler tells you when it
% is evaluating conditional compile statements.
scan_c_endif_found=02033_CL_ENDIF $1 encontrado
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifdef_found=02034_CL_IFDEF $1 encontrado, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifopt_found=02035_CL_IFOPT $1 encontrado $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_if_found=02036_CL_IF $1 encontrado, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_ifndef_found=02037_CL_IFNDEF $1 encontrado, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_else_found=02038_CL_ELSE $1 encontrado, $2
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements.
scan_c_skipping_until=02039_CL_Desprezando até...
% When you turn on conditional messages(\var{-vc}), the compiler tells you
% where it encounters conditional statements, and whether it is skipping or
% compiling parts.
scan_i_press_enter=02040_I_Tecle <return> para continuar
% When the \var{-vi} switch is used, the compiler stops compilation
% and waits for the \var{Enter} key to be pressed when it encounters
% a \var{\{\$STOP\}} directive.
scan_w_unsupported_switch=02041_W_Diretiva não suportada "$1"
% When warnings are turned on (\var{-vw}) the compiler warns you about
% unsupported switches. This means that the switch is used in Delphi or
% Turbo Pascal, but not in \fpc
scan_w_illegal_directive=02042_W_Diretiva de compilador inválida "$1"
% When warings are turned on (\var{-vw}) the compiler warns you about
% unrecognised switches. For a list of recognised switches, \progref
scan_t_back_in=02043_TL_De volta em $1
% When you use (\var{-vt}) the compiler tells you when it has finished
% reading an include file.
scan_w_unsupported_app_type=02044_W_Application type não suportado: "$1"
% You get this warning, if you specify an unknown application type
% with the directive \var{\{\$APPTYPE\}}
scan_w_app_type_not_support=02045_W_APPTYPE não suportado pelo Sist.Op. de execução
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_w_description_not_support=02046_W_Diretiva DESCRIPTION não é suportada pelo Sist.Op. de execução
% The \var{\{\$DESCRIPTION\}} directive is not supported on this target OS
scan_n_version_not_support=02047_N_Diretiva VERSION não é suportada pelo Sist.Op. de execução
% The \var{\{\$VERSION\}} directive is not supported on this target OS
scan_n_only_exe_version=02048_N_Diretiva VERSION é só para executáveis ou DLLs
% The \var{\{\$VERSION\}} directive is only used for executable or DLL sources.
scan_w_wrong_version_ignored=02049_W_Formato da diretiva VERSION errado "$1"
% The \var{\{\$VERSION\}} directive format is majorversion.minorversion
% where majorversion and minorversion are words.
scan_e_illegal_asmmode_specifier=02050_E_Valor inválido na diretiva ASMMODE "$1"
% When you specify an assembler mode with the \var{\{\$ASMMODE xxx\}}
% the compiler didn't recognize the mode you specified.
scan_w_no_asm_reader_switch_inside_asm=02051_W_Não é possível trocar tipo de assembly dentro de 1 trecho em assembly, a diretiva "$1" será válida só no próximo trecho assembly
% It is not possible to switch from one assembler reader to another
% inside an assembler block. The new reader will be used for next
% assembler statements only.
scan_e_wrong_switch_toggle=02052_E_Valor de diretiva inválido, use ON/OFF ou +/-
% You need to use ON or OFF or a + or - to toggle the switch
scan_e_resourcefiles_not_supported=02053_E_Resource files não são suportados no Sist.Op. de execução
% The target you are compiling for doesn't support resource files.
scan_w_include_env_not_found=02054_W_Variável de ambiente "$1" não encontrada
% The included environment variable can't be found in the environment, it will
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_Valor inválido para diretiva MAXFPUREGISTERS
% Valid values for this directive are 0..8 and NORMAL/DEFAULT
scan_w_only_one_resourcefile_supported=02056_W_O Sist.Op. de execução suporta somente 1 resource file
% The target you are compiling for supports only one resource file.
% The first resource file found is used, the others are discarded.
scan_w_macro_support_turned_off=02057_W_Suporte a macros foi desativado
% A macro declaration has been found, but macro support is currently off,
% so the declaration will be ignored. To turn macro support on compile with
% -Sm on the commandline or add \{\$MACRO ON\} in the source
scan_e_invalid_interface_type=02058_E_Tipo de interface inválido. Valores válidos são COM, CORBA or DEFAULT.
% The interface type that was specified is not supported
scan_w_appid_not_support=02059_W_APPID só é suportada pelo Sist.Op. de execução PalmOS
% The \var{\{\$APPID\}} directive is only supported for the PalmOS target.
scan_w_appname_not_support=02060_W_APPNAME só é suportada pelo Sist.Op. de execução PalmOS
% The \var{\{\$APPNAME\}} directive is only supported for the PalmOS target.
scan_e_string_exceeds_255_chars=02061_E_Constantes string não podem exceder 255 caracteres
% A single string constant can contain at most 255 chars. Try splitting up the
% string in multiple smaller parts and concatenate them with a + operator.
scan_f_include_deep_ten=02062_F_Número de include files aninhados excede 16 níveis
% When including include files the files have been nested to a level of 16.
% The compiler will expand no further, since this may be a sign that
% recursion is used.
scan_e_too_many_push=02063_F_Número máximo de níveis de PUSH excedido
% A maximum of 20 levels is allowed. This error occurs only in mode MacPas.
scan_e_too_many_pop=02064_E_POP não precedido de PUSH
% This error occurs only in mode MacPas.
scan_e_error_macro_lacks_value=02065_E_Macro ou variável de tempo de compilação "$1" vazia ou sem valor atribuído
% Thus the conditional compile time expression cannot be evaluated.
scan_e_wrong_switch_toggle_default=02066_E_Valor de diretiva inválido, use ON/OFF/DEFAULT ou +/-/*
% You need to use ON or OFF or DEFAULT or a + or - or * to toggle the switch
scan_e_mode_switch_not_allowed=02067_E_Diretiva Mode "$1" não permitida aqui
% A mode switch has already been encountered, or, in case of option -Mmacpas,
% a mode switch occur after UNIT.
scan_e_error_macro_undefined=02068_E_Variável de tempo de compilação "$1" não definida
% Thus the conditional compile time expression cannot be evaluated. Only in mode MacPas.
scan_e_utf8_bigger_than_65535=02069_E_Encontrado caracter com código UTF-8 maior do que 65535
% \fpc handles utf-8 strings internally as widestrings e.g. the char codes are limited to 65535
scan_e_utf8_malformed=02070_E_String UTF-8 com caracteres inválidos
% The given string isn't a valid UTF-8 string
scan_c_switching_to_utf8=02071_C_Encontrada assinatura UTF-8, conteúdo do arquivo será tratado como sendo UTF-8
% The compiler found an UTF-8 encoding signature (\$ef, \$bb, \$bf) at the beginning of a file,
% so it interprets it as an UTF-8 file
scan_e_compile_time_typeerror=02072_E_Expressão em tempo de compilação: Esperado $1 mas encontrado $2 em $3
% Type check of a compile time expression failed.
scan_n_app_type_not_support=02073_N_Diretiva APPTYPE não suportada pelo sistema operacional de destino
% The \var{\{\$APPTYPE\}} directive is supported by certain operating systems only.
scan_e_illegal_optimization_specifier=02074_E_Especificada otimizacao ilegal "$1"
% When you specify an optimization with the \var{\{\$OPTIMIZATION xxx\}}
% the compiler didn't recognize the optimization you specified.
scan_w_setpeflags_not_support=02075_W_Diretiva SETPEFLAGS não suportada pelo sistema operacional de destino
% The \var{\{\$SETPEFLAGS\}} directive is not supported by the target OS
scan_w_imagebase_not_support=02076_W_Diretiva IMAGEBASE não suportada pelo sistema operacional de destino
% The \var{\{\$IMAGEBASE\}} directive is not supported by the target OS
scan_w_minstacksize_not_support=02077_W_Diretiva MINSTACKSIZE não suportada pelo sistema operacional de destino
% The \var{\{\$MINSTACKSIZE\}} directive is not supported by the target OS
scan_w_maxstacksize_not_support=02078_W_Diretiva MAXSTACKSIZE não suportada pelo sistema operacional de destino
% The \var{\{\$MAXSTACKSIZE\}} directive is not supported by the target OS
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
parser_e_syntax_error=03000_E_Parser - Erro de sintaxe
% An error against the Turbo Pascal language was encountered. This happens
% typically when an illegal character is found in the sources file.
parser_e_dont_nest_interrupt=03004_E_INTERRUPT procedure não pode ser local tem que ser global
% An \var{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_Tipo de procedure "$1" ignorado
% The specified is ignored by FPC programs.
parser_e_no_overload_for_all_procs=03006_E_OVERLOAD usado para "$1". Todas declarações devem usar OVERLOAD
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_export_name_double=03008_E_Funções exportadas com o nome externo "$1" igual
% Exported function names inside a specific DLL must all be different
parser_e_export_ordinal_double=03009_E_Funções exportadas com o index externo "$1" igual
% Exported function names inside a specific DLL must all be different
parser_e_export_invalid_index=03010_E_Valor de index externo de função exportada inválido
% DLL function index must be in the range \var{1..\$FFFF}
parser_w_parser_reloc_no_debug=03011_W_Informações de depuração para DLL relocável ou para executável $1 não funcionaram. Depuração ignorada.
parser_w_parser_win32_debug_needs_WN=03012_W_Para permitir depuração de código win32 voce precisa desabilitar opção de relocação com a opção -WN
% Stabs info is wrong for relocatable DLL or EXES use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_Nome do método Constructor deve ser INIT
% You are declaring an object constructor with a name which is not \var{init}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_destructorname_must_be_done=03014_E_Nome do método Destructor deve ser DONE
% You are declaring an object destructor with a name which is not \var{done}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_proc_inline_not_supported=03016_E_Uso de procedures tipo INLINE como no C++ não suportado
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler doesn't support C++
% styled inlining by default.
parser_w_constructor_should_be_public=03018_W_Métodos Constructor devem estar em escopo "public" na classe
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_Métodos Destructor devem estar em escopo "public" na classe
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_Classes devem ter somente 1 método destructor
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_Declaração de classes em escopo "local" não permitida. Elas devem estar em escopo "global"
% Classes must be defined globally. They cannot be defined inside a
% procedure or function
parser_f_no_anonym_objects=03022_F_Classes sem métodos e não derivadas de outras(anônimas) não são permitidas
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_n_object_has_no_vmt=03023_N_Não há entrada na VMT para o objeto "$1"
% This is a note indicating that the declared object has no
% virtual method table.
parser_e_illegal_parameter_list=03024_E_Lista de argumentos inválida(tipos de dados diferem dos parâmetros)
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_size=03026_E_Número de parâmetros errado
% There is an error in the parameter list of the function or procedure,
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_Método que está fazendo overload e base do overload "$1" são de tipo diferente (function/procedure)
% The compiler encountered a symbol with the same name as an overloaded
% function, but it is not a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_Método tem mesma lista parâmetros do método sobre o qual queremos fazer overload
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_Tipo de retorno ou qualificador da função difere na declaração forward "$1"
% You declared a function with same parameters but
% different result type or function modifiers.
parser_e_header_different_var_names=03030_E_Header de função "$1" não combina com o do forward : nome de variável trocado $2 => $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but define it with a different parameter list.
parser_n_duplicate_enum=03031_N_Valores especificados em tipos enumerados devem estar em ordem crescente
% \fpc allows enumeration constructions as in C. Given the following
% declaration two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_e_no_with_for_variable_in_other_segments=03033_E_"With" não pode ser usado para variáveis in segmentos diferentes de memória
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_Mais de 31 funções aninhadas
% You can nest function definitions only 31 times.
parser_e_range_check_error=03035_E_Erro de "range" durante apuração de constantes
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_Erro de "range" durante apuração de constantes
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_Condição duplicada em 1 comando case
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_O valor superior de 1 "range" especificado num "case" é menor que o valor inferior
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless
parser_e_type_const_not_possible=03039_E_Constantes do tipo "class" ou "object" não são permitidas
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_Não é permitido uso de variáveis tipo "function" em functions que sofrerão overload
% You are trying to assign an overloaded function to a procedural variable.
% This is not allowed
parser_e_invalid_string_size=03041_E_Tamanho de string deve estar entre 1 e 255 bytes
% The length of a shortstring in Pascal is limited to 255 characters. You are
% trying to declare a string with length lower than 1 or greater than 255
parser_w_use_extended_syntax_for_objects=03042_W_Use a sintaxe extendida de NEW e DISPOSE para instâncias de objetos
% If you have a pointer \var{a} to a class type, then the statement
% \var{new(a)} will not initialize the class (i.e. the constructor isn't
% called), although space will be allocated. you should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the object
parser_w_no_new_dispose_on_void_pointers=03043_W_Não há sentido em usar NEW ou DISPOSE para pointers untyped
parser_e_no_new_dispose_on_void_pointers=03044_E_Não é possível usar NEW e DISPOSE para pointers untyped
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% Accepted for compatibility in \var{tp} and \var{delphi} modes.
parser_e_class_id_expected=03045_E_Identificador de classe ou objeto esperado antes do ponto
% This happens when the compiler scans a procedure declaration that contains
% a dot,
% i.e., a object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_Identificador de type não permitido neste contexto
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_Método esperado após o ponto
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_Header de procedure/função "$1" não identifica nenhum método desta classe
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_d_procedure_start=03049_DL_procedure ou function $1
% When using the \var{-vd} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_Constante de ponto flutuante inválida
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_"FAIL" só pode ser usado em "constructors"
% You are using the \var{fail} keyword outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_Métodos Destructor não podem ter parâmetros
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_methods_via_class_ref=03053_E_Somente métodos de classe podem ser usados numa referência de classe
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
parser_e_only_class_methods=03054_E_Somente métodos da classe podem ser acessados em métodos de classe
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
parser_e_case_mismatch=03055_E_Constante usada numa cláusula de um comando "case" não combina com o tipo da variável do "case"
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_O símbolo não pode ser exportado de 1 library. Só procedures ou functions
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_Um método herdado está escondido(hidden) por "$1". Verifique se não tem que usar a diretiva OVERRIDE
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendent class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_Não há método em classe ancestral com este nome para sofrer overriden : "$1"
% You are trying to \var{override} a virtual method of a parent class that does
% not exist.
parser_e_no_procedure_to_access_property=03059_E_Faltou a cláusula "read" na declaração da propriedade
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_Descontinuada
% The \var{stored} directive is not yet implemented
parser_e_ill_property_access_sym=03061_E_Símbolo inválido para acesso a uma propriedade(a propriedade é 1 array?)
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code would cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Campo de objeto declarado como protected não pode ser acessado aqui
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module where the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_Campo de objeto declarado como private não pode ser acessado aqui
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_e_overridden_methods_not_same_ret=03066_E_metodos envolvidos em override devem ter o mesmo tipo de retorno: "$2" faz override em "$1" que tem outro tipo de retorno
% If you declare overridden methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_Functions/procedures não podem ser declaradas aninhadas em functions/procedures que serão exportadas
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_métodos não podem ser exportados
% You cannot declare a procedure that is a method for an object as
% \var{export}ed.
parser_e_call_by_ref_without_typeconv=03069_E_argumentos de chamadas por referência tem que ter mesmo atributo que o parâmetro correspondente: Achado "$1" esperado "$2"
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_Metodo não é de classe ancestral da que o está chamando
% When calling inherited methods, you are trying to call a method of a non-related
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_Tentativa de usar "SELF" fora de um método
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_só pode usar uma chamada no formato classe.metodo dentro de um metodo
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_Uso de ':' em local inválido
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_Erro de range na declaração de set: Elemento(s) duplicado(s) ou fora de faixa p/o tipo de dado
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, either two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_Esperada pointer para um objeto
% You specified an illegal type in a \var{new} statement.
% The extended syntax of \var{new} needs an object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_Expressão deve ser uma chamada a um constructor
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_Expressão deve ser uma chamada a um destructor
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_Campos declarados em ordem diferente da declaração do record
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_O tipo de expressão deve ser de class ou de record
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_Subrotinas/métodos do tipo "procedure" não podem retornar valor
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures cannot return a value.
parser_e_constructors_always_objects=03081_E_Constructors e destructors devem ser métodos
% You're declaring a procedure as destructor or constructor, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_Um dos operandos não está na definição deste operador
% You're trying to use an overloaded operator when it is not overloaded for
% this type.
parser_e_no_such_assignment=03083_E_Impossível overload com dados do mesmo tipo
% You can not overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_Combinação de tipos do operador, dos argumentos e tipo de retorno incompatível
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_Uso de raise não é possível aqui
% You are trying to raise an exception where it is not allowed. You can only
% raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_Esta sintaxe de new ou dispose não pode ser aplicada a classes
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_procedure_overloading_is_off=03088_E_Overload de procedures está desabilitado(ver diretiva -S)
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_Este tipo de operador não pode sofrer overload
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_Function que faça overload no operador = deve retornar "boolean"
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_Somente métodos virtuais podem ser declarados "abstract"
% You are declaring a method as abstract, when it is not declared to be
% virtual.
parser_f_unsupported_feature=03092_F_Feature não suportada!
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_A mistura de diferentes tipos de objetos(class, object, interface, etc) não é permitida
% You cannot derive \var{objects}, \var{classes}, \var{cppclasses} and \var{interfaces} interttwined . E.g.
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_Diretiva desconhecida foi ignorada: "$1"
% The procedure directive you specified is unknown.
parser_e_absolute_only_one_var=03095_E_Absolute pode ser usado em linha que declare somente 1 variável e redefinir variável ou "typed constant"
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
parser_e_absolute_only_to_var_or_const=03096_E_Absolute só pode redefinir 1 variável ou constante
% The address of a \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_Não pode usar valor inicial em linha que declare mais de 1 variável
% You cannot specify more than one variable with a initial value
% in Delphi mode.
parser_e_abstract_no_definition=03098_E_Métodos abstract só podem ser declarados. Não podem conter código
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_Esta função que está sofrendo overload não pode ser local (deve aparecer na interface)
% You are defining a overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_Objeto/classe contém métodos virtuais mas não contém contructor em "$1"
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_c_macro_defined=03101_CL_Macro definida: $1
% When \var{-vc} is used, the compiler tells you when it defines macros.
parser_c_macro_undefined=03102_CL_Macro liberada ou "des-definida": $1
% When \var{-vc} is used, the compiler tells you when it undefines macros.
parser_c_macro_set_to=03103_CL_Macro $1 assumiu valor $2
% When \var{-vc} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_Compilando "unit" $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_UL_Interpretando "interface" da "unit" $1
% This tells you that the reading of the interface
% of the current unit starts
parser_u_parsing_implementation=03106_UL_Interpretando "implementation" da "unit" $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_DL_Recompilando $1
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_property_found_to_override=03109_E_Propriedade para sofrer override não existe
% You want to override a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_Tentativa de definir propriedade default para classe que já tem 1 default
% You specified a property as \var{Default}, but the class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_A propriedade default deve ser uma "array property"
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_Virtual constructors podem ser usados somente para classes
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_Não há propriedade default disponível(nem por herança)
% You are trying to access a default property of a class, but this class (or one of
% its ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_A classe não pode ter uma "published section", use a diretiva {$M+}
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, whch turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_A declaração com forward da classe "$1" precisa ser resolvida aqui para poder usar esta classe como ancestral
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% Where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_Não pode fazer overload de operadores locais
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_A diretiva "$1" não é permitida seção "interface"
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_A diretiva "$1" não é permitida seção "implementation"
% This procedure directive is not defined in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_A diretiva "$1" não é permitida na área de declaração de variáveis de 1 procedure/function
% This procedure directive cannot be part of a procedural or function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_Procedure/function já foi declarada como Public/Forward "$1"
% You will get this error if a function is defined as \var{forward} twice.
% Or it is once in the \var{interface} section, and once as a \var{forward}
% declaration in the \var{implmentation} section.
parser_e_not_external_and_export=03121_E_Não pode usar EXPORT e EXTERNAL simultaneamente
% These two procedure directives are mutually exclusive
parser_w_not_supported_for_inline=03123_W_"$1" ainda não é suportada em "inline procedure/function"
% Inline procedures don't support this declaration.
parser_w_inlining_disabled=03124_W_O uso de INLINE está desabilitado(diretiva $I)
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_Avisa que gravou log do símbolo $1 gerado pelo "symbol browser"
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+ \}} switch).
parser_h_maybe_deref_caret_missing=03126_H_Pode estar faltando uma "pointer dereference". Meramente um comentário p/questões de sintaxe em relação ao Delphi p/exemplo
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_Tipo de assembly não suportado
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_Diretiva "$1" conflita com outras diretivas
% You specified a procedure directive that conflicts with other directives.
% for instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_Interface/implementation "calling conventions" não combinam(cdecl?)
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_property_cant_have_a_default_value=03131_E_A propriedade não pode ter valor default(é tipo array ou set?)
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_O valor default de 1 propriedade deve ser uma constante
% The value of a \var{default} declared property must be known at compile
% time. The value you specified is only known at run time. This happens
% .e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_Simbolo não pode ser published. Somente propriedades e variáveis "class type"
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_Propriedades com este tipo de dados não podem ser "published"
% Properties in a \var{published} section cannot be array properties.
% they must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_e_empty_import_name=03136_E_Procedures/functions que são exportadas de DLL's devem ter 1 "external name"
% Some targets need a name for the imported procedure or a \var{cdecl} specifier
parser_e_division_by_zero=03138_E_Divisão por zero
% There is a division by zero encounted
parser_e_invalid_float_operation=03139_E_Operação de ponto flutuante inválida
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_Fim da faixa menor que o início da faixa
% The upper bound of a an array declaration is less than the lower bound and this
% is not possible
parser_w_string_too_long=03141_W_Constante string "$1" é maior que a declaração da variável string "$2"
% The size of the constant string is larger than the size you specified in
% string type definition
parser_e_string_larger_array=03142_E_Constante string maior do que o "array of char" a qual está sendo atribuída
% The size of the constant string is larger than the size you specified in
% the array[x..y] of char definition
parser_e_ill_msg_expr=03143_E_Expressão inválida após uma diretiva
% \fpc supports only integer or string values as message constants
parser_e_ill_msg_param=03144_E_Métodos declarados com a diretiva {message}(message handlers) só podem ter 1 parâmetro e recebido por referêcia
% A method declared with the \var{message}-directive as message handler
% can take only one parameter which must be declared as call by reference
% Parameters are declared as call by reference using the \var{var}-directive
parser_e_duplicate_message_label=03145_E_Label de mensagem duplicado: "$1"
% A label for a message is used twice in one object/class
parser_e_self_in_non_message_handler=03146_E_"Self" só pode ser usado como parâmetro explícito em métodos declarados com diretiva {message}(message handlers)
% The self parameter can only be passed explicitly to a method which
% is declared as message handler.
parser_e_threadvars_only_sg=03147_E_Variáveis "Threadvars" s´ podem ser "static" ou "global"
% Threadvars must be static or global, you can't declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has its own stack and local variables
% are stored on the stack
parser_f_direct_assembler_not_allowed=03148_F_"Direct assembly" não é suportado quando usando formato binário de saída
% You can't use direct assembler when using a binary writer, choose an
% other outputformat or use an other assembler reader
parser_w_no_objpas_use_mode=03149_W_Não carregue OBJPAS unit com "uses", use \{\$mode objfpc\} ou \{\$mode delphi\} ao invés disto
% You are trying to load the ObjPas unit manually from a uses clause. This is
% not a good idea. Use the \var{\{\$mode objfpc\}} or
% \var{\{\$mode delphi\}}
% directives which load the unit automatically
parser_e_no_object_override=03150_E_Não é possível fazer OVERRIDE em métodos de "object's"
% Override is not supported for objects, use \var{virtual} instead to override
% a method of a parent object
parser_e_cant_use_inittable_here=03151_E_Tipos de dados que requeiram inicialização/finalização não podem ser usados em registros "variant"
% Some data type (e.g. \var{ansistring}) needs initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% can't be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_Resourcestrings só podem ser declarados static ou global
% Resourcestring can not be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_"Exit" com argumento neste contexto, não é permitido
% an exit statement with an argument for the return value can't be used here, this
% can happen e.g. in \var{try..except} or \var{try..finally} blocks
parser_e_stored_property_must_be_boolean=03154_E_Símbolo de tipo de memoria deve ser booleano
% If you specify a storage symbol in a property declaration, it must be of
% the type boolean
parser_e_ill_property_storage_sym=03155_E_Este símbolo não é permitido como símbolo memória
% You can't use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants
parser_e_only_publishable_classes_can__be_published=03156_E_Só é permitido publicar classes compiladas com a diretiva $M+
% In the published section of a class can be only class as fields used which
% are compiled in \var{\{\$M+\}} or which are derived from such a class. Normally
% such a class should be derived from TPersitent
parser_e_proc_directive_expected=03157_E_Esperada diretiva de procedure
% When declaring a procedure in a const block you used a ; after the
% procedure declaration after which a procedure directive must follow.
% Correct declarations are:
% \begin{verbatim}
% const
%   p : procedure;stdcall=nil;
%   p : procedure stdcall=nil;
% \end{verbatim}
parser_e_invalid_property_index_value=03158_E_O valor do "index" de uma propriedade deve ser do tipo ordinal
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_Nome de procedure/function que será exportada tem que ter no mínimo 2 caracteres
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correct with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_Não foi possível gerar 1 DEFFILE para as variáveis globais da "unit"
parser_e_dlltool_unit_var_problem2=03161_E_Compile sem a opção -WD
% You need to compile this file without the -WD switch on the
% commandline
parser_f_need_objfpc_or_delphi_mode=03162_F_Voce precisa de ObjFpc (-S2) ou Delphi (-Sd) mode para compilar este módulo
% You need to use \{\$mode objfpc\} or \{\$mode delphi\} to compile this file.
% Or use the equivalent commandline switches -S2 or -Sd.
parser_e_no_export_with_index_for_target=03163_E_Não é possível exportar functions/procedures para este Sist.Op. $1
% Exporting of functions or procedures with a specified index is not
% supported on this target.
parser_e_no_export_of_variables_for_target=03164_E_Exportação de variáveis não é permitida no Sist.Op. $1
% Exporting of variables is not supported on this target.
parser_e_improper_guid_syntax=03165_E_Sintaxe de declaração GUID inválida
parser_w_interface_mapping_notfound=03168_W_Procedure/function "$1" não encontrada em contexto aplicável para implementar $2.$3
parser_e_interface_id_expected=03169_E_Esperado identificador de interface
% This happens when the compiler scans a \var{class} declaration that contains
% \var{interface} function name mapping code like this:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% and the \var{interface} before the dot not listed in the inheritance list.
parser_e_type_cant_be_used_in_array_index=03170_E_Tipo "$1" não pode ser usado como "array index type"
% Types like \var{qword} or \var{int64} aren't allowed as array index type
parser_e_no_con_des_in_interfaces=03171_E_Constructors/destructors não são permitidos em interfaces
% Constructor and destructor declarations aren't allowed in interface
% In the most cases the method \var{QueryInterface} of \var{IUnknown} can
% be used to create a new interface.
parser_e_no_access_specifier_in_interfaces=03172_E_Especificacoes de visibilidade não podem ser usadas em INTERFACES
% The access specifiers \var{public}, \var{private}, \var{protected} and
% \var{pusblished} can't be used in interfaces because all methods
% of an interfaces must be public.
parser_e_no_vars_in_interfaces=03173_E_Interfaces não podem conter definições de campos
% Declarations of fields aren't allowed in interfaces. An interface
% can contain only methods
parser_e_no_local_proc_external=03174_E_Não é permitido declarar procedures locais como EXTERNAL
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance of errors very high
parser_w_skipped_fields_before=03175_W_Alguns campos antes de "$1" não foram inicializados
% In Delphi mode, not all fields of a typed constant record have to be
% initialized, but the compiler warns you when it detects such situations.
parser_e_skipped_fields_before=03176_E_Alguns campos antes de "$1" não foram inicializados
% In all syntax modes but Delphi mode, you can't leave some fields uninitialized
% in the middle of a typed constant record
parser_w_skipped_fields_after=03177_W_Alguns campos após "$1" não foram inicializados
% You can leave some fields at the end of a type constant record uninitialized
% (the compiler will initialize them to zero automatically). This may be the cause
% of subtle problems.
parser_e_varargs_need_cdecl_and_external=03178_E_A diretiva VarArgs usada em procedure/function não CDecl nem External
% The varargs directive can only be used with procedures or functions
% that are declared with \var{cdecl} and \var{external} directives. The varargs directive
% is only meant to provide a compatible interface to C functions like printf.
parser_e_self_call_by_value=03179_E_Self deve ser 1 parâmetro recebido p/valor(não pode ser uma variável/constante)
% You can't declare self as a const or var parameter, it must always be
% a call-by-value parameter
parser_e_interface_has_no_guid=03180_E_Interface "$1" precisa ter 1 identificação GUID
% When you want to assign an interface to a constant, then the interface
% must have a GUID value set.
parser_e_illegal_field_or_method=03181_E_Campo de classe ou nome de método desconhecido "$1"
% Properties must refer to a field or method in the same class.
parser_w_proc_overriding_calling=03182_W_"Calling convention" "$2" substituirá a "$1"
% There are two directives in the procedure declaration that specify a calling
% convention. Only the last directive will be used
parser_e_no_procvarobj_const=03183_E_"Typed" constantes do tipo "procedure of object" só podem ser inicializadas com NIL
% You can't assign the address of a method to a typed constant which has a
% 'procedure of object' type, because such a constant requires two addresses:
% that of the method (which is known at compile time) and that of the object or
% class instance it operates on (which can not be known at compile time).
parser_e_default_value_only_one_para=03184_E_Valor default só pode ser atribuido a 1 parametro
parser_e_default_value_expected_for_para=03185_E_Parâmetro default requerido para "$1"
parser_w_unsupported_feature=03186_W_Use de feature não suportada
% You're trying to force the compiler into doing something it cannot do yet.
parser_h_c_arrays_are_references=03187_H_Parametros passados a functions/procedures em C devem ser passados por referência(pointer)
% Any array passed to a C functions is passed
% by a pointer (i.e. by reference).
parser_e_C_array_of_const_must_be_last=03188_E_Array de constantes deve ser o último argumento de uma chamada do C
% You can not add any other argument after an \var{array of const} for
% \var{cdecl} functions, as the size pushed on stack for this argument is
% not known.
parser_h_type_redef=03189_H_O "type" "$1" já foi criado, tentativa de redefinição
% This is an indicator that a previously declared type is
% being redefined as something else. This may, or may not
% be, a cause for errors.
parser_w_cdecl_has_no_high=03190_W_Functions/procedures declaradas CDECL não suportam funções built in do compilador FPC
% Functions declared with cdecl modifier do not pass an extra implicit parameter.
parser_w_cdecl_no_openstring=03191_W_Functions/procedures declaradas CDECL devem ser chamadas com string de tamanho igual ao declarado no cabecalho da Function/procedure
% Openstring is not supported for cdecl'ared functions.
parser_e_initialized_not_for_threadvar=03192_E_Variáveis "threadvar" não podem ser inicializadas
% Variables declared as threadvar can not be initialized with a default value.
% The variables will always be filled with zero at the start of a new thread.
parser_e_msg_only_for_classes=03193_E_A diretiva MESSAGE só é permitida em Classes
% The message directive is only supported for Class types.
parser_e_procedure_or_function_expected=03194_E_Procedure/Function esperada
% A class method can only be specified for procedures and functions.
parser_e_illegal_calling_convention=03195_W_Diretiva de "calling convention" ignorada: "$1"
% Some calling conventions are supported only by certain CPUs. I.e. most non-i386 ports support
% only the standard ABI calling convention of the CPU.
parser_e_no_object_reintroduce=03196_E_REINTRODUCE não pode ser usado em objetos
% \var{reintroduce} is not supported for objects.
parser_e_paraloc_only_one_para=03197_E_Cada argumento tem sua devida posição
% If locations for arguments are specified explicitly as it is required by
% some syscall conventions, each argument must have it's only location, things
% like \var{procedure p(i,j : longint 'r1');} aren't allowed
parser_e_paraloc_all_paras=03198_E_Cada argumento tem sua posição específica
% If one argument has an explicit argument location, all arguments of a procedure
% must have one.
parser_e_illegal_explicit_paraloc=03199_E_Posicao do argumento desconhecida
% The location specified for an argument isn't recognized by the compiler
parser_e_32bitint_or_pointer_variable_expected=03200_E_Esperado um inteiro de 32 bits ou uma pointer
% The libbase for MorphOS/AmigaOS can be give only as \var{longint}, \var{dword} or any pointer variable.
parser_e_goto_outside_proc=03201_E_Comando GOTO não pode desviar para label fora da procedure/function
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
parser_f_too_complex_proc=03202_F_Procedure muito complexa, ela requer registradores demais
% Your procedure body is too long for the compiler. You should split the
% procedure into multiple smaller procedures.
parser_e_illegal_expression=03203_E_Expressão inválida
% This can occur under many circumstances. Mostly when trying to evaluate
% constant expressions.
parser_e_invalid_integer=03204_E_Resultado da expressão não é "integer"
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
parser_e_invalid_qualifier=03205_E_Qualificador inválido
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
parser_e_upper_lower_than_lower=03206_E_Limite superior < limite inferior
% You are declaring a subrange, and the lower limit is higher than the high
% limit of the range.
parser_e_macpas_exit_wrong_param=03207_E_O parâmetro do Exit deve ser o nome da procedure onde ele é usado
% Non local exit is not allowed. This error occurs only in mode MacPas.
parser_e_illegal_assignment_to_count_var=03208_E_Assinalamento inválido para a variável de controle do loop "for" "$1"
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
parser_e_no_local_var_external=03209_E_Variáveis locais não podem ser declaradas EXTERNAL
% Declaring local variables as external is not allowed. Only global variables can reference
% to external variables.
parser_e_proc_already_external=03210_E_Procedure já foi declarada EXTERNAL
% The procedure is already declared with the EXTERNAL directive in an interface or
% forward declaration.
parser_w_implicit_uses_of_variants_unit=03211_W_Assumido uso de Variants unit
% The Variant type is used in the unit without any used unit using the Variants unit. The
% compiler has implicitly added the Variants unit to the uses list. To remove this warning
% the Variants unit needs to be added to the uses statement.
parser_e_no_static_method_in_interfaces=03212_E_Classes e métodos static não podem ser usados em INTERFACES
% The specifier \var{class} and directive \var{static} can't be used in interfaces
% because all methods of an interfaces must be public.
parser_e_arithmetic_operation_overflow=03213_E_Overflow em operação aritmética
% An operation on two integers values produced an overflow
parser_e_protected_or_private_expected=03214_E_Protected ou private esperados
% \var{strict} can be only used together with \var{protected} or \var{private}.
parser_e_illegal_slice=03215_E_SLICE não pode ser usado fora de uma lista de parâmetros
% \var{slice} can be used only for arguments accepting an open array parameter
parser_e_dispinterface_cant_have_parent=03216_E_Uma DISPINTERFACE não pode ter classe pai
% A DISPINMTERFACE is a special type of interface which can't have a parent class
parser_e_dispinterface_needs_a_guid=03217_E_Uma DISPINTERFACE precisa de uma identificação
% A DISPINMTERFACE always needs an interface identification
parser_w_overridden_methods_not_same_ret=03218_W_Metodos que fazem override em outros devem ter tipos de retorno relacionados ("$2" sofre override de "$1" que tem outro tipo de retorno))
% If you declare overridden methods in a class definition, they must
% have the same return type. Some versions of Delphi allow you to change the
% return type of interface methods, and even to change procedures into
% functions, but the resulting code may crash depending on the types used
% and the way the methods are called.
parser_e_dispid_must_be_ord_const=03219_E_Identificadores de prioridade de seleção devem ser constantes ordinais
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
type_e_mismatch=04000_E_Type não combina
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_Tipos de dado incompatíveis. Encontrado "$1" esperado "$2"
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
type_e_not_equal_types=04002_E_Tipos de "$1" e "$2" não combinam
% The types are not equal
type_e_type_id_expected=04003_E_Identificador de tipo(type) esperado
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_Esperada uma variável
% This happens when you pass a constant to a routine (such as \var{Inc} var or \var{Dec})
% when it expects a variable. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_Esperada expressão do tipo "integer", mas encontrada "$1"
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_Esperada expressão do tipo "boolean", mas encontrada "$1"
% The expression must be a boolean type, it should be return true or
% false.
type_e_ordinal_expr_expected=04007_E_Esperada expressão do tipo "ordinal"
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_Esperado tipo pointer, mas encontrado "$1"
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_Esperada uma classe, mas encontrado "$1"
% The variable of expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_cant_eval_constant_expr=04011_E_Não pude resolver a expressão
% This error can occur when the bounds of an array you declared does
% not evaluate to ordinal constants
type_e_set_element_are_not_comp=04012_E_Os elementos dos "sets" são incompatíveis
% You are trying to make an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union
type_e_set_operation_unknown=04013_E_Operação não implementada para "sets"
% several binary operations are not defined for sets
% like div mod ** (also >= <= for now)
type_w_convert_real_2_comp=04014_W_Conversão automática de floating type para COMP o qual é do tipo "integer"
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_Use o operador DIV para obter um resultado "integer"
% When hints are on, then an integer division with the '/' operator will
% procuce this message, because the result will then be of type real
type_e_strict_var_string_violation=04016_E_String types não combinam, por causa da diretiva $V+
% When compiling in \var{\{\$V+\}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_Succ ou Pred em tipos "enumerated" que contenham atribuições não é possível
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use the \var{Succ} or \var{Pred} functions on them.
type_e_cant_read_write_type=04018_E_Não é possível ler ou gravar variáveis deste "type"
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that. Only integer types,
% reals, pchars and strings can be read from/written to a text file.
% Booleans can only be written to text files.
type_e_no_readln_writeln_for_typed_file=04019_E_Não pode usar readln ou writeln em arquivo "typed"
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_Não pode usar read ou write em arquivo "untyped"
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_Conflito de tipo entre 1 ou mais elementos e o tipo de dados do SET
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_lo/hi(dword/qword) retorna upper/lower nos formatos word/dword
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword/int64/qword}
% which returns the lower/upper word/dword of the argument. TP always uses
% a 16 bit \var{lo/hi} which returns always bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the TP behavior you have
% to type cast the argument to \var{word/integer}
type_e_integer_or_real_expr_expected=04023_E_Esperada expressão "integer" ou "real"
% The first argument to \var{str} must a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_Tipo "$1" inválido em "array constructor"
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_Tipo incompatível para o arg no. $1: Recebido "$2", esperado "$3"
% You are trying to pass an invalid type for the specified parameter.
type_e_no_method_and_procedure_not_compatible=04026_E_Variáveis do tipo "procedure of object" não são compatíveis com variáveis do tipo "procedure"
% You can't assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_Constante inválida passada para função matemática
% The constant argument passed to a ln or sqrt function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_Não é possível obter endereço de constantes
% It is not possible to get the address of a constant, because they
% aren't stored in memory, you can try making it a typed constant.
type_e_argument_cant_be_assigned=04029_E_Não se pode atribuir dados a este argumento
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference argument
% Remark: Properties can be only
% used on the left side of an assignment, but they cannot be used as arguments
type_e_cannot_local_proc_to_procvar=04030_E_Não pode atribuir procedure/function local a uma variável do tipo "procedure"
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling of local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_Não pode atribuir valores a um endereço
% It is not allowed to assign a value to an address of a variable,constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_Não pode atribuir valores a uma variável declarada como "const"
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const, to allow
% changing the value make the parameter as a value parameter or a var.
type_e_array_required=04033_E_Requerido tipo array
% If you are accessing a variable using an index '[<x>]' then
% the type must be an array. In FPC mode also a pointer is allowed.
type_e_interface_type_expected=04034_E_Esperado tipo "interface" mas encontrado "$1"
type_w_mixed_signed_unsigned=04035_W_Misturar expressões com sinal e "longword" gera resultado de 64 bits
% If you divide (or calculate the modulus of) a signed expression by a longword (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetic
% expression (+, -, *, div, mod) in which both signed numbers and longwords appear,
% then everything has to be evaluated in 64bit which is slower than normal
% 32bit arithmetic. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_Misturar expressões com sinal e "cardinals" neste ponto pode causar "range check error"
% If you use a binary operator (and, or, xor) and one of
% the operands is a longword while the other one is a signed expression, then,
% if range checking is turned on, you may get a range check error because in
% such a case both operands are converted to longword before the operation is
% carried out. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_e_typecast_wrong_size_for_assignment=04037_E_Variável que sofreu typecast ficou com tamanho diferente ($1 -> $2)
% Type casting to a type with a different size is not allowed when the variable is
% used for assigning.
type_e_array_index_enums_with_assign_not_possible=04038_E_Membro de "enumerated" que contenha atribuições, não pode ser usado como índice de array
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use it as index of an array.
type_e_classes_not_related=04039_E_Classes ou "objects" de tipos "$1" and "$2" não estão relacionados(os tipos)
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors
type_w_classes_not_related=04040_W_Classes "types" "$1" e "$2" não estão relacionados
% There is a typecast from one class or object to another while the class/object
% are not related. This will probably lead to errors
type_e_class_or_interface_type_expected=04041_E_Esperada uma classe ou um interface mas encontrado "$1"
type_e_type_is_not_completly_defined=04042_E_O "type" "$1" não está completamente definido
type_w_string_too_long=04043_W_Literal tem mais caracteres que o tamanho de "short string"
% The size of the constant string, which is assigned to a shortstring,
% is longer than the maximum size of the shortstring
type_w_signed_unsigned_always_false=04044_W_Comparação será sempre falsa devido ao "range" de valores
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% false. Exlicitly typecast the constant to the correct range to avoid this problem.
type_w_signed_unsigned_always_true=04045_W_Comparação será sempre verdadeira devido ao "range" de valores
% There is a comparison between an unsigned value and a signed constant which is
% less than zero. Because of type promotion, the statement will always evaluate to
% true. Exlicitly typecast the constant to the correct range to avoid this problem.
type_w_instance_with_abstract=04046_W_Construindo classe "$1" mas superclasse tem métodos virtuais que não sofreram override
% An instance of a class is created which contains non-implemented abstract
% methods. This will probably lead to a runtime error 211 in the code if that
% routine is ever called. All abstract methods should be overriden.
type_h_in_range_check=04047_H_Operando a esquerada de um operador IN tem que ter tamanho de 1 byte
% The left operand of the \var{in} operator is not an ordinal or enumeration which fits
% within 8-bits, this may lead to range check errors. The \var{in} operator
% currently only supports a left operand which fits within a byte. In the case of
% enumerations, the size of an element of an enumeration can be controlled with
% the \var{\{\$PACKENUM\}} or \var{\{\$Zn\}} switches.
type_w_smaller_possible_range_check=04048_W_"Types" de tamanho diferente, haverá possível perda de dados e/ou "range check error"
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_h_smaller_possible_range_check=04049_H_Tamanho dos types não combina, possível perda de dados ou erro de "range"
% There is an assignment to a smaller type than the source type. This means that
% this may cause a range-check error, or may lead to possible loss of data.
type_e_cant_take_address_of_abstract_method=04050_E_Não foi possível obter o endereço de 1 método abstrato
% An abstract method has no body, so the address of an abstract method can't be taken.
type_e_operator_not_allowed=04051_E_O operador não se aplica ao tipo de operando
% You are trying an operator that is not available for the type of the
% operands
type_e_constant_expr_expected=04052_E_Esperada expressão do tipo constante
% The compiler expects an constant expression, but gets a variable expression.
type_e_operator_not_supported_for_types=04053_E_Operação "$1" não permitida para os tipos "$2" and "$3"
% The operation is not allowed for the supplied types
type_e_illegal_type_conversion=04054_E_Conversão ilegal de tipos: "$1" para "$2"
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
type_h_pointer_to_longint_conv_not_portable=04055_H_Conversão entre "ordinals" e "pointer" não é portável
% If you typecast a pointer to a longint (or vice-versa), this code will not compile
% on a machine using 64-bit for pointer storage.
type_w_pointer_to_longint_conv_not_portable=04056_W_Conversão entre "ordinals" e "pointer" não é portável
% If you typecast a pointer to a ordinal type of a different size (or vice-versa), this can
% cause problems. This is a warning to help finding the 32bit specific code where cardinal/longint is used
% to typecast pointers to ordinals. A solution is to use the ptrint/ptruint types instead.
type_e_cant_choose_overload_function=04057_E_Não foi possível determinar qual das functions que sofreram overload deve ser chamada
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
type_e_illegal_count_var=04058_E_Variável usada como contador é de tipo inválido
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
type_w_double_c_varargs=04059_W_Convertendo constante tipo real p/tipo double p/ser usada como variável de C, adicione typecast explícito p/evitar isto
% In C, constant real values are double by default. For this reason, if you
% pass a constant real value to a variable argument part of a C function, FPC
% by default converts this constant to double as well. If you want to prevent
% this from happening, add an explicit typecast around the constant.
type_e_class_or_cominterface_type_expected=04060_E_Esperada classe ou interface tipo COM, mas encontrado "$1"
% Some operators like the AS operator are only appliable to classes or COM interfaces.
% \end{description}
#
# Symtable
#
# 05055 is the last used one
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Identificador não encontrado "$1"
% The compiler doesn't know this symbol. Usually happens when you misspel
% the name of a variable or procedure, or when you forgot to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_Erro interno no SymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_Identificador duplicado "$1"
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_Identificador já definido em $1 na linha $2
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_Identificador desconhecido "$1"
% The identifier encountered has not been declared, or is used outside the
% scope where it is defined.
sym_e_forward_not_resolved=05005_E_Declaração "forward" não resolvida "$1"
% This can happen in two cases:
% \begin{itemize}
% \item This happens when you declare a function (in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_e_error_in_type_def=05007_E_Erro na definição de um "type"
% There is an error in your definition of a new array type:
% \item One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_forward_type_not_resolved=05009_E_"Forward type" "$1" não resolvido
% A symbol was forward defined, but no declaration was encountered.
sym_e_only_static_in_static=05010_E_Apenas variáveis "static" podem ser usadas em metodos "static" ou metodos externos ao bloco
% A static method of an object can only access static variables.
sym_f_type_must_be_rec_or_class=05012_F_Esperado um record ou uma classe
% The variable or expression isn't of the type \var{record} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Não é permitido instanciar classes ou objetos através de métodos abstratos
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_"Label" não definido "$1"
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_"Label" usado mas não definido "$1"
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_Declaração de label inválida
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO e LABEL não suportados (use a diretiva -Sg)
% You must compile a program which has \var{label}s and \var{goto} statements
% with the \var{-Sg} switch. By default, \var{label} and \var{goto} aren't
% supported.
sym_e_label_not_found=05018_E_Label não encontrado
% A \var{goto label} was encountered, but the label isn't declared.
sym_e_id_is_no_label_id=05019_E_Identificador não é um label
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_Lable já definido
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_Type inválido na declaração dos elementos de um set
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_Definição da classe citada num "forward" não resolvida "$1"
% You declared a class, but you did not implement it.
sym_n_unit_not_used=05023_H_"Unit" "$1" não usada em $2
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_Identificador "$1" não usado
% The identifier was declared (locally or globally) but
% was not used (locally or globally).
sym_n_local_identifier_not_used=05025_N_Variável local "$1" não usada
% You have declared, but not used a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_Identificador "$1" declarado com valor mas nunca usado
% The identifier was declared (locally or globally)
% set but not used (locally or globally).
sym_n_local_identifier_only_set=05027_N_Variável local "$1" é declarada/inicializada mas nunca usada
% The variable in a procedure or function
% implementation is declared, set but never used.
sym_h_local_symbol_not_used=05028_H_Símbolo local $1 "$2" não é usado
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_Campo "private" "$1.$2" nunca é usado
sym_n_private_identifier_only_set=05030_N_Campo "private" "$1.$2" é inicializado mas nunca usado
sym_n_private_method_not_used=05031_N_Metodo "private" "$1.$2" nunca usado
sym_e_set_expected=05032_E_Esperado tipo "set"
% The variable or expression is not of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_Resultado de uma function parece não ter sido atribuído
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_Type "$1" parece não estar alinhado corretamente no registro corrente para em estruturas C
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_Campo de "record" desconhecido "$1"
% The field doesn't exist in the record/object definition.
sym_w_uninitialized_local_variable=05036_W_Variável local "$1" parece não ter sido inicializada
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_w_uninitialized_variable=05037_W_Variavel "$1" não parece ter sido inicializada
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_e_id_no_member=05038_E_Identificador não identifica nenhum dado ou método "$1"
% This error is generated when an identifier of a record,
% field, or method is accessed while it is not defined.
sym_h_param_list=05039_H_Declaração encontrada mas procedure que sofre o overload não: $1
% You get this when you use the \var{-vh} switch. In case an overloaded
% procedure is not found, then all candidate overloaded procedures are
% listed, with their parameter lists.
sym_e_segment_too_large=05040_E_Elemento de dados muito grande
% You get this when you declare a data element whose size exceeds the
% prescribed limit (2 Gb on 80386+/68020+ processors)
sym_e_no_matching_implementation_found=05042_E_Não encontrada implementação de metodo para a interface "$1"
% There was no matching method found which could implement the interface
% method. Check argument types and result type of the methods.
sym_w_deprecated_symbol=05043_W_Symbol "$1" está obsoleto(a)
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{deprecated} is used. Deprecated symbols may no longer
% be available in newer versions of the unit / library. Usage of this symbol
% should be avoided as much as possible.
sym_w_non_portable_symbol=05044_W_Símbolo "$1" não é portavel
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{platform} is used. This symbol's value, usage
% and availability is platform specific and should not be used
% if the source code must be portable.
sym_w_non_implemented_symbol=05055_W_Símbolo "$1" não está implementado
% This means that a symbol (a variable, routine, etc...) which is
% declared as \var{unimplemented} is used. This symbol is defined,
% but is not yet implemented on this specific platform.
sym_e_cant_create_unique_type=05056_E_Não é possível criar 1 "type" que seja único a partir deste "type"
% Only simple types like ordinal, float and string types are supported when
% redefining a type with \var{type newtype = type oldtype;}.
sym_h_uninitialized_local_variable=05057_H_Variável local "$1" não parece ter sido inicializada
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_h_uninitialized_variable=05058_H_Variável "$1" não parece ter sido inicializada
% This message is displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% was not initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_w_function_result_uninitialized=05059_W_Variável resultado de uma function não parece estar sendo inicializada
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. appears in the right-hand-side of an expression)
% before it is initialized (i.e. appeared in the left-hand side of an
% assigment)
sym_h_function_result_uninitialized=05060_H_Variável resultado de uma function não parece estar sendo inicializada
% This message is displayed if the compiler thinks that the function result
% variable will be used (i.e. appears in the right-hand-side of an expression)
% before it is initialized (i.e. appeared in the left-hand side of an
% assigment)
sym_w_identifier_only_read=05061_W_Variavel "$1" usada em um read mas não sofre assinalamento em lugar algum
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
cg_e_parasize_too_big=06009_E_Lista de parâmetros excede 65535 bytes
% The I386 processor limits the parameter list to 65535 bytes (the \var{RET}
% instruction causes this)
cg_e_file_must_call_by_reference=06012_E_"File types" devem ser variáveis
% You cannot specify files as value parameters, i.e. they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_O uso de pointer tipo "far" não é permitido
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_dont_call_exported_direct=06015_E_Functions declaradas EXPORT não podem ser chamadas
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_Possível chamada inválida a constructor ou destructor
% The compiler detected that a constructor or destructor is called within a
% a method. This will probably lead to problems, since constructors / destructors
% require parameters on entry.
cg_n_inefficient_code=06017_N_Código pouco eficiente
% Your statement seems dubious to the compiler.
cg_w_unreachable_code=06018_W_Código nunca será executado
% You specified a construct which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_cant_call_abstract_method=06020_E_Métodos abstratos não podem ser chamadaos diretamente
% You cannot call an abstract method directy, instead you must call a
% overriding child method, because an abstract method isn't implemented.
cg_d_register_weight=06027_DL_Registrador $1 tem prioridade sobre $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_d_stackframe_omited=06029_DL_"Stack frame" omitido
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_e_unable_inline_object_methods=06031_E_Objetos ou métodos não podem ser inline.
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_Call a variáveis do tipo procedure não podem ser inline
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_Nenhum código foi carregado para a procedure inline
% The compiler couldn't store code for the inline procedure.
cg_e_can_access_element_zero=06035_E_O elemento zero de 1 ansi/wide- ou longstring não pode ser acessado, use SetLength
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such a string types
cg_e_cannot_call_cons_dest_inside_with=06037_E_Constructors ou destructors não podem ser chamados de dentro de 1 cláusula 'with'
% Inside a \var{with} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_Não é possível acessar diretamente métodos do tipo "message handler"(declarados com diretiva message)
% A message method handler method cannot be called directly if it contains an
% explicit self argument
cg_e_goto_inout_of_exception_block=06039_E_Pulo para dentro ou para fora de 1 "exception block"
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
cg_e_control_flow_outside_finally=06040_E_Comandos de controle de fluxo de execução não são permitidos dentro de blocos "finally"
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
cg_w_parasize_too_big=06041_W_Tamanho total dos parâmetros excede o limite de alguns processadores
% This indicates that you are declaring more than 64K of parameters, which
% might not be supported on other processor targets.
cg_w_localsize_too_big=06042_W_O tamanho total das variáveis locais excede o limite de alguns processadores
% This indicates that you are declaring more than 32K of local variables, which
% might not be supported on other processor targets.
cg_e_localsize_too_big=06043_E_O tamanho das variáveis locais excede o limite suportado
% This indicates that you are declaring more than 32K of local variables, which
% is not supported by this processor.
cg_e_break_not_allowed=06044_E_BREAK não permitido neste ponto
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06045_E_CONTINUE não permitido neste ponto
% You're trying to use \var{continue} outside a loop construction.
cg_f_unknown_compilerproc=06046_F_Procedure de compilador "$1" inválida. Verifique se usou a library de run time correta
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
asmr_d_start_reading=07000_DL_Iniciando a montagem do bloco assembly $1
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_DL_Montagem do bloco assembly $1 finalizada
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_Encontrado o caracter @ em identificador que não é label
% A identifier which isn't a label can't contain a @.
asmr_e_building_record_offset=07004_E_Erro durante a apuração de um deslocamento(offset)
% There has an error occured while building the offset of a record/object
% structure, this can happend when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_Deslocamento(OFFSET) usado sem um identificador
% You can only use OFFSET with an identifier. Other syntaxes aren't
% supported
asmr_e_type_without_identifier=07006_E_"TYPE" usado sem identificador
% You can only use TYPE with an identifier. Other syntaxes aren't
% supported
asmr_e_no_local_or_para_allowed=07007_E_Não pode usar variável local ou parâmetros aqui
% You can't use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the frame pointer register so the
% address can't be obtained directly.
asmr_e_need_offset=07008_E_É necessário o uso do operador OFFSET aqui
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_Precisa usar $ aqui
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Não pode usar múltiplos símbolos relocáveis
% You can't have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_Simbolo relocável só pode ser usado para adição
% Relocatable symbols (variable/typed constant) can't be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_Expressão constante inválida
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_Símbolo relocável não permitido aqui
% You can't use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_Erro de sintaxe
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_Nao permitido acionar $1 a partir deste código
% You can not read directly the value of a local variable or parameter
% of a higher level procedure in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_Símbolos e labels locais não são permitidos como referências
% You can't use local symbols/labels as references
asmr_e_wrong_base_index=07017_E_Uso inválido de registradores base e indexador
% There is an error with the base and index register, they are
% probably incorrect
asmr_w_possible_object_field_bug=07018_W_Possível erro no manuseio de algum campo de objeto
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_Especificado fator de escala errado
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_Uso de múltiplos registradores indexadores
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_Tipo de operando inválido
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_Opcode inválido: $1
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE e @DATA não suportados
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_Null label references não permitidas
asmr_e_expr_zero_divide=07025_E_Divisão por zero no analisador do assembler
% There is a division by zero in a constant expression
asmr_e_expr_illegal=07026_E_Expressão ilegal
% There is an illegal expression in a constant expression
asmr_e_escape_seq_ignored=07027_E_Sequencia de escape ignorada: $1
% There is a C-styled string, but the escape sequence in the string
% is unknown, and is therefore ignored
asmr_e_invalid_symbol_ref=07028_E_Referência a símbolo inválida
asmr_w_fwait_emu_prob=07029_W_Fwait pode causar problemas de emulação com emu387
asmr_w_fadd_to_faddp=07030_W_$1 sem operandos traduzida para $1P
asmr_w_enter_not_supported_by_linux=07031_W_Instrução ENTER não é suportada pelo kernel do Linux
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_Chamada em assembly a 1 método que sofreu overload
% There is a call to an overloaded method in the assembler block,
% this might be the sign there is a problem
asmr_e_unsupported_symbol_type=07033_E_Símbolo de "type" não suportado para operando
asmr_e_constant_out_of_bounds=07034_E_Constante fora da faixa
asmr_e_error_converting_decimal=07035_E_Erro convertendo decimal $1
% A constant decimal value does not have the correct syntax
asmr_e_error_converting_octal=07036_E_Erro convertendo octal $1
% A constant octal value does not have the correct syntax
asmr_e_error_converting_binary=07037_E_Erro convertendo binario $1
% A constant binary value does not have the correct syntax
asmr_e_error_converting_hexadecimal=07038_E_Error convertendo hexadecimal $1
% A constant hexadecimal value does not have the correct syntax
asmr_h_direct_global_to_mangled=07039_H_$1 traduzido para $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 está associado a uma function que sofreu overload
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Não pode usar SELF fora de um método
% There is a reference to the \var{self} symbol while it is not
% allowed. \var{self} can only be referenced inside methods
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Não pode usar OLDEBP fora de uma procedure aninhada
% There is a reference to the \var{oldebp} symbol while it is not
% allowed. \var{oldebp} can only be referenced inside nested routines
asmr_e_void_function=07043_W_Procedures não podem retornar nenhum valor no código assembly
% Trying to return a value while in a procedure. A procedure
% does not have any return value
asmr_e_SEG_not_supported=07044_E_SEG não suportado
asmr_e_size_suffix_and_dest_dont_match=07045_E_Sufixo de tamanho e o destino ou tamanho da origem não combinam
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_w_size_suffix_and_dest_dont_match=07046_W_Tamanho do "suffix" e o tamanho do destino ou da origem não combinam
% The register size and the opcode size suffix don't match. This is
% probably an error in the assembler statement
asmr_e_syntax_error=07047_E_Erro de sintaxe assembly
% There is an assembler syntax error
asmr_e_invalid_opcode_and_operand=07048_E_Combinação de opcode e operandos inválida
% The opcode cannot be used with this type of operand
asmr_e_syn_operand=07049_E_Erro de sintaxe assembly em operando
asmr_e_syn_constant=07050_E_Erro de sintaxe assembly em constante
asmr_e_invalid_string_expression=07051_E_Expressão string inválida
asmr_w_const32bit_for_address=07052_W_Constante com símbolo $1 para endereço que não está numa pointer
% A constant expression represents an address which does not fit
% into a pointer. The address is probably incorrect
asmr_e_unknown_opcode=07053_E_Opcode não reconhecido $1
% This opcode is not known
asmr_e_invalid_or_missing_opcode=07054_E_Opcode inválido ou ausente
asmr_e_invalid_prefix_and_opcode=07055_E_Combinação de opcode e prefixo inválida: $1
asmr_e_invalid_override_and_opcode=07056_E_Combinação inválida de opcode e override: $1
asmr_e_too_many_operands=07057_E_Operandos em excesso numa linha
% There are too many operands for this opcode. Check your
% assembler syntax
asmr_w_near_ignored=07058_W_NEAR ignorado
asmr_w_far_ignored=07059_W_FAR ignorado
asmr_e_dup_local_sym=07060_E_Símbolo local duplicado $1
asmr_e_unknown_local_sym=07061_E_Símbolo local não definido $1
asmr_e_unknown_label_identifier=07062_E_Label desconhecido $1
asmr_e_invalid_register=07063_E_Nome de registrador inválido
% There is an unknown register name used as operand.
asmr_e_invalid_fpu_register=07064_E_Nome de registrador de ponto flutuante inválido
% There is an unknown register name used as operand.
asmr_w_modulo_not_supported=07066_W_Função "mod"(resto) não suportada
asmr_e_invalid_float_const=07067_E_Constante de ponto flutuante inválida $1
% The floating point constant declared in an assembler block is
% invalid.
asmr_e_invalid_float_expr=07068_E_Expressão de ponto flutuante inválida
% The floating point expression declared in an assembler block is
% invalid.
asmr_e_wrong_sym_type=07069_E_Símbolo de tipo inválido
asmr_e_cannot_index_relative_var=07070_E_Nao pode indexar com registrador uma variável local ou parâmetro(que já são indexados a 1 reg. base)
% Trying to index using a base register a symbol which is already relative
% to a register. This is not possible, and will probably lead to crashes.
asmr_e_invalid_seg_override=07071_E_Expressão com override inválido de segmento
asmr_w_id_supposed_external=07072_W_Identificador $1 assumido como external
% There is a reference to an undefined symbol. This will not result
% in an error, since the symbol might be external, but may cause
% problems at link time if the symbol is not defined anywhere.
asmr_e_string_not_allowed_as_const=07073_E_Strings não permitidos como constantes
% Character strings are not allowed as constants.
asmr_e_no_var_type_specified=07074_Nenhum tipo de variável especificado
% The syntax expects a type idenfitifer after the dot, but
% none was found.
asmr_w_assembler_code_not_returned_to_text=07075_E_O codigo assembly não gera retorno a seção de texto
% There was a directive in the assembler block to change sections,
% but there is a missing return to the text section at the end
% of the assembler block. This might cause errors during link time.
asmr_e_not_directive_or_local_symbol=07076_E_Não é uma diretiva nem um símbolo local $1
% This symbol is unknown.
asmr_w_using_defined_as_local=07077_E_Usando um nome auto-definido como label local
asmr_e_dollar_without_identifier=07078_E_Um símbolo "$" usado sem estar no início de 1 identificador
% A constant expression has an identifier which does not start with
% the $ symbol.
asmr_w_32bit_const_for_address=07079_W_Criada uma constante de 32 bits para endereçamento
% A constant was used as an address. This is probably an error,
% since using absolute addresses will probably not work.
asmr_n_align_is_target_specific=07080_N_.align é especifico de plataforma, use .balign ou .p2align
% Using the .align directive is platform specific, and its meaning will vary
% from one platform to another.
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Não pode acessar diretamente os campos de parâmetros
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_Não pode acessar diretamente campos de objetos/classes
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
asmr_e_unable_to_determine_reference_size=07083_E_Não especificado tamanho e não foi possível determinar o tamanho dos operandos
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference.
asmr_e_cannot_use_RESULT_here=07084_E_Não pode usar RESULT nesta function
% Some functions which return complex types cannot use the \var{result}
% keyword.
asmr_w_adding_explicit_args_fXX=07086_W_"$1" sem operando traduzido para "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" traduzido para "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" traduzido para "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_O caracter < não é permitido aqui
% The shift operator requires the << characters. Only one
% of those characters was found.
asmr_e_invalid_char_greater=07090_E_O caracter > não é permitido aqui
% The shift operator requires the >> characters. Only one
% of those characters was found.
asmr_w_align_not_supported=07093_W_ALIGN não suportado
asmr_e_no_inc_and_dec_together=07094_E_"Inc" e "Dec" não podem estar juntos
% Trying to use an increment and a decrement within the same
% opcode on the 680x0. This is impossible.
asmr_e_invalid_reg_list_in_movem=07095_E_Registrador(es) inválido(s) para "movem"
% Trying to use the \var{movem} opcode with invalid registers
% to save or restore.
asmr_e_invalid_reg_list_for_opcode=07096_E_Registrador(es) inválidos para o opcode
asmr_e_higher_cpu_mode_required=07097_E_Requerido modo de CPU mais avançado ($1)
% Trying to use an instruction which is not supported in the current
% cpu mode. Use a higher cpu generation to be able to use this
% opcode in your assembler block
asmr_w_unable_to_determine_reference_size_using_dword=07098_W_Não foi especificado tamanho e não é possivel determinar o tamanho dos operandos, DWORD usado como default
% You should specify explicitly a size for the reference, because
% the compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference. This warning is only used in Delphi mode where
% it falls back to use DWORD as default.
asmr_e_illegal_shifterop_syntax=07099_E_Erro de sintaxe enquanto tentava montar uma operação de "shift"
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
asmw_f_too_many_asm_files=08000_F_Excesso de arquivos assembly
% With smartlinking enabled, there are too many assembler
% files generated. Disable smartlinking.
asmw_f_assembler_output_not_supported=08001_F_Tipo de saida do assembler não suportado
asmw_f_comp_not_supported=08002_F_"Comp" não suportado
asmw_f_direct_not_supported=08003_F_"Direct" não suportado para "binary writers"
% Direct assembler mode is not supported for binary writers.
asmw_e_alloc_data_only_in_bss=08004_E_Alocação de dados só é permitida na seção bss
asmw_f_no_binary_writer_selected=08005_F_Nenhum "binary writer" selecionado
asmw_e_opcode_not_in_table=08006_E_Assembler: Opcode $1 não existe na tabela
asmw_e_invalid_opcode_and_operands=08007_E_Asssmbler: $1 combinação de opcode e operadores inválida
asmw_e_16bit_not_supported=08008_E_Assembler: Referências de 16 bits não suportadas
asmw_e_invalid_effective_address=08009_E_Assembler: Endereço efetivo inválido
asmw_e_immediate_or_reference_expected=08010_E_Assembler: Esperados um operador imediato ou uma referência
asmw_e_value_exceeds_bounds=08011_E_Assembler: Valor de $1 excede os limites de $2
asmw_e_short_jmp_out_of_range=08012_E_Assembler: "Short jump" fora de faixa $1("short jump" gera troca de segmento de memória?)
asmw_e_undefined_label=08013_E_Assembler: Label indefinido $1
asmw_e_comp_not_supported=08014_E_Assembler: Type "Comp" não suportado na plataforma destino
asmw_e_extended_not_supported=08015_E_Assembler: "Type" Extended não suportado na plataforma destino
asmw_e_duplicate_label=08016_E_Assembler: Label duplicado $1
asmw_e_redefined_label=08017_E_Assembler: Label redefinido $1
asmw_e_first_defined_label=08018_E_Assembler: Definido a primeira vez aqui
asmw_e_invalid_register=08019_E_Assembler: Registrador inválido $1
asmw_e_16bit_32bit_not_supported=08020_E_Asm: Referencias em 16 e 32 bits não suportadas
asmw_e_64bit_not_supported=08021_E_Asm: Operandos de 64 bits não suportados

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
exec_w_source_os_redefined=09000_W_Sist Oper fonte redefinido
exec_i_assembling_pipe=09001_I_Montagem em andamento $1
exec_d_cant_create_asmfile=09002_E_Assembler não pode criar o arquivo $1
% The mentioned file can't be created. Check if you have got
% access permissions to create this file
exec_e_cant_create_objectfile=09003_E_Não pode criar o arquivo objeto: $1
% The mentioned file can't be created. Check if you've
% got access permissions to create this file
exec_e_cant_create_archivefile=09004_E_Não pode criar o arquivo "archive": $1
% The mentioned file can't be created. Check if you've
% access permissions to create this file
exec_e_assembler_not_found=09005_E_Assembler $1 não encontrado, mudando para montagem assembly externa
exec_t_using_assembler=09006_T_Usando o assembler: $1
exec_e_error_while_assembling=09007_E_Erro durante a montagem assembly -> exitcode $1
% There was an error while assembling the file using an external assembler.
% Consult the documentation of the assembler tool to find out more information
% on this error.
exec_e_cant_call_assembler=09008_E_Não pode chamar o assembler, erro $1. Mudando para montagem assembly externa
exec_i_assembling=09009_I_Montando $1
exec_i_assembling_smart=09010_I_Montagem com smartlinking $1
exec_w_objfile_not_found=09011_W_Objeto $1 não encontrado, a link-edição pode falhar!
% One of the object file is missing, and linking will probably fail.
% Check your paths.
exec_w_libfile_not_found=09012_W_Library $1 não encontrada, a link-edição pode falhar!
% One of the library file is missing, and linking will probably fail.
% Check your paths.
exec_e_error_while_linking=09013_E_Erro durante a link-edição
% Generic error while linking.
exec_e_cant_call_linker=09014_E_Não pode chamar o linkage editor interno, mudando para link-edição externa
exec_i_linking=09015_I_Link edição de $1
exec_e_util_not_found=09016_E_Utilitário $1 não encontrado, mudando para link-edição externa
exec_t_using_util=09017_T_Usando utilitário $1
exec_e_exe_not_supported=09018_E_Criação de executáveis não suportada
exec_e_dll_not_supported=09019_E_Criação de libraries Dynamic/Shared não suportada
exec_i_closing_script=09020_I_Fechando script $1
exec_e_res_not_found=09021_E_Recurso do compilador não encontrado, mudando para modo externo
exec_i_compilingresource=09022_I_Recurso de compilação $1
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_A "unit" $1 não pode ser link-editada de forma estática, mudando para o modo "smart linking"
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_A "unit" $1 não pode ser link-editada no modo "smart linking", mudando para link-edição estática
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_A "unit" $1 nao pode ser link-editada em modo "shared linking", mudando para link-edicao estática
exec_e_unit_not_smart_or_static_linkable=09026_E_A "unit" $1 não pode ser link-editada nem no modo "smart linking" nem em link-edição estática
exec_e_unit_not_shared_or_static_linkable=09027_E_A "unit" $1 não pode ser link-editada nem no modo "shared linking" nem em link-edição estática
exec_d_resbin_params=09028_D_Chamando o compilador "$1" com a linha de comando "$2"
%\end{description}
# EndOfTeX

#
# Executable information
#
execinfo_f_cant_process_executable=09128_F_Não pode pós-processar o executável $1
execinfo_f_cant_open_executable=09129_F_Não pode abrir o executável $1
execinfo_x_codesize=09130_X_Tamanho do código: $1 bytes
execinfo_x_initdatasize=09131_X_Tamanho da área de dados inicializados: $1 bytes
execinfo_x_uninitdatasize=09132_X_Tamanho da área de dados não-inicializados: $1 bytes
execinfo_x_stackreserve=09133_X_Espaço reservado para o stack: $1 bytes
execinfo_x_stackcommit=09134_X_Espaço realmente usado pelo stack: $1 bytes

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
unit_t_unitsearch=10000_T_Caminho onde o compilador tentará achar "units": $1
% When you use the \var{-vt}, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_"Units" carregadas pelo compilador $1 (.PPU)
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_Nome da "unit"(.PPU): $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_Flags da "unit"(.PPU): $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_Check sum da "unit"(.PPU): $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_Data de compilação da "unit"(.PPU): $1
% When you use the \var{-vu} flag, the time the unit was compiled is shown.
unit_u_ppu_file_too_short=10006_U_Arquivo PPU da "unit" muito pequeno
% The ppufile is too short, not all declarations are present.
unit_u_ppu_invalid_header=10007_U_Header de arquivo PPU(de uma "unit") inválido(não tem PPU no início)
% A unit file contains as the first three bytes the ascii codes of \var{PPU}
unit_u_ppu_invalid_version=10008_U_Versão de arquivo PPU(de uma "unit") inválida $1
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_"Unit"(arquivo PPU) compilada para outro processador
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_invalid_target=10010_U_"Unit"(arquivo PPU) foi compilada para outro sistema operacional
% This unit file was compiled for a different target, and
% cannot be read
unit_u_ppu_source=10011_U_Check sum do arquivo PPU da "unit": $1
% When you use the \var{-vu} flag, the unit source file name is shown.
unit_u_ppu_write=10012_U_Gravando as "units"(arquivos PPU) em: $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_Nao pode gravar o arquivo PPU
% An error occurred when writing the unit file.
unit_f_ppu_read_error=10014_F_Erro durante a leitura de um arquivo PPU
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_read_unexpected_end=10015_F_Fim de arquivo PPU inesperado
% Unexpected end of file. This may mean that the PPU file is
% corrupted.
unit_f_ppu_invalid_entry=10016_F_Arquivo PPU em formato inválido: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_Problema na contagem Dbx
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_Nome da "unit" inválido $1. Diverge do nome do arquivo que a contem.
% The name of the unit does not match the file name.
unit_f_too_much_units=10019_F_Há "units" demais. Excedem o limite definido para a compilação(diretiva maxunits)
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{fmodule.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_"Units" $1 e $2 referem-se mutuamente(referência circular)
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_"Unit" $1 precisa ser recompilada mas o fonte não foi encontrado
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_"Unit"(arquivo PPU) "$1" não pode ser encontrada
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your configuration file for the unit paths
unit_w_unit_name_error=10023_W_"Unit" $1 não foi encontrada mas a $2 existe
unit_f_unit_name_error=10024_F_"Unit" $1 procurada mas a $2 foi encontrada
% Dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_Para compilar a "unit" System tem que usar a diretiva -Us
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} must be specified.
unit_f_errors_in_unit=10026_F_Houve $1 erros compilando o módulo, compilação abortada
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_Carregando $3 a partir da "unit" $1. A carga foi requisitada em $2.
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_Recompilando a "unit" $1, checksum mudou para $2
unit_u_recompile_source_found_alone=10029_U_Recompilando $1 porque foi encontrado somente o fonte
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_Recompilando a "unit", algum das "static libraries" é mais antiga que o arquivo PPU
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit are older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_Recompilando a "unit", a "shared lib" ‚ mais antiga que o ppufile
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit are older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_Recompilando a "unit". Módulos obj e asm tem data mais antiga que o PPU
% When you use the \var{-vu} flag, the compiler warns if the assembler or
% object file of the unit are older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_Recompilando a "unit". Módulo obj mais antigo que o asm
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_parsing_interface=10034_U_Interpretando interface da "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit
unit_u_parsing_implementation=10035_U_Interpretando implementation da "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit
unit_u_second_load_unit=10036_U_Segunda carga da "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_check_time=10037_U_Verificando arquivo PPU $1 data/hora $2
% When you use the \var{-vu} flag, the compiler show the filename and
% date and time of the file which a recompile depends on
### The following two error msgs is currently disabled.
#unit_h_cond_not_set_in_last_compile=10038_H_Conditional $1 was not set at startup in last compilation of $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that currently is defined, but was not used the last
#% time the unit was compiled.
#unit_h_cond_set_in_last_compile=10039_H_Conditional $1 was set at startup in last compilation of $2
#% when recompilation of an unit is required the compiler will check that
#% the same conditionals are set for the recompiliation. The compiler has
#% found a conditional that was used the last time the unit was compiled, but
#% the conditional is currently not defined.
unit_w_cant_compile_unit_with_changed_incfile=10040_W_Não pode recompilar "unit" $1, mas foram encontrados arquivos de include modificados
% A unit was found to have modified include files, but
% some source files were not found, so recompilation is impossible.
unit_h_source_modified=10041_H_Arquivo $1 é mais recente que o arquivo PPU $2 que foi compilado com flag de release -Ur
% A modified source file for a unit was found that was compiled with the
% release flag (-Ur). The unit will not implicitly be recompiled because
% this release flag is set.
unit_u_ppu_invalid_fpumode=10042_U_Usando uma "unit" que foi compilada com tipo diferente de emulação de ponto flutuante(FPU)
% Trying to compile code while using units which were not compiled with
% the same floating point format mode. Either all code should be compiled
% with FPU emulation on, or with FPU emulation off.
unit_u_loading_interface_units=10043_U_Incluindo "units" relacionadas na interface da "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the interface part of the unit.
unit_u_loading_implementation_units=10044_U_Incluindo "units" relacionadas na implementation da "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the units defined in the implementation part of the unit.
unit_u_interface_crc_changed=10045_U_Check sum da Interface modificado para a "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated for the interface has been changed after the implementation
% has been parsed.
unit_u_implementation_crc_changed=10046_U_Check sum da Implementation modificado para a "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it the
% CRC calculated has been changed after the implementation
% has been parsed.
unit_u_finished_compiling=10047_U_Compilação da "unit" $1 terminada
% When you use the \var{-vu} flag, the compiler warns that it
% has finished compiling the unit.
unit_u_add_depend_to=10048_U_Dependência de $1 adicionada a $2
% When you use the \var{-vu} flag, the compiler warns that it
% has added a dependency between the two units.
unit_u_no_reload_is_caller=10049_U_Não precisa carregar de novo, "unit" já está carregada $1
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is the unit that wants
% to load this unit
unit_u_no_reload_in_second_compile=10050_U_Não precisa carregar a unit de novo, é uma recompilação $1
% When you use the \var{-vu} flag, the compiler warns that it
% has will not reload the unit because it is already in a second recompile
unit_u_flag_for_reload=10051_U_Recarregando: $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to reload the unit
unit_u_forced_reload=10052_U_Recarga forçada
% When you use the \var{-vu} flag, the compiler warns that it
% has is reloading the unit because it was required
unit_u_previous_state=10053_U_Estado anterior de $1: $2
% When you use the \var{-vu} flag, the compiler shows the
% previous state of the unit
unit_u_second_compile_unit=10054_U_Já compilando $1, setando opção de recompilação
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_loading_unit=10055_U_Carregando a "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% loading the unit.
unit_u_finished_loading_unit=10056_U_Carga da "unit" $1 terminada
% When you use the \var{-vu} flag, the compiler warns that it finished
% loading the unit.
unit_u_registering_new_unit=10057_U_Registrando a nova "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it has
% found a new unit and registers it in the internal lists.
unit_u_reresolving_unit=10058_U_Recalculando área de dados da "unit" $1
% When you use the \var{-vu} flag, the compiler warns that it
% has to recalculate the internal data of the unit
unit_u_skipping_reresolving_unit=10059_U_Pulando o recalculo da área de dados da "unit" $1, "units" usadas já carregadas
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
option_usage=11000_O_$1 [opções] <arquivo entrada> [opções]
# BeginOfTeX
%
% \section{Command-line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_only_one_source_support=11001_W_Suportado apenas um arquivo fonte
% You can specify only one source file on the command line. The first
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_Arquivo DEF só pode ser criado para OS/2
% This option can only be specified when you're compiling for OS/2
option_no_nested_response_file=11003_E_Response files aninhados não permitidos
% you cannot nest response files with the \var{@file} command-line option.
option_no_source_found=11004_F_Nenhum arquivo fonte especifica na linha de comando
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_Nenhuma opção dentro do arquivo de configuração $1
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_Parâmetro inválido: $1
% You specified an unknown option.
option_help_pages_para=11007_H_"-?" exibe páginas de help
% When an unknown option is given, this message is diplayed.
option_too_many_cfg_files=11008_F_Muitos arquivos de configuração aninhados(+ de 16 níveis)
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_Não foi possível abrir arquivo $1
% The option file cannot be found.
option_reading_further_from=11010_D_Lendo opções adicionais de $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_Sistema operacional destino já está setado para: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_Libraries shared não suportados em DOS, revertendo para libraries estáticas
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos
option_too_many_ifdef=11013_F_IF(N)DEFs em excesso
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_ENDIFs em excesso
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_Comando de compilação condicional não fechado no final de arquivo
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_Este executável do compilador não suporta geração de informações de debug
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_Tente recompilar com -dGDB
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_W_Voce está usando uma diretiva obsoleta $1
% this warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the switch meaning may change.
option_obsolete_switch_use_new=11019_W_Voce está usando a diretiva obsoleta $1, por favor use $2
% this warns you when you use a switch that is not supported anymore. You
% must now use the second switch instead.
% It is recommended that you change the switch to overcome problems in the
% future, when the switch meaning may change.
option_switch_bin_to_src_assembler=11020_N_Mudando assembler para o modo default de fonte
% this notifies you that the assembler has been changed because you used the
% -a switch which can't be used with a binary assembler writer.
option_incompatible_asm=11021_W_Tipo de saída assembler selecionada "$1" não compatível com "$2"
option_asm_forced=11022_W_Forçado uso de assembler "$1"
% The assembler output selected can not generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
option_using_file=11026_T_Lendo opções a partir do arquivo $1
% Options are also read from this file
option_using_env=11027_T_Lendo opções da variável de ambiente $1
% Options are also read from this environment string
option_handling_option=11028_D_Tratando opção "$1"
% Debug info that an option is found and will be handled
option_help_press_enter=11029__*** pressione enter ***
option_start_reading_configfile=11030_H_Início do tratamento do arquivo de configuração $1
% Starting of config file parsing.
option_end_reading_configfile=11031_H_Fim do arquivo durante leitura do arquivo de configuração $1
% End of config file parsing.
option_interpreting_option=11032_D_Interpretando a opção "$1"
option_interpreting_firstpass_option=11036_D_Interpretando a opção de primeiro passo "$1"
option_interpreting_file_option=11033_D_Interpretando opção de arquivo "$1"
option_read_config_file=11034_D_Lendo o arquivo de configuração "$1"
option_found_file=11035_D_Encontrado o arquivo fonte "$1"
% Additional infos about options, displayed
% when you have debug option turned on.
option_code_page_not_available=11039_E_Página de código desconhecida
option_config_is_dir=11040_F_Arquivo de configuração $1 é um diretório
% Directories can not be used as configuration files.
%\end{description}
# EndOfTeX

#
# Logo (option -l)
#
option_logo=11023_[
Free Pascal Compiler versão $FPCFULLVERSION [$FPCDATE] para $FPCCPU
Copyright (c) 1993-2008 por Florian Klaempfl
]

#
# Info (option -i)
#
option_info=11024_[
Free Pascal Compiler versão $FPCVERSION

Data do compilador : $FPCDATE
Processador destino: $FPCCPU

Sistemas operacionais atendidos
  $OSTARGETS

Conjuntos de instruções atendidos:
  $INSTRUCTIONSETS

Conjuntos de instruções de ponto flutuante atendidos:
  $FPUINSTRUCTIONSETS

Este programa vem sob a licença GNU General Public Licence
Para mais informações leia COPYING.FPC

Informações de erros detectados, sugestões, etc para:
                 bugs@freepascal.org
]
# Help pages
option_help_pages=11025_[
**0*_coloque + após 1 opção booleana para habilita-la, - para desabilita-la
**1a_o compilador não deletou o arquivo assembly gerado
**2al_Lista as linhas de código fonte do arquivo assembly
**2an_lista as informações de node do arquivo assembly
*L2ap_use pipes(redirecione) ao invés de criar arquivos assembly temporários
**2ar_lista informações de alocação/liberação de registradores no arquivo asembly
**2at_lista informaçoes de alocação/liberação temporárias no arquivo assembly
**1A<x>_Formato de saída:
**2Adefault_use o assembly default
3*2Aas_Montagem feita usando o GNU AS
3*2Anasmcoff_coff (Go32v2) file gerado usando Nasm
3*2Anasmelf_elf32 (Linux) file gerado usando Nasm
3*2Anasmwin32_Win32 object file gerado usando Nasm
3*2Anasmwdosx_Win32/WDOSX object file gerado usando Nasm
3*2Awasm_obj file gerado usando Wasm (Watcom)
3*2Anasmobj_obj file gerado usando Nasm
3*2Amasm_obj file gerado usando Masm (Microsoft)
3*2Atasm_obj file gerado usando Tasm (Borland)
3*2Aelf_elf32 (Linux) gerado usando uma internal writer
3*2Acoff_coff (Go32v2) gerado usando uma internal writer
3*2Apecoff_pecoff (Win32) gerado usando uma internal writer
4*2Aas_Montagem feita usando o GNU AS
6*2Aas_Unix o-file gerado usando o GNU AS
6*2Agas_GNU Motorola assembler
6*2Amit_MIT Syntax (old GAS)
6*2Amot_Standard Motorola assembler
A*2Aas_Montagem feita usando o GNU AS
P*2Aas_Montagem feita usando o GNU AS
S*2Aas_Montagem feita usando o GNU AS
**1b_gera informações do browser
**2bl_gera a local symbol information
**1B_Compila/monta todos os módulos
**1C<x>_opções de geração do código:
**2Cc<x>_default calling convention setada para <x>
**2CD_criar também a dynamic library (não suportado)
**2Ce_Compilação com opcodes de ponto flutuante emulados
**2Cf<x>_Selecione o conjunto de instruções de ponto flutuante a ser usado, veja diretiva -i do FPC para valores possíveis
**2Cg_Gerar código com endereçamento relativo(endereçar usando só o deslocamento do operando)
**2Ch<n>_define heap com <n> bytes (entre 1023->1KB e 67107840->64 MB)
**2Ci_IO-checking(validação dos comandos de I/O)
**2Cn_Não executar estágio de link-edição
**2Co_Checar overflow de operações com inteiros
**2Cp<x>_seleciona o conjunto de instruções, veja diretiva FPC -i para valores possíveis
**2Cr_range checking(validação da faixa de validade de valores)
**2CR_verificar validade das chamadas de métodos de objetos
**2Cs<n>_seta o tamanho do stack para o valor <n>
**2Ct_stack checking(verifica estouros da capacidade do stack)
**2CX_criar também a smartlinked library
**1d<x>_define o símbolo <x>
**1D_gera um DEF file
**2Dd<x>_seta a descrição para <x>
**2Dv<x>_set a versão das DLL para <x>
*O2Dw_PM application(usa o Presentation manager do OS/2)
**1e<x>_seta o path para o executável
**1E_o mesmo que -Cn
**1F<x>_seta nomes de arquivos e paths:
**2Fa<x>[,y]_para o programa primeiro carregar as "units" <x> e [y] antes de interpretar o "uses"
**2Fc<x>_seta a página de código da entrada para <x>
**2FD<x>_seta o diretório onde procurar pelas utilities do compilador
**2Fe<x>_redireciona a saída de erros para <x>
**2FE<x>_seta a saída de exe/units para o path <x>
**2Fi<x>_adiciona <x> ao(s) path(s) de include
**2Fl<x>_adiciona <x> ao library path
**2FL<x>_usa <x> como dynamic linker
**2Fo<x>_adiciona <x> ao object path
**2Fr<x>_carrega o arquivo de mensagens de erro <x>
**2Fu<x>_adiciona <x> ao unit path
**2FU<x>_seta o unit output path para <x>, anula o informado na diretiva -FE
*g1g_gerar informações para o depurador(debugger)
*g2gc_gerar checagem de pointers
*g2gd_usar dbx
*g2gg_usar gsym
*g2gh_Usar "unit" de trace de heap (para depuração de descontinuidade de memória)
*g2gl_usar unit com informações de números de linhas para obter msgs de erro com seu número de linha
*g2gv_gerar programas traçáveis com valgrind
*g2gw_gerar informações de depuração para dwarf
**1i_informações
**2iD_retorna a data de compilação do compilador
**2iV_retorna a versão do compilador
**2iSO_retorna o sistema operacional onde está sendo feita a compilação
**2iSP_retorna o processador do compilador
**2iTO_retorna o sistema operacional para o qual será gerado o executável
**2iTP_retorna o processador para o qual será gerado o executável
**1I<x>_adiciona <x> ao(s) path(s) de include
**1k<x>_Passa <x> para o linkage editor
**1l_escreva as mensagens de identificação
**1M<x>_seta language mode para <x>(delphi, FPC, etc.)
**2Mfpc_dialeto free pascal (default)
**2Mobjfpc_ativa algumas extensões do Delphi 2
**2Mdelphi_Tenta manter compatibilidade com o Delphi
**2Mtp_Tenta manter a compatibilidade com TP/BP 7.0
**2Mgpc_Tenta manter compatibilidade com gpc
**2Mmacpas_Tenta manter compatibilidade com os dialetos do macintosh pascal
**1n_não ler o arquivo de configuração default
**2Nu_expanda o conteúdo dos loops
**1o<x>_Troca o nome do executável produzido para <x>
**1O<x>_otimizações:
3*2Og_gera codigo(executável) menor
3*2OG_gera codigo(executável) mais rápido(default)
**2Or_mantem certas variáveis em registradores
3*2Ou_ativa otimizações incertas (veja documentação)
3*2O1_otimizações de nível 1 (optimizações rápidas)
3*2O2_otimizações de nível 2(-O1 optimizações + lentas)
3*2O3_otimizações de nível 3(-O2 repetidamente, max 5 vezes)
3*2Op<x>_processador para o qual será gerado o executável(target)
3*3Op1_seta processador destino como 386/486
3*3Op2_seta processador destino como Pentium/PentiumMMX (tm)
3*3Op3_seta processador destino como PPro/PII/c6x86/K6 (tm)
6*2Og_gera código(executável) menor
6*2OG_gera código(executável) mais rápido(default)
6*2Ox_otimize ao máximo(ainda com problemas!!!)
6*2O0_seta como processador destino(gera executável para) o MC68000
6*2O2_seta como processador destino(gera executável para) o MC68020+ (default)
**1pg_gera perfil para gprof (cria o FPC_PROFILE)
**1R<x>_estilo de leitura do assembler:
**2Rdefault_use o assembler default
3*2Ratt_ler assembly no estilo AT&T
3*2Rintel_ler assembly no estilo Intel
6*2RMOT_ler assembly no estilo motorola
**1S<x>_opções de sintaxe:
**2S2_o mesmo que -Mobjfpc
**2Sc_suporta operadores como o C (*=,+=,/= and -=)
**2Sa_include assertion code.
**2Sd_o mesmo que -Mdelphi
**2Se<x>_opções de erro. <x> é uma combinação do seguinte:
**3*_<n> : compilador para após <n> erros (o default é 1)
**3*_w : compilador também para após warnings
**3*_n : compilador também para após notes
**3*_h : compilador também para após hints
**2Sg_permitir LABEL e GOTO
**2Sh_Use strings do tipo ansistring
**2Si_dar suporte a INLINE estilo C++
**2SI<x>_setar estilo de interface para <x>
**3SIcom_interface compatível com COM (default)
**3SIcorba_interface compatível com CORBA
**2Sm_suporta macros como o C (global)
**2So_o mesmo que -Mtp
**2Sp_o mesmo que -Mgpc
**2Ss_constructor tem que se chamar init (destructor tem que se chamar done)
**2St_permitir a keyword static em objetos
**1s_não chamar o assembler nem linker
**2sh_Gerar script para link edição no host
**2st_Gerar script para link edição no sistema operacional de destino
**2sr_Pular a fase de alocação de registradores (usar com -alr)
**1T<x>_Sistema operacional para o qual será gerado o executável(target):
3*2Temx_OS/2 via EMX (incluindo o extensor EMX/RSX)
3*2Tfreebsd_FreeBSD
3*2Tgo32v2_Versao 2 do DJ Delorie DOS extender
3*2Tlinux_Linux
3*2Tnetbsd_NetBSD
3*2Tnetware_Novell Netware Module (clib)
3*2Tnetwlibc_Novell Netware Module (libc)
3*2Topenbsd_OpenBSD
3*2Tos2_OS/2 / eComStation
3*2Tsunos_SunOS/Solaris
3*2Twatcom_Watcom compatible DOS extender
3*2Twdosx_WDOSX DOS extender
3*2Twin32_Windows 32 Bit
4*2Tlinux_Linux
6*2Tamiga_Commodore Amiga
6*2Tatari_Atari ST/STe/TT
6*2Tlinux_Linux/m68k
6*2Tmacos_Macintosh m68k (not supported)
6*2Tpalmos_PalmOS
A*2Tlinux_Linux
P*2Tdarwin_Darwin and MacOS X on PowerPC
P*2Tlinux_Linux on PowerPC
P*2Tmacos_MacOS (classic) on PowerPC
P*2Tmorphos_MorphOS
S*2Tlinux_Linux
**1u<x>_excluir definição do símbolo <x>
**1U_opções de "unit":
**2Un_não verifique o nome da "unit"
**2Ur_gerar arquivos de "unit" do tipo "release"
**2Us_compilar uma "unit" system
**1v<x>_Mostrar opções de compilação. <x> é uma combinação das seguintes letras:
**2*_e : Mostrar erros (default)     0 : Mostrar somente erros
**2*_w : Mostrar avisos              u : Mostrar informações da "unit"
**2*_n : Mostrar "notes"             t : Mostrar arquivos acessados/usados
**2*_h : Mostrar "hints"             c : Mostrar "conditionals"
**2*_i : Mostrar informações gerais  d : Mostrar informações de depuração(debug)
**2*_l : Mostrar números de linha    r : Modo de compatibilidade Rhide/GCCode
**2*_a : Mostrar tudo                x : Informações do executável (Win32 apenas)
**2*_v : grava fpcdebug.txt com      p : Grava tree.log com a árvore de parse
**2*_    muitas informações de depuração
3*1W<x>_opções do sistema operacional de destino no formato Win32
3*2WB_Criar uma imagem relocável
3*2WB<x>_Setar a base da imagem para o valor hexadecimal <x>
3*2WC_Especifica aplicação tipo console
3*2WD_Use DEFFILE para exportar functions de DLL ou EXE
3*2WF_Especifica aplicação tipo tela cheia (apenas OS/2)
3*2WG_Especifica aplicação gráfica
3*2WN_Não gerar código relocável (necessário para depuração)
3*2WR_Gerar código relocável
P*2WC_Especifica aplicação tipo console (apenas MacOS)
P*2WG_Especifica aplicação gráfica (apenas MacOS)
P*2WT_Especifica aplicação do tipo ferramenta (ferramenta MPW, apenas MacOS)
**1X_opções de executável:
**2Xc_passar --shared ao linkage editor (apenas Unix)
**2Xd_não usar caminho padrão de pesquisa em library (necessário para cross compile)
**2XD_tentar link editar "units" dinâmicas(definir FPC_LINK_DYNAMIC)
**2Xm_gerar mapa de link edição
**2XM<x>_seta o nome da rotina principal do programa (default é 'main')
**2XP<x>_prefixa os nomes dos binutils com o prefixo <x>
**2Xr<x>_setar o caminho de pesquisa de libraries para <x> (necessário para cross compile)
**2Xs_elimine todos os símbolos do executável
**2XS_tente link editar as units como static (default) (definido com FPC_LINK_STATIC)
**2Xt_link editar com static libraries (-static é passado ao linkage editor)
**2XX_tente link editar as unidades com smart linking(definido com FPC_LINK_SMART)
**1?_Mostra esta ajuda
**1h_mostra esta ajuda sem espera
]
# End of help
%%% scan_w_only_pack_records=02015_W_O alinhamento de campos de registro só pode ser em 1,2,4,8,16 or 32 bytes
%%% scan_w_only_pack_enum=02016_W_Tipo enumerado pode ser salvo em apenas 1, 2 ou 4 bytes
%%% parser_e_exit_with_argument_not_possible=03153_E_Não é possível usar "Exit" com argumento neste contexto
%%% parser_e_only_publishable_classes_can_be_published=03156_E_Só podem ter atributo "published" classes compiladas com $M+
%%% type_w_smaller_possible_range_check=04048_W_"Types" de tamanho diferente, haverá possível perda de dados e/ou "range check error"
%%% mr_e_local_para_unreachable=07015_E_Não pode acessar a variável $1 direto deste código
%%% asmr_e_size_suffix_and_dest_dont_match=07045_E_Sufixo de tamanho e o destino ou tamanho da origem não combinam
%%% option_no_debug_support=11016_W_Este executável do compilador não suporta geração de informações de debug
%%% scan_e_illegal_pack_records=02015_E_Illegal record alignment specifier "$1"
% You are specifying the \var{\{\$PACKRECORDS n\} } or \var{\{\$ALIGN n\} }
% with an illegal value for \var{n}. For \$PACKRECORDS valid alignments are 1, 2, 4, 8, 16, 32, C,
% NORMAL, DEFAULT, and for \$ALIGN valid alignment are 1, 2, 4, 8, 16, 32, ON,
% OFF. Under mode MacPas \$ALIGN also supports MAC68K, POWER and RESET.
%%% scan_e_illegal_pack_enum=02016_E_Illegal enum minimum-size specifier "$1"
% You are specifying the \var{\{\$PACKENUM n\}} with an illegal value for
% \var{n}. Only 1,2,4, NORMAL or DEFAULT are valid here.
%%% scan_e_compile_time_typeerror=02072_E_Compile time expression: Wanted $1 but got $2 at $3
% Type check of a compile time expression failed.
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
%%% cg_f_unknown_compilerproc=06046_F_Unknown compilerproc "$1". Check if you use the correct run time library.
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
%%% exec_d_resbin_params=09028_D_Calling resource compiler "$1" with "$2" as command line
%\end{description}
# EndOfTeX

#
# Executable information
#
