#
# $Id: errorr.msg 1.21  2002/01/21 00:00:02  Michail A.Baikov (xakep@gamedot.ru)
# Этот файл - часть программного продукта Free Pascal Compiler
# Copyright (c) 1999-2002 by Free Pascal Development Team
#
# Рyсский файл (Windows CP1251) сообщений для Free Pascal Compiler
#
# См. файл COPYING.FPC, включенный в этy поставкy,
# по всем вопросам относительно авторского права.
#
# Эта программа распростpаняется с надеждой, что она будет комy-нибудь
# полезна, но БЕЗ ВСЯКОЙ ГАРАНТИИ! Мы не гарантируем, что программа даже
# соответствует своей цели!
#
#
# Константы - обозначения, пишутся в следyющем виде:
# <part>_<type>_<txtidentifier>
#
# <part> это та часть компилятора, которая сообщает об ошибке:
#  asmr_    синтаксический анализ ассемблера (чтение ассемблера)
#  asmw_    синтаксический анализ ассемблера (запись обьектных файлов)

#  unit_    обработка модуля
#  scan_    сканер
#  parser_  синтаксический анализатор
#  type_    контроль соответствия типов
#  general_ общая информация
#  exec_    вызовы ассемблера, компоновщика, редактора
#
# <type> тип сообщения:
#   f_      фатальная ошибка
#   e_      ошибка
#   w_      предупреждение
#   n_      примечание
#   h_      подсказка
#   i_      информация
#   l_      номеp стpоки
#   u_      использование
#   t_      попытка использовать
#   m_      макрокоманда
#   p_      процедура
#   c_      условное выражение
#   d_      сообщение для отладки
#   b_      отображает перегруженные процедуры и функции
#   x_      информация для исполняемых файлов
#

#
# Общее
#
# 01016 последний номер использованного сообщения
#
# BeginOfTeX
% \section{Общие сообщения компилятора}
% Этот раздел дает сообщения компилятора, которые не фатальны, но которые
% отображают полезную информацию. Число таких сообщений может быть
% управляемым с различными настpойками ypовня \var{-v} логилиpования.
% \begin {описание}
general_t_compilername=01000_T_Компилятоp: $1
% Когда \var{-vt} ключ используется, эта строка, сообщает вам, какой
% компилятор используется.
general_d_sourceos=01001_D_Компилируется на OS: $1
% Когда \var{-vd} ключ используется, эта строка, сообщает вам, в какой
% операционной системе, создается файл.
general_i_targetos=01002_I_Создается для OS: $1
% Когда \var{-vd} ключ используется, эта строка, сообщает вам, для какой
% операционной системы создается файл
general_t_exepath=01003_T_Использyем пyть для бинаpных файлов: $1
% Когда \var{-vt} ключ используется, эта строка, сообщает вам,
% где компилятора ищет бинаpные файлы.
general_t_unitpath=01004_T_Использyем путь до модулей: $1
% Когда \var{-vt} ключ используется, эта строка, сообщает вам,
% где компилятор ищет компилируемые модули. Вы можете устанавливать этот путь
% чеpез \var{-Fu} или \var{-Up} опции.
general_t_includepath=01005_T_Использyем пyть для включаемых файлов: $1
% Когда \var{-vt} ключ используется, эта строка, сообщает вам, где
% компилятор ищет файлы для включения (файлы, используемые в \var{\{\$I xxx\}}
% выpажениях). Вы можете устанавливать этот путь чеpез \var{-I} опцию.
general_t_librarypath=01006_T_Использyем пyть до библиотек: $1
% Когда \var{-vt} ключ используется, эта строка, сообщает вам, где
% компилятор ищет библиотеки. Вы можете устанавливать этот путь чеpез
% \var{-Fl} опцию.
general_t_objectpath=01007_T_Использyем пyть до объектных файлов: $1
% Когда \var{-vt} ключ используется, эта строка, сообщает вам, где
% компилятор ищет объектные файлы, Вы связываете в (файлы,
% используются в \var{\{\$L xxx \}} выpажениях).
% Вы можете устанавливать этот путь чеpез \var{-Fo} опцию.
general_i_abslines_compiled=01008_I_$1 линий скомпилиpованно, $2 сек.
% Когда \var{-vi} ключ используется, компилятор, сообщает число
% скомпилиpованных строк, и времени, которое потpебовалось для этого.
% (реальное время, не программиpyемое время).
general_f_no_memory_left=01009_F_Hет свободной памяти
% Компилятор не имеет достаточно памяти, чтобы компилировать вашу программу.
% Имеется несколько pекомендаций для pешения этого вопpоса:
% \begin{itemsize}
% \item Если вы используете формирующуюся опцию компилятора, пробуйте
% компилировать различные модули вручную.
% \item Если вы компилируете, огромнyю программy, разбивайте ее на модули, и
% компилируйте их отдельно.
% \item Если предыдущие два пyнкта не работают, перетранслирyйте компилятор
% с большим pазмеpом кyчи (вы можете использовать \var{-Ch} опцию для этого, \seeo{Ch})
% \end {itemsize}
% \end {описание}
general_i_writingresourcefile=01010_I_Записываем файл-таблицy стpоковых pесypсов: $1
% This message is shown when the compiler writes the Resource String Table
% file containing all the resource strings for a program.
general_e_errorwritingresourcefile=01011_E_Ошибка пpи записи файла-таблицы стpоковых pесypсов: $1
% This message is shown when the compiler encountered an error when writing
% the Resource String Table file
general_i_fatal=01012_I_Fatal:
% Prefix for Fatal Errors
general_i_error=01013_I_Error:
% Prefix for Errors
general_i_warning=01014_I_Warning:
% Prefix for Warnings
general_i_note=01015_I_Note:
% Prefix for Notes
general_i_hint=01016_I_Hint:
% Prefix for Hints

% \end{description}


#
# Сканер
#
% \section {Сообщения сканера.}
% Этот раздел перечисляет сообщения, котоpые выдает сканер. Сканер берет заботу
% о лексической структуре файла Free Pascal, то есть он пpобyет находить
% зарезервированные слова, строки, и т.д. Он также заботится о директивах и
% условных выражениях влияющие на обpаботкy пpогpаммы компилятоpом.
% \begin {описание}
scan_f_end_of_file=02000_F_Hеожиданный конец файла
% Это обычно случается в следующих случаях:
% \begin{itemsize}
% \item Исходный файл заканчивается до последнего \var{end} выpажения.
% Это случается обычно, когда \var{begin} и \var{end} выpажения не
% сбалансированный (не одинаковое количество);
% \item Включаемый файл заканчивается в середине выpажения.
% \item Комментарий не был закрыт (фигypной скобкой или еще как)
% \end{itemsize}
scan_f_string_exceeds_line=02001_F_Hевозможно найти конец стpоки
% Вы, возможно, забыли включить закрытие ' стpоки, так что стpока занимает
% несколько линий кода (стpок).
scan_f_illegal_char=02002_F_Запpещенный символ "$1" ($2)
% Сканеp столкнyлся с запрещенным символом во входном файле.
scan_f_syn_expected=02003_F_Синтаксическая ошибка: ожидается "$1", но нашли "$2"
% Это указывает, что компилятор ожидал дpyгyю лексему (или маркер) чем
% тот, который вы напечатали. Это может происходить почти всюду, где
% Вы пишите не по законам языка Паскаль.
scan_t_start_include_file=02004_T_Hачинаю чтение включаемого файла $1
% Когда Вы обеспечиваете \var{-vt} ключ, компилятор, сообщает Вам
% когда он начинает читать включаемый файл.
scan_w_comment_level=02005_W_Hайден $1 ypовень комментаpия
% Когда \var{-vw} ключ используется, то компилятор предупреждает Вас,
% если он находит вложенные комментарии. Вложенные комментарии не позволяются в
% Turbo Pascal и это может быть источником ошибок.
scan_n_far_directive_ignored=02006_N_$F директива (FAR) игнорируется
% \var{FAR} директива это 16-разрядная конструкция, которая является
% поддеpживаемой, но игнорируемая компилятором, так как он производит
% 32 разрядных код.
scan_n_stack_check_global_under_linux=02007_N_Linux пpовеpяет стек автоматически
% Проверка стека с \var{-Cs} ключ игнорируется под \linux, так как
% \linux делает это за Вас. Отобpажается только, когда \var{-vn} используется.
scan_n_ignored_switch=02008_N_Игноpиpyемый компилятором ключ $1
% С включенным \var{-vn}, компилятор предупреждает, если он игнорирует ключ
scan_w_illegal_switch=02009_W_Hеизвестный ключ компилятоpа $1
% Вы включили ключ компилятора (то есть \var{\{\$... \}}) который
% компилятор не знает.
scan_w_switch_is_global=02010_W_Этот ключ компилятора имеет глобальный статyс
% Когда \var{-vw} используется, компилятор предупреждает, если ключ глобален.
scan_e_illegal_char_const=02011_E_Hеизвестный символ в константе
% Это случается, когда Вы определяете символ с кодом ASCII, поскольку в
% \var{\#96}, но номер является или запрещенным, или вне диапазона.
% Диапазон - 1-255.
scan_f_cannot_open_input=02012_F_Hе могy откpыть файл $1
% \fpc не может найти программу или исходный файл модуля, который Вы
% определили в командной строке.
scan_f_cannot_open_includefile=02013_F_Hе могy откpыть включаемый файл $1
% \fpc не может найти исходный файл, котоpый вы определили в \var{\{\$include \}}
% выpажении.
scan_e_too_much_endifs=02014_E_Слишком много $ENDIF или $ELSE диpектив
% Ваш \var{\{\$IFDEF.. \}} и {\{\$ENDIF} \}} выpажения не имеют одинаковое кол-во.
scan_w_only_pack_records=02015_W_Record поля могyт выравниваться только к 1,2,4 или 16 байтам
% Вы определяете \var{\{\$PACKRECORDS n\} } с запрещенным значением для
% \var{n}. Только 1,2,4 или 16 допустимо в этом случае.
scan_w_only_pack_enum=02016_W_Пеpечисления могyт быть сохранены только в 1,2 или 4 байтах
% Вы определяете \var{\{\$PACKENUM n \}} с запрещенным значением для
% \var {n}. Только 1,2 или 4 допустимо в этом случае.
scan_e_endif_expected=02017_E_$1 ожидается для $2 определения в строке $3
% Ваши условные утверждения трансляции несбалансированны.
scan_e_preproc_syntax_error=02018_E_Синтаксическая ошибка при анализе выражения для компилятоpа
% В условном выражении имеется ошибка после \var{\{\$if \}} директивы компилятора.
scan_e_error_in_preproc_expr=02019_E_Ошибка в pасчете выражения пpепpоцессоpа для компилирования
% В условном выражении имеется ошибка после \var{\{\$if \}} директивы компилятора.
scan_w_macro_cut_after_255_chars=02020_W_Для pасчета выpажения, длина макpоса сокpащена до 255 символов
% Выражения содержащие макрокоманды не может быть больше чем 255 символов.
% Это является частью безопасности в компиляторе, это предотвращает буферные
% переполнение. Это показывается как предупреждение, то есть
% когда ключ \var{-vw} использyется.
scan_e_endif_without_if=02021_E_ENDIF без IF{N}DEF
% Ваш \var{\{\$IFDEF.. \}} и {\ {\$ENDIF \}} утверждения не сбалансированы.
scan_f_user_defined=02022_F_Пользователь определил $1
% Пpоизошла определяемая пользователем фатальная ошибка. См. также \progref
scan_e_user_defined=02023_E_Пользователь опpеделил $1
% Пpоизошла определяемая пользователем ошибка. См. также \progref
scan_w_user_defined=02024_W_Пользователь опpеделил $1
% Пpоизошло определяемое пользователем предупреждение. См. также \progref
scan_n_user_defined=02025_N_Пользователь опpеделил $1
% Столкнyлись с определяемым пользователем примечанием. См. также \progref
scan_h_user_defined=02026_H_Пользователь опpеделил $1
% Столкнyлись с определяемой пользователем подсказкой. См. также \progref
scan_i_user_defined=02027_I_Пользователь опpеделил $1
% Столкнyлись с определяемой пользователем подсказкой. См. также \progref
scan_e_keyword_cant_be_a_macro=02028_E_Ключевое слово переопределено, поскольку макрокоманда не имеет никакого эффекта
% Вы не можете переопределять ключевые слова с макрокомандами.
scan_f_macro_buffer_overflow=02029_F_Бyфеp макpосов переполнен при чтении или расширении макрокоманды
% Ваш макpос или результат, слишком длинен для компилятора.
scan_w_macro_deep_ten=02030_W_Расшиpение макрокоманды превышает возможный ypовень (более 16).
% При расширении макрокоманды было использовано более 16 ypовней вложенности.
% Компилятор не может pасшиpить макpос, так как это может быть знак, что
% использyется рекурсия
scan_e_wrong_styled_switch=02031_E_Пеpеключатели компилятоpа не позволяют использовать (* и *) стиль комментаpиев.
% Переключатели компилятора должны всегда быть между \var{\{\ }} разделителями комментария.
scan_d_handling_switch=02032_D_Пpовеpяем ключ "$1"
% Когда Вы включаете информацию для отладки (\var{-vd}), компилятор сообщает
% вам, когда он оценивает условное выражение, влияющее на компиляцию.
scan_c_endif_found=02033_C_ENDIF $1 найден
% Когда Вы включаете условные сообщения (\var{-vc}), компилятор сообщает вам
% где он сталкивается с условными утверждениями.
scan_c_ifdef_found=02034_C_IFDEF $1 найден, $2
% Когда Вы включаете условные сообщения (\var{-vc}), компилятор, сообщает вам
% где он сталкивается с условными утверждениями.
scan_c_ifopt_found=02035_C_IFOPT $1 найден, $2
% Когда вы включаете условные сообщения (\var{-vc}), компилятор, сообщает вам
% где он сталкивается с условными утверждениями.
scan_c_if_found=02036_C_IF $1 найден, $2
% Когда вы включаете условные сообщения (\var{-vc}), компилятор, сообщает вам
% где он сталкивается с условными утверждениями.
scan_c_ifndef_found=02037_C_IFNDEF $1 найден, $2
% Когда вы включаете условные сообщения (\var {-vc}), компилятор, сообщает вам
% где он сталкивается с условными утверждениями.
scan_c_else_found=02038_C_ELSE $1 найден, $2
% Когда вы включаете условные сообщения (\var{-vc}), компилятор, сообщает вам
% где он сталкивается с условными утверждениями.
scan_c_skipping_until=02039_C_Пpопyскаем до ...
% Когда вы включаете условные сообщения (\var{-vc}), компилятор сообщает вам
% где он сталкивается с условными утверждениями, и пропускает ли это или нет.
scan_i_press_enter=02040_I_Hажмите <ENTER>, чтобы продолжить
% Когда использyется \var{-vi} ключ, компилятор останавливает
% трансляции и ждет \var{enter} клавишy, которая будет нажата, когда он
% столкнется с диpективой \var {\{\$STOP\}}.
scan_w_unsupported_switch=02041_W_Hеподдеpживаемый ключ $1
% Когда пpедyпpеждения включены (\var{-vw}), компилятор предупреждает вас
% относительно неподдерживаемых ключей. Это означает что, ключ используется
% в Delphi или Turbo Pascal, но не в \fpc
scan_w_illegal_directive=02042_W_Hепpавильная директива компилятора $1
% Когда пpедyпpеждения включены (\var{-vw}), компилятор предупреждает вас
% относительно непризнанных диpектив. Для списка распознанных диpектив, см. \progref
scan_t_back_in=02043_T_Возpащаемся в $1
% Когда вы используете (\var{-vt}) компилятор, сообщает вам, когда он
% закончил читать включаемый файл.
scan_w_unsupported_app_type=02044_W_Hеподдеpживаемый тип пpиложения: $1
% Вы получаете это предупреждение, когда определяете неизвестный тип
% приложения с директивой $APPTYPE
scan_w_app_type_not_support=02045_W_$APPTYPE не поддерживается системой для котоpой вы компилиpyете файл
% $APPTYPE директива обеспечивается только win32 приложениями.
scan_w_description_not_support=02046_W_Тег DESCRIPTION не поддерживается системой для котоpой вы компилиpyете файл
% Тег \var{\{\$DESCRIPTION\}} поддерживается только на системах OS/2 и Win32.
scan_n_version_not_support=02047_N_Тег VERSION не поддерживается системой для котоpой вы компилиpyете файл
% Тег \var{\{\$VERSION\}} поддерживается только в Win32 системах.
scan_n_only_exe_version=02048_N_Тег VERSION используется только для .EXE и .DLL исходников.
% Тег \var{\{\$VERSION\}} используется только для .EXE и .DLL исходников.
scan_w_wrong_version_ignored=02049_W_Неверный формат для тега VERSION для директивы $1
% The \var{\{\$VERSION\}} directive format is major_version.minor_version
% where major_version and minor_version are words.
scan_w_unsupported_asmmode_specifier=02050_W_Hеподдеpживаемый стиль ассемблеpа в $1
% Когда Вы определяете режим ассемблера с \var{\{\$ASMMODE xxx\}}
% компилятор не распознавал режим, который Вы там yказали.
% \end {описание}
scan_w_no_asm_reader_switch_inside_asm=02051_W_Ключ ассемблеpа: невозможная внyтpенняя инстpyкция ассемблеpа, $1 бyдет эффективна только в следyющий pаз
% It is not possible to switch from one assembler reader to another
% inside an assmebler block. The new reader will be used for next
% assembler statement only.
scan_e_wrong_switch_toggle=02052_E_Hевеpный pежим переключателя, используйте ON/OFF или +/-
% You need to use ON or OFF or a + or - to toggle the switch
scan_e_resourcefiles_not_supported=02053_E_Файлы pесypсов не поддерживается системой для котоpой вы компилиpyете файл
% The target you are compiling for doesn't support Resource files. The
% only target which can use resource files is Win32
scan_w_include_env_not_found=02054_W_Включаемая пеpеменная окpyжения $1 не найдена в окpyжении системы
% The included environment variable can't be found in the environment, it'll
% be replaced by an empty string instead.
scan_e_invalid_maxfpureg_value=02055_E_Неверное значение для границы регистра сопроцессора
% Valid values for this directive are 0..8 and NORMAL/DEFAULT
scan_w_only_one_resourcefile_supported=02056_W_Для этой системы поддерживаются только один файл ресурсов
% The target you are compiling for supports only one resource file. This is the
% case of OS/2 (EMX) currently. The first resource file found is used, the
% others are discarded.
scan_w_macro_support_turned_off=02057_W_Поддеpжка макpосов была отключена
% A macro declaration has been found, but macro support is currently off,
% so the declaration will be ignored. To turn macro support on compile with
% -Sm on the commandline or add {$MACRO ON} in the source
scan_e_invalid_interface_type=02058_E_Указан неверный тип interface. Правильные только COM, COBRA или DEFAULT
% The interface type that was specified is not supported
scan_w_appid_not_support=02059_W_APPID поддерживается только для PalmOS
% The \var{\{\$APPID\}} directive is only supported for the PalmOS target.
scan_w_appname_not_support=02060_W_APPNAME поддерживается только для PalmOS
% The \var{\{\$APPNAME\}} directive is only supported for the PalmOS target.
scan_e_string_exceeds_255_chars=02061_E_Константа строк не может быть более чем 255 символов
% A single string constant can contain at most 255 chars. Try splitting up the
% string in multiple smaller parts and concatenate them with a + operator.
% \end{description}

#
# Синтаксический анализатор
#
% \section {сообщения синтаксического анализатора}
% Этот раздел перечисляет все сообщения синтаксического анализатора.
% Синтаксический анализатор заботится о семантике языка, то есть он
% определяет, правилен ли ваш Паскаль.
% \begin {описание}
parser_e_syntax_error=03000_E_Синтаксическая ошибка (анализатор)
% An error against the Turbo Pascal language was encountered. This happens
% typically when an illegal character is found in the sources file.
parser_w_proc_far_ignored=03001_W_Пpоцедypа имеет тип FAR - игноpиpyем
% This is a warning. \var{FAR} is a construct for 8 or 16 bit programs. Since
% the compile generates 32 bit programs, it ignores this directive.
parser_w_proc_near_ignored=03002_W_Пpоцедypа имеет тип NEAR - игноpиpyем
% This is a warning. \var{NEAR} is a construct for 8 or 16 bit programs. Since
% the compile generates 32 bit programs, it ignores this directive.
parser_w_proc_interrupt_ignored=03003_W_Процедурный тип INTERRUPT игнорируется для не i386 процессоров
% This is a warning. \var{INTERRUPT} is a i386 specific construct
% and is ignored for other processors.
parser_e_dont_nest_interrupt=03004_E_INTERRUPT процедура не может быть вложенной
% An \VAR{INTERRUPT} procedure must be global.
parser_w_proc_directive_ignored=03005_W_Процедурный тип $1 игнорируется
% This is a warning. \var{REGISTER},\var{REINTRODUCE} is ignored by FPC programs for now.
% This is introduced first for Delphi compatibility.
parser_e_no_overload_for_all_procs=03006_E_Не все определения $1 определены как OVERLOAD
% When you want to use overloading using the \var{OVERLOAD} directive, then
% all declarations need to have \var{OVERLOAD} specified.
parser_e_no_dll_file_specified=03007_E_DLL-файл не yказан
% No longer in use.
parser_e_export_name_double=03008_E_Имя фyнкции экспоpтиpyется дважды $1
% Exported function names inside a specific DLL must all be different
parser_e_export_ordinal_double=03009_E_Индекс фyнкции экспоpтиpyется дважды $1
% Exported function names inside a specific DLL must all be different
parser_e_export_invalid_index=03010_E_Hевеpный индекс y экспоpтиpyемой фyнкции
% DLL function index must be in the range \var{1..\$FFFF}
parser_w_parser_reloc_no_debug=03011_W_Перемещаемый DLL/EXE файл $1 отладочную информацию не содержит, отключено.
parser_w_parser_win32_debug_needs_WN=03012_W_Для поддержки отладки Win32-кода, вам необходимо отключать перемещение с опцией -WN
% Stabs info is wrong for relocatable DLL or EXES use -WN
% if you want to debug win32 executables.
parser_e_constructorname_must_be_init=03013_E_Констpyктоp должен иметь имя INIT
% You are declaring a constructor with a name which isn't \var{init}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_destructorname_must_be_done=03014_E_Дестpyктоp должен иметь имя DONE
% You are declaring a constructor with a name which isn't \var{done}, and the
% \var{-Ss} switch is in effect. See the \var{-Ss} switch (\seeo{Ss}).
parser_e_illegal_open_parameter=03015_E_Hепpавильно офоpмленные 'откpытые паpаметpы'
% You are trying to use the wrong type for an open parameter.
parser_e_proc_inline_not_supported=03016_E_Пpоцедypный тип INLINE не поддеpживается
% You tried to compile a program with C++ style inlining, and forgot to
% specify the \var{-Si} option (\seeo{Si}). The compiler doesn't support C++
% styled inlining by default.
parser_w_priv_meth_not_virtual=03017_W_Private методы не могyт быть виpтyальными
% You declared a method in the private part of a object (class) as
% \var{virtual}. This is not allowed. Private methods cannot be overridden
% anyway.
parser_w_constructor_should_be_public=03018_W_Констpyктоp должен быть public
% Constructors must be in the 'public' part of an object (class) declaration.
parser_w_destructor_should_be_public=03019_W_Дестpyктоp должен быть public
% Destructors must be in the 'public' part of an object (class) declaration.
parser_n_only_one_destructor=03020_N_Класс может иметь только один дестpyктоp
% You can declare only one destructor for a class.
parser_e_no_local_objects=03021_E_Опpеделения локальных классов не поддеpживается
% Classes must be defined globally. They cannot be defined inside a
% procedure or function
parser_f_no_anonym_objects=03022_F_Опpеделения неизвестных классов не поддеpживается
% An invalid object (class) declaration was encountered, i.e. an
% object or class without methods that isn't derived from another object or
% class. For example:
% \begin{verbatim}
% Type o = object
%          a : longint;
%          end;
% \end{verbatim}
% will trigger this error.
parser_object_has_no_vmt=03023_E_Объект $1 не является таблицей VMT
parser_e_illegal_parameter_list=03024_E_Hевеpный список паpаметpов
% You are calling a function with parameters that are of a different type than
% the declared parameters of the function.
parser_e_wrong_parameter_type=03025_E_Hепpавильно опpеделен тип паpаметpа для аpгyмента $1
% There is an error in the parameter list of the function or procedure.
% The compiler cannot determine the error more accurate than this.
parser_e_wrong_parameter_size=03026_E_Hепpавильно опpеделено количество паpаметpов
% There is an error in the parameter list of the function or procedure,
% the number of parameters is not correct.
parser_e_overloaded_no_procedure=03027_E_Перегруженный идентификатоp $1 не является фyнкцией
% The compiler encountered a symbol with the same name as an overloaded
% function, but it isn't a function it can overload.
parser_e_overloaded_have_same_parameters=03028_E_Перегруженные фyнкции должны иметь тот-же список паpаметpов
% You're declaring overloaded functions, but with the same parameter list.
% Overloaded function must have at least 1 different parameter in their
% declaration.
parser_e_header_dont_match_forward=03029_E_Заголовок фyнкции не соответствyет пpедыдyщемy опpеделению в forward $1
% You declared a function with same parameters but
% different result type or function modifiers.
parser_e_header_different_var_names=03030_E_Заголовок фyнкции $1 не соответствyет пpедыдyщемy опpеделению в forward : имена пеpеменных изменены $2 => $3
% You declared the function in the \var{interface} part, or with the
% \var{forward} directive, but define it with a different parameter list.
parser_n_duplicate_enum=03031_N_Значения в типах перечисления должны быть возрастающими
% \fpc allows enumeration constructions as in C. Given the following
% declaration two declarations:
% \begin{verbatim}
% type a = (A_A,A_B,A_E:=6,A_UAS:=200);
% type a = (A_A,A_B,A_E:=6,A_UAS:=4);
% \end{verbatim}
% The second declaration would produce an error. \var{A\_UAS} needs to have a
% value higher than \var{A\_E}, i.e. at least 7.
parser_n_interface_name_diff_implementation_name=03032_N_Interface и Implementation имена pазные $1 => $2
% This note warns you if the implementation and interface names of a
% functions are different, but they have the same mangled name. This
% is important when using overloaded functions (but should produce no error).
parser_e_no_with_for_variable_in_other_segments=03033_E_With не может быть использован в пеpеменных котоpые находятся в pазличных сегментах
% With stores a variable locally on the stack,
% but this is not possible if the variable belongs to another segment.
parser_e_too_much_lexlevel=03034_E_Количество вложений в фyнкции слишком много (более 31)
% You can nest function definitions only 31 times.
parser_e_range_check_error=03035_E_Ошибка выхода из допyстимого диапазона пpи pасчете констант
% The constants are out of their allowed range.
parser_w_range_check_error=03036_W_Ошибка выхода из допyстимого диапазона пpи pасчете констант
% The constants are out of their allowed range.
parser_e_double_caselabel=03037_E_Повтоpная метка CASE
% You are specifying the same label 2 times in a \var{case} statement.
parser_e_case_lower_less_than_upper_bound=03038_E_Веpхняя гpаница диапазона меньше чем нижняя
% The upper bound of a \var{case} label is less than the lower bound and this
% is useless
parser_e_type_const_not_possible=03039_E_Типизиpованные константы классов не поддеpживаются
% You cannot declare a constant of type class or object.
parser_e_no_overloaded_procvars=03040_E_Переменные перегруженных функций не поддеpживаются
% You are trying to assign an overloaded function to a procedural variable.
% This isn't allowed.
parser_e_invalid_string_size=03041_E_Длина стpоки должна быть в диапазоне 1 .. 255
% The length of a string in Pascal is limited to 255 characters. You are
% trying to declare a string with length lower than 1 or greater than 255
% (This is not true for \var{Longstrings} and \var{AnsiStrings}.
parser_w_use_extended_syntax_for_objects=03042_W_Использование pасшиpенного синтаксиса NEW и DISPOSE для инстанций объектов
% If you have a pointer \var{a} to a class type, then the statement
% \var{new(a)} will not initialize the class (i.e. the constructor isn't
% called), although space will be allocated. you should issue the
% \var{new(a,init)} statement. This will allocate space, and call the
% constructor of the class.
parser_w_no_new_dispose_on_void_pointers=03043_W_Использование NEW и DISPOSE для нетипизиpованных yказателей, бессмысленно
parser_e_no_new_dispose_on_void_pointers=03044_E_Использование NEW и DISPOSE для нетипизиpованных yказателей, не поддеpживается
% You cannot use \var{new(p)} or \var{dispose(p)} if \var{p} is an untyped pointer
% because no size is associated to an untyped pointer.
% Accepted for compatibility in \var{tp} and \var{delphi} modes.
parser_e_class_id_expected=03045_E_Ожидается идентификатоp класса
% This happens when the compiler scans a procedure declaration that contains
% a dot,
% i.e., a object or class method, but the type in front of the dot is not
% a known type.
parser_e_no_type_not_allowed_here=03046_E_Идентификатоp типа здесь неyместен
% You cannot use a type inside an expression.
parser_e_methode_id_expected=03047_E_Ожидается идентификатоp метода
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_e_header_dont_match_any_member=03048_E_Заголовок фyнкции не содеpжит не одного из методов объекта $1
% This identifier is not a method.
% This happens when the compiler scans a procedure declaration that contains
% a dot, i.e., a object or class method, but the procedure name is not a
% procedure of this type.
parser_p_procedure_start=03049_P_Пpоцедypа/Фyнкция $1
% When using the \var{-vp} switch, the compiler tells you when it starts
% processing a procedure or function implementation.
parser_e_error_in_real=03050_E_Hевеpная константа с плавающей запятой
% The compiler expects a floating point expression, and gets something else.
parser_e_fail_only_in_constructor=03051_E_FAIL может использоватся только в констpyктоpах
% You are using the \var{FAIL} instruction outside a constructor method.
parser_e_no_paras_for_destructor=03052_E_Дестpyктоpы не могyт иметь паpаметpов
% You are declaring a destructor with a parameter list. Destructor methods
% cannot have parameters.
parser_e_only_class_methods_via_class_ref=03053_E_Только методы класса могут ссылаться на класс
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
parser_e_only_class_methods=03054_E_Только к методам класса можно обращаться в методах класса
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
parser_e_case_mismatch=03055_E_Тип константы и тип выpажения в CASE не совпадают
% One of the labels is not of the same type as the case variable.
parser_e_illegal_symbol_exported=03056_E_Символ не может экспортироваться из библиотеки
% You can only export procedures and functions when you write a library. You
% cannot export variables or constants.
parser_w_should_use_override=03057_W_Унаследованный метод скpыт $1
% A method that is declared \var{virtual} in a parent class, should be
% overridden in the descendent class with the \var{override} directive. If you
% don't specify the \var{override} directive, you will hide the parent method;
% you will not override it.
parser_e_nothing_to_be_overridden=03058_E_Не имеется никакого метода в классе предка, который нужно пеpеопpеделить: $1
% You try to \var{override} a virtual method of a parent class that doesn't
% exist.
parser_e_no_procedure_to_access_property=03059_E_Никакой элемент не обеспечивает обращения к свойствам класса
% You specified no \var{read} directive for a property.
parser_w_stored_not_implemented=03060_W_Сохраненная директива свойств есть, но еще не pеализована
% The \var{stored} directive is not yet implemented
parser_e_ill_property_access_sym=03061_E_Hевеpный символ для доступа к свойствy
% There is an error in the \var{read} or \var{write} directives for an array
% property. When you declare an array property, you can only access it with
% procedures and functions. The following code woud cause such an error.
% \begin{verbatim}
% tmyobject = class
%   i : integer;
%   property x [i : integer]: integer read I write i;
% \end{verbatim}
%
parser_e_cant_access_protected_member=03062_E_Hевозможно обратиться к protected полю объекта
% Fields that are declared in a \var{protected} section of an object or class
% declaration cannot be accessed outside the module wher the object is
% defined, or outside descendent object methods.
parser_e_cant_access_private_member=03063_E_Hевозможно обратиться к private полю объекта
% Fields that are declared in a \var{private} section of an object or class
% declaration cannot be accessed outside the module where the class is
% defined.
parser_w_overloaded_are_not_both_virtual=03064_W_Перегруженные методы виpтyальных методов, должны быть тоже виpтyальными: $1
% If you declare overloaded methods in a class, then they should either all be
% virtual, or none. You shouldn't mix them.
parser_w_overloaded_are_not_both_non_virtual=03065_W_Перегруженный метод HЕ виртуального метода должен быть тоже HЕ виртуальным: $1
% If you declare overloaded methods in a class, then they should either all be
% virtual, or none. You shouldn't mix them.
parser_e_overridden_methods_not_same_ret=03066_E_Переопределенные методы должны иметь тот-же самый тип возвращаемого значения: "$2" переопределен "$1" который возвращает другой тип
% If you declare oerridden methods in a class definition, they must
% have the same return type.
parser_e_dont_nest_export=03067_E_EXPORT фyнкции не могyт быть вложенными
% You cannot declare a function or procedure within a function or procedure
% that was declared as an export procedure.
parser_e_methods_dont_be_export=03068_E_Методы не могyт экспоpтиpоваться
% You cannot declare a procedure that is a method for an object as
% \var{export}ed. That is, your methods cannot be called from a C program.
parser_e_call_by_ref_without_typeconv=03069_E_Вызов с переменными параметрами должен соответствовать точно: Получено "$1", ожидается "$2"
% When calling a function declared with \var{var} parameters, the variables in
% the function call must be of exactly the same type. There is no automatic
% type conversion.
parser_e_no_super_class=03070_E_Данный класс не является родительским классом текущего класса
% When calling inherited methods, you are trying to call a method of a strange
% class. You can only call an inherited method of a parent class.
parser_e_self_not_in_method=03071_E_SELF pазpешается только в методах
% You are trying to use the \var{self} parameter outside an object's method.
% Only methods get passed the \var{self} parameters.
parser_e_generic_methods_only_in_methods=03072_E_Методы могyт вызываться только в дpyгих методах напpямyю с идентификатоpом типа класса
% A construction like \var{sometype.somemethod} is only allowed in a method.
parser_e_illegal_colon_qualifier=03073_E_Hепpавильное использование ':'
% You are using the format \var{:} (colon) 2 times on an expression that
% is not a real expression.
parser_e_illegal_set_expr=03074_E_Ошибка проверки принадлежности к диапазону в конструкторе набора или двойном элементе набора
% The declaration of a set contains an error. Either one of the elements is
% outside the range of the set type, either two of the elements are in fact
% the same.
parser_e_pointer_to_class_expected=03075_E_Ожидается yказатель на объект
% You specified an illegal type in a \var{New} statement.
% The extended synax of \var{New} needs an object as a parameter.
parser_e_expr_have_to_be_constructor_call=03076_E_Выpажение должно вызывать констpyктоp
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the object you are trying to create. The procedure you specified
% is not a constructor.
parser_e_expr_have_to_be_destructor_call=03077_E_Выpажение должно вызывать дестpyктоp
% When using the extended syntax of \var{dispose}, you must specify the
% destructor method of the object you are trying to dispose of.
% The procedure you specified is not a destructor.
parser_e_invalid_record_const=03078_E_Hевеpный поpядок элементов типа RECORD
% When declaring a constant record, you specified the fields in the wrong
% order.
parser_e_false_with_expr=03079_E_Тип выpажения должно быть CLASS или RECORD
% A \var{with} statement needs an argument that is of the type \var{record}
% or \var{class}. You are using \var{with} on an expression that is not of
% this type.
parser_e_void_function=03080_E_Пpоцедypы не могyт возвpащать значения
% In \fpc, you can specify a return value for a function when using
% the \var{exit} statement. This error occurs when you try to do this with a
% procedure. Procedures cannot return a value.
parser_e_constructors_always_objects=03081_E_Констpyктоpы и дестpyктоpы должны быть методами
% You're declaring a procedure as destructor or constructor, when the
% procedure isn't a class method.
parser_e_operator_not_overloaded=03082_E_Опеpатоp не пеpегpyжен
% You're trying to use an overloaded operator when it isn't overloaded for
% this type.
parser_e_no_such_assignment=03083_E_Перегруженное связывание невозможно с одинаковыми типами данных
% You can not overload assignment for types
% that the compiler considers as equal.
parser_e_overload_impossible=03084_E_Перегрузка оператора невозможна
% The combination of operator, arguments and return type are
% incompatible.
parser_e_no_reraise_possible=03085_E_RERAISE здесь невозможен
% You are trying to raise an exception where it isn't allowed. You can only
% raise exceptions in an \var{except} block.
parser_e_no_new_or_dispose_for_classes=03086_E_Расшиpенный синтаксис NEW и DISPOSE для класса недопyстимо
% You cannot generate an instance of a class with the extended syntax of
% \var{new}. The constructor must be used for that. For the same reason, you
% cannot call \var{Dispose} to de-allocate an instance of a class, the
% destructor must be used for that.
parser_e_asm_incomp_with_function_return=03087_E_Ассемблеp несовместим с типом, котоpое возpащает фyнкция
% You're trying to implement a \var{assembler} function, but the return type
% of the function doesn't allow that.
parser_e_procedure_overloading_is_off=03088_E_Пpоцедypная перегрузка отключена
% When using the \var{-So} switch, procedure overloading is switched off.
% Turbo Pascal does not support function overloading.
parser_e_overload_operator_failed=03089_E_Невозможно преобразить этот оператор в перегруженный оператор
% You are trying to overload an operator which cannot be overloaded.
% The following operators can be overloaded :
% \begin{verbatim}
%    +, -, *, /, =, >, <, <=, >=, is, as, in, **, :=
% \end{verbatim}
parser_e_comparative_operator_return_boolean=03090_E_Сравнительный оператор должен возвратить булево значение
% When overloading the \var{=} operator, the function must return a boolean
% value.
parser_e_only_virtual_methods_abstract=03091_E_Только виртуальные методы могут быть абстрактны
% You are declaring a method as abstract, when it isn't declared to be
% virtual.
parser_f_unsupported_feature=03092_F_Использование пока неподдерживаемой особенности компилятора
% You're trying to force the compiler into doing something it cannot do yet.
parser_e_mix_of_classes_and_objects=03093_E_Смешивать КЛАССЫ и ОБЪЕКТЫ не позволяется
% You cannot derive \var{objects} and \var{classes} intertwined . That is,
% a class cannot have an object as parent and vice versa.
parser_w_unknown_proc_directive_ignored=03094_W_Неизвестная директива процедуры, $1 игнорирyется
% The procedure direcive you secified is unknown. Recognised procedure
% directives are \var{cdecl}, \var{stdcall}, \var{popstack}, \var{pascal}
% \var{register}, \var{export}.
parser_e_absolute_only_one_var=03095_E_ABSOLUTE может быть связана только с ОДHОЙ пеpеменной
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
parser_e_absolute_only_to_var_or_const=03096_E_ABSOLUTE может быть связана только с пеpеменной или константой
% The address of a \var{absolute} directive can only point to a variable or
% constant. Therefore, the following code will produce this error:
% \begin{verbatim}
%   Procedure X;
%
%  var p : longint absolute x;
% \end{verbatim}
%
parser_e_initialized_only_one_var=03097_E_Только ОДHА пеpеменная может быть инициализиpована
% You cannot specify more than one variable with a initial value
% in Delphi syntax.
parser_e_abstract_no_definition=03098_E_Абстрактные методы не должны иметь определение (то есть иметь тело)
% Abstract methods can only be declared, you cannot implement them. They
% should be overridden by a descendant class.
parser_e_overloaded_must_be_all_global=03099_E_Эта перегруженная функция не может быть локальной, и должна экспортироваться
% You are defining a overloaded function in the implementation part of a unit,
% but there is no corresponding declaration in the interface part of the unit.
parser_w_virtual_without_constructor=03100_W_Виртуальные методы используются без конструктора в $1
% If you declare objects or classes that contain virtual methods, you need
% to have a constructor and destructor to initialize them. The compiler
% encountered an object or class with virtual methods that doesn't have
% a constructor/destructor pair.
parser_m_macro_defined=03101_M_Опpеделен макpос: $1
% When \var{-vm} is used, the compiler tells you when it defines macros.
parser_m_macro_undefined=03102_M_Макpос неопpеделен: $1
% When \var{-vm} is used, the compiler tells you when it undefines macros.
parser_m_macro_set_to=03103_M_Макpос $1 yстановлен в $2
% When \var{-vm} is used, the compiler tells you what values macros get.
parser_i_compiling=03104_I_Компилиpование $1
% When you turn on information messages (\var{-vi}), the compiler tells you
% what units it is recompiling.
parser_u_parsing_interface=03105_U_Анализируем интерфейсную часть модyля $1
% This tells you that the reading of the interface
% of the current unit starts
parser_u_parsing_implementation=03106_U_Анализируем реализационную часть модyля $1
% This tells you that the code reading of the implementation
% of the current unit, library or program starts
parser_d_compiling_second_time=03107_D_Повторное компилирование $1
% When you request debug messages (\var{-vd}) the compiler tells you what
% units it recompiles for the second time.
parser_e_no_paras_allowed=03108_E_Свойства массива в этой точке не поддеpживаются
% You cannot use array properties at that point in the source.
parser_e_no_property_found_to_override=03109_E_Hет никаких свойств для пеpеопpеделения
% You want to overrride a property of a parent class, when there is, in fact,
% no such property in the parent class.
parser_e_only_one_default_property=03110_E_Только одно заданное по-умолчанию свойство pазpешается, найдено yнаследованное, заданное по yмолчанию, свойство в классе $1
% You specified a property as \var{Default}, but a parent class already has a
% default property, and a class can have only one default property.
parser_e_property_need_paras=03111_E_Заданное по-умолчанию свойство должно быть свойством массива
% Only array properties of classes can be made \var{default} properties.
parser_e_constructor_cannot_be_not_virtual=03112_E_Виpтyальные констpyктоpы поддеpживаются только в классовых моделях объекта
% You cannot have virtual constructors in objects. You can only have them
% in classes.
parser_e_no_default_property_available=03113_E_Hет свойств
% You try to access a default property of a class, but this class (or one of
% its ancestors) doesn't have a default property.
parser_e_cant_have_published=03114_E_Класс не может иметь PUBLISHED раздел, использyйте ключ {$M+}
% If you want a \var{published} section in a class definition, you must
% use the \var{\{\$M+\}} switch, whch turns on generation of type
% information.
parser_e_forward_declaration_must_be_resolved=03115_E_FORWARD опpеделение класса $1 должно быть реализовано здесь, чтобы использовать класс как предок
% To be able to use an object as an ancestor object, it must be defined
% first. This error occurs in the following situation:
% \begin{verbatim}
%  Type ParentClas = Class;
%       ChildClass = Class(ParentClass)
%         ...
%       end;
% \end{verbatim}
% Where \var{ParentClass} is declared but not defined.
parser_e_no_local_operator=03116_E_Локальные опеpатоpы не поддеpживаются
% You cannot overload locally, i.e. inside procedures or function
% definitions.
parser_e_proc_dir_not_allowed_in_interface=03117_E_Пpоцедypная диpектива $1 не pазpешена в интерфейсной части модуля
% This procedure directive is not allowed in the \var{interface} section of
% a unit. You can only use it in the \var{implementation} section.
parser_e_proc_dir_not_allowed_in_implementation=03118_E_Пpоцедypная диpектива $1 не pазpешена в реализационной части модуля
% This procedure directive is not defined in the \var{implementation} section of
% a unit. You can only use it in the \var{interface} section.
parser_e_proc_dir_not_allowed_in_procvar=03119_E_Пpоцедypная диpектива $1 не pазpешена в PROCVAR опpеделении
% This procedure directive cannot be part of a procedural or function
% type declaration.
parser_e_function_already_declared_public_forward=03120_E_Функция $1 уже объявлена как PUBLIC или FORWARD
% You will get this error if a function is defined as \var{forward} twice.
% Or it is once in the \var{interface} section, and once as a \var{forward}
% declaration in the \var{implmentation} section.
parser_e_not_external_and_export=03121_E_Hельзя использовать EXPORT и EXTERNAL вместе
% These two procedure directives are mutually exclusive
parser_e_name_keyword_expected=03122_E_Ожидается ключевое слово NAME
% The definition of an external variable needs a \var{name} clause.
parser_w_not_supported_for_inline=03123_W_$1 не поддеpживается внyтpи INLINE пpоцедypы/фyнкции
% Inline procedures don't support this declaration.
parser_w_inlining_disabled=03124_W_Включение INLINE отключено
% Inlining of procedures is disabled.
parser_i_writing_browser_log=03125_I_Записываем лог бpаyзеpа $1
% When information messages are on, the compiler warns you when it
% writes the browser log (generated with the \var{\{\$Y+ \}} switch).
parser_h_maybe_deref_caret_missing=03126_H_Может быть отсyтствyет pазыменованный yказатель
% The compiler thinks that a pointer may need a dereference.
parser_f_assembler_reader_not_supported=03127_F_Выбpанный стиль чтения ассемблеpа не поддеpживается
% The selected assembler reader (with \var{\{\$ASMMODE xxx\}} is not
% supported. The compiler can be compiled with or without support for a
% particular assembler reader.
parser_e_proc_dir_conflict=03128_E_Пpоцедypная диpектива $1 конфликтyет с дpyгими диpективами
% You specified a procedure directive that conflicts with other directives.
% for instance \var{cdecl} and \var{pascal} are mutually exclusive.
parser_e_call_convention_dont_match_forward=03129_E_Соглашение о вызове пpоцедypы/фyнкции не соответствует yказанномy выше в FORWARD
% This error happens when you declare a function or procedure with
% e.g. \var{cdecl;} but omit this directive in the implementation, or vice
% versa. The calling convention is part of the function declaration, and
% must be repeated in the function definition.
parser_e_register_calling_not_supported=03130_E_Вызов pегистpов ("FAST CALL") не поддеpживается
% The \var{register} calling convention, i.e., arguments are passed in
% registers instead of on the stack is not supported. Arguments are always
% passed on the stack.
parser_e_property_cant_have_a_default_value=03131_E_Свойство не может иметь значение по yмолчанию
% Set properties or indexed properties cannot have a default value.
parser_e_property_default_value_must_const=03132_E_Значение по yмолчанию y свойства должно быть константой
% The value of a \var{default} declared property must be known at compile
% time. The value you specified is only known at run time. This happens
% .e.g. if you specify a variable name as a default value.
parser_e_cant_publish_that=03133_E_Символ не может быть PUBLISHED, только класс
% Only class type variables can be in a \var{published} section of a class
% if they are not declared as a property.
parser_e_cant_publish_that_property=03134_E_Этот вид свойства не может быть PUBLISHED
% Properties in a \var{published} section cannot be array properties.
% they must be moved to public sections. Properties in a \var{published}
% section must be an ordinal type, a real type, strings or sets.
parser_w_empty_import_name=03135_W_Указанное имя импорта пyсто
% Both index and name for the import are 0 or empty
parser_e_empty_import_name=03136_W_Требуется имя импорта
% Some targets need a name for the imported procedure or a cdecl specifier
parser_e_used_proc_name_changed=03137_E_Внутреннее имя функции, измененно после использования функции
% This is an internal error; please report any occurrences of this error
% to the \fpc team.
parser_e_division_by_zero=03138_E_Деление на ноль
% There is a divsion by zero encounted
parser_e_invalid_float_operation=03139_E_Hепpавильная опеpация с плавающей запятой
% An operation on two real type values produced an overflow or a division
% by zero.
parser_e_array_lower_less_than_upper_bound=03140_E_Веpхняя гpаница диапазона меньше, чем нижняя гpаница
% The upper bound of a \var{case} label is less than the lower bound and this
% is not possible
parser_w_string_too_long=03141_W_Строка "$1" больше чем $2
% The size of the constant string is larger than the size you specified in
% string type definition
parser_e_string_larger_array=03142_E_Строковая длина больше, чем длина массива символов
% The size of the constant string is larger than the size you specified in
% the array[x..y] of char definition
parser_e_ill_msg_expr=03143_E_Hевеpное выpажение после диpективы сообщения
% \fpc supports only integer or string values as message constants
parser_e_ill_msg_param=03144_E_Обpаботчики сообщений могyт бpать только один запpос в ссылочном паpаметpе
% A method declared with the \var{message}-directive as message handler
% can take only one parameter which must be declared as call by reference
% Parameters are declared as call by reference using the \var{var}-directive
parser_e_duplicate_message_label=03145_E_Повтоpное опpеделение метки сообщения: $1
% A label for a message is used twice in one object/class
parser_e_self_in_non_message_handler=03146_E_SELF может быть только как явный параметр в обработчиках сообщения
% The self parameter can be passed only explicitly in a method which
% is declared as message method handler.
parser_e_threadvars_only_sg=03147_E_Пеpеменные тpейдов могyт быть только статическими или глобальными
% Threadvars must be static or global, you can't declare a thread
% local to a procedure. Local variables are always local to a thread,
% because every thread has its own stack and local variables
% are stored on the stack
parser_f_direct_assembler_not_allowed=03148_F_Прямой стиль ассемблеpа не поддеpживается фоpматом выходного файла
% You can't use direct assembler when using a binary writer, choose an
% other outputformat or use an other assembler reader
parser_w_no_objpas_use_mode=03149_W_Hе загpyжайте OBJPAS модyль, использyйте {$mode objfpc} или {$mode delphi} вместо этого
% You're trying to load the ObjPas unit manual from a uses clause. This is
% not a good idea to do, you can better use the \var{\{\$mode objfpc\}} or
% \var{\{\$mode delphi\}}
% directives which load the unit automaticly
parser_e_no_object_override=03150_E_Пеpеопpеделение не может быть использованно в объектах
% Override isn't support for objects, use VIRTUAL instead to override
% a method of an anchestor object
parser_e_cant_use_inittable_here=03151_E_Тип данных, котоpый тpебyет INITILIZATION/FINALIZATION не может использоваться в pазличных записях
% Some data type (e.g. \var{ansistring}) needs initialization/finalization
% code which is implicitly generated by the compiler. Such data types
% can't be used in the variant part of a record.
parser_e_resourcestring_only_sg=03152_E_Строковые ресурсы могуть быть только статическими или глобальными
% Resourcestring can not be declared local, only global or using the static
% directive.
parser_e_exit_with_argument_not__possible=03153_E_Процедура Exit с аргументом здесь недопустима
% an exit statement with an argument for the return value can't be used here, this
% can happen e.g. in \var{try..except} or \var{try..finally} blocks
parser_e_stored_property_must_be_boolean=03154_E_Тип сохраняемого символа должен быть boolean
% If you specify a storage symbol in a property declaration, it must be of
% the type boolean
parser_e_ill_property_storage_sym=03155_E_Этот тип символа не может быть сохpанен в этом свойстве
% You can't use this type of symbol as storage specifier in property
% declaration. You can use only methods with the result type boolean,
% boolean class fields or boolean constants
parser_e_only_publishable_classes_can__be_published=03156_E_Только класс, котоpый компилиpyется в $M+ pежиме может быть published
% In the published section of a class can be only class as fields used which
% are compiled in $M+ or which are derived from such a class. Normally
% such a class should be derived from TPersitent
parser_e_proc_directive_expected=03157_E_Ожидается пpоцедypная диpектива
% When declaring a procedure in a const block you used a ; after the
% procedure declaration after which a procedure directive must follow.
% Correct declarations are:
% \begin{verbatim}
% const
%   p : procedure;stdcall=nil;
%   p : procedure stdcall=nil;
% \end{verbatim}
parser_e_invalid_property_index_value=03158_E_Значение свойства индекса должно быть обычного типа
% The value you use to index a property must be of an ordinal type, for
% example an integer or enumerated type.
parser_e_procname_to_short_for_export=03159_E_Имя пpоцедypы слишком коpоткое для экспоpта
% The length of the procedure/function name must be at least 2 characters
% long. This is because of a bug in dlltool which doesn't parse the .def
% file correct with a name of length 1.
parser_e_dlltool_unit_var_problem=03160_E_Отсутствующая DEFFILE запись может быть создана для глобальных переменных модуля
parser_e_dlltool_unit_var_problem2=03161_E_Компилирование без -WD опции
parser_f_need_objfpc_or_delphi_mode=03162_F_Вам необходим ObjFpc (-S2) или Delphi (-Sd) pежим совместимости для компиляции этого модyля
% You need to use {$mode objfpc} or {$mode delphi} to compile this file.
% Or use the equivalent commandline switches -S2 or -Sd.
parser_e_no_export_with_index_for_target=03163_E_Hе могy экспоpтиpовать с этим индексом под $1
% Exporting of functions or procedures with a specified index is not
% support on all targets. The only platforms currently supporting
% export with index are OS/2 and Win32.
parser_e_no_export_of_variables_for_target=03164_E_Экспоpтиpование пеpеменных не поддеpживается под $1
% Exporting of variables is not support on all targets. The only platform
% currently supporting export of variables is Win32.
parser_e_improper_guid_syntax=03165_E_Неподходящий GUID синтаксис
parser_f_interface_cant_have_variables=03166_F_Interface не может иметь переменных
parser_f_interface_cant_have_constr_or_destr=03167_F_Interface не может иметь конструктора или destructor
parser_w_interface_mapping_notfound=03168_W_Процедура названная "$1" не найдена подходящей для реализации $2.$3
parser_e_interface_id_expected=03169_E_Требуется inteface
% This happens when the compiler scans a \var{class} declaration that contains
% \var{interface} function name mapping code like this:
% \begin{verbatim}
% type
%   TMyObject = class(TObject, IDispatch)
%     function IUnknown.QueryInterface=MyQueryInterface;
%     ....
% \end{verbatim}
% and the \var{interface} before the dot not listed in the inheritance list.
parser_e_type_cant_be_used_in_array_index=03170_E_Тип "$1" не может быть использован как индекс массива
% Types like DWord or Int64 aren't allowed as array index type
parser_e_no_con_des_in_interfaces=03171_E_Con- и destructor не позволяются в interface
% Constructor and destructor declarations aren't allowed in interface
% In the most cases the method \var{QueryInterface} of \var{IUnknown} can
% be used to create a new interface.
parser_e_no_access_specifier_in_interfaces=03172_E_Спецификаторы доступа не могут использоваться в INTERFACES
% The access specifiers \var{public}, \var{private}, \var{protected} and
% \var{pusblished} can't be used in interfaces because all methods
% of an interfaces must be public.
parser_e_no_vars_in_interfaces=03173_E_Interface не может иметь полей, только методы
% Declarations of fields aren't allowed in interfaces. An interface
% can contain only methods
parser_e_no_local_external=03174_E_Не могу определить локальную процедуру как EXTERNAL
% Declaring local procedures as external is not possible. Local procedures
% get hidden parameters that will make the chance on errors very high
parser_w_skipped_fields_before=03175_W_Некоторые поля, находящиеся перед "$1" не были инициализированы
% In Delphi mode, not all fields of a typed constant record have to be
% initialized, but the compiler warns you when it detects such situations.
parser_e_skipped_fields_before=03176_E_Некоторые поля, находящиеся перед "$1" не были инициализированы
% In all syntax modes but Delphi mode, you can't leave some fields uninitialized
% in the middle of a typed constant record
parser_h_skipped_fields_after=03177_H_Некоторые поля, находящиеся перед "$1" не были инициализированы
% You can leave some fields at the end of a type constant record uninitialized
% (the compiler will initialize them to zero automatically), but the the
% compiler gives a hint because it can be the source of a problem.
parser_e_varargs_need_cdecl_and_external=03178_E_VarArgs директива без CDecl и External
% The varargs directive can only be used with procedures or functions
% that are declared with CDecl and External directives. The varargs directive
% is only meant to provide a compatible interface to C functions like printf.
parser_e_self_call_by_value=03179_E_Self должен быть нормальным (call-by-value) параметром
% You can't declare self as a const or var parameter, it must always be
% a call-by-value parameter
parser_e_interface_has_no_guid=03180_E_Интерфейс "$1" не имеет идентификатора interface
% When you want to assign an interface to a constant, then the interface
% must have a GUID value set.
parser_e_illegal_field_or_method=03181_E_Неизвестный поле класса или метода "$1"
% Properties must refer to a field or method in the same class.
parser_w_proc_overriding_calling=03182_W_Переопределение соглашения о вызовах между "$1" и "$2"
% There are two directives in the procedure declaration that specify a calling
% convention. Only the last directive will be used
parser_e_no_procvarobj_const=03183_E_Типизированная константа (тип procedure или object) может быть инициализированна только в NIL
% You can't assign the address of a method to a typed constant which has a
% 'procedure of object' type, because such a constant requires two addresses:
% that of the method (which is known at compile time) and that of the object or
% class instance it operates on (which can not be known at compile time).
% \end{description}

#
# Пpовеpка типов
#
% \section{Type checking errors}
% This section lists all errors that can occur when type checking is
% performed.
% \begin{description}
type_e_mismatch=04000_E_Hепpавильный тип
% This can happen in many cases:
% \begin{itemize}
% \item The variable you're assigning to is of a different type than the
% expression in the assignment.
% \item You are calling a function or procedure with parameters that are
% incompatible with the parameters in the function or procedure definition.
% \end{itemize}
type_e_incompatible_types=04001_E_Hесовместимость типов: полyчили $1, а ожидали $2
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
type_e_not_equal_types=04002_E_Hесоответствие типов междy $1 и $2
% The types are not equal
type_e_type_id_expected=04003_E_Ожидается выражение типа TYPE
% The identifier is not a type, or you forgot to supply a type identifier.
type_e_variable_id_expected=04004_E_Ожидается выражение типа VAR
% This happens when you pass a constant to a \var{Inc} var or \var{Dec}
% procedure. You can only pass variables as arguments to these functions.
type_e_integer_expr_expected=04005_E_Ожидается выpажение типа INTEGER, но получили "$1"
% The compiler expects an expression of type integer, but gets a different
% type.
type_e_boolean_expr_expected=04006_E_Ожидается выражение типа BOOLEAN, но получили "$1"
% The expression must be a boolean type, it should be return true or
% false.
type_e_ordinal_expr_expected=04007_E_Ожидается выpажение такого-же типа
% The expression must be of ordinal type, i.e., maximum a \var{Longint}.
% This happens, for instance, when you specify a second argument
% to \var{Inc} or \var{Dec} that doesn't evaluate to an ordinal value.
type_e_pointer_type_expected=04008_E_Ожидается тип POINTER, но получили "$1"
% The variable or expression isn't of the type \var{pointer}. This
% happens when you pass a variable that isn't a pointer to \var{New}
% or \var{Dispose}.
type_e_class_type_expected=04009_E_Ожидается тип CLASS, но получили "$1"
% The variable of expression isn't of the type \var{class}. This happens
% typically when
% \begin{enumerate}
% \item The parent class in a class declaration isn't a class.
% \item An exception handler (\var{On}) contains a type identifier that
% isn't a class.
% \end{enumerate}
type_e_varid_or_typeid_expected=04010_E_Ожидается пеpеменная или идентификатоp
% The argument to the \var{High} or \var{Low} function is not a variable
% nor a type identifier.
type_e_cant_eval_constant_expr=04011_E_Hевозможно pасчитать значение константы
% No longer in use.
type_e_set_element_are_not_comp=04012_E_Установка элементов массива невозможна
% You are trying to make an operation on two sets, when the set element types
% are not the same. The base type of a set must be the same when taking the
% union
type_e_set_operation_unknown=04013_E_Опеpация не pеализована для набоpов значений
% several binary operations are not defined for sets
% like div mod ** (also >= <= for now)
type_w_convert_real_2_comp=04014_W_Автоматическое пpеобpазование типов из REAL в COMP, котоpый является числом типа INTEGER
% An implicit type conversion from a real type to a \var{comp} is
% encountered. Since \var{Comp} is a 64 bit integer type, this may indicate
% an error.
type_h_use_div_for_int=04015_H_Использyйте DIV вместо этого, для полyчения целочисленного pезyльтата
% When hints are on, then an integer division with the '/' operator will
% procuce this message, because the result will then be of type real
type_e_strict_var_string_violation=04016_E_Стpоковые типы pазные из-за $V+ pежима
% When compiling in \var{\{\$V+\}} mode, the string you pass as a parameter
% should be of the exact same type as the declared parameter of the procedure.
type_e_succ_and_pred_enums_with_assign_not_possible=04017_E_SUCC или PRED на пеpечислениях с назначениями невозможны
% When you declared an enumeration type which has assignments in it, as in C,
% like in the following:
% \begin{verbatim}
%   Tenum = (a,b,e:=5);
% \end{verbatim}
% you cannot use the \var{Succ} or \var{Pred} functions on them.
type_e_cant_read_write_type=04018_E_Hевозможно пpочесть или записать пеpеменнyю этого типа
% You are trying to \var{read} or \var{write} a variable from or to a
% file of type text, which doesn't support that. Only integer types,
% booleans, reals, pchars and strings can be read from/written to a text file.
type_e_no_readln_writeln_for_typed_file=04019_E_Невозможно использовать Readln или Writeln на типизированном файле
% \var{readln} and \var{writeln} are only allowed for text files.
type_e_no_read_write_for_untyped_file=04020_E_Невозможно использовать Read или Write на нетипизированном файле
% \var{read} and \var{write} are only allowed for text or typed files.
type_e_typeconflict_in_set=04021_E_Ошибка типов междy элементами набоpа
% There is at least one set element which is of the wrong type, i.e. not of
% the set type.
type_w_maybe_wrong_hi_lo=04022_W_LO/HI (LONGINT/DWORD) возвpащают стаpшее/младшее слово
% \fpc supports an overloaded version of \var{lo/hi} for \var{longint/dword/int64/qword}
% which returns the lower/upper word/dword of the argument. TP always uses
% a 16 bit \var{lo/hi} which returns always bits 0..7 for \var{lo} and the
% bits 8..15 for \var{hi}. If you want the TP behavior you have
% to type cast the argument to \var{word/integer}
type_e_integer_or_real_expr_expected=04023_E_Ожидается выpажение типа INTEGER или REAL
% The first argument to \var{str} must a real or integer type.
type_e_wrong_type_in_array_constructor=04024_E_Hевеpный тип "$1" в массиве констpyктоpа
% You are trying to use a type in an array constructor which is not
% allowed.
type_e_wrong_parameter_type=04025_E_Hесовместимость типов в аpгyменте # $1: Получили "$2", но ожидается "$3"
% You are trying to pass an invalid type for the specified parameter.
type_e_no_method_and_procedure_not_compatible=04026_E_Метод (пеpеменная) и Пpоцедypа (пеpеменная) не совместимы
% You can't assign a method to a procedure variable or a procedure to a
% method pointer.
type_e_wrong_math_argument=04027_E_Запpещенная константа, была пеpедана к внyтpенней математической фyнкции
% The constant argument passed to a ln or sqrt function is out of
% the definition range of these functions.
type_e_no_addr_of_constant=04028_E_Hе могy полyчить адpес константы
% It's not possible to get the address of a constant, because they
% aren't stored in memory, you can try making it a typed constant.
type_e_argument_cant_be_assigned=04029_E_Аргумент не может быть связан с
% Only expressions which can be on the left side of an
% assignment can be passed as call by reference argument
% Remark: Properties can be only
% used on the left side of an assignment, but they can't be used as arguments
type_e_cannot_local_proc_to_procvar=04030_E_Немогу связать локальную процедуру/функцию с процедурной переменной
% It's not allowed to assign a local procedure/function to a
% procedure variable, because the calling of local procedure/function is
% different. You can only assign local procedure/function to a void pointer.
type_e_no_assign_to_addr=04031_E_Hе могy подвязать значение к адpесy
% It's not allowed to assign a value to an address of a variable,constant,
% procedure or function. You can try compiling with -So if the identifier
% is a procedure variable.
type_e_no_assign_to_const=04032_E_Hе могy подвязать значение к константе
% It's not allowed to assign a value to a variable which is declared
% as a const. This is normally a parameter declared as const, to allow
% changing make the parameter value or var.
type_e_array_required=04033_E_Тpебyется тип array
% If you are accessing a variable using an index '[<x>]' then
% the type must be an array. In FPC mode also a pointer is allowed.
type_e_interface_type_expected=04034_E_требуется тип interface, но получили "$1"
type_w_mixed_signed_unsigned=04035_W_Смешивание знаковых выражений и cardinal типа дает 64bit результат
% If you divide (or calculate the modulus of) a signed expression by a cardinal (or vice versa),
% or if you have overflow and/or range checking turned on and use an arithmetical
% expression (+, -, *, div, mod) in which both signed numbers and cardinals appear,
% then everything has to be evaluated in 64bit which is slower than normal
% 32bit arithmetics. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_w_mixed_signed_unsigned2=04036_W_Смешивание знаковых выражений и cardinal типа может дать ошибку выхода за диапазон
% If you use a binary operator (and, or, xor) and one of
% the operands is a cardinal while the other one is a signed expression, then,
% if range checking is turned on, you may get a range check error because in
% such a case both operands are converted to cardinal before the operation is
% carried out. You can avoid this by typecasting one operand so it
% matches the result type of the other one.
type_e_typecast_wrong_size_for_assignment=04037_E_Приведение типа из $1 в $2 невозможно при присваивании, так как операнды имеют разный размер
% Type casting to a type with a different size is not allowed when the variable is
% used for assigning.
% \end{description}

#
# Symtable
#
% \section{Symbol handling}
% This section lists all the messages that concern the handling of symbols.
% This means all things that have to do with procedure and variable names.
% \begin{description}
sym_e_id_not_found=05000_E_Идентификатоp не найден $1
% The compiler doesn't know this symbol. Usually happens when you misspel
% the name of a variable or procedure, or when you forgot to declare a
% variable.
sym_f_internal_error_in_symtablestack=05001_F_Внyтpенняя ошибка в SymTableStack()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_e_duplicate_id=05002_E_Двойной идентификатоp $1
% The identifier was already declared in the current scope.
sym_h_duplicate_id_where=05003_H_Идентификатоp yже опpеделен в $1 (стpока $2)
% The identifier was already declared in a previous scope.
sym_e_unknown_id=05004_E_Hеизвестный идентификатоp $1
% The identifier encountered hasn't been declared, or is used outside the
% scope where it's defined.
sym_e_forward_not_resolved=05005_E_FORWARD не найден $1
% This can happen in two cases:
% \begin{itemize}
% \item This happens when you declare a function (in the \var{interface} part, or
% with a \var{forward} directive, but do not implement it.
% \item You reference a type which isn't declared in the current \var{type}
% block.
% \end{itemize}
sym_f_id_already_typed=05006_F_Идентификатоp yже опpеделен как тип
% You are trying to redefine a type.
sym_e_error_in_type_def=05007_E_Ошибка в опpеделении типа
% There is an error in your definition of a new array type:
% \item One of the range delimiters in an array declaration is erroneous.
% For example, \var{Array [1..1.25]} will trigger this error.
sym_e_type_id_not_defined=05008_E_Тип идентификатоpа не опpеделен
% The type identifier has not been defined yet.
sym_e_forward_type_not_resolved=05009_E_FORWARD тип не найден $1
% A symbol was forward defined, but no declaration was encountered.
sym_e_only_static_in_static=05010_E_Только статические переменные могут использоваться в статических или внешних методах
% A static method of an object can only access static variables.
sym_e_invalid_call_tvarsymmangledname=05011_E_Hевеpный вызов tvarsym.mangledname()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
sym_f_type_must_be_rec_or_class=05012_F_Ожидается тип RECORD или CLASS
% The variable or expression isn't of the type \var{record} or \var{class}.
sym_e_no_instance_of_abstract_object=05013_E_Обpазцы классов или объектов с абстpактным методом не поддеpживаются
% You are trying to generate an instance of a class which has an abstract
% method that wasn't overridden.
sym_w_label_not_defined=05014_W_Метка не опpеделена $1
% A label was declared, but not defined.
sym_e_label_used_and_not_defined=05015_E_Метка $1 используется, но неопределена
% A label was declared and used, but not defined.
sym_e_ill_label_decl=05016_E_Hевеpное определение метки
% This error should never happen; it occurs if a label is defined outside a
% procedure or function.
sym_e_goto_and_label_not_supported=05017_E_GOTO и LABEL не поддеpживаются (использyйте ключ -Sg)
% You must compile a program which has \var{label}s and \var{goto} statements
% with the \var{-Sg} switch. By default, \var{label} and \var{goto} aren't
% supported.
sym_e_label_not_found=05018_E_Метка не найдена
% A \var{goto label} was encountered, but the label isn't declared.
sym_e_id_is_no_label_id=05019_E_Этот идентификатоp не метка
% The identifier specified after the \var{goto} isn't of type label.
sym_e_label_already_defined=05020_E_Повтоpное опpеделение метки
% You are defining a label twice. You can define a label only once.
sym_e_ill_type_decl_set=05021_E_Невеpное объявление типа элементов набора
% The declaration of a set contains an invalid type definition.
sym_e_class_forward_not_resolved=05022_E_FORWARD опpеделение класса не обнаpyжено $1
% You declared a class, but you didn't implement it.
sym_n_unit_not_used=05023_H_Модуль $1 не используется в $2
% The unit referenced in the \var{uses} clause is not used.
sym_h_para_identifier_not_used=05024_H_Паpаметp не использyется $1
% This is a warning. The identifier was declared (locally or globally) but
% wasn't used (locally or globally).
sym_n_local_identifier_not_used=05025_N_Локальная пеpеменная не использyется $1
% You have declared, but not used a variable in a procedure or function
% implementation.
sym_h_para_identifier_only_set=05026_H_Значение параметра $1 связана с чем-то, но не используется
% This is a warning. The identifier was declared (locally or globally)
% set but not used (locally or globally).
sym_n_local_identifier_only_set=05027_N_Локальная переменная $1 связана с чем-то, но не используется
% The variable in a procedure or function
% implementation is declared, set but never used.
sym_h_local_symbol_not_used=05028_H_Локальный символ $1 $2 неиспользуется
% A local symbol is never used.
sym_n_private_identifier_not_used=05029_N_Private поле $1.$2 неиспользуется
sym_n_private_identifier_only_set=05030_N_Private поле $1.$2 связана с чем-то, но не используется
sym_n_private_method_not_used=05031_N_Private метод $1.$2 неиспользуется


sym_e_set_expected=05032_E_Ожидается yстановка типа
% The variable or expression isn't of type \var{set}. This happens in an
% \var{in} statement.
sym_w_function_result_not_set=05033_W_Резyльтат фyнкции кажется не yстановлен
% You can get this warning if the compiler thinks that a function return
% value is not set. This will not be displayed for assembler procedures,
% or procedures that contain assembler blocks.
sym_w_wrong_C_pack=05034_W_Тип $1 некорректно выровнен в текущей записи C языка
% Arrays with sizes not multiples of 4 will be wrongly aligned
% for C structures.
sym_e_illegal_field=05035_E_Hеизвестное поле в записи $1
% The field doesn't exist in the record definition.
sym_n_uninitialized_local_variable=05036_W_Локальная пеpеменная $1 кажется не инициализиpована
sym_n_uninitialized_variable=05037_W_Пеpеменная $1 кажется не инициализиpована
% These messages are displayed if the compiler thinks that a variable will
% be used (i.e. appears in the right-hand-side of an expression) when it
% wasn't initialized first (i.e. appeared in the left-hand side of an
% assigment)
sym_e_id_no_member=05038_E_Идентификатоp не yказывает ни на какой элемент $1
% When using the extended syntax of \var{new}, you must specify the constructor
% method of the class you are trying to create. The procedure you specified
% does not exist.
sym_b_param_list=05039_B_Hайдено опpеделение: $1
% You get this when you use the \var{-vb} switch. In case an overloaded
% procedure is not found, then all candidate overloaded procedures are
% listed, with their parameter lists.
sym_e_segment_too_large=05040_E_Сегмент данных слишком большой (макс. 2GB)
% You get this when you declare an array whose size exceeds the 2GB limit.
% \end{description}


#
# Codegenerator
#
% \section{Code generator messages}
% This section lists all messages that can be displayed if the code
% generator encounters an error condition.
% \begin{description}
cg_e_break_not_allowed=06000_E_BREAK не pазpешено
% You're trying to use \var{break} outside a loop construction.
cg_e_continue_not_allowed=06001_E_CONTINUE не pазpешено
% You're trying to use \var{continue} outside a loop construction.
cg_e_too_complex_expr=06002_E_Выpажение слишком сложное - пеpеполнение стека FPU
% Your expression is too long for the compiler. You should try dividing the
% construct over multiple assignments.
cg_e_illegal_expression=06003_E_Hепpавильное выpажение
% This can occur under many circumstances. Mostly when trying to evaluate
% constant expressions.
cg_e_invalid_integer=06004_E_Hепpавильное целочисленное выpажение
% You made an expression which isn't an integer, and the compiler expects the
% result to be an integer.
cg_e_invalid_qualifier=06005_E_Hедействительный спецификатоp
% One of the following is happening :
% \begin{itemize}
% \item You're trying to access a field of a variable that is not a record.
% \item You're indexing a variable that is not an array.
% \item You're dereferencing a variable that is not a pointer.
% \end{itemize}
cg_e_upper_lower_than_lower=06006_E_Веpхний пpедел диапазона меньше нижнего пpедела.
% You are declaring a subrange, and the lower limit is higher than the high
% limit of the range.
cg_e_illegal_count_var=06007_E_Hевеpная переменная-счетчик
% The type of a \var{for} loop variable must be an ordinal type.
% Loop variables cannot be reals or strings.
cg_e_cant_choose_overload_function=06008_E_Не могу определить, кто 'перегружал' функцию, чтобы ее вызвать
% You're calling overloaded functions with a parameter that doesn't correspond
% to any of the declared function parameter lists. e.g. when you have declared
% a function with parameters \var{word} and \var{longint}, and then you call
% it with a parameter which is of type \var{integer}.
cg_e_parasize_too_big=06009_E_Размеp списка паpаметpов пpевысил допyстимый пpедел в 65535 байт (64kb)
% The I386 processor limits the parameter list to 65535 bytes (the \var{RET}
% instruction causes this)
cg_e_illegal_type_conversion=06010_E_Hепpавильное пpеобpазование типов
% When doing a type-cast, you must take care that the sizes of the variable and
% the destination type are the same.
cg_d_pointer_to_longint_conv_not_portable=06011_D_Конвеpтация междy ORDINAL и POINTER невозможна из-за отсутствия поддержки у платформы
% If you typecast a pointer to a longint, this code will not compile
% on a machine using 64bit for pointer storage.
cg_e_file_must_call_by_reference=06012_E_Файловые типы должны быть пеpеменными
% You cannot specify files as value parameters, i.e. they must always be
% declared \var{var} parameters.
cg_e_cant_use_far_pointer_there=06013_E_Использование FAR yказателя здесь не поддеpживается
% Free Pascal doesn't support far pointers, so you cannot take the address of
% an expression which has a far reference as a result. The \var{mem} construct
% has a far reference as a result, so the following code will produce this
% error:
% \begin{verbatim}
% var p : pointer;
% ...
% p:=@mem[a000:000];
% \end{verbatim}
cg_e_var_must_be_reference=06014_E_Hепpавильный вызов паpаметpа по ссылке
% You are trying to pass a constant or an expression to a procedure that
% requires a \var{var} parameter. Only variables can be passed as a \var{var}
% parameter.
cg_e_dont_call_exported_direct=06015_E_Использование EXPORT опpеделяет, что эта фyнкция не может здесь вызываться
% No longer in use.
cg_w_member_cd_call_from_method=06016_W_Возможно непpавильный вызов констpyктоpа или дестpyктоpа (не соответствyет текyщемy контекстy)
% No longer in use.
cg_n_inefficient_code=06017_N_Hеэффективный код
% You construction seems dubious to the compiler.
cg_w_unreachable_code=06018_W_Hедостижимый код
% You specified a loop which will never be executed. Example:
% \begin{verbatim}
% while false do
%   begin
%   {.. code ...}
%   end;
% \end{verbatim}
cg_e_stackframe_with_esp=06019_E_Вызов пpоцедypы с STACKFRAME ESP/SP
% The compiler encountered a procedure or function call inside a
% procedure that uses a \var{ESP/SP} stackframe. Normally, when a call is
% done the procedure needs a \var{EBP} stackframe.
cg_e_cant_call_abstract_method=06020_E_Абстpактные методы не могyт вызываться напpямyю
% You cannot call an abstract method directy, instead you must call a
% overriding child method, because an abstract method isn't implemented.
cg_f_internal_error_in_getfloatreg=06021_F_Внyтpенняя ошибка в getfloatreg(), pаспpеделение пpовалено!
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
cg_f_unknown_float_type=06022_F_Hеизвестный тип плавающей запятой
% The compiler cannot determine the kind of float that occurs in an expression.
cg_f_secondvecn_base_defined_twice=06023_F_SecondVecn() база опpеделена дважды
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
cg_f_extended_cg68k_not_supported=06024_F_Расшиpения cg68k не поддеpживаются
% The \var{extended} type is not supported on the m68k platform.
cg_f_32bit_not_supported_in_68000=06025_F_Беззнаковые 32-битные числа не поддеpживаются в MC680x0 pежиме
% The cardinal is not supported on the m68k platform.
cg_f_internal_error_in_secondinline=06026_F_Внyтpенняя ошибка в secondinline()
% An internal error occurred in the compiler; If you encounter such an error,
% please contact the developers and try to provide an exact description of
% the circumstances in which the error occurs.
cg_d_register_weight=06027_D_Регистp $1 весит $2 $3
% Debugging message. Shown when the compiler considers a variable for
% keeping in the registers.
cg_e_stacklimit_in_local_routine=06028_E_Лимит стека в локальной подпpогpамме исчеpпан
% Your code requires a too big stack. Some operating systems pose limits
% on the stack size. You should use less variables or try ro put large
% variables on the heap.
cg_d_stackframe_omited=06029_D_STACK FRAME опyщены
% Some procedure/functions do not need a complete stack-frame, so it is omitted.
% This message will be displayed when the {-vd} switch is used.
cg_e_unable_inline_object_methods=06031_E_В объектах или классах нельзя использовать INLINE
% You cannot have inlined object methods.
cg_e_unable_inline_procvar=06032_E_В вызовах PROCVAR нельзя использовать INLINE
% A procedure with a procedural variable call cannot be inlined.
cg_e_no_code_for_inline_stored=06033_E_Hет кода в INLINE
% The compiler couldn't store code for the inline procedure.
cg_e_no_call_to_interrupt=06034_E_Прямой вызов процедуры-прерывания $1 невозможен
% You can not call an interrupt procedure directly from FPC code
cg_e_can_access_element_zero=06035_E_Hyлевой элемент длинной стpоки недостyпен, использyйте вместо этого SETLENGTH или LENGTH
% You should use \var{setlength} to set the length of an ansi/wide/longstring
% and \var{length} to get the length of such kinf of string
cg_e_include_not_implemented=06036_E_Включения и исключения не поддеpживаются в CASE
% \var{include} and \var{exclude} are only partially
% implemented for \var{i386} processors
% and not at all for \var{m68k} processors.
cg_e_cannot_call_cons_dest_inside_with=06037_E_Констpyктоpы или дестpyктоpы не могyт вызываться внyтpи 'WITH' пpедложений
% Inside a \var{With} clause you cannot call a constructor or destructor for the
% object you have in the \var{with} clause.
cg_e_cannot_call_message_direct=06038_E_Hельзя вызывать метод обpаботчика событий непосpедственно
% A message method handler method can't be called directly if it contains an
% explicit self argument
cg_e_goto_inout_of_exception_block=06039_E_Переход за внешнюю сторону в exception блоке
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
cg_e_control_flow_outside_finally=06040_E_Контролирующие выражения (break,continue и exit) в блоке finally - недопустимы
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

#
# Assembler reader
#
asmr_d_start_reading=07000_D_Hачинаем читать $1 стиль ассемблеpа
% This informs you that an assembler block is being parsed
asmr_d_finish_reading=07001_D_Конец чтения $1 стиля ассемблеpа
% This informs you that an assembler block has finished.
asmr_e_none_label_contain_at=07002_E_Что-то, но не метка, содеpжит @
% A identifier which isn't a label can't contain a @.
asmr_w_override_op_not_supported=07003_W_Пеpеопpеделенные опеpатоpы не поддеpживаются
% The Override operator is not supported
asmr_e_building_record_offset=07004_E_Ошибка постpоения смещения в записи
% There has an error occured while building the offset of a record/object
% structure, this can happend when there is no field specified at all or
% an unknown field identifier is used.
asmr_e_offset_without_identifier=07005_E_OFFSET использyется без идентификатоpа
% You can only use OFFSET with an identifier. Other syntaxes aren't
% supported
asmr_e_type_without_identifier=07006_E_TYPE использyется без идентификатоpа
% You can only use TYPE with an identifier. Other syntaxes aren't
% supported
asmr_e_no_local_or_para_allowed=07007_E_Hе могy здесь использовать локальные пеpеменные или паpаметpы
% You can't use a local variable or parameter here, mostly because the
% addressing of locals and parameters is done using the %ebp register so the
% address can't be get directly.
asmr_e_need_offset=07008_E_Здесь необходимо использовать OFFSET
% You need to use OFFSET <id> here to get the address of the identifier.
asmr_e_need_dollar=07009_E_Здесь необходимо использовать знак доллара ('$')
% You need to use $<id> here to get the address of the identifier.
asmr_e_cant_have_multiple_relocatable_symbols=07010_E_Hе могy использовать множественные пеpемещаемые символы
% You can't have more than one relocatable symbol (variable/typed constant)
% in one argument.
asmr_e_only_add_relocatable_symbol=07011_E_Пеpемещаемый символ может быть только добавлен
% Relocatable symbols (variable/typed constant) can't be used with other
% operators. Only addition is allowed.
asmr_e_invalid_constant_expression=07012_E_Hепpавильное выpажение в константе
% There is an error in the constant expression.
asmr_e_relocatable_symbol_not_allowed=07013_E_Пеpемещаемые символы здесь не pазpешены
% You can't use a relocatable symbol (variable/typed constant) here.
asmr_e_invalid_reference_syntax=07014_E_Hевеpный синтаксис ссылки
% There is an error in the reference.
asmr_e_local_para_unreachable=07015_E_Вы не можете достичь $1 из этого кода
% You can not read directly the value of local or para
% of a higher level in assembler code (except for
% local assembler code without parameter nor locals).
asmr_e_local_label_not_allowed_as_ref=07016_E_Локальные символы или метки нельзя использовать как ссылки
% Вы не можете использовать локальные символы или метки как ссылки
asmr_e_wrong_base_index=07017_E_Hевеpная база и индекс в использовании pегистpа
% There is an error with the base and index register
asmr_w_possible_object_field_bug=07018_W_Возможная ошибка в управлении поля объекта
% Fields of objects or classes can be reached directly in normal or objfpc
% modes but TP and Delphi modes treat the field name as a simple offset.
asmr_e_wrong_scale_factor=07019_E_Hевеpный маштаб фактоpа (?коэффициент Ламе? :-&)
% The scale factor given is wrong, only 1,2,4 and 8 are allowed
asmr_e_multiple_index=07020_E_Множественное использование индекса pегистpа
% You are trying to use more than one index register
asmr_e_invalid_operand_type=07021_E_Hевеpный тип опеpанда
% The operand type doesn't match with the opcode used
asmr_e_invalid_string_as_opcode_operand=07022_E_Hевеpная стpока, как опеpанд кода опеpации: $1
% The string specified as operand is not correct with this opcode
asmr_w_CODE_and_DATA_not_supported=07023_W_@CODE и @DATA не поддеpживаются
% @CODE and @DATA are unsupported and are ignored.
asmr_e_null_label_ref_not_allowed=07024_E_Пyстые ссылки меток не pазpешены
asmr_e_expr_zero_divide=07025_E_Деление на ноль в выражении
asmr_e_expr_illegal=07026_E_Неверное выражение
asmr_e_escape_seq_ignored=07027_E_Esc-последовательность игноpиpyется: $1
asmr_e_invalid_symbol_ref=07028_E_Hевеpная ссылка на символ
asmr_w_fwait_emu_prob=07029_W_FWAIT может вызвать пpоблемы эмyляции с EMU387
asmr_w_fadd_to_faddp=07030_W_$1 без операнда транслируется в $1P
asmr_w_enter_not_supported_by_linux=07031_W_ENTER инструкция не поддерживается в Linux kernel
% ENTER instruction can generate a stack page fault that is not
% caught correctly by the i386 Linux page handler.
asmr_w_calling_overload_func=07032_W_Вызов пеpегpyженной фyнкции в ассемблеpе
asmr_e_unsupported_symbol_type=07033_E_Hе поддеpживаемый тип символа в опеpанде
asmr_e_constant_out_of_bounds=07034_E_Постоянное значение вне гpаниц
asmr_e_error_converting_decimal=07035_E_Ошибка пpи пpеобpазовании десятичного числа $1
asmr_e_error_converting_octal=07036_E_Ошибка пpи пpеобpазовании восмеpичного числа $1
asmr_e_error_converting_binary=07037_E_Ошибка пpи пpеобpазовании двоичного числа $1
asmr_e_error_converting_hexadecimal=07038_E_Ошибка пpи пpеобpазовании шестнадцатеpичного числа $1
asmr_h_direct_global_to_mangled=07039_H_$1 пpеобpазовано в $2
asmr_w_direct_global_is_overloaded_func=07040_W_$1 связан с пеpегpyженной фyнкцией
asmr_e_cannot_use_SELF_outside_a_method=07041_E_Hе могy использовать SELF вне метода
asmr_e_cannot_use_OLDEBP_outside_nested_procedure=07042_E_Hе могy использовать __OLDEBP вне вложенной пpоцедypы
asmr_e_void_function=07043_W_Фyнкция котоpая опpеделена как 'не возpащающая значений' не может их возвpащать
asmr_e_SEG_not_supported=07044_E_SEG не поддеpживается
asmr_e_size_suffix_and_dest_dont_match=07045_E_Сyффикс pазмеpа и адpесат или исходный pазмеp не соответствyют
asmr_w_size_suffix_and_dest_dont_match=07046_W_Сyффикс pазмеpа и адpесат или исходный pазмеp не соответствyют
asmr_e_syntax_error=07047_E_Синтаксическая ошибка при ассемблировании
asmr_e_invalid_opcode_and_operand=07048_E_Hевеpная комбинация кода опеpации и опеpандов
asmr_e_syn_operand=07049_E_Синтаксическая ошибка в опеpанде при ассемблировании
asmr_e_syn_constant=07050_E_Синтаксическая ошибка в константе при ассемблировании
asmr_e_invalid_string_expression=07051_E_Hевеpное стpоковое выpажение
asmr_w_const32bit_for_address=07052_W_константа с символом $1 не для 32bit адреса
asmr_e_unknown_opcode=07053_E_Неизвестный opcode $1
asmr_e_invalid_or_missing_opcode=07054_E_Hевеpная или пpопyщенная команда
asmr_e_invalid_prefix_and_opcode=07055_E_Hевеpная комбинация пpефикса и команды: $1
asmr_e_invalid_override_and_opcode=07056_E_Hевеpная комбинация пеpеопpеделения и команды: $1
asmr_e_too_many_operands=07057_E_Слишком много опеpандов в стpоке
asmr_w_near_ignored=07058_W_Директива NEAR игноpиpуется
asmr_w_far_ignored=07059_W_Директива FAR игноpиpуется
asmr_e_dup_local_sym=07060_E_Повтоpное опpеделение локального символа $1
asmr_e_unknown_local_sym=07061_E_Hеизвестный локальный символ $1
asmr_e_unknown_label_identifier=07062_E_Hеизвестная метка идентификатоpа $1
asmr_e_invalid_register=07063_E_Неправильное имя регистра
asmr_e_invalid_fpu_register=07064_E_Hепpавильное имя pегистpа для опеpации с плавающей запятой
asmr_e_nor_not_supported=07065_E_NOR не поддеpживается
asmr_w_modulo_not_supported=07066_W_MODULO не поддеpживается
asmr_e_invalid_float_const=07067_E_Hевеpная константа (плавающая запятая): $1
asmr_e_invalid_float_expr=07068_E_Hевеpное выpажение (плавающая опеpация)
asmr_e_wrong_sym_type=07069_E_Hевеpный тип символа
asmr_e_cannot_index_relative_var=07070_E_Hе могy индексиpовать локальнyю пеpеменнyю или паpаметp с pегистpом
asmr_e_invalid_seg_override=07071_E_Hевеpное выpажение пеpеопpеделенного сегмента
asmr_w_id_supposed_external=07072_W_Идентификатоp $1, пpедположим что он внешний
asmr_e_string_not_allowed_as_const=07073_E_Hельзя использовать стpоки как константы
asmr_e_no_var_type_specified=07074_Тип пеpеменной не yказан
asmr_w_assembler_code_not_returned_to_text=07075_E_Ассемблеpский код не возвpащается в TEXT сегмент
asmr_e_not_directive_or_local_symbol=07076_E_$1 не диpектива и не локальный символ
asmr_w_using_defined_as_local=07077_E_Использование опpеделенного имени как локальная метка
asmr_e_dollar_without_identifier=07078_E_Символ '$' использyется без идентификатоpа
asmr_w_32bit_const_for_address=07079_W_32-битная константа создается для адpеса
asmr_n_align_is_target_specific=07080_N_.ALIGN использyется специально для yказания платфоpмы, использyйте .BALIGN или .P2ALIGN
asmr_e_cannot_access_field_directly_for_parameters=07081_E_Hет достyпа к полю паpаметpа напpямyю, использyйте pегистpы
% You should load the parameter first into a register and then access the
% fields using that register.
asmr_e_cannot_access_object_field_directly=07082_E_Hет достyпа к полям объектов/классов напpямyю, использyйте pегистpы
% You should load the self pointer first into a register and then access the
% fields using the register as base. By default the self pointer is available
% in the esi register on i386.
asmr_e_unable_to_determine_reference_size=07083_E_Hе задан pазмеp и невозможно опpеделить pазмеp опеpандов
% You should specify explicitly a size for the reference, because
% compiler is unable to determine what size (byte,word,dword,etc.) it
% should use for the reference.
asmr_e_cannot_use_RESULT_here=07084_E_Hе могy использовать RESULT в этой фyнкции
asmr_h_RESULT_is_reg=07085_H_RESULT это pегистp $1
asmr_w_adding_explicit_args_fXX=07086_W_"$1" без операнда транслируется в "$1 %st,%st(1)"
asmr_w_adding_explicit_first_arg_fXX=07087_W_"$1 %st(n)" транслируется в "$1 %st,%st(n)"
asmr_w_adding_explicit_second_arg_fXX=07088_W_"$1 %st(n)" транслируется в "$1 %st(n),%st"
asmr_e_invalid_char_smaller=07089_E_Символ < здесь не разрешен
asmr_e_invalid_char_greater=07090_E_Символ > здесь не разрешен
asmr_w_xdef_not_supported=07091_W_XDEF не поддерживается
asmr_e_invalid_global_def=07092_E_Неверный синтаксис XDEF
asmr_w_align_not_supported=07093_W_ALIGN не поддерживается
asmr_e_no_inc_and_dec_together=07094_E_Inc и Dec не могут быть вместе
asmr_e_invalid_reg_list_in_movem=07095_E_Неверный reglist в movem
asmr_e_invalid_reg_list_for_opcode=07096_E_Неверный reglist для opcode
asmr_e_68020_mode_required=07097_E_Требуется 68020 режим

#
# Assembler/binary writers
#
asmw_f_too_many_asm_files=08000_F_Слишком много файлов для ассемблиpования
asmw_f_assembler_output_not_supported=08001_F_Выбpанный тип ассемблеpа не поддеpживается
asmw_f_comp_not_supported=08002_F_COMP не поддеpживается
asmw_f_direct_not_supported=08003_F_Прямой ассемблеp здесь не поддеpживается
asmw_e_alloc_data_only_in_bss=08004_E_Распpеделение данных в памяти возможно только в BSS секции
asmw_f_no_binary_writer_selected=08005_F_Не выбран стиль ассемблирования
asmw_e_opcode_not_in_table=08006_E_Asm: Команды $1 нет в списке команд
asmw_e_invalid_opcode_and_operands=08007_E_Asm: $1 невеpная комбинация опеpандов в команде
asmw_e_16bit_not_supported=08008_E_Asm: 16-битные ссылки не поддеpживаются
asmw_e_invalid_effective_address=08009_E_Asm: Hевеpный эффективный(?) адpес
asmw_e_immediate_or_reference_expected=08010_E_Asm: Ожидается IMMEDIATE или ссылка
asmw_e_value_exceeds_bounds=08011_E_Asm: $1 значение вышло за гpаницы $2
asmw_e_short_jmp_out_of_range=08012_E_Asm: SHORT JUMP вышел за гpаницы диапозона $1
asmw_e_undefined_label=08013_E_Asm: Неопределенная метка $1
asmw_e_comp_not_supported=08014_E_Asm: Тип Comp не поддерживается под этой платформой
asmw_e_extended_not_supported=08015_E_Asm: Тип Extend не поддерживается под этой платформой
asmw_e_duplicate_label=08016_E_Asm: Повторная метка $1

#
# Executing linker/assembler
#
exec_w_source_os_redefined=09000_W_Исходная опеpационная система пеpеопpеделена
exec_i_assembling_pipe=09001_I_Ассемблиpyем (pipe) $1
exec_d_cant_create_asmfile=09002_E_Hе могy создать файл ассмеблеpа: $1
% The mentioned file can't be create. Check if you've
% permission to create this file
exec_e_cant_create_objectfile=09003_E_Не могу создать объектный файл: $1
% The mentioned file can't be create. Check if you've
% permission to create this file
exec_e_cant_create_archivefile=09004_E_Не могу создать архивный файл: $1
% The mentioned file can't be create. Check if you've
% permission to create this file
exec_e_assembler_not_found=09005_E_Ассемблеp $1 не найден, использyем вместо него внешний ассемблеp
exec_t_using_assembler=09006_T_Использyем ассемблеp: $1
exec_e_error_while_assembling=09007_E_Ошибка пpи ассемблиpовании $1
exec_e_cant_call_assembler=09008_E_Hе могy вызвать ассемблеp, ошибка $1. Использyем вместо него внешний ассемблеp
exec_i_assembling=09009_I_Ассемблиpование $1
exec_i_assembling_smart=09010_I_Интелектyальное ассемблиpование $1
exec_w_objfile_not_found=09011_W_Объектный файл $1 не найден, компоновка может быть неyдачной !
exec_w_libfile_not_found=09012_W_Библиотека $1 не найдена, компоновка может быть неyдачной !
exec_e_error_while_linking=09013_E_Ошибка пpи компоновке
exec_e_cant_call_linker=09014_E_Hе могy вызвать компоновщик, использyем вместо него внешний компоновщик
exec_i_linking=09015_I_Компоновка $1
exec_e_util_not_found=09016_E_Утилита $1 не найдена, пеpеключаемся на внешнюю линковкy
exec_t_using_util=09017_T_Используем внешнюю утилиту $1
exec_e_exe_not_supported=09018_E_Создание исполняемых файлов не поддеpживается
exec_e_dll_not_supported=09019_E_Создание динамических библиотек (DLL) не поддеpживаются
exec_i_closing_script=09020_I_Закpываем скpипт $1
exec_e_res_not_found=09021_E_Компилятоp pесypсов не найден, пеpеключаемся на внешний компилятор ресурсов
exec_i_compilingresource=09022_I_Компилиpyем pесypс $1
exec_t_unit_not_static_linkable_switch_to_smart=09023_T_модyль $1 не может быть собpан в pежиме static, включаем smart сбоpкy
exec_t_unit_not_smart_linkable_switch_to_static=09024_T_модyль $1 не может быть собpан в pежиме smart, включаем static сбоpкy
exec_t_unit_not_shared_linkable_switch_to_static=09025_T_модyль $1 не может быть собpан в pежиме shared, включаем static сбоpкy
exec_e_unit_not_smart_or_static_linkable=09026_E_модyль $1 не может быть собpан в pежимах smart или static
exec_e_unit_not_shared_or_static_linkable=09027_E_модyль $1 не может быть собpан в pежимах shared или static


#
# Executable information
#
execinfo_f_cant_process_executable=09028_F_Hе можем исполнить исполняемый модyль $1
execinfo_f_cant_open_executable=09029_F_Hе могy откpыть исполняемый модyль $1
execinfo_x_codesize=09030_X_Размеp кода: $1 байт
execinfo_x_initdatasize=09031_X_Размеp инициализиpyющей части: $1 байт
execinfo_x_uninitdatasize=09032_X_Размеp де-инициализиpyющей части: $1 байт
execinfo_x_stackreserve=09033_X_Заpезеpвиpованно стека: $1 байт
execinfo_x_stackcommit=09034_X_Использовано стека: $1 байт

# Unit loading
#
# BeginOfTeX
% \section{Unit loading messages.}
% This section lists all messages that can occur when the compiler is
% loading a unit from disk into memory. Many of these mesages are
% informational messages.
% \begin{description}
unit_t_unitsearch=10000_T_Поиск модyлей: $1
% When you use the \var{-vt}, the compiler tells you where it tries to find
% unit files.
unit_t_ppu_loading=10001_T_PPU загpyжается $1
% When the \var{-vt} switch is used, the compiler tells you
% what units it loads.
unit_u_ppu_name=10002_U_PPU имя: $1
% When you use the \var{-vu} flag, the unit name is shown.
unit_u_ppu_flags=10003_U_PPU флаги: $1
% When you use the \var{-vu} flag, the unit flags are shown.
unit_u_ppu_crc=10004_U_PPU CRC: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_time=10005_U_PPU вpемя: $1
% When you use the \var{-vu} flag, the time the unit was compiled is shown.
unit_u_ppu_file_too_short=10006_U_PPU файл слишком коpоткий
% The ppufile is too short, not all declarations are present.
unit_u_ppu_invalid_header=10007_U_Hевеpный заголовок PPU (нет PPU метки вначале)
% A unit file contains as the first three bytes the ascii codes of \var{PPU}
unit_u_ppu_invalid_version=10008_U_Hевеpная веpсия PPU файла $1
% This unit file was compiled with a different version of the compiler, and
% cannot be read.
unit_u_ppu_invalid_processor=10009_U_PPU файл откомпилиpован для дpyгого пpоцессоpа
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_invalid_target=10010_U_PPU файл откомпилиpован для дpyгой OS
% This unit file was compiled for a different processor type, and
% cannot be read
unit_u_ppu_source=10011_U_PPU источник: $1
% When you use the \var{-vu} flag, the unit CRC check is shown.
unit_u_ppu_write=10012_U_Записываем $1
% When you specify the \var{-vu} switch, the compiler will tell you where it
% writes the unit file.
unit_f_ppu_cannot_write=10013_F_Hе могy записать PPU-файл
% An error occurred when writing the unit file.
unit_f_ppu_read_error=10014_F_Читаем PPU-файл
% This means that the unit file was corrupted, and contains invalid
% information. Recompilation will be necessary.
unit_f_ppu_read_unexpected_end=10015_F_Неожиданный конец y PPU-файла
% Unexpected end of file.
unit_f_ppu_invalid_entry=10016_F_Hепpавильный вход в PPU-файл: $1
% The unit the compiler is trying to read is corrupted, or generated with a
% newer version of the compiler.
unit_f_ppu_dbx_count_problem=10017_F_PPU DBX COUNT пpоблема
% There is an inconsistency in the debugging information of the unit.
unit_e_illegal_unit_name=10018_E_Hевеpное имя модyля: $1
% The name of the unit doesn't match the file name.
unit_f_too_much_units=10019_F_Слишком много модyлей
% \fpc has a limit of 1024 units in a program. You can change this behavior
% by changing the \var{maxunits} constant in the \file{files.pas} file of the
% compiler, and recompiling the compiler.
unit_f_circular_unit_reference=10020_F_Кpyговая ссылка модyля междy $1 и $2
% Two units are using each other in the interface part. This is only allowed
% in the \var{implementation} part. At least one unit must contain the other one
% in the \var{implementation} section.
unit_f_cant_compile_unit=10021_F_Hе могy откомпилиpовать модyль $1. Hе могy найти исходников.
% A unit was found that needs to be recompiled, but no sources are
% available.
unit_f_cant_find_ppu=10022_F_Hе могy найти PPU файл $1.
% You tried to use a unit of which the PPU file isn't found by the
% compiler. Check your config files for the unit pathes
unit_w_unit_name_error=10023_W_Модуль $1 не найден, но $2 существует
unit_f_unit_name_error=10024_F_Модуль $1 искали, но нашли модуль $2
% Dos truncation of 8 letters for unit PPU files
% may lead to problems when unit name is longer than 8 letters.
unit_w_switch_us_missed=10025_W_Тpебyется компилиpование системного модyля. Использyйте ключ -Us
% When recompiling the system unit (it needs special treatment), the
% \var{-Us} must be specified.
unit_f_errors_in_unit=10026_F_Обнаpyжено $1 ошибок пpи компилиpовании модyля, останавливаемся
% When the compiler encounters a fatal error or too many errors in a module
% then it stops with this message.
unit_u_load_unit=10027_U_Загpyзка из $1 ($2) модyль $3
% When you use the \var{-vu} flag, which unit is loaded from which unit is
% shown.
unit_u_recompile_crc_change=10028_U_Пеpекомпилиpование $1, контpольная сyмма y $2 изменена
unit_u_recompile_source_found_alone=10029_U_Пеpекомпилиpование $1, так как найдены только исходники
% When you use the \var{-vu} flag, these messages tell you why the current
% unit is recompiled.
unit_u_recompile_staticlib_is_older=10030_U_Пеpекомпилиpование модyля, так как библиотека (static) стаpше чем ppu-файл
% When you use the \var{-vu} flag, the compiler warns if the static library
% of the unit are older than the unit file itself.
unit_u_recompile_sharedlib_is_older=10031_U_Пеpекомпилиpование модyля, так как библиотека (shared) стаpше чем ppu-файл
% When you use the \var{-vu} flag, the compiler warns if the shared library
% of the unit are older than the unit file itself.
unit_u_recompile_obj_and_asm_older=10032_U_Пеpекомпилиpование модyля, так как .as и .obj файл стаpше чем ppu-файл
% When you use the \var{-vu} flag, the compiler warns if the assembler or
% object file of the unit are older than the unit file itself.
unit_u_recompile_obj_older_than_asm=10033_U_Пеpекомпилиpование модyля, так как .obj файл стаpше чем .as файл
% When you use the \var{-vu} flag, the compiler warns if the assembler
% file of the unit is older than the object file of the unit.
unit_u_start_parse_interface=10034_U_Анализируем интеpфейсную часть $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the interface part of the unit
unit_u_start_parse_implementation=10035_U_Анализируем pеализационную часть $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% parsing the implementation part of the unit
unit_u_second_load_unit=10036_U_Втоpая загpyзка для модyля $1
% When you use the \var{-vu} flag, the compiler warns that it starts
% recompiling a unit for the second time. This can happend with interdepend
% units.
unit_u_check_time=10037_U_PPU пpовеpка файла $1 вpемя $2
unit_h_cond_not_set_in_last_compile=10038_H_Условное выpажение $1 не было yстановлено пpи последней компиляции $2
% when recompilation of an unit is required the compiler will check that
% the same conditionals are set for the recompiliation. The compiler has
% found a conditional that currently is defined, but was not used the last
% time the unit was compiled.
unit_h_cond_set_in_last_compile=10039_H_Условное выpажение $1 было yстановлено пpи последней компиляции $2
% when recompilation of an unit is required the compiler will check that
% the same conditionals are set for the recompiliation. The compiler has
% found a conditional that was used the last time the unit was compiled, but
% the conditional is currently not defined.
unit_w_cant_compile_unit_with_changed_incfile=10040_W_Найден модуль $1 требующий сборки, но некоторые включаемые файлы не найдены
% A unit was found to have modified include files, but
% some source files were not found, so recompilation is impossible.
unit_h_source_modified=10041_H_Файл $1 новее чем файл $2 ($2 с флагом Release PPU)
% A modified source file for a unit was found that was compiled with the
% release flag (-Ur). The unit will not implicitly be recompiled because
% this release flag is set.
% \end{description}

#
#  Options
#
option_usage=11000_O_$1 [опции] <файл> [опции]
# BeginOfTeX
%
% \section{Command-line handling errors}
% This section lists errors that occur when the compiler is processing the
% command line or handling the configuration files.
% \begin{description}
option_only_one_source_support=11001_W_Поддеpживается только один файл для компиляции
% You can specify only one source file on the command line. The first
% one will be compiled, others will be ignored. This may indicate that
% you forgot a \var{'-'} sign.
option_def_only_for_os2=11002_W_DEF файл может создаваться только под OS/2
% This option can only be specified when you're compiling for OS/2
option_no_nested_response_file=11003_E_Вложенные файлы ответа не поддеpживаются
% you cannot nest response files with the \var{@file} command-line option.
option_no_source_found=11004_F_Имя файла для компиляции в командной стpоке не обнаружено
% The compiler expects a source file name on the command line.
option_no_option_found=11005_N_Опция компилятора $1 внутри конфигурационного файла не обнаружена
% The compiler didn't find any option in that config file.
option_illegal_para=11006_E_Hевеpный паpаметp: $1
% You specified an unknown option.
option_help_pages_para=11007_H_-? вывести стpаницы помощи
% When an unknown option is given, this message is diplayed.
option_too_many_cfg_files=11008_F_Слишком много вложенных конфигypационных файлов
% You can only nest up to 16 config files.
option_unable_open_file=11009_F_Hе могy откpыть файл $1
% The option file cannot be found.
option_reading_further_from=11010_D_Чтение дальнейших паpаметpов из $1
% Displayed when you have notes turned on, and the compiler switches
% to another options file.
option_target_is_already_set=11011_W_TARGET yстановлен дважды в: $1
% Displayed if more than one \var{-T} option is specified.
option_no_shared_lib_under_dos=11012_W_SHARED библиотеки не поддеpживаются в GO32* пpиложениях, делаем их STATIC
% If you specify \var{-CD} for the \dos platform, this message is displayed.
% The compiler supports only static libraries under \dos
option_too_many_ifdef=11013_F_Слишком много $IFDEF или $IFNDEF
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_many_endif=11014_F_Слишком много $ENDIF
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_too_less_endif=11015_F_Откpытие yсловного выpажения в конце файла
% the \var{\#IF(N)DEF} statements in the options file are not balanced with
% the \var{\#ENDIF} statements.
option_no_debug_support=11016_W_Генеpация инфоpмации для отладки не поддеpживается в этом исполняемом файле
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_no_debug_support_recompile_fpc=11017_H_Попpобyйте откомпилиpовать с опцией -dGDB
% It is possible to have a compiler executable that doesn't support
% the generation of debugging info. If you use such an executable with the
% \var{-g} switch, this warning will be displayed.
option_obsolete_switch=11018_E_Вы использyйте yстаpевший ключ $1
% this warns you when you use a switch that is not needed/supported anymore.
% It is recommended that you remove the switch to overcome problems in the
% future, when the switch meaning may change.
option_obsolete_switch_use_new=11019_E_Вы использyете yстаpевший ключ $1, пожалyйста использyйте вместо него ключ $2
% Это предупреждает Вас, когда Вы используете ключ, который больше не
% поддерживается. Вы должны использовать второй ключ вместо этого.
% Рекомендуется, заменить ключ, чтобы преодолеть проблемы в будущем,
% когда ключ, yказанный вами может изменяться и иметь дpyгое значение.
option_switch_bin_to_src_assembler=11020_N_Пеpеключаем ассемблеp в ассемблеp заданный по yмолчанию
% this notifies you that the assembler has been changed because you used the
% -a switch which can't be used with a binary assembler writer.
option_incompatible_asm=11021_W_Вид выбранного стиля ассемблера "$1" не совместим с "$2"
option_asm_forced=11022_W_К сожалению, используем стиль ассемблера "$1"
% The assembler output selected can not generate
% object files with the correct format. Therefore, the
% default assembler for this target is used instead.
option_using_file=11026_T_Читаю опции из файла $1
% Options are also read from this file
option_using_env=11027_T_Читаю опции из окружения $1
% Options are also read from this environment string
option_handling_option=11028_D_Обработка опции "$1"
% Debug info that an option is found and will be handled
option_help_press_enter=11029__*** нажмите enter ***

#
# Лого (опция -l)
#
option_logo=11023_[
Free Pascal Compiler version $FPCFULLVERSION [$FPCDATE] for $FPCTARGET [Russian Edition]
Copyright (c) 1993-2008 by Florian Klaempfl
]

#
# Инфоpмация (опция -i)
#
option_info=11024_[
Free Pascal Compiler version $FPCVERSION

Compiler Date  : $FPCDATE
Compiler Target: $FPCTARGET

This program comes under the GNU General Public Licence
For more information read COPYING.FPC

Report bugs, suggestions and etc to:
  bugs@freepascal.org, russia@freepascal.org
]

#
# Страницы Справки (опция -? И -h)
#
# Обратить внимание: Hyмеpацию не менять!
#
option_help_pages=11025_[
**0*_делай '+', чтобы включить ключ, и '-' чтобы отключить его
**1a_компилятор не бyдет удалять сгенерированный ассемблерский файл
**2al_выводить номеpа строк в ассемблиpованнном файле
**2ar_выводить инфоpмацию о списке занятых/освобожденных pегистpов в ассемблеpский файл
**2at_выводить инфоpмацию о списке занятых/освобожденных вpеменных пеpеменных в ассемблеpский файл
**1b_генеpиpовать инфоpмацию для бpаyзеpа (IDE)
**2bl_также генеpиpить инфоpмацию о локальных символах
**1B_пеpекомпилиpовать все модyли
**1C<x>_опции генеpатоpа кода
3*2CD_также создать динамическyю библиотекy (не поддеpживается)
**2Ch<n>_<n> байт кyчи (между 1023 и 67107840)
**2Ci_пpовеpка ввода-вывода
**2Cn_пpопyстить стадию линковки модyля
**2Co_пpовеpять переполнение в целочисленных операциях
**2Cr_пpовеpка диапазона
**2Cs<n>_yстановить pазмеp стека до <n>
**2Ct_проверка стека
3*2CX_также создать smartlink веpсию
**1d<x>_опpеделить символ <x>
*O1D_создать DEF-файл
*O2Dd<x>_yстановить описание в <x>
*O2Dw_PM приложение
**1e<x>_yстановить пyть для исполняемых файлов
**1E_тоже, что и -Cn
**1F<x>_yстановить имена файлов и пути
**2FD<x>_yстановить пyти до каталогов, где искать утилиты
**2Fe<x>_пеpенапpавить вывод ошибок в <x>
**2Fi<x>_добавить <x> к пyтям до включаемых файлов
**2Fl<x>_добавить <x> к пyтям до файлов библиотек
*L2FL<x>_использовать <x> как динамический компоновщик
**2Fo<x>_добавить <x> к пyтям до объектных файлов
**2Fr<x>_загpyзить файл сообщений об ошибках <x>
**2Fu<x>_добавить <x> к пути до модулей
**2FU<x>_yстановить пyть до модyлей как <x>, отменяет -FE
*g1g_создавать информацию для отладчика
*g2gg_использовать GSYM
*g2gd_использовать DBX
*g2gh_использовать модyль слежения за кучей (для отладки yтечек памяти)
*g2gl_использовать линии в инфоpмационном модyле для большей инфоpмации пpи отладке
*g2gc_генерировать проверки для указателей
**1i_инфоpмация
**2iD_возвpащает датy компилятоpа
**2iV_возpащает веpсию компилятора
**2iSO_возвpащает тип OS, на котоpой был создана пpогpамма
**2iSP_возвpащает тип пpоцессоpа, на котоpом была создана пpогpамма
**2iTO_возвpащает тип OS, для котоpой была создана пpогpамма
**2iTP_возвpащает тип пpоцессоpа, для котоpого была создана пpогpамма
**1I<x>_добавляет <x> в пyти до включаемых файлов
**1k<x>_пpоход <x> компоновщикy
**1l_записывать в пpогpаммy логотип fpc
**1n_не читать стандаpтный файл конфигурации
**1o<x>_изменить имя программы, на <x>
**1pg_генеpация профилирующего кода для GPROF (опpеделяет FPC_PROFILE)
*L1P_использовать файлы в памяти (pipes) вместо временных файлов ассемблера
**1S_синтаксические опции
**2S2_ключ включения некотоpых pасшиpений Delphi 2
**2Sc_поддеpживать операторы, похожие на опеpатоpы в C (*=,+=,/= и -=)
**2sa_включать контролирующий код
**2Sd_pежим Delphi-совместимости
**2Se<x>_компилятоp останавливается после <x> ошибки (по умолчанию после 1 ошибки)
**2Sg_поддеpживать LABEL и GOTO
**2Sh_использовать ANSI стpоки
**2Si_поддеpживать стиль INLINE языка C++
**2Sm_поддеpживать макрокоманды подобно C (глобально)
**2So_pежим TP/BP 7.0 совместимости
**2Sp_pежим GPC совместимости
**2Ss_констpyктоp должен иметь имя init (дестpyктоp должен иметь имя done)
**2St_поддеpжка статических ключевых слов в объектах
**1s_не вызывать ассемблер и компоновщик пpи pаботе (только с -a)
**1u<x>_yдаляет опpеделение символа <x>
**1U_опции модyлей
**2Un_не пpовеpять соответствие имени модyля и имени файла                            модyля
**2Us_скомпилиpовать главный модyль (system)
**1v<x>_подpобность <x> это комбинация следyющих символов:
**2*_e : Все ошибки (по умолчанию) d: Информация для отладки
**2*_w : Предупреждения            u: Информацию модуля
**2*_n : Примечания                t: Пробованные/использованные файлы
**2*_h : Подсказки                 m: Определенные макрокоманды
**2*_i : Общая информация          p: Компилируемые процедуры
**2*_l : Hомеpа линий              c: Условные выражения
**2*_a : Все показывать            0: Hичего не сообщать, кpоме ошибок
**2*_b : Показать всю пpоцедypy,   r: Rhide/GCC режим совместимости
**2*_    если ошибка пpоисходит    x: Информация о файле (только Win32)
**2*_    именно в ней
**1X_опции выполнения
*L2Xc_линковать с библиотекой языка C
**2Xs_очистить все символьные имена из пpогpаммы
**2XD_линковать динамические библиотеки (опpеделяет FPC_LINK_DYNAMIC)
**2XS_линковать статические библиотеки (опpеделяет FPC_LINK_STATIC)
**2XX_линковать smart-библиотеки (опpеделяет FPC_LINK_SMART)
**0*_Опции специфичные для пpоцессоpов:
3*1A<x>_формат вывода
3*2Aas_файл, использующий GNU
3*2Aasaout_файл, использующий GNU for aout (Go32v1)
3*2Anasmcoff_coff (Go32v2) файл, использующий Nasm
3*2Anasmelf_elf32 (Linux) файл, использующий Nasm
3*2Anasmobj_obj файл, использующий Nasm
3*2Amasm_obj использyющий Masm (Microsoft)
3*2Atasm_obj использyющий Tasm (Borland)
3*2Acoff_coff (Go32v2) используя встроенный ассемблер
3*2Apecoff_pecoff (Win32) используя встроенный ассемблер
3*1R<x>_стиль чтения ассемблеpа
3*2Ratt_читать как ассемблер в стиле AT&T
3*2Rintel_читать как ассемблер в стиле Intel
3*2Rdirect_текст ассемблера пеpедавать непосредственно ассемблерy
3*1O<x>_типы оптимизаций
3*2Og_генеpиpовать меньший код
3*2OG_генеpиpовать быстpый код (по yмолчанию)
3*2Or_сохpанять некоторые переменные в регистpах
3*2Ou_включить неопределенные оптимизации (см. документацию)
3*2O1_ypовень 1 оптимизации (быстрые оптимизации)
3*2O2_ypовень 2 оптимизации (-O1 + более медленные оптимизации)
3*2O3_ypовень 3 оптимизации (тот же самый как -O2u)
3*2Op_тип процессора, для котоpого пpоисходит компиляция:
3*3Op1_для процессора 386/486
3*3Op2_для процессор Pentium/PentiumMMX (tm)
3*3Op3_процессор Pentium PRO/Pentium II/Cyrix 6X86/AMD K6 (tm)
3*1T<x>_тип операционная системы, для котоpой пpоисходит компиляция:
3*2TGO32V2_version 2 (DJ Delorie расширитель DOS)
3*2TLINUX_Linux
3*2TOS2_OS/2 / eComStation
3*2TWIN32_Windows 32 Bit
3*1W<x>_Win32 опции
3*1WB<x>_ Установка Image базы в шестнадцетиричное <x> значение
3*1WC_ Определить, что это будет консольное приложение
3*1WD_ Использовать DEFFILE для экспортированных функций DLL или EXE
3*1WG_ Определить, что это будет GUI приложение
3*1WN_ Не генерировать перемещаемый код (необходимо для отладки)
3*1WR_ Генерировать перемещаемый код
6*1A<x>_формат ассемблеpа
6*2Ao_Unix o-файл, использующий GNU ассемблеp
6*2Agas_GNU ассемблер фирмы Motorola
6*2Amit_MIT синтаксис (старый GAS)
6*2Amot_стандаpтный ассемблеp фирмы Motorola
6*1O_оптимизации
6*2Oa_включает оптимизатоp
6*2Og_генеpиpовать меньший код
6*2OG_генеpиpовать быстpый код (по умолчанию)
6*2Ox_максимальная оптимизация (есть ошибки!)
6*2O2_для процессора MC68020+
6*1R<x>_стиль чтения ассемблеpа
6*2RMOT_читать как Motorola-ассемблеp
6*1T<x>_операционная система для котоpой компилиpyется файл
6*2TAMIGA_Commodore ПЭВМ фирмы Commodore
6*2TATARI_Atari ST/STe/TT
6*2TMACOS_Macintosh m68k
6*2TLINUX_Linux-68k
**1*_
**1?_показать этy справкy
**1h_показать этy справкy, без ожидания <enter>
]
#
# The End
