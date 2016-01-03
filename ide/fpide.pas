{
    This file is part of the Free Pascal Integrated Development Environment
    Copyright (c) 1998 by Berczi Gabor

    Main IDEApp object

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit fpide;

{2.0 compatibility}
{$ifdef VER2_0}
  {$macro on}
  {$define resourcestring := const}
{$endif}

interface

{$i globdir.inc}

uses
  Objects,Drivers,Views,App,Gadgets,MsgBox,Tabs,
  WEditor,WCEdit,
  Comphook,Browcol,
  WHTMLScn,
  FPViews,FPSymbol
  {$ifndef NODEBUG}
  ,fpevalw
  {$endif};

type
    TExecType = (exNormal,exNoSwap,exDosShell);
    Tdisplaymode = (dmIDE,dmUser);

    TIDEApp = object(TApplication)
      IsRunning : boolean;
      displaymode : Tdisplaymode;
      constructor Init;
      procedure   InitDesktop; virtual;
      procedure   LoadMenuBar;
      procedure   InitMenuBar; virtual;
      procedure   reload_menubar;
      procedure   InitStatusLine; virtual;
      procedure   Open(FileName: string;FileDir:string);
      function    OpenSearch(FileName: string) : boolean;
      function    AskSaveAll: boolean;
      function    SaveAll: boolean;
      function    AutoSave: boolean;
      procedure   Idle; virtual;
      procedure   Update;
      procedure   UpdateMode;
      procedure   UpdateRunMenu(DebuggeeRunning : boolean);
      procedure   UpdateTarget;
      procedure   GetEvent(var Event: TEvent); virtual;
      procedure   HandleEvent(var Event: TEvent); virtual;
      procedure   GetTileRect(var R: TRect); virtual;
      function    GetPalette: PPalette; virtual;
      procedure   DosShell; {virtual;}
      procedure   ShowReadme;
      destructor  Done; virtual;
      procedure   ShowUserScreen;
      procedure   ShowIDEScreen;
      function    IsClosing : boolean;
    private
      procedure NewEditor;
      procedure NewFromTemplate;
      procedure OpenRecentFile(RecentIndex: integer);
      procedure ChangeDir;
      procedure Print;
      procedure PrinterSetup;
      procedure ShowClipboard;
      procedure FindProcedure;
      procedure Objects;
      procedure Modules;
      procedure Globals;
      procedure SearchSymbol;
      procedure RunDir;
      procedure Parameters;
      procedure DoStepOver;
      procedure DoTraceInto;
      procedure DoRun;
      procedure DoResetDebugger;
      procedure DoContToCursor;
      procedure DoContUntilReturn;
      procedure Target;
      procedure DoCompilerMessages;
      procedure DoPrimaryFile;
      procedure DoClearPrimary;
      procedure DoUserScreenWindow;
      procedure DoCloseUserScreenWindow;
      procedure DoUserScreen;
      procedure DoOpenGDBWindow;
      procedure DoToggleBreak;
      procedure DoShowCallStack;
      procedure DoShowDisassembly;
      procedure DoShowBreakpointList;
      procedure DoShowWatches;
      procedure DoAddWatch;
      procedure do_evaluate;
      procedure DoShowRegisters;
      procedure DoShowFPU;
      procedure DoShowVector;
      function  AskRecompileIfModified:boolean;
      procedure Messages;
      procedure Calculator;
      procedure DoAsciiTable;
      procedure ExecuteTool(Idx: integer);
      procedure SetSwitchesMode;
      procedure DoCompilerSwitch;
      procedure MemorySizes;
      procedure DoLinkerSwitch;
      procedure DoDebuggerSwitch;
{$ifdef SUPPORT_REMOTE}
      procedure DoRemote;
      procedure TransferRemote;
{$endif SUPPORT_REMOTE}
      procedure Directories;
      procedure Tools;
      procedure DoGrep;
      procedure Preferences;
      procedure EditorOptions(Editor: PEditor);
      procedure CodeComplete;
      procedure CodeTemplates;
      procedure BrowserOptions(Browser: PBrowserWindow);
      procedure DesktopOptions;
      procedure ResizeApplication(x, y : longint);
      procedure Mouse;
      procedure StartUp;
      procedure Colors;
      procedure OpenINI;
      procedure SaveINI;
      procedure SaveAsINI;
      procedure CloseAll;
      procedure WindowList;
      procedure HelpContents;
      procedure HelpHelpIndex;
      procedure HelpTopicSearch;
      procedure HelpPrevTopic;
      procedure HelpUsingHelp;
      procedure HelpFiles;
      procedure About;
      procedure CreateAnsiFile;
    public
      procedure SourceWindowClosed;
      function  DoExecute(ProgramPath, Params, InFile, OutFile, ErrFile: string; ExecType: TExecType): boolean;
    private
      SaveCancelled: boolean;
      InsideDone : boolean;
      LastEvent: longint;
      procedure AddRecentFile(AFileName: string; CurX, CurY: sw_integer);
      function  SearchRecentFile(AFileName: string): integer;
      procedure RemoveRecentFile(Index: integer);
      procedure CurDirChanged;
      procedure UpdatePrimaryFile;
      procedure UpdateINIFile;
      procedure UpdateRecentFileList;
      procedure UpdateTools;
    end;

procedure PutEvent(TargetView: PView; E: TEvent);
procedure PutCommand(TargetView: PView; What, Command: Word; InfoPtr: Pointer);

var
  IDEApp: TIDEApp;

implementation

uses
{$ifdef HasSignal}
  fpcatch,
{$endif HasSignal}
{$ifdef WinClipSupported}
  WinClip,
{$endif WinClipSupported}
{$ifdef Unix}
  fpKeys,
{$endif Unix}
  FpDpAnsi,WConsts,
  Video,Mouse,Keyboard,
  Compiler,Version,
  FVConsts,
  Dos{,Memory},Menus,Dialogs,StdDlg,timeddlg,
  Systems,
  WUtils,WHlpView,WViews,WHTMLHlp,WHelp,WConsole,
  FPConst,FPVars,FPUtils,FPSwitch,FPIni,FPIntf,FPCompil,FPHelp,
  FPTemplt,FPCalc,FPUsrScr,FPTools,
{$ifndef NODEBUG}
  FPDebug,FPRegs,
{$endif}
  FPRedir,
  FPDesk,FPCodCmp,FPCodTmp;

type
   TTargetedEvent = record
     Target: PView;
     Event: TEvent;
   end;

const
     TargetedEventHead   : integer = 0;
     TargetedEventTail   : integer = 0;
var
     TargetedEvents      : array[0..10] of TTargetedEvent;

resourcestring  menu_local_gotosource = '~G~oto source';
                menu_local_tracksource = '~T~rack source';
                menu_local_options = '~O~ptions...';
                menu_local_clear = '~C~lear';
                menu_local_saveas = 'Save ~a~s';


{                menu_key_common_helpindex      = 'Shift+F1';
                menu_key_common_topicsearch    = 'Ctrl+F1';
                menu_key_common_prevtopic      = 'Alt+F1';}

                { menu entries }
                menu_file              = '~F~ile';
                menu_file_new          = '~N~ew';
                menu_file_template     = 'New from ~t~emplate...';
                menu_file_open         = '~O~pen...';
                menu_file_save         = '~S~ave';
                menu_file_saveas       = 'Save ~a~s...';
                menu_file_saveall      = 'Save a~l~l';
                menu_file_reload       = '~R~eload';
                menu_file_print        = '~P~rint';
                menu_file_printsetup   = 'Print s~e~tup';
                menu_file_changedir    = '~C~hange dir...';
                menu_file_dosshell     = 'Comman~d~ shell';
                menu_file_exit         = 'E~x~it';

                menu_edit              = '~E~dit';
                {$ifdef HASAMIGA}
                {$ifdef AROS}
                menu_edit_copywin      = 'Cop~y~ to AROS';
                menu_edit_pastewin     = 'Paste from A~R~OS';
                {$else}
                menu_edit_copywin      = 'Cop~y~ to System';
                menu_edit_pastewin     = 'Paste from Syste~m~';
                {$endif}
                {$else}
                menu_edit_copywin      = 'Cop~y~ to Windows';
                menu_edit_pastewin     = 'Paste from ~W~indows';
                {$endif}
                menu_edit_undo         = '~U~ndo';
                menu_edit_redo         = '~R~edo';
                menu_edit_cut          = 'Cu~t~';
                menu_edit_copy         = '~C~opy';
                menu_edit_paste        = '~P~aste';
                menu_edit_clear        = 'C~l~ear';
                menu_edit_showclipboard= '~S~how clipboard';
                menu_edit_selectall    = 'Select ~A~ll';
                menu_edit_unselect     = 'U~n~select';

                menu_search            = '~S~earch';
                menu_search_find       = '~F~ind...';
                menu_search_replace    = '~R~eplace...';
                menu_search_searchagain= '~S~earch again';
                menu_search_jumpline   = '~G~o to line number...';
                menu_search_findproc   = 'Find ~p~rocedure...';
                menu_search_objects    = '~O~bjects';
                menu_search_modules    = 'Mod~u~les';
                menu_search_globals    = 'G~l~obals';
                menu_search_symbol     = 'S~y~mbol...';

                menu_run               = '~R~un';
                menu_run_run           = '~R~un';
                menu_run_continue      = '~C~ontinue';
                menu_run_stepover      = '~S~tep over';
                menu_run_traceinto     = '~T~race into';
                menu_run_conttocursor  = '~G~oto Cursor';
                menu_run_untilreturn   = '~U~ntil return';
                menu_run_rundir        = 'Run ~D~irectory...';
                menu_run_parameters    = 'P~a~rameters...';
                menu_run_resetdebugger = '~P~rogram reset';

                menu_compile           = '~C~ompile';
                menu_compile_compile   = '~C~ompile';
                menu_compile_make      = '~M~ake';
                menu_compile_build     = '~B~uild';
                menu_compile_target    = '~T~arget...';
                menu_compile_primaryfile = '~P~rimary file...';
                menu_compile_clearprimaryfile = 'C~l~ear primary file';
                menu_compile_information = '~I~nformation...';
                menu_compile_compilermessages = 'C~o~mpiler messages';

                menu_debug             = '~D~ebug';
                menu_debug_output      = '~O~utput';
                menu_debug_userscreen  = '~U~ser screen';
                menu_debug_breakpoint  = '~B~reakpoint';
                menu_debug_callstack   = '~C~all stack';
                menu_debug_remote      = '~S~end to remote';
                menu_debug_registers   = '~R~egisters';
                menu_debug_fpu_registers   = '~F~loating Point Unit';
                menu_debug_vector_registers   = '~V~ector Unit';
                menu_debug_addwatch    = '~A~dd Watch';
                menu_debug_watches     = '~W~atches';
                menu_debug_breakpointlist = 'Breakpoint ~L~ist';
                menu_debug_gdbwindow   = '~G~DB window';
                menu_debug_disassemble = '~D~isassemble';

                menu_tools             = '~T~ools';
                menu_tools_messages    = '~M~essages';
                menu_tools_msgnext     = 'Goto ~n~ext';
                menu_tools_msgprev     = 'Goto ~p~revious';
                menu_tools_grep        = '~G~rep';
                menu_tools_calculator  = '~C~alculator';
                menu_tools_asciitable  = 'Ascii ~t~able';

                menu_options           = '~O~ptions';
                menu_options_mode      = 'Mode~.~..';
                menu_options_compiler  = '~C~ompiler...';
                menu_options_memory    = '~M~emory sizes...';
                menu_options_linker    = '~L~inker...';
                menu_options_debugger  = 'De~b~ugger...';
                menu_options_remote    = '~R~emote...';
                menu_options_directories = '~D~irectories...';
                menu_options_browser   = 'Bro~w~ser...';
                menu_options_tools     = '~T~ools...';
                menu_options_env       = '~E~nvironment';
                menu_options_env_preferences = '~P~references...';
                menu_options_env_editor= '~E~ditor...';
                menu_options_env_codecomplete = 'Code~C~omplete...';
                menu_options_env_codetemplates = 'Code~T~emplates...';
                menu_options_env_desktop = '~D~esktop...';
                menu_options_env_keybmouse = 'Keyboard & ~m~ouse...';
                menu_options_env_startup = '~S~tartup...';
                menu_options_env_colors= '~C~olors';
                menu_options_learn_keys= 'Learn ~K~eys';
                menu_options_open      = '~O~pen...';
                menu_options_save      = '~S~ave';
                menu_options_saveas    = 'Save ~a~s...';

                menu_window            = '~W~indow';
                menu_window_tile       = '~T~ile';
                menu_window_cascade    = 'C~a~scade';
                menu_window_closeall   = 'Cl~o~se all';
                menu_window_resize     = '~S~ize/Move';
                menu_window_zoom       = '~Z~oom';
                menu_window_next       = '~N~ext';
                menu_window_previous   = '~P~revious';
                menu_window_hide       = '~H~ide';
                menu_window_close      = '~C~lose';
                menu_window_list       = '~L~ist...';
                menu_window_update     = '~R~efresh display';

                menu_help              = '~H~elp';
                menu_help_contents     = '~C~ontents';
                menu_help_index        = '~I~ndex';
                menu_help_topicsearch  = '~T~opic search';
                menu_help_prevtopic    = '~P~revious topic';
                menu_help_using        = '~U~sing help';
                menu_help_files        = '~F~iles...';
                menu_help_about        = '~A~bout...';

                { Source editor local menu items }
                menu_srclocal_openfileatcursor = 'Open ~f~ile at cursor';
                menu_srclocal_browseatcursor = '~B~rowse symbol at cursor';
                menu_srclocal_topicsearch  = 'Topic ~s~earch';
                menu_srclocal_options      = '~O~ptions...';
                menu_srclocal_reload       = '~R~eload modified file';
                { Help viewer local menu items }
                menu_hlplocal_contents     = '~C~ontents';
                menu_hlplocal_index        = '~I~ndex';
                menu_hlplocal_topicsearch  = '~T~opic search';
                menu_hlplocal_prevtopic    = '~P~revious topic';
                menu_hlplocal_copy         = '~C~opy';

                { Messages local menu items }
                menu_msglocal_clear        = '~C~lear';
                menu_msglocal_gotosource   = '~G~oto source';
                menu_msglocal_tracksource  = '~T~rack source';
{                menu_msglocal_saveas = menu_local_saveas;}

                { short cut entries in menu }
                menu_key_file_open     = 'F3';
                menu_key_file_save     = 'F2';
                menu_key_file_exit     = 'Alt+X';

                menu_key_edit_undo     = 'Alt+BkSp';
                menu_key_edit_cut_borland      = 'Shift+Del';
                menu_key_edit_copy_borland     = menu_key_common_copy_borland;
                menu_key_edit_paste_borland    = 'Shift+Ins';
                menu_key_edit_cut_microsoft    = 'Ctrl+X';
                menu_key_edit_copy_microsoft   = menu_key_common_copy_microsoft;
                menu_key_edit_paste_microsoft  = 'Ctrl+V';
                menu_key_edit_clear    = 'Ctrl+Del';
                menu_key_edit_all_microsoft = 'Ctrl+A';
                menu_key_edit_all_borland = '';

                menu_key_run_run       = 'Ctrl+F9';
                menu_key_run_stepover  = 'F8';
                menu_key_run_traceinto = 'F7';
                menu_key_run_conttocursor = 'F4';
                menu_key_run_untilreturn= 'Alt+F4';
                menu_key_run_resetdebugger = 'Ctrl+F2';

                menu_key_compile_compile = 'Alt+F9';
                menu_key_compile_make = 'F9';
                menu_key_compile_compilermessages = 'F12';

                menu_key_debug_userscreen = 'Alt+F5';
                menu_key_debug_breakpoint = 'Ctrl+F8';
                menu_key_debug_callstack = 'Ctrl+F3';
                menu_key_debug_addwatch = 'Ctrl+F7';

                menu_key_tools_messages= 'F11';
                menu_key_tools_msgnext = 'Alt+F8';
                menu_key_tools_msgprev = 'Alt+F7';
                menu_key_tools_grep    = 'Shift+F2';

                menu_key_window_resize = 'Ctrl+F5';
                menu_key_window_zoom   = 'F5';
                menu_key_window_next   = 'F6';
                menu_key_window_previous = 'Shift+F6';
                menu_key_window_close  = 'Alt+F3';
                menu_key_window_list   = 'Alt+0';
                menu_key_window_hide   = 'Ctrl+F6';

                menu_key_help_helpindex = menu_key_common_helpindex;
                menu_key_help_topicsearch = menu_key_common_topicsearch;
                menu_key_help_prevtopic= menu_key_common_prevtopic;

                menu_key_hlplocal_index = menu_key_common_helpindex;
                menu_key_hlplocal_topicsearch = menu_key_common_topicsearch;
                menu_key_hlplocal_prevtopic = menu_key_common_prevtopic;
                menu_key_hlplocal_copy_borland = menu_key_common_copy_borland;
                menu_key_hlplocal_copy_microsoft = menu_key_common_copy_microsoft;

                { status line entries }
                status_help            = '~F1~ Help';
                status_help_on_help    = '~F1~ Help on help';
                status_help_previoustopic = '~Alt+F1~ Previous topic';
                status_help_index      = '~Shift+F1~ Help index';
                status_help_close      = '~Esc~ Close help';
                status_save            = '~F2~ Save';
                status_open            = '~F3~ Open';
                status_compile         = '~Alt+F9~ Compile';
                status_make            = '~F9~ Make';
                status_localmenu       = '~Alt+F10~ Local menu';
                status_transferchar    = '~Ctrl+Enter~ Transfer char';
                status_msggotosource   = '~'+EnterSign+'~ Goto source';
                status_msgtracksource  = '~Space~ Track source';
                status_close           = '~Esc~ Close';
                status_calculatorpaste = '~Ctrl+Enter~ Transfer result';
                status_disassemble     = '~Alt+I~ Disassemble';

                { error messages }
                error_saving_cfg_file  = 'Error saving configuration.';
                error_saving_dsk_file  = 'Error saving desktop file.'#13+
                                         'Desktop layout could not be stored.';
                error_user_screen_not_avail = 'Sorry, user screen not available.';

                { standard button texts }
                button_OK          = 'O~K~';
                button_Cancel      = 'Cancel';
                button_New         = '~N~ew';
                button_Delete      = '~D~elete';
                button_Show        = '~S~how';
                button_Hide        = '~H~ide';

                { dialogs }
                dialog_fillintemplateparameter = 'Fill in template parameter';
                dialog_calculator       = 'Calculator';
                dialog_openafile        = 'Open a file';
                dialog_browsesymbol = 'Browse Symbol';

                msg_confirmsourcediradd = 'Directory %s is not in search path for source files. '+
                                         'Should we add it ?';
                msg_quitconfirm         = 'Do You really want to quit?';
                msg_printernotopened = 'Can''t open printer,'#13#3'check device name in "print setup"';
                msg_printerror = 'Error while printing';
                msg_impossibletoreachcursor = 'Impossible to reach current cursor';
                msg_impossibletosetbreakpoint = 'Impossible to set breakpoints here';
                msg_nothingtorun = 'Oooops, nothing to run.';
                msg_cannotrununit = 'Can''t run a unit';
                msg_cannotrunlibrary = 'Can''t run a library';
                msg_errorexecutingshell = 'Error cannot run shell';

                msg_userscreennotavailable = 'Sorry, user screen not available.';
                msg_cantsetscreenmode = #3'Impossible to set'#13#3'%dx%d mode';
                msg_confirmnewscreenmode = 'Please, confirm that new mode'#13 +
                                           'is displayed correctly';

                { Debugger confirmations and messages }
                msg_nodebuginfoavailable = 'No debug info available.';
                msg_nodebuggersupportavailable = 'No debugger support available.';

                msg_invalidfilename = 'Invalid filename %s';

                { File|New from template dialog }
                msg_notemplatesavailable = 'No templates available.';
                dialog_newfromtemplate   = 'New from template';
                label_availabletemplates = 'Available ~t~emplates';

                label_filetoopen        = 'File to ope~n~';
                label_lookingfor        = 'Looking for %s';

                {Printing.}
                dialog_setupprinter = 'Setup printer';
                label_setupprinter_device = '~D~evice';

                {Find procedure.}
                dialog_proceduredialog = 'Find Procedure';
                label_enterproceduretofind = 'Enter ~m~atching expr.';
                label_sym_findprocedure = 'Procedures';
                label_sym_findprocedure2 = 'Matching ';

                { Browser messages }
{                msg_symbolnotfound = #3'Symbol %s not found';
                msg_nobrowserinfoavailable = 'No Browser info available';}
                msg_toomanysymbolscantdisplayall= 'Too many symbols. Can''t display all of them.';

                label_sym_objects = 'Objects';
                label_sym_globalscope = 'Global scope';
                label_sym_globals = 'Globals';

                dialog_units = 'Units';

                label_entersymboltobrowse = 'Enter S~y~mbol to browse';

                {Program parameters dialog.}
                dialog_programparameters = 'Program parameters';
                label_parameters_parameter = '~P~arameter';
                msg_programnotrundoserroris = #3'Program %s'#13#3'not run'#13#3'DosError = %d';
                msg_programfileexitedwithexitcode = #3'Program %s'#13#3'exited with '#13#3'exitcode = %d';

                {Target platform dialog.}
                dialog_target = 'Target';
                label_target_platform = 'Target platform';

                {Primary file dialog.}
                label_primaryfile_primaryfile = 'Primary file';

                {Switches mode.}
                dialog_switchesmode = 'SwitchesMode';
                static_switchesmode_switchesmode = 'Switches Mode';

                {Compiler options.}
                dialog_compilerswitches = 'Compiler Switches';
                label_compiler_syntaxswitches = 'S~y~ntax Switches';
                label_compiler_mode = 'Compiler ~m~ode';
                label_compiler_codegeneration = 'Code generation';
                label_compiler_optimizations = 'Optimizations';
                label_compiler_opt_targetprocessor = 'Optimization target processor';
                label_compiler_codegen_targetprocessor = 'Code generation target processor';
                label_compiler_linkafter = 'Linking stage';
                label_compiler_verboseswitches = 'Verbose Switches';
                label_compiler_browser = 'Browser';
                label_compiler_assemblerreader = 'Assembler reader';
                label_compiler_assemblerinfo = 'Assembler info';
                label_compiler_assembleroutput = 'Assembler output';
                page_compiler_syntax = 'S~y~ntax';
                page_compiler_codegeneration = '~G~enerated code';
                page_compiler_verbose = '~V~erbose';
                page_compiler_browser = '~B~rowser';
                page_compiler_assembler = '~A~ssembler';

                {Memory sizes dialog.}
                dialog_memorysizes = 'Memory sizes';

                {Linker options dialog.}
                dialog_linker = 'Linker';
                label_linker_preferredlibtype = 'Preferred library type';

                {Debugger options dialog.}
                dialog_debugger = 'Browsing/Debugging/Profiling';
                label_debugger_debuginfo = 'Debugging information';
                label_debugger_profileswitches = 'Profiling Switches';
                label_debugger_compilerargs = 'Additional ~c~ompiler args';
                label_debugger_useanotherconsole = '~U~se another console';
                label_debugger_redirection = 'Debuggee ~R~edirection';
                label_debugger_useanothertty = '~U~se Another tty for Debuggee';

                { Remote options dialog }
                dialog_remote = 'Remote setup';
                label_remote_machine = 'Remote machine ~n~ame';
                label_remote_port = 'Remote ~p~ort number';
                label_remote_dir = 'Remote ~d~irectory';
                label_remote_config = 'Remote config ~o~ptions';
                label_remote_ident = 'Remote ~i~dent';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                label_remote_send_command = 'Remote ~S~end command';
                label_remote_exec_command = 'Remote ~E~xec command';
                label_remote_ssh_exec_command = 'Remote Ss~h~ exec command';
                label_remote_copy = 'Remote copy executable';
                label_remote_shell = 'Remote shell executable';
                label_remote_gdbserver = 'Remote gdbserver executable';
=======
                label_remote_command = 'Remote ~c~ommand';
                label_remote_scp = 'Scp executable';
                label_remote_ssh = 'Ssh executable';
>>>>>>> graemeg/cpstrnew
=======
                label_remote_command = 'Remote ~c~ommand';
                label_remote_scp = 'Scp executable';
                label_remote_ssh = 'Ssh executable';
>>>>>>> graemeg/cpstrnew
=======
                label_remote_command = 'Remote ~c~ommand';
                label_remote_scp = 'Scp executable';
                label_remote_ssh = 'Ssh executable';
>>>>>>> graemeg/cpstrnew
=======
                label_remote_command = 'Remote ~c~ommand';
                label_remote_scp = 'Scp executable';
                label_remote_ssh = 'Ssh executable';
>>>>>>> origin/cpstrnew

                {Directories dialog.}
                dialog_directories = 'Directories';

                {Editor options window.}
                dialog_defaulteditoroptions = 'Default Editor Options';
                dialog_editoroptions = 'Editor Options';
                label_editor_backupfiles = 'Create backup ~f~iles';
                label_editor_insertmode = '~I~nsert mode';
                label_editor_autoindentmode = '~A~uto indent mode';
                label_editor_usetabcharacters = '~U~se tab characters';
                label_editor_backspaceunindents = '~B~ackspace unindents';
                label_editor_persistentblocks = '~P~ersistent blocks';
                label_editor_syntaxhighlight = '~S~yntax highlight';
                label_editor_blockinsertcursor = 'B~l~ock insert cursor';
                label_editor_verticalblocks = '~V~ertical blocks';
                label_editor_highlightcolumn = 'Highlight ~c~olumn';
                label_editor_highlightrow = 'Highlight ~r~ow';
                label_editor_autoclosingbrackets = 'Aut~o~-closing brackets';
                label_editor_keeptrailingspaces = '~K~eep trailing spaces';
                label_editor_codecomplete = 'Co~d~eComplete enabled';
                label_editor_folds = 'E~n~able folds';
                label_editor_editoroptions = '~E~ditor options';
                label_editor_tabsize = '~T~ab size';
                label_editor_indentsize = 'Indent si~z~e';
                label_editor_highlightextensions = '~H~ighlight extensions';
                label_editor_filepatternsneedingtabs = 'File ~p~atterns needing tabs';

                {Browser options dialog.}
                dialog_browseroptions = 'Browser Options';
                dialog_localbrowseroptions = 'Local Browser Options';
                label_browser_labels = '~L~abels';
                label_browser_constants = '~C~onstants';
                label_browser_types = '~T~ypes';
                label_browser_variables = '~V~ariables';
                label_browser_procedures = '~P~rocedures';
                label_browser_inherited = '~I~nherited';
                label_browser_symbols = 'Symbols';
                label_browser_newbrowser = '~N~ew browser';
                label_browser_currentbrowser = '~R~eplace current';
                label_browser_subbrowsing = 'Sub-browsing';
                label_browser_scope = '~S~cope';
                label_browser_reference = 'R~e~ference';
                label_browser_preferredpane = 'Preferred pane';
                label_browser_qualifiedsymbols = '~Q~ualified symbols';
                label_browser_sortsymbols = 'S~o~rt always';
                label_browser_display = 'Display';

                {Preferences dialog.}
                dialog_preferences = 'Preferences';
                label_preferences_videomode = 'Video mode';
                label_preferences_currentdirectory = 'C~u~rrent directory';
                label_preferences_configdirectory = 'Conf~i~g file directory';
                label_preferences_desktopfile = 'Desktop file';
                label_preferences_editorfiles = 'Editor ~f~iles';
                label_preferences_environment = '~E~nvironment';
                label_preferences_desktop = '~D~esktop';
                label_preferences_autosave = 'Auto save';
                label_preferences_autotracksource = '~A~uto track source';
                label_preferences_closeongotosource = 'C~l~ose on go to source';
                label_preferences_changedironopen = 'C~h~ange dir on open';
                label_preferences_options = 'Options';

                {Desktop preferences dialog.}
                dialog_desktoppreferences = 'Desktop Preferences';
                label_desktop_historylists = '~H~istory lists';
                label_desktop_clipboard = '~C~lipboard content';
                label_desktop_watches = '~W~atch expressions';
                label_desktop_breakpoints = '~B~reakpoints';
                label_desktop_openwindow = '~O~pen windows';
                label_desktop_symbolinfo = '~S~ymbol information';
                label_desktop_codecompletewords = 'Co~d~eComplete wordlist';
                label_desktop_codetemplates = 'Code~T~emplates';
                label_desktop_preservedacrosssessions = '~P~reserved across sessions';

                {Mouse options dialog.}
                dialog_mouseoptions = 'Mouse Options';
                label_mouse_speedbar = 'Fast       Medium      Slow';
                label_mouse_doubleclickspeed = 'Mouse ~d~ouble click';
                label_mouse_reversebuttons = '~R~everse mouse buttons';
                label_mouse_crtlrightmousebuttonaction = 'Ctrl+Right mouse button';
                label_mouse_altrightmousebuttonaction = 'Alt+Right mouse button';
                label_mouse_act_nothing = 'Nothing';
                label_mouse_act_topicsearch = 'Topic search';
                label_mouse_act_gotocursor = 'Go to cursor';
                label_mouse_act_breakpoint = 'Breakpoint';
                label_mouse_act_evaluate = 'Evaluate';
                label_mouse_act_addwatch = 'Add watch';
                label_mouse_act_browsesymbol = 'Browse symbol';

                {Open options dialog.}
                dialog_openoptions = 'Open Options';
                msg_cantopenconfigfile = 'Can''t open config file.';
                msg_errorsavingconfigfile = 'Error saving config file.';

                {Save options dialog.}
                dialog_saveoptions = 'Save Options';
                dialog_ini_filename = 'Name of INI file';

                {Window list dialog.}
                dialog_windowlist = 'Window List';
                label_wndlist_windows = '~W~indows';
                msg_windowlist_hidden = 'hidden';

                {Help files dialog.}
                dialog_helpfiles = 'Install Help Files';
                label_helpfiles_helpfiles = '~H~elp files';

                {Install help file.}
                dialog_installhelpfile = 'Install a help file';
                label_installhelpfile_filename = '~H~elp file name';

                {Topic title dialog.}
                dialog_topictitle = 'Topic title';
                label_topictitle_title = 'Title';

                { About window }
{                dialog_about = 'About';
                label_about_compilerversion = 'Compiler Version';
                label_about_debugger = 'Debugger';}

                msg_errorparsingtoolparams = 'Error parsing tool params.';
                msg_executingtool = 'Executing tool %s ...';
                msg_errorreadingoutput = 'Error reading output.';
                msg_executingfilterfor = 'Executing filter for %s ...';
                msg_cantfindfilteredoutput = 'Can''t find filtered output.';
                msg_errorprocessingfilteredoutput = 'Error processing filtered output.';
                msg_errorexecutingfilter = 'Error executing filter %s';
                msg_errorexecutingtool = 'Error executing tool %s';
                msg_filterexecutionsuccessfulexitcodeis = 'Filter execution successful. Exit code = %d';
                msg_toolexecutionsuccessfulexitcodeis = 'Tool execution successful. Exit code = %d';
                msg_xmustbesettoyforz_doyouwanttochangethis =
                  '%s must be set to "%s" for %s. '+
                  'Do you want to change this option automatically?';

                dialog_greparguments = 'Grep arguments';
                msg_grepprogramnotfound = 'Grep program not found';
                label_grep_texttofind = '~T~ext to find';
                label_grep_greparguments = '~G~rep arguments';
                msg_runninggrepwithargs = 'Running Grep -n %s';
                msg_errorrunninggrep = #3'Error running Grep'#13#3'DosError = %d'#13#3'Exit code = %d';
                msg_errorreadinggrepoutput = #3'Error reading Grep output'#13#3'In line %d of %s';
                msg_filealreadyexistsoverwrite = 'File %s already exists. Overwrite?';
                msg_createkeywordindexforhelpfile = 'Create keyword index from help file?';

                msg_pleasewaitwhilecreatingindex = 'Please wait while creating index...';
                msg_buildingindexfile = 'Building index file %s';
                msg_filedoesnotcontainanylinks = '%s doesn''t contain any links, thus it isn''t suitable for indexing.';
                msg_storinghtmlindexinfile = 'Storing HTML index in %s';
                msg_errorstoringindexdata = 'Error storing index data (%d)';

                msg_cantcreatefile = 'Can''t create %s';

                {ANSI screenshots.}
                msg_saveansifile = 'Save previous screen as Ansi File';
                msg_click_upper_left = 'Click to select upper left corner; Escape to cancel; Enter to select (0,0)';
                msg_click_lower_right = 'Click to select lower right corner; Escape to cancel; Enter to select (maxX,maxY)';

function IncTargetedEventPtr(I: integer): integer;
begin
  Inc(I);
  if I>High(TargetedEvents) then I:=Low(TargetedEvents);
  IncTargetedEventPtr:=I;
end;

procedure PutEvent(TargetView: PView; E: TEvent);
begin
  if IncTargetedEventPtr(TargetedEventHead)=TargetedEventTail then Exit;
  with TargetedEvents[TargetedEventHead] do
  begin
    Target:=TargetView;
    Event:=E;
  end;
  TargetedEventHead:=IncTargetedEventPtr(TargetedEventHead);
end;

procedure PutCommand(TargetView: PView; What, Command: Word; InfoPtr: Pointer);
var E: TEvent;
begin
  FillChar(E,Sizeof(E),0);
  E.What:=What;
  E.Command:=Command;
  E.InfoPtr:=InfoPtr;
  PutEvent(TargetView,E);
end;

function GetTargetedEvent(var P: PView; var E: TEvent): boolean;
var OK: boolean;
begin
  OK:=TargetedEventHead<>TargetedEventTail;
  if OK then
  begin
    with TargetedEvents[TargetedEventTail] do
    begin
      P:=Target;
      E:=Event;
    end;
    TargetedEventTail:=IncTargetedEventPtr(TargetedEventTail);
  end;
  GetTargetedEvent:=OK;
end;

function IDEUseSyntaxHighlight(Editor: PFileEditor): boolean;
begin
  IDEUseSyntaxHighlight:=(Editor^.IsFlagSet(efSyntaxHighlight)) and ((Editor^.FileName='') or MatchesFileList(NameAndExtOf(Editor^.FileName),HighlightExts));
end;

function IDEUseTabsPattern(Editor: PFileEditor): boolean;
begin
  { the commented code lead all new files
    to become with TAB use enabled which is wrong in my opinion PM }
  IDEUseTabsPattern:={(Editor^.FileName='') or }MatchesFileList(NameAndExtOf(Editor^.FileName),TabsPattern);
end;

constructor TIDEApp.Init;
var R: TRect;
begin
  displaymode:=dmIDE;
  UseSyntaxHighlight:=@IDEUseSyntaxHighlight;
  UseTabsPattern:=@IDEUseTabsPattern;
  inherited Init;
  InitAdvMsgBox;
  InsideDone:=false;
  IsRunning:=true;
  MenuBar^.GetBounds(R); R.A.X:=R.B.X-8;
  New(ClockView, Init(R));
  ClockView^.GrowMode:=gfGrowLoX+gfGrowHiX;
  Application^.Insert(ClockView);
  New(ClipboardWindow, Init);
  Desktop^.Insert(ClipboardWindow);
  New(CalcWindow, Init); CalcWindow^.Hide;
  Desktop^.Insert(CalcWindow);
  New(CompilerMessageWindow, Init);
  CompilerMessageWindow^.Hide;
  Desktop^.Insert(CompilerMessageWindow);
  Message(@Self,evBroadcast,cmUpdate,nil);
  CurDirChanged;
  { heap viewer }
  GetExtent(R); Dec(R.B.X); R.A.X:=R.B.X-9; R.A.Y:=R.B.Y-1;
  New(HeapView, InitKb(R));
  if (StartupOptions and soHeapMonitor)=0 then HeapView^.Hide;
  Insert(HeapView);
  Drivers.ShowMouse;
{$ifdef Windows}
  // WindowsShowMouse;
{$endif Windows}
end;

procedure TIDEApp.InitDesktop;
var
  R: TRect;
begin
  GetExtent(R);
  Inc(R.A.Y);
  Dec(R.B.Y);
  Desktop:=New(PFPDesktop, Init(R));
end;

procedure TIDEApp.LoadMenuBar;

var R: TRect;
    WinPMI : PMenuItem;

begin
  GetExtent(R); R.B.Y:=R.A.Y+1;
  WinPMI:=nil;
{$ifdef WinClipSupported}
  if WinClipboardSupported then
    WinPMI:=NewLine(
      NewItem(menu_edit_copywin,'', kbNoKey, cmCopyWin, hcCopyWin,
      NewItem(menu_edit_pastewin,'', kbNoKey, cmPasteWin, hcPasteWin,
      nil)));
{$endif WinClipSupported}
  MenuBar:=New(PAdvancedMenuBar, Init(R, NewMenu(
    NewSubMenu(menu_file,hcFileMenu, NewMenu(
      NewItem(menu_file_new,'',kbNoKey,cmNew,hcNew,
      NewItem(menu_file_template,'',kbNoKey,cmNewFromTemplate,hcNewFromTemplate,
      NewItem(menu_file_open,menu_key_file_open,kbF3,cmOpen,hcOpen,
      NewItem(menu_file_reload,'',kbNoKey,cmDoReload,hcDoReload,
      NewItem(menu_file_save,menu_key_file_save,kbF2,cmSave,hcSave,
      NewItem(menu_file_saveas,'',kbNoKey,cmSaveAs,hcSaveAs,
      NewItem(menu_file_saveall,'',kbNoKey,cmSaveAll,hcSaveAll,
      NewLine(
      NewItem(menu_file_print,'',kbNoKey,cmPrint,hcPrint,
      NewItem(menu_file_printsetup,'',kbNoKey,cmPrinterSetup,hcPrinterSetup,
      NewLine(
      NewItem(menu_file_changedir,'',kbNoKey,cmChangeDir,hcChangeDir,
      NewItem(menu_file_dosshell,'',kbNoKey,cmDOSShell,hcDOSShell,
      NewItem(menu_file_exit,menu_key_file_exit,kbNoKey,cmQuit,hcQuit,
      nil))))))))))))))),
    NewSubMenu(menu_edit,hcEditMenu, NewMenu(
      NewItem(menu_edit_undo,menu_key_edit_undo, kbAltBack, cmUndo, hcUndo,
      NewItem(menu_edit_redo,'', kbNoKey, cmRedo, hcRedo,
{$ifdef DebugUndo}
      NewItem('~D~ump Undo','', kbNoKey, cmDumpUndo, hcUndo,
      NewItem('U~n~do All','', kbNoKey, cmUndoAll, hcUndo,
      NewItem('R~e~do All','', kbNoKey, cmRedoAll, hcRedo,
{$endif DebugUndo}
      NewLine(
      NewItem(menu_edit_cut,menu_key_edit_cut, cut_key, cmCut, hcCut,
      NewItem(menu_edit_copy,menu_key_edit_copy, copy_key, cmCopy, hcCopy,
      NewItem(menu_edit_paste,menu_key_edit_paste, paste_key, cmPaste, hcPaste,
      NewItem(menu_edit_clear,menu_key_edit_clear, kbCtrlDel, cmClear, hcClear,
      NewItem(menu_edit_selectall,menu_key_edit_all, all_Key, cmSelectAll, hcSelectAll,
      NewItem(menu_edit_unselect,'', kbNoKey, cmUnselect, hcUnselect,
      NewLine(
      NewItem(menu_edit_showclipboard,'', kbNoKey, cmShowClipboard, hcShowClipboard,
      WinPMI))))))))
{$ifdef DebugUndo}))){$endif DebugUndo}
      )))),
    NewSubMenu(menu_search,hcSearchMenu, NewMenu(
      NewItem(menu_search_find,'', kbNoKey, cmFind, hcFind,
      NewItem(menu_search_replace,'', kbNoKey, cmReplace, hcReplace,
      NewItem(menu_search_searchagain,'', kbNoKey, cmSearchAgain, hcSearchAgain,
      NewLine(
      NewItem(menu_search_jumpline,'', kbNoKey, cmJumpLine, hcGotoLine,
      NewItem(menu_search_findproc,'', kbNoKey, cmFindProcedure, hcFindProcedure,
      NewLine(
      NewItem(menu_search_objects,'', kbNoKey, cmObjects, hcObjects,
      NewItem(menu_search_modules,'', kbNoKey, cmModules, hcModules,
      NewItem(menu_search_globals,'', kbNoKey, cmGlobals, hcGlobals,
      NewLine(
      NewItem(menu_search_symbol,'', kbNoKey, cmSymbol, hcSymbol,
      nil))))))))))))),
    NewSubMenu(menu_run,hcRunMenu, NewMenu(
      NewItem(menu_run_run,menu_key_run_run, kbCtrlF9, cmRun, hcRun,
      NewItem(menu_run_stepover,menu_key_run_stepover, kbF8, cmStepOver, hcRun,
      NewItem(menu_run_traceinto,menu_key_run_traceinto, kbF7, cmTraceInto, hcRun,
      NewItem(menu_run_conttocursor,menu_key_run_conttocursor, kbF4, cmContToCursor, hcContToCursor,
      NewItem(menu_run_untilreturn,menu_key_run_untilreturn, kbAltF4,cmUntilReturn,hcUntilReturn,
      NewItem(menu_run_rundir,'', kbNoKey, cmRunDir, hcRunDir,
      NewItem(menu_run_parameters,'', kbNoKey, cmParameters, hcParameters,
      NewItem(menu_run_resetdebugger,menu_key_run_resetdebugger, kbCtrlF2, cmResetDebugger, hcResetDebugger,
      nil))))))))),
    NewSubMenu(menu_compile,hcCompileMenu, NewMenu(
      NewItem(menu_compile_compile,menu_key_compile_compile, kbAltF9, cmCompile, hcCompile,
      NewItem(menu_compile_make,menu_key_compile_make, kbF9, cmMake, hcMake,
      NewItem(menu_compile_build,'', kbNoKey, cmBuild, hcBuild,
      NewLine(
      NewItem(menu_compile_target,'', kbNoKey, cmTarget, hcTarget,
      NewItem(menu_compile_primaryfile,'', kbNoKey, cmPrimaryFile, hcPrimaryFile,
      NewItem(menu_compile_clearprimaryfile,'', kbNoKey, cmClearPrimary, hcClearPrimary,
      NewLine(
      NewItem(menu_compile_compilermessages,menu_key_compile_compilermessages, kbF12, cmCompilerMessages, hcCompilerMessages,
      nil)))))))))),
    NewSubMenu(menu_debug, hcDebugMenu, NewMenu(
      NewItem(menu_debug_output,'', kbNoKey, cmUserScreenWindow, hcUserScreenWindow,
      NewItem(menu_debug_userscreen,menu_key_debug_userscreen, kbAltF5, cmUserScreen, hcUserScreen,
      NewLine(
{$ifdef SUPPORT_REMOTE}
      NewItem(menu_debug_remote,'', kbNoKey, cmTransferRemote, hcTransferRemote,
{$endif SUPPORT_REMOTE}
      NewItem(menu_debug_addwatch,menu_key_debug_addwatch, kbCtrlF7, cmAddWatch, hcAddWatch,
      NewItem(menu_debug_watches,'', kbNoKey, cmWatches, hcWatchesWindow,
      NewItem(menu_debug_breakpoint,menu_key_debug_breakpoint, kbCtrlF8, cmToggleBreakpoint, hcToggleBreakpoint,
      NewItem(menu_debug_breakpointlist,'', kbNoKey, cmBreakpointList, hcBreakpointList,
      NewItem('~E~valuate...','Ctrl+F4', kbCtrlF4, cmEvaluate, hcEvaluate,
      NewItem(menu_debug_callstack,menu_key_debug_callstack, kbCtrlF3, cmStack, hcStackWindow,
      NewLine(
      NewItem(menu_debug_disassemble,'', kbNoKey, cmDisassemble, hcDisassemblyWindow,
      NewItem(menu_debug_registers,'', kbNoKey, cmRegisters, hcRegistersWindow,
      NewItem(menu_debug_fpu_registers,'', kbNoKey, cmFPURegisters, hcFPURegisters,
      NewItem(menu_debug_vector_registers,'', kbNoKey, cmVectorRegisters, hcVectorRegisters,
      NewLine(
      NewItem(menu_debug_gdbwindow,'', kbNoKey, cmOpenGDBWindow, hcOpenGDBWindow,
      nil
{$ifdef SUPPORT_REMOTE}
      )
{$endif SUPPORT_REMOTE}
      ))))))))))))))))),
    NewSubMenu(menu_tools, hcToolsMenu, NewMenu(
      NewItem(menu_tools_messages,menu_key_tools_messages, kbF11, cmToolsMessages, hcToolsMessages,
      NewItem(menu_tools_msgnext,menu_key_tools_msgnext, kbAltF8, cmToolsMsgNext, hcToolsMsgNext,
      NewItem(menu_tools_msgprev,menu_key_tools_msgprev, kbAltF7, cmToolsMsgPrev, hcToolsMsgPrev,
      NewLine(
      NewItem(menu_tools_grep,menu_key_tools_grep, kbShiftF2, cmGrep, hcGrep,
      NewItem(menu_tools_calculator, '', kbNoKey, cmCalculator, hcCalculator,
      NewItem(menu_tools_asciitable, '', kbNoKey, cmAsciiTable, hcAsciiTable,
      nil)))))))),
    NewSubMenu(menu_options, hcOptionsMenu, NewMenu(
      NewItem(menu_options_mode,'', kbNoKey, cmSwitchesMode, hcSwitchesMode,
      NewItem(menu_options_compiler,'', kbNoKey, cmCompiler, hcCompiler,
      NewItem(menu_options_memory,'', kbNoKey, cmMemorySizes, hcMemorySizes,
      NewItem(menu_options_linker,'', kbNoKey, cmLinker, hcLinker,
      NewItem(menu_options_debugger,'', kbNoKey, cmDebugger, hcDebugger,
{$ifdef SUPPORT_REMOTE}
      NewItem(menu_options_remote,'', kbNoKey, cmRemoteDialog, hcRemoteDialog,
{$endif SUPPORT_REMOTE}
      NewItem(menu_options_directories,'', kbNoKey, cmDirectories, hcDirectories,
      NewItem(menu_options_browser,'',kbNoKey, cmBrowser, hcBrowser,
      NewItem(menu_options_tools,'', kbNoKey, cmTools, hcTools,
      NewLine(
      NewSubMenu(menu_options_env, hcEnvironmentMenu, NewMenu(
        NewItem(menu_options_env_preferences,'', kbNoKey, cmPreferences, hcPreferences,
        NewItem(menu_options_env_editor,'', kbNoKey, cmEditor, hcEditor,
        NewItem(menu_options_env_codecomplete,'', kbNoKey, cmCodeCompleteOptions, hcCodeCompleteOptions,
        NewItem(menu_options_env_codetemplates,'', kbNoKey, cmCodeTemplateOptions, hcCodeTemplateOptions,
        NewItem(menu_options_env_desktop,'', kbNoKey, cmDesktopOptions, hcDesktopOptions,
        NewItem(menu_options_env_keybmouse,'', kbNoKey, cmMouse, hcMouse,
{        NewItem(menu_options_env_startup,'', kbNoKey, cmStartup, hcStartup,
        NewItem(menu_options_env_colors,'', kbNoKey, cmColors, hcColors,}
{$ifdef Unix}
        NewItem(menu_options_learn_keys,'', kbNoKey, cmKeys, hcKeys,
{$endif Unix}
        nil
{$ifdef Unix}
        )
{$endif Unix}
        {))}))))))),
      NewLine(
      NewItem(menu_options_open,'', kbNoKey, cmOpenINI, hcOpenINI,
      NewItem(menu_options_save,'', kbNoKey, cmSaveINI, hcSaveINI,
      NewItem(menu_options_saveas,'', kbNoKey, cmSaveAsINI, hcSaveAsINI,
      nil
{$ifdef SUPPORT_REMOTE}
      )
{$endif SUPPORT_REMOTE}
      ))))))))))))))),
    NewSubMenu(menu_window, hcWindowMenu, NewMenu(
      NewItem(menu_window_tile,'', kbNoKey, cmTile, hcTile,
      NewItem(menu_window_cascade,'', kbNoKey, cmCascade, hcCascade,
      NewItem(menu_window_closeall,'', kbNoKey, cmCloseAll, hcCloseAll,
      NewLine(
      NewItem(menu_window_resize,menu_key_window_resize, kbCtrlF5, cmResize, hcResize,
      NewItem(menu_window_zoom,menu_key_window_zoom, kbF5, cmZoom, hcZoom,
      NewItem(menu_window_next,menu_key_window_next, kbF6, cmNext, hcNext,
      NewItem(menu_window_previous,menu_key_window_previous, kbShiftF6, cmPrev, hcPrev,
      NewItem(menu_window_hide,menu_key_window_hide, kbCtrlF6, cmHide, hcHide,
      NewItem(menu_window_close,menu_key_window_close, kbAltF3, cmClose, hcClose,
      NewLine(
      NewItem(menu_window_list,menu_key_window_list, kbAlt0, cmWindowList, hcWindowList,
      NewItem(menu_window_update,'', kbNoKey, cmUpdate, hcUpdate,
      nil)))))))))))))),
    NewSubMenu(menu_help, hcHelpMenu, NewMenu(
      NewItem(menu_help_contents,'', kbNoKey, cmHelpContents, hcHelpContents,
      NewItem(menu_help_index,menu_key_help_helpindex, kbShiftF1, cmHelpIndex, hcHelpIndex,
      NewItem(menu_help_topicsearch,menu_key_help_topicsearch, kbCtrlF1, cmHelpTopicSearch, hcHelpTopicSearch,
      NewItem(menu_help_prevtopic,menu_key_help_prevtopic, kbAltF1, cmHelpPrevTopic, hcHelpPrevTopic,
      NewItem(menu_help_using,'',kbNoKey, cmHelpUsingHelp, hcHelpUsingHelp,
      NewItem(menu_help_files,'',kbNoKey, cmHelpFiles, hcHelpFiles,
      NewLine(
      NewItem(menu_help_about,'',kbNoKey, cmAbout, hcAbout,
      nil))))))))),
    nil)))))))))))));
   SetCmdState(ToClipCmds+FromClipCmds+NulClipCmds+UndoCmd+RedoCmd,false);
end;

procedure TIDEApp.InitMenuBar;

begin
  LoadMenuBar;
  DisableCommands(EditorCmds+SourceCmds+CompileCmds);
  // Update; Desktop is still nil at that point ...
end;

procedure Tideapp.reload_menubar;

begin
   delete(menubar);
   dispose(menubar,done);
   case EditKeys of
     ekm_microsoft:
       begin
         menu_key_edit_cut:=menu_key_edit_cut_microsoft;
         menu_key_edit_copy:=menu_key_edit_copy_microsoft;
         menu_key_edit_paste:=menu_key_edit_paste_microsoft;
         menu_key_edit_all:=menu_key_edit_all_microsoft;
         menu_key_hlplocal_copy:=menu_key_hlplocal_copy_microsoft;
         cut_key:=kbCtrlX;
         copy_key:=kbCtrlC;
         paste_key:=kbCtrlV;
         all_key:=kbCtrlA;
       end;
     ekm_borland:
       begin
         menu_key_edit_cut:=menu_key_edit_cut_borland;
         menu_key_edit_copy:=menu_key_edit_copy_borland;
         menu_key_edit_paste:=menu_key_edit_paste_borland;
         menu_key_edit_all:=menu_key_edit_all_borland;
         menu_key_hlplocal_copy:=menu_key_hlplocal_copy_borland;
         cut_key:=kbShiftDel;
         copy_key:=kbCtrlIns;
         paste_key:=kbShiftIns;
         all_key:=kbNoKey;
       end;
   end;
   loadmenubar;
   insert(menubar);
end;

procedure TIDEApp.InitStatusLine;
var
  R: TRect;
begin
  GetExtent(R);
  R.A.Y := R.B.Y - 1;
  StatusLine:=New(PIDEStatusLine, Init(R,
    NewStatusDef(hcDragging, hcDragging,
      NewStatusKey(status_help, kbF1, cmHelp,
      StdStatusKeys(
      NewStatusKey('~Cursor~ Move', kbNoKey, 65535,
      NewStatusKey('~Shift+Cursor~ Size', kbNoKey, 65535,
      NewStatusKey('~'#17'ды~ Done', kbNoKey, 65535, {#17 = left arrow}
      NewStatusKey('~Esc~ Cancel', kbNoKey, 65535,
      nil)))))),
    NewStatusDef(hcStackWindow, hcStackWindow,
      NewStatusKey(status_help, kbF1, cmHelp,
      NewStatusKey(status_disassemble, kbAltI, cmDisassemble,
      StdStatusKeys(
      nil))),
    NewStatusDef(hcFirstCommand, hcLastNormalCommand,
      NewStatusKey(status_help, kbF1, cmHelp,
      StdStatusKeys(
      nil)),
    NewStatusDef(hcFirstNoAltXCommand, hcLastCommand,
      NewStatusKey(status_help, kbF1, cmHelp,
      NewStatusKey('', kbF10, cmMenu,
      NewStatusKey('', kbAltF3, cmClose,
      NewStatusKey('', kbF5, cmZoom,
      NewStatusKey('', kbCtrlF5, cmResize,
      NewStatusKey('', kbF6, cmNext,
      NewStatusKey('', kbShiftF6, cmPrev,
      nil))))))),
    NewStatusDef(hcHelpWindow, hcHelpWindow,
      NewStatusKey(status_help_on_help, kbF1, cmHelpUsingHelp,
      NewStatusKey(status_help_previoustopic, kbAltF1, cmHelpPrevTopic,
      NewStatusKey(status_help_index, kbShiftF1, cmHelpIndex,
      NewStatusKey(status_help_close, kbEsc, cmClose,
      StdStatusKeys(
      nil))))),
    NewStatusDef(hcSourceWindow, hcSourceWindow,
      NewStatusKey(status_help, kbF1, cmHelp,
      NewStatusKey(status_save, kbF2, cmSave,
      NewStatusKey(status_open, kbF3, cmOpen,
      NewStatusKey(status_compile, kbAltF9, cmCompile,
      NewStatusKey(status_make, kbF9, cmMake,
      NewStatusKey(status_localmenu, kbAltF10, cmLocalMenu,
      StdStatusKeys
      (
      nil))))))),
    NewStatusDef(hcASCIITableWindow, hcASCIITableWindow,
      NewStatusKey(status_help, kbF1, cmHelp,
      NewStatusKey(status_transferchar, kbCtrlEnter, cmTransfer,
      StdStatusKeys(
      nil))),
    NewStatusDef(hcMessagesWindow, hcMessagesWindow,
      NewStatusKey(status_help, kbF1, cmHelp,
      NewStatusKey(status_msggotosource, kbEnter, cmMsgGotoSource,
      NewStatusKey(status_msgtracksource, kbNoKey, cmMsgTrackSource,
      NewStatusKey(status_localmenu, kbAltF10, cmLocalMenu,
      NewStatusKey('', kbEsc, cmClose,
      StdStatusKeys(
      nil)))))),
    NewStatusDef(hcCalcWindow, hcCalcWindow,
      NewStatusKey(status_help, kbF1, cmHelp,
      NewStatusKey(status_close, kbEsc, cmClose,
      NewStatusKey(status_calculatorpaste, kbCtrlEnter, cmCalculatorPaste,
      StdStatusKeys(
      nil)))),
    NewStatusDef(0, $FFFF,
      NewStatusKey(status_help, kbF1, cmHelp,
      NewStatusKey(status_open, kbF3, cmOpen,
      NewStatusKey(status_compile, kbAltF9, cmCompile,
      NewStatusKey(status_make, kbF9, cmMake,
      NewStatusKey(status_localmenu, kbAltF10, cmLocalMenu,
      StdStatusKeys(
      nil)))))),
    nil))))))))))));
end;

procedure TIDEApp.Idle;
begin
  inherited Idle;
  Message(Application,evIdle,0,nil);
end;

procedure TIDEApp.GetEvent(var Event: TEvent);
var P: PView;
begin
  { first of all dispatch queued targeted events }
  while GetTargetedEvent(P,Event) do
    P^.HandleEvent(Event);
  { Handle System events directly }
  Drivers.GetSystemEvent(Event);         { Load system event }
  If (Event.What <> evNothing) Then
    HandleEvent(Event);

  inherited GetEvent(Event);
{$ifdef DEBUG}
  if (Event.What=evKeyDown) and (Event.KeyCode=kbAltF11) then
    begin
{$ifdef HasSignal}
      Generate_SIGSEGV;
{$else}
      Halt(1);
{$endif}
    end;
  if (Event.What=evKeyDown) and (Event.KeyCode=kbCtrlF11) then
    begin
      RunError(250);
    end;
{$endif DEBUG}
  if (Event.What=evKeyDown) and (Event.KeyCode=kbAltF12) then
    begin
      CreateAnsiFile;
      ClearEvent(Event);
    end;
  if Event.What<>evNothing then
    LastEvent:=GetDosTicks
  else
    begin
      if abs(GetDosTicks-LastEvent)>SleepTimeOut then
        GiveUpTimeSlice;
    end;
end;

procedure TIDEApp.HandleEvent(var Event: TEvent);
var DontClear: boolean;
    TempS: string;
    ForceDlg: boolean;
    W  : PSourceWindow;
    DS : DirStr;
    NS : NameStr;
    ES : ExtStr;
{$ifdef HasSignal}
    CtrlCCatched : boolean;
{$endif HasSignal}
begin
{$ifdef HasSignal}
  if (Event.What=evKeyDown) and (Event.keyCode=kbCtrlC) and
     (CtrlCPressed) then
    begin
      CtrlCCatched:=true;
{$ifdef DEBUG}
      Writeln(stderr,'One Ctrl-C caught');
{$endif DEBUG}
    end
  else
    CtrlCCatched:=false;
{$endif HasSignal}
  case Event.What of
       evKeyDown :
         begin
           DontClear:=true;
           { just for debugging purposes }
         end;
       evCommand :
         begin
           DontClear:=false;
           case Event.Command of
             cmUpdate        : Message(Application,evBroadcast,cmUpdate,nil);
           { -- File menu -- }
             cmNew           : NewEditor;
             cmNewFromTemplate: NewFromTemplate;
             cmOpen          : begin
                                 ForceDlg:=false;
                                 if (OpenFileName<>'') and
                                    ((DirOf(OpenFileName)='') or (Pos(ListSeparator,OpenFileName)<>0)) then
                                   begin
                                     TempS:=LocateSourceFile(OpenFileName,false);
                                     if TempS='' then
                                       ForceDlg:=true
                                     else
                                       OpenFileName:=TempS;
                                   end;
                                 if ForceDlg then
                                   OpenSearch(OpenFileName)
                                 else
                                   begin
                                     W:=LastSourceEditor;
                                     if assigned(W) then
                                       FSplit(W^.Editor^.FileName,DS,NS,ES)
                                     else
                                       DS:='';
                                     Open(OpenFileName,DS);
                                   end;
                                 OpenFileName:='';
                               end;
             cmPrint         : Print;
             cmPrinterSetup  : PrinterSetup;
             cmSaveAll       : SaveAll;
             cmChangeDir     : ChangeDir;
             cmDOSShell      : DOSShell;
             cmRecentFileBase..
             cmRecentFileBase+10
                             : OpenRecentFile(Event.Command-cmRecentFileBase);
           { -- Edit menu -- }
             cmShowClipboard : ShowClipboard;
           { -- Search menu -- }
             cmFindProcedure : FindProcedure;
             cmObjects       : Objects;
             cmModules       : Modules;
             cmGlobals       : Globals;
             cmSymbol        : SearchSymbol;
           { -- Run menu -- }
             cmRunDir        : RunDir;
             cmParameters    : Parameters;
             cmStepOver      : DoStepOver;
             cmTraceInto     : DoTraceInto;
             cmRun,
             cmContinue      : DoRun;
             cmResetDebugger : DoResetDebugger;
             cmContToCursor  : DoContToCursor;
             cmUntilReturn   : DoContUntilReturn;
           { -- Compile menu -- }
             cmCompile       : DoCompile(cCompile);
             cmBuild         : DoCompile(cBuild);
             cmMake          : DoCompile(cMake);
             cmTarget        : Target;
             cmPrimaryFile   : DoPrimaryFile;
             cmClearPrimary  : DoClearPrimary;
             cmCompilerMessages : DoCompilerMessages;
           { -- Debug menu -- }
             cmUserScreen    : DoUserScreen;
             cmToggleBreakpoint : DoToggleBreak;
             cmStack         : DoShowCallStack;
             cmDisassemble   : DoShowDisassembly;
             cmBreakpointList : DoShowBreakpointList;
             cmWatches       :  DoShowWatches;
             cmAddWatch      :  DoAddWatch;
             cmOpenGDBWindow : DoOpenGDBWindow;
             cmRegisters     : DoShowRegisters;
             cmFPURegisters     : DoShowFPU;
             cmVectorRegisters : DoShowVector;
             cmEvaluate      : do_evaluate;
           { -- Options menu -- }
             cmSwitchesMode  : SetSwitchesMode;
             cmCompiler      : DoCompilerSwitch;
             cmMemorySizes   : MemorySizes;
             cmLinker        : DoLinkerSwitch;
             cmDebugger      : DoDebuggerSwitch;
{$ifdef SUPPORT_REMOTE}
             cmRemoteDialog  : DoRemote;
             cmTransferRemote: TransferRemote;
{$endif SUPPORT_REMOTE}
             cmDirectories   : Directories;
             cmTools         : Tools;
             cmPreferences   : Preferences;
             cmEditor        : EditorOptions(nil);
             cmEditorOptions : EditorOptions(Event.InfoPtr);
             cmCodeTemplateOptions: CodeTemplates;
             cmCodeCompleteOptions: CodeComplete;
             cmBrowser       : BrowserOptions(nil);
             cmBrowserOptions : BrowserOptions(Event.InfoPtr);
             cmMouse         : Mouse;
             cmStartup       : StartUp;
             cmDesktopOptions: DesktopOptions;
             cmColors        : Colors;
{$ifdef Unix}
             cmKeys          : LearnKeysDialog;
{$endif Unix}
             cmOpenINI       : OpenINI;
             cmSaveINI       : SaveINI;
             cmSaveAsINI     : SaveAsINI;
           { -- Tools menu -- }
             cmToolsMessages : Messages;
             cmCalculator    : Calculator;
             cmAsciiTable    : DoAsciiTable;
             cmGrep          : DoGrep;
             cmToolsBase+1..
             cmToolsBase+MaxToolCount
                             : ExecuteTool(Event.Command-cmToolsBase);
           { -- Window menu -- }
             cmCloseAll      : CloseAll;
             cmWindowList    : WindowList;
             cmUserScreenWindow: DoUserScreenWindow;
           { -- Help menu -- }
             cmHelp,
             cmHelpContents  : HelpContents;
             cmHelpIndex     : HelpHelpIndex;
             cmHelpDebug     : HelpDebugInfos;
             cmHelpTopicSearch: HelpTopicSearch;
             cmHelpPrevTopic : HelpPrevTopic;
             cmHelpUsingHelp : HelpUsingHelp;
             cmHelpFiles     : HelpFiles;
             cmAbout         : About;
             cmShowReadme    : ShowReadme;
             cmResizeApp     : ResizeApplication(Event.Id, Event.InfoWord);
             cmQuitApp       : Message(@Self, evCommand, cmQuit, nil);
           else DontClear:=true;
           end;
           if DontClear=false then ClearEvent(Event);
         end;
       evBroadcast :
         case Event.Command of
           cmSaveCancelled :
             SaveCancelled:=true;
           cmUpdateTools :
             UpdateTools;
           cmCommandSetChanged :
             UpdateMenu(MenuBar^.Menu);
           cmUpdate              :
             Update;
           cmSourceWndClosing :
             begin
               with PSourceWindow(Event.InfoPtr)^ do
                 if Editor^.FileName<>'' then
                   AddRecentFile(Editor^.FileName,Editor^.CurPos.X,Editor^.CurPos.Y);
               {$ifndef NODEBUG}
               if assigned(Debugger) and (PView(Event.InfoPtr)=Debugger^.LastSource) then
                 Debugger^.LastSource:=nil;
               {$endif}
             end;

         end;
  end;
  inherited HandleEvent(Event);
{$ifdef HasSignal}
  { Reset flag if CrtlC was handled }
  if CtrlCCatched and (Event.What=evNothing) then
    begin
      CtrlCPressed:=false;
{$ifdef DEBUG}
      Writeln(stderr,'One CtrlC handled');
{$endif DEBUG}
    end;
{$endif HasSignal}
end;


procedure TIDEApp.GetTileRect(var R: TRect);
begin
  Desktop^.GetExtent(R);
{ Leave the compiler messages window in the bottom }
  if assigned(CompilerMessageWindow) and (CompilerMessageWindow^.GetState(sfVisible)) then
   R.B.Y:=Min(CompilerMessageWindow^.Origin.Y,R.B.Y);
{ Leave the messages window in the bottom }
  if assigned(MessagesWindow) and (MessagesWindow^.GetState(sfVisible)) then
   R.B.Y:=Min(MessagesWindow^.Origin.Y,R.B.Y);
{$ifndef NODEBUG}
{ Leave the watch window in the bottom }
  if assigned(WatchesWindow) and (WatchesWindow^.GetState(sfVisible)) then
   R.B.Y:=Min(WatchesWindow^.Origin.Y,R.B.Y);
{$endif NODEBUG}
end;


{****************************************************************************
                                 Switch Screens
****************************************************************************}

procedure TIDEApp.ShowUserScreen;
begin
  displaymode:=dmUser;
  if Assigned(UserScreen) then
    UserScreen^.SaveIDEScreen;
  DoneSysError;
  DoneEvents;
  { DoneKeyboard should be called last to
    restore the keyboard correctly PM }
{$ifndef go32v2}
  donevideo;
{$endif ndef go32v2}
  DoneKeyboard;
  If UseMouse then
    DoneMouse
  else
    ButtonCount:=0;
{  DoneDosMem;}

  if Assigned(UserScreen) then
    UserScreen^.SwitchToConsoleScreen;
end;


procedure TIDEApp.ShowIDEScreen;
begin
  if Assigned(UserScreen) then
    UserScreen^.SaveConsoleScreen;
{  InitDosMem;}
  InitKeyboard;
  If UseMouse then
    InitMouse
  else
    ButtonCount:=0;
{$ifndef go32v2}
  initvideo;
{$endif ndef go32v2}
  {Videobuffer has been reallocated, need passive video situation detection
   again.}
  initscreen;
{$ifdef Windows}
  { write the empty screen to dummy console handle }
  UpdateScreen(true);
{$endif ndef Windows}
  InitEvents;
  InitSysError;
  CurDirChanged;
{$ifndef Windows}
  Message(Application,evBroadcast,cmUpdate,nil);
{$endif Windows}
{$ifdef Windows}
  // WindowsShowMouse;
{$endif Windows}

  if Assigned(UserScreen) then
    UserScreen^.SwitchBackToIDEScreen;
  Video.SetCursorType(crHidden);
{$ifdef Windows}
  { This message was sent when the VideoBuffer was smaller
    than was the IdeApp thought => writes to random memory and random crashes... PM }
  Message(Application,evBroadcast,cmUpdate,nil);
{$endif Windows}
{$ifdef Unix}
  SetKnownKeys;
{$endif Unix}
 {$ifndef Windows}
{$ifndef go32v2}
  UpdateScreen(true);
{$endif go32v2}
{$endif Windows}
  displaymode:=dmIDE;
end;

function TIDEApp.AutoSave: boolean;
var IOK,SOK,DOK: boolean;
begin
  IOK:=true; SOK:=true; DOK:=true;
  if (AutoSaveOptions and asEnvironment)<>0 then
    begin
      IOK:=WriteINIFile(false);
      if IOK=false then
        ErrorBox(error_saving_cfg_file,nil);
    end;
  if (AutoSaveOptions and asEditorFiles)<>0 then { was a typo here ("=0") - Gabor }
      SOK:=SaveAll;
  if (AutoSaveOptions and asDesktop)<>0 then
    begin
      { destory all help & browser windows - we don't want to store them }
      { UserScreenWindow is also not registered PM }
      DoCloseUserScreenWindow;
      {$IFNDEF NODEBUG}
      DoneDisassemblyWindow;
      {$ENDIF}
      CloseHelpWindows;
      CloseAllBrowsers;
      DOK:=SaveDesktop;
      if DOK=false then
        ErrorBox(error_saving_dsk_file,nil);
    end;
  AutoSave:=IOK and SOK and DOK;
end;

function TIDEApp.DoExecute(ProgramPath, Params, InFile,OutFile,ErrFile: string; ExecType: TExecType): boolean;
var CanRun: boolean;
    ConsoleMode : TConsoleMode;
{$ifndef Unix}
    PosExe: sw_integer;
{$endif Unix}
begin
  SaveCancelled:=false;
  CanRun:=AutoSave;
  if (CanRun=false) and (SaveCancelled=false) then
    CanRun:=true; { do not care about .DSK or .INI saving problems - just like TP }
  if CanRun then
  begin
    if UserScreen=nil then
     begin
       ErrorBox(error_user_screen_not_avail,nil);
       Exit;
     end;

    if ExecType<>exNoSwap then
      ShowUserScreen;
    SaveConsoleMode(ConsoleMode);

    if ExecType=exDosShell then
      WriteShellMsg
    else if ExecType<>exNoSwap then
      Writeln('Running "'+ProgramPath+' '+Params+'"');
     { DO NOT use COMSPEC for exe files as the
      ExitCode is lost in those cases PM }
{$ifdef HASAMIGA}
  DosExecute(ProgramPath, Params);
{$else}
{$ifndef Unix}
    posexe:=Pos('.EXE',UpCaseStr(ProgramPath));
    { if programpath was three char long => bug }
    if (posexe>0) and (posexe=Length(ProgramPath)-3) then
      begin
{$endif Unix}
        if (InFile='') and (OutFile='') and (ErrFile='') then
          DosExecute(ProgramPath,Params)
        else
          begin
            if ErrFile='' then
              ErrFile:='stderr';
            ExecuteRedir(ProgramPath,Params,InFile,OutFile,ErrFile);
          end;
{$ifndef Unix}
      end
    else if (InFile='') and (OutFile='') and (ErrFile='') then
      DosExecute(GetEnv('COMSPEC'),'/C '+ProgramPath+' '+Params)
    else
      begin
        if ErrFile='' then
          ErrFile:='stderr';
        ExecuteRedir(GetEnv('COMSPEC'),'/C '+ProgramPath+' '+Params,
          InFile,OutFile,ErrFile);
     end;
{$endif Unix}
{$endif HASAMIGA}

{$ifdef Unix}
    if (DebuggeeTTY='') and (OutFile='') and (ExecType<>exDosShell) then
      begin
        Write(' Press any key to return to IDE');
        InitKeyBoard;
        Keyboard.GetKeyEvent;
        while (Keyboard.PollKeyEvent<>0) do
         Keyboard.GetKeyEvent;
        DoneKeyboard;
      end;
{$endif}
    RestoreConsoleMode(ConsoleMode);
    if ExecType<>exNoSwap then
      ShowIDEScreen;
  end;
  DoExecute:=CanRun;
end;


procedure TIDEApp.Update;
begin
  SetCmdState([cmSaveAll],IsThereAnyEditor);
  SetCmdState([cmCloseAll,cmWindowList],IsThereAnyWindow);
  SetCmdState([cmTile,cmCascade],IsThereAnyVisibleWindow);
  SetCmdState([cmFindProcedure,cmObjects,cmModules,cmGlobals,cmSymbol],IsSymbolInfoAvailable);
{$ifndef NODEBUG}
  SetCmdState([cmResetDebugger,cmUntilReturn],assigned(debugger) and debugger^.debuggee_started);
{$endif}
  SetCmdState([cmToolsMsgNext,cmToolsMsgPrev],MessagesWindow<>nil);
  UpdateTools;
  UpdateRecentFileList;
  UpdatePrimaryFile;
  UpdateINIFile;
  Message(Application,evBroadcast,cmCommandSetChanged,nil);
  application^.redraw;
end;

procedure TIDEApp.SourceWindowClosed;
begin
  if not IsClosing then
    Update;
end;

procedure TIDEApp.CurDirChanged;
begin
  Message(Application,evBroadcast,cmUpdateTitle,nil);
  UpdatePrimaryFile;
  UpdateINIFile;
  UpdateMenu(MenuBar^.Menu);
end;


procedure TIDEApp.UpdatePrimaryFile;
begin
  SetMenuItemParam(SearchMenuItem(MenuBar^.Menu,cmPrimaryFile),SmartPath(PrimaryFile));
  SetCmdState([cmClearPrimary],PrimaryFile<>'');
  if PrimaryFile<>'' then
     SetCmdState(CompileCmds,true);
  UpdateMenu(MenuBar^.Menu);
end;

procedure TIDEApp.UpdateINIFile;
begin
  SetMenuItemParam(SearchMenuItem(MenuBar^.Menu,cmSaveINI),SmartPath(IniFileName));
end;

procedure TIDEApp.UpdateRecentFileList;
var P: PMenuItem;
    {ID,}I: word;
    FileMenu: PMenuItem;
begin
{  ID:=cmRecentFileBase;}
  FileMenu:=SearchSubMenu(MenuBar^.Menu,menuFile);
  repeat
{    Inc(ID);
    P:=SearchMenuItem(FileMenu^.SubMenu,ID);
    if FileMenu^.SubMenu^.Default=P then
      FileMenu^.SubMenu^.Default:=FileMenu^.SubMenu^.Items;
    if P<>nil then RemoveMenuItem(FileMenu^.SubMenu,P);}
    P:=GetMenuItemBefore(FileMenu^.SubMenu,nil);
    if (P<>nil) then
    begin
      if (cmRecentFileBase<P^.Command) and (P^.Command<=cmRecentFileBase+MaxRecentFileCount) then
        begin
          RemoveMenuItem(FileMenu^.SubMenu,P);
          if FileMenu^.SubMenu^.Default=P then
            FileMenu^.SubMenu^.Default:=FileMenu^.SubMenu^.Items;
        end
      else
        P:=nil;
    end;
  until P=nil;
  P:=GetMenuItemBefore(FileMenu^.SubMenu,nil);
  if (P<>nil) and IsSeparator(P) then
     RemoveMenuItem(FileMenu^.SubMenu,P);

  if RecentFileCount>0 then
     AppendMenuItem(FileMenu^.SubMenu,NewLine(nil));
  for I:=1 to RecentFileCount do
  begin
    P:=NewItem('~'+IntToStr(I)+'~ '+ShrinkPath(SmartPath(RecentFiles[I].FileName),27),' ',
        kbNoKey,cmRecentFileBase+I,hcRecentFileBase+I,nil);
    AppendMenuItem(FileMenu^.SubMenu,P);
  end;
end;

procedure TIDEApp.UpdateTools;
var P: PMenuItem;
{    ID,}I: word;
    ToolsMenu: PMenuItem;
    S1,S2,S3: string;
    W: word;
begin
{  ID:=cmToolsBase;}
  ToolsMenu:=SearchSubMenu(MenuBar^.Menu,menuTools);
  repeat
    P:=GetMenuItemBefore(ToolsMenu^.SubMenu,nil);
    if (P<>nil) then
    begin
      if (cmToolsBase<P^.Command) and (P^.Command<=cmToolsBase+MaxToolCount) then
        begin
          RemoveMenuItem(ToolsMenu^.SubMenu,P);
          if ToolsMenu^.SubMenu^.Default=P then
            ToolsMenu^.SubMenu^.Default:=ToolsMenu^.SubMenu^.Items;
        end
      else
        P:=nil;
    end;
  until P=nil;
  P:=GetMenuItemBefore(ToolsMenu^.SubMenu,nil);
  if (P<>nil) and IsSeparator(P) then
     RemoveMenuItem(ToolsMenu^.SubMenu,P);

  if GetToolCount>0 then
     AppendMenuItem(ToolsMenu^.SubMenu,NewLine(nil));
  for I:=1 to GetToolCount do
  begin
    GetToolParams(I-1,S1,S2,S3,W);
    P:=NewItem(S1,KillTilde(GetHotKeyName(W)),W,cmToolsBase+I,hcToolsBase+I,nil);
    AppendMenuItem(ToolsMenu^.SubMenu,P);
  end;
end;

procedure TIDEApp.DosShell;
var
  s : string;
begin
{$ifdef HASAMIGA}
  s := 'C:NewShell';
{$else}
{$ifdef Unix}
  s:=GetEnv('SHELL');
  if s='' then
    if ExistsFile('/bin/sh') then
      s:='/bin/sh';
{$else}
  s:=GetEnv('COMSPEC');
  if s='' then
    if ExistsFile('c:\command.com') then
      s:='c:\command.com'
    else
      begin
        s:='command.com';
        if Not LocateExeFile(s) then
          s:='';
      end;
{$endif}
{$endif}
  if s='' then
    ErrorBox(msg_errorexecutingshell,nil)
  else
    DoExecute(s, '', '', '', '', exDosShell);
  { In case we have something that the compiler touched }
  AskToReloadAllModifiedFiles;
end;

procedure TIDEApp.ShowReadme;
var R,R2: TRect;
    D: PCenterDialog;
    M: PFPMemo;
    VSB: PScrollBar;
    S: PFastBufStream;
begin
  New(S, Init(ReadmeName, stOpenRead, 4096));
  if S^.Status=stOK then
  begin
    R.Assign(0,0,63,18);
    New(D, Init(R, 'Free Pascal IDE'));
    with D^ do
    begin
      GetExtent(R);
      R.Grow(-2,-2); Inc(R.B.Y);
      R2.Copy(R); R2.Move(1,0); R2.A.X:=R2.B.X-1;
      New(VSB, Init(R2)); VSB^.GrowMode:=0; Insert(VSB);
      New(M, Init(R,nil,VSB,nil));
      M^.LoadFromStream(S);
      M^.ReadOnly:=true;
      Insert(M);
    end;
    InsertOK(D);
    ExecuteDialog(D,nil);
  end;
  Dispose(S, Done);
end;

{$I FPMFILE.INC}

{$I FPMEDIT.INC}

{$I FPMSRCH.INC}

{$I FPMRUN.INC}

{$I FPMCOMP.INC}

{$I FPMDEBUG.INC}

{$I FPMTOOLS.INC}

{$I FPMOPTS.INC}

{$I FPMWND.INC}

{$I FPMHELP.INC}

{$I fpmansi.inc}

procedure TIDEApp.AddRecentFile(AFileName: string; CurX, CurY: sw_integer);
begin
  if SearchRecentFile(AFileName)<>-1 then Exit;
  if RecentFileCount>0 then
   Move(RecentFiles[1],RecentFiles[2],SizeOf(RecentFiles[1])*Min(RecentFileCount,High(RecentFiles)-1));
  if RecentFileCount<High(RecentFiles) then Inc(RecentFileCount);
  with RecentFiles[1] do
  begin
    FileName:=AFileName;
    LastPos.X:=CurX; LastPos.Y:=CurY;
  end;
  UpdateRecentFileList;
end;

function TIDEApp.SearchRecentFile(AFileName: string): integer;
var Idx,I: integer;
begin
  Idx:=-1;
  for I:=1 to RecentFileCount do
    if UpcaseStr(AFileName)=UpcaseStr(RecentFiles[I].FileName) then
      begin Idx:=I; Break; end;
  SearchRecentFile:=Idx;
end;

procedure TIDEApp.RemoveRecentFile(Index: integer);
begin
  if Index<RecentFileCount then
     Move(RecentFiles[Index+1],RecentFiles[Index],SizeOf(RecentFiles[1])*(RecentFileCount-Index));
  Dec(RecentFileCount);
  UpdateRecentFileList;
end;

function TIDEApp.GetPalette: PPalette;
begin
  GetPalette:=@AppPalette;
end;

function TIDEApp.IsClosing: Boolean;
begin
  IsClosing:=InsideDone;
end;

destructor TIDEApp.Done;
begin
  InsideDone:=true;
  IsRunning:=false;
  inherited Done;
  Desktop:=nil;
  RemoveBrowsersCollection;
  DoneHelpSystem;
end;



end.
