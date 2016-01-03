{
    This file is part of the Free Pascal run time library.
    This unit contains the record definition for the Win32 API
    Copyright (c) 1999-2002 by Marco van de Voort,
    member of the Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 ************************************************************************}

{ leave out unused functions so the unit can be used on win2000 as well }
{$ifndef NO_SMART_LINK}
{$smartlink on}
{$endif}

{$PACKRECORDS C}
{$calling stdcall}
{$mode objfpc}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef FPC_OS_UNICODE}
  {$define UNICODE}
{$endif}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Unit ShellApi;

//+-------------------------------------------------------------------------
//
//  Microsoft Windows
//  Copyright (c) Microsoft Corporation. All rights reserved.
//
//  File: shellapi.h
//
//  Header translation by Marco van de Voort for Free Pascal Platform
//  SDK dl'ed January 2002
//
//--------------------------------------------------------------------------


Interface

Uses Windows, ActiveX;
  {
    shellapi.h -  SHELL.DLL functions, types, and definitions
    Copyright (c) Microsoft Corporation. All rights reserved.             }

<<<<<<< HEAD
<<<<<<< HEAD
const
   shell32 =  'shell32.dll';

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
Const 
    SHGDN_NORMAL             = $0000;  // default (display purpose)
    SHGDN_INFOLDER           = $0001;  // displayed under a folder (relative)
    SHGDN_FOREDITING         = $1000;  // for in-place editing
    SHGDN_FORADDRESSBAR      = $4000;  // UI friendly parsing name (remove ugly stuff)
    SHGDN_FORPARSING         = $8000;  // parsing name for ParseDisplayName()

    SHCONTF_FOLDERS             = $0020;   // only want folders enumerated (SFGAO_FOLDER)
    SHCONTF_NONFOLDERS          = $0040;   // include non folders
    SHCONTF_INCLUDEHIDDEN       = $0080;   // show items normally hidden
    SHCONTF_INIT_ON_FIRST_NEXT  = $0100;   // allow EnumObject() to return before validating enum
    SHCONTF_NETPRINTERSRCH      = $0200;   // hint that client is looking for printers
    SHCONTF_SHAREABLE           = $0400;   // hint that client is looking sharable resources (remote shares)
    SHCONTF_STORAGE             = $0800;   // include all items with accessible storage and their ancestors

    SHCIDS_ALLFIELDS        = $80000000;
    SHCIDS_CANONICALONLY    = $10000000;
    SHCIDS_BITMASK          = $FFFF0000;
    SHCIDS_COLUMNMASK       = $0000FFFF;
    SFGAO_CANCOPY           = DROPEFFECT_COPY; // Objects can be copied    (= $1)
    SFGAO_CANMOVE           = DROPEFFECT_MOVE; // Objects can be moved     (= $2)
    SFGAO_CANLINK           = DROPEFFECT_LINK; // Objects can be linked    (= $4)
    SFGAO_STORAGE           = $00000008;     // supports BindToObject(IID_IStorage)
    SFGAO_CANRENAME         = $00000010;     // Objects can be renamed
    SFGAO_CANDELETE         = $00000020;     // Objects can be deleted
    SFGAO_HASPROPSHEET      = $00000040;     // Objects have property sheets
    SFGAO_DROPTARGET        = $00000100;     // Objects are drop target
    SFGAO_CAPABILITYMASK    = $00000177;
    SFGAO_ENCRYPTED         = $00002000;     // object is encrypted (use alt color)
    SFGAO_ISSLOW            = $00004000;     // 'slow' object
    SFGAO_GHOSTED           = $00008000;     // ghosted icon
    SFGAO_LINK              = $00010000;     // Shortcut (link)
    SFGAO_SHARE             = $00020000;     // shared
    SFGAO_READONLY          = $00040000;     // read-only
    SFGAO_HIDDEN            = $00080000;     // hidden object
    SFGAO_DISPLAYATTRMASK   = $000FC000;
    SFGAO_FILESYSANCESTOR   = $10000000;     // may contain children with SFGAO_FILESYSTEM
    SFGAO_FOLDER            = $20000000;     // support BindToObject(IID_IShellFolder)
    SFGAO_FILESYSTEM        = $40000000;     // is a win32 file system object (file/folder/root)
    SFGAO_HASSUBFOLDER      = $80000000;     // may contain children with SFGAO_FOLDER
    SFGAO_CONTENTSMASK      = $80000000;
    SFGAO_VALIDATE          = $01000000;     // invalidate cached information
    SFGAO_REMOVABLE         = $02000000;     // is this removeable media?
    SFGAO_COMPRESSED        = $04000000;     // Object is compressed (use alt color)
    SFGAO_BROWSABLE         = $08000000;     // supports IShellFolder, but only implements CreateViewObject() (non-folder view)
    SFGAO_NONENUMERATED     = $00100000;     // is a non-enumerated object
    SFGAO_NEWCONTENT        = $00200000;     // should show bold in explorer tree
    SFGAO_CANMONIKER        = $00400000;     // defunct
    SFGAO_HASSTORAGE        = $00400000;     // defunct
    SFGAO_STREAM            = $00400000;     // supports BindToObject(IID_IStream)
    SFGAO_STORAGEANCESTOR   = $00800000;     // may contain children with SFGAO_STORAGE or SFGAO_STREAM
    SFGAO_STORAGECAPMASK    = $70C50008;     // for determining storage capabilities, ie for open/save semantics
    
    SHCOLSTATE_TYPE_STR	        =  $1;
    SHCOLSTATE_TYPE_INT	        =  $2;
    SHCOLSTATE_TYPE_DATE	=  $3;
    SHCOLSTATE_TYPEMASK	        =  $f;
    SHCOLSTATE_ONBYDEFAULT	= $10;
    SHCOLSTATE_SLOW	        = $20;
    SHCOLSTATE_EXTENDED	        = $40;
    SHCOLSTATE_SECONDARYUI	= $80;
    SHCOLSTATE_HIDDEN 	        = $100;
    SHCOLSTATE_PREFER_VARCMP	= $200;

    GIL_OPENICON     = $0001;      // allows containers to specify an "open" look
    GIL_FORSHELL     = $0002;      // icon is to be displayed in a ShellFolder
    GIL_ASYNC        = $0020;      // this is an async extract, return E_PENDING
    GIL_DEFAULTICON  = $0040;      // get the default icon location if the final one takes too long to get
    GIL_FORSHORTCUT  = $0080;      // the icon is for a shortcut to the object

// GetIconLocation() return flags

    GIL_SIMULATEDOC  = $0001;      // simulate this document icon for this
    GIL_PERINSTANCE  = $0002;      // icons from this class are per instance (each file has its own)
    GIL_PERCLASS     = $0004;      // icons from this class per class (shared for all files of this type)
    GIL_NOTFILENAME  = $0008;      // location is not a filename, must call ::ExtractIcon
    GIL_DONTCACHE    = $0010;      // this icon should not be cached

// QueryContextMenu uFlags
    CMF_NORMAL              = $00000000;
    CMF_DEFAULTONLY         = $00000001;
    CMF_VERBSONLY           = $00000002;
    CMF_EXPLORE             = $00000004;
    CMF_NOVERBS             = $00000008;
    CMF_CANRENAME           = $00000010;
    CMF_NODEFAULT           = $00000020;
    CMF_INCLUDESTATIC       = $00000040;
    CMF_EXTENDEDVERBS       = $00000100;      // rarely used verbs
    CMF_RESERVED            = $ffff0000;      // View specific

// GetCommandString uFlags
    GCS_VERBA        = $00000000;     // canonical verb
    GCS_HELPTEXTA    = $00000001;     // help text (for status bar)
    GCS_VALIDATEA    = $00000002;     // validate command exists
    GCS_VERBW        = $00000004;     // canonical verb (unicode)
    GCS_HELPTEXTW    = $00000005;     // help text (unicode version)
    GCS_VALIDATEW    = $00000006;     // validate command exists (unicode)
    GCS_UNICODE      = $00000004;     // for bit testing - Unicode string

{$ifdef UNICODE}
    GCS_VERB        = GCS_VERBW;
    GCS_HELPTEXT    = GCS_HELPTEXTW;
    GCS_VALIDATE    = GCS_VALIDATEW;
{$else}
    GCS_VERB        = GCS_VERBA;
    GCS_HELPTEXT    = GCS_HELPTEXTA;
    GCS_VALIDATE    = GCS_VALIDATEA;
{$endif}

    SLDF_HAS_ID_LIST         = $00000001;   // Shell link saved with ID list
    SLDF_HAS_LINK_INFO       = $00000002;   // Shell link saved with LinkInfo
    SLDF_HAS_NAME            = $00000004;
    SLDF_HAS_RELPATH         = $00000008;
    SLDF_HAS_WORKINGDIR      = $00000010;
    SLDF_HAS_ARGS            = $00000020;
    SLDF_HAS_ICONLOCATION    = $00000040;
    SLDF_UNICODE             = $00000080;   // the strings are unicode
    SLDF_FORCE_NO_LINKINFO   = $00000100;   // don't create a LINKINFO (make a dumb link)
    SLDF_HAS_EXP_SZ          = $00000200;   // the link contains expandable env strings
    SLDF_RUN_IN_SEPARATE     = $00000400;   // Run the 16-bit target exe in a separate VDM/WOW
    SLDF_HAS_LOGO3ID         = $00000800;   // this link is a special Logo3/MSICD link
    SLDF_HAS_DARWINID        = $00001000;   // this link is a special Darwin link
    SLDF_RUNAS_USER          = $00002000;   // Run this link as a different user
    SLDF_HAS_EXP_ICON_SZ     = $00004000;   // contains expandable env string for icon path
    SLDF_NO_PIDL_ALIAS       = $00008000;   // don't ever resolve to a logical location
    SLDF_FORCE_UNCNAME       = $00010000;   // make GetPath() prefer the UNC name to the local name
    SLDF_RUN_WITH_SHIMLAYER  = $00020000;   // Launch the target of this link w/ shim layer active
    SLDF_RESERVED            = $80000000;   // Reserved-- so we can use the low word as an index value in the future

// Define File View Show Info Flags.
    FVSIF_RECT      = $00000001;      // The rect variable has valid data.
    FVSIF_PINNED    = $00000002;      // We should Initialize pinned
    FVSIF_NEWFAILED = $08000000;      // The new file passed back failed
   				    // to be viewed.
    FVSIF_NEWFILE   = dword($80000000);      // A new file to view has been returned
    FVSIF_CANVIEWIT = $40000000;      // The viewer can view it.

    FCIDM_SHVIEWFIRST           = $0000;
    FCIDM_SHVIEWLAST            = $7fff;
    FCIDM_BROWSERFIRST          = $a000;
    FCIDM_BROWSERLAST           = $bf00;
    FCIDM_GLOBALFIRST           = $8000;
    FCIDM_GLOBALLAST            = $9fff;

//
// Global submenu IDs and separator IDs
//
    FCIDM_MENU_FILE             = (FCIDM_GLOBALFIRST+$0000);
    FCIDM_MENU_EDIT             = (FCIDM_GLOBALFIRST+$0040);
    FCIDM_MENU_VIEW             = (FCIDM_GLOBALFIRST+$0080);
    FCIDM_MENU_VIEW_SEP_OPTIONS = (FCIDM_GLOBALFIRST+$0081);
    FCIDM_MENU_TOOLS            = (FCIDM_GLOBALFIRST+$00c0); // for Win9x compat
    FCIDM_MENU_TOOLS_SEP_GOTO   = (FCIDM_GLOBALFIRST+$00c1); // for Win9x compat
    FCIDM_MENU_HELP             = (FCIDM_GLOBALFIRST+$0100);
    FCIDM_MENU_FIND             = (FCIDM_GLOBALFIRST+$0140);
    FCIDM_MENU_EXPLORE          = (FCIDM_GLOBALFIRST+$0150);
    FCIDM_MENU_FAVORITES        = (FCIDM_GLOBALFIRST+$0170);

    PANE_NONE        = -1;
    PANE_ZONE        = 1;
    PANE_OFFLINE     = 2;
    PANE_PRINTER     = 3;
    PANE_SSL         = 4;
    PANE_NAVIGATION  = 5;
    PANE_PROGRESS    = 6;
    PANE_PRIVACY     = 7;
    CDBOSC_SETFOCUS         = $00000000;
    CDBOSC_KILLFOCUS        = $00000001;
    CDBOSC_SELCHANGE        = $00000002;
    CDBOSC_RENAME           = $00000003;
    CDBOSC_STATECHANGE      = $00000004;
    CDB2N_CONTEXTMENU_DONE  = $00000001;
    CDB2N_CONTEXTMENU_START = $00000002;

//GetViewFlags
    CDB2GVF_SHOWALLFILES        = $00000001;

    CSIDL_DESKTOP                   = $0000;        // <desktop>
    CSIDL_INTERNET                  = $0001;        // Internet Explorer (icon on desktop)
    CSIDL_PROGRAMS                  = $0002;        // Start Menu\Programs
    CSIDL_CONTROLS                  = $0003;        // My Computer\Control Panel
    CSIDL_PRINTERS                  = $0004;        // My Computer\Printers
    CSIDL_PERSONAL                  = $0005;        // My Documents
    CSIDL_FAVORITES                 = $0006;        // <user name>\Favorites
    CSIDL_STARTUP                   = $0007;        // Start Menu\Programs\Startup
    CSIDL_RECENT                    = $0008;        // <user name>\Recent
    CSIDL_SENDTO                    = $0009;        // <user name>\SendTo
    CSIDL_BITBUCKET                 = $000a;        // <desktop>\Recycle Bin
    CSIDL_STARTMENU                 = $000b;        // <user name>\Start Menu
    CSIDL_MYDOCUMENTS               = $000c;        // logical "My Documents" desktop icon
    CSIDL_MYMUSIC                   = $000d;        // "My Music" folder
    CSIDL_MYVIDEO                   = $000e;        // "My Videos" folder
    CSIDL_DESKTOPDIRECTORY          = $0010;        // <user name>\Desktop
    CSIDL_DRIVES                    = $0011;        // My Computer
    CSIDL_NETWORK                   = $0012;        // Network Neighborhood (My Network Places)
    CSIDL_NETHOOD                   = $0013;        // <user name>\nethood
    CSIDL_FONTS                     = $0014;        // windows\fonts
    CSIDL_TEMPLATES                 = $0015;
    CSIDL_COMMON_STARTMENU          = $0016;        // All Users\Start Menu
    CSIDL_COMMON_PROGRAMS           = $0017;        // All Users\Start Menu\Programs
    CSIDL_COMMON_STARTUP            = $0018;        // All Users\Startup
    CSIDL_COMMON_DESKTOPDIRECTORY   = $0019;        // All Users\Desktop
    CSIDL_APPDATA                   = $001a;        // <user name>\Application Data
    CSIDL_PRINTHOOD                 = $001b;        // <user name>\PrintHood
    CSIDL_LOCAL_APPDATA             = $001c;        // <user name>\Local Settings\Applicaiton Data (non roaming)
    CSIDL_ALTSTARTUP                = $001d;        // non localized startup
    CSIDL_COMMON_ALTSTARTUP         = $001e;        // non localized common startup
    CSIDL_COMMON_FAVORITES          = $001f;
    CSIDL_INTERNET_CACHE            = $0020;
    CSIDL_COOKIES                   = $0021;
    CSIDL_HISTORY                   = $0022;
    CSIDL_COMMON_APPDATA            = $0023;        // All Users\Application Data
    CSIDL_WINDOWS                   = $0024;        // GetWindowsDirectory()
    CSIDL_SYSTEM                    = $0025;        // GetSystemDirectory()
    CSIDL_PROGRAM_FILES             = $0026;        // C:\Program Files
    CSIDL_MYPICTURES                = $0027;        // C:\Program Files\My Pictures
    CSIDL_PROFILE                   = $0028;        // USERPROFILE
    CSIDL_SYSTEMX86                 = $0029;        // x86 system directory on RISC
    CSIDL_PROGRAM_FILESX86          = $002a;        // x86 C:\Program Files on RISC
    CSIDL_PROGRAM_FILES_COMMON      = $002b;        // C:\Program Files\Common
    CSIDL_PROGRAM_FILES_COMMONX86   = $002c;        // x86 Program Files\Common on RISC
    CSIDL_COMMON_TEMPLATES          = $002d;        // All Users\Templates
    CSIDL_COMMON_DOCUMENTS          = $002e;        // All Users\Documents
    CSIDL_COMMON_ADMINTOOLS         = $002f;        // All Users\Start Menu\Programs\Administrative Tools
    CSIDL_ADMINTOOLS                = $0030;        // <user name>\Start Menu\Programs\Administrative Tools
    CSIDL_CONNECTIONS               = $0031;        // Network and Dial-up Connections
    CSIDL_COMMON_MUSIC              = $0035;        // All Users\My Music
    CSIDL_COMMON_PICTURES           = $0036;        // All Users\My Pictures
    CSIDL_COMMON_VIDEO              = $0037;        // All Users\My Video
    CSIDL_RESOURCES                 = $0038;        // Resource Direcotry
    CSIDL_RESOURCES_LOCALIZED       = $0039;        // Localized Resource Direcotry
    CSIDL_COMMON_OEM_LINKS          = $003a;        // Links to All Users OEM specific apps
    CSIDL_CDBURN_AREA               = $003b;        // USERPROFILE\Local Settings\Application Data\Microsoft\CD Burning
// unused                               = $003c
    CSIDL_COMPUTERSNEARME           = $003d;        // Computers Near Me (computered from Workgroup membership)

    CSIDL_FLAG_CREATE               = $8000;        // combine with CSIDL_ value to force folder creation in SHGetFolderPath()
    CSIDL_FLAG_DONT_VERIFY          = $4000;        // combine with CSIDL_ value to return an unverified folder path
    CSIDL_FLAG_NO_ALIAS             = $1000;        // combine with CSIDL_ value to insure non-alias versions of the pidl
    CSIDL_FLAG_PER_USER_INIT        = $0800;        // combine with CSIDL_ value to indicate per-user init (eg. upgrade)
    CSIDL_FLAG_MASK                 = $FF00;        // mask for all possible flag values
    FCS_READ                    = $00000001;
    FCS_FORCEWRITE              = $00000002;
    FCS_WRITE                   = (FCS_READ or FCS_FORCEWRITE);

    FCS_FLAG_DRAGDROP           = 2;

// Mask which values have been retreived or being set.
    FCSM_VIEWID                 = $00000001;
    FCSM_WEBVIEWTEMPLATE        = $00000002;
    FCSM_INFOTIP                = $00000004;
    FCSM_CLSID                  = $00000008;
    FCSM_ICONFILE               = $00000010;
    FCSM_LOGO                   = $00000020;
    FCSM_FLAGS                  = $00000040;

// Browsing for directory.
    BIF_RETURNONLYFSDIRS   = $0001;  // For finding a folder to start document searching
    BIF_DONTGOBELOWDOMAIN  = $0002;  // For starting the Find Computer
    BIF_STATUSTEXT         = $0004;   // Top of the dialog has 2 lines of text for BROWSEINFO.lpszTitle and one line if
                                        // this flag is set.  Passing the message BFFM_SETSTATUSTEXTA to the hwnd can set the
                                        // rest of the text.  This is not used with BIF_USENEWUI and BROWSEINFO.lpszTitle gets
                                        // all three lines of text.
    BIF_RETURNFSANCESTORS  = $0008;
    BIF_EDITBOX            = $0010;   // Add an editbox to the dialog
    BIF_VALIDATE           = $0020;   // insist on valid result (or CANCEL)

    BIF_NEWDIALOGSTYLE     = $0040;   // Use the new dialog layout with the ability to resize
                                        // Caller needs to call OleInitialize() before using this API

    BIF_USENEWUI           = (BIF_NEWDIALOGSTYLE or BIF_EDITBOX);

    BIF_BROWSEINCLUDEURLS  = $0080;   // Allow URLs to be displayed or entered. (Requires BIF_USENEWUI)
    BIF_UAHINT             = $0100;   // Add a UA hint to the dialog, in place of the edit box. May not be combined with BIF_EDITBOX
    BIF_NONEWFOLDERBUTTON  = $0200;   // Do not add the "New Folder" button to the dialog.  Only applicable with BIF_NEWDIALOGSTYLE.
    BIF_NOTRANSLATETARGETS = $0400;   // don't traverse target as shortcut

    BIF_BROWSEFORCOMPUTER  = $1000;  // Browsing for Computers.
    BIF_BROWSEFORPRINTER   = $2000;  // Browsing for Printers
    BIF_BROWSEINCLUDEFILES = $4000;  // Browsing for Everything
    BIF_SHAREABLE          = $8000;  // sharable resources displayed (remote shares, requires BIF_USENEWUI)

// message from browser
    BFFM_INITIALIZED        = 1;
    BFFM_SELCHANGED         = 2;
    BFFM_VALIDATEFAILEDA    = 3;   // lParam:szPath ret:1(cont),0(EndDialog)
    BFFM_VALIDATEFAILEDW    = 4;   // lParam:wzPath ret:1(cont),0(EndDialog)
    BFFM_IUNKNOWN           = 5;   // provides IUnknown to client. lParam: IUnknown*

// messages to browser
    BFFM_SETSTATUSTEXTA     = (WM_USER + 100);
    BFFM_ENABLEOK           = (WM_USER + 101);
    BFFM_SETSELECTIONA      = (WM_USER + 102);
    BFFM_SETSELECTIONW      = (WM_USER + 103);
    BFFM_SETSTATUSTEXTW     = (WM_USER + 104);
    BFFM_SETOKTEXT          = (WM_USER + 105); // Unicode only
    BFFM_SETEXPANDED        = (WM_USER + 106); // Unicode only
    {$IFDEF UNICODE}
      BFFM_SETSTATUSTEXT    =      BFFM_SETSTATUSTEXTW;
      BFFM_SETSELECTION     =      BFFM_SETSELECTIONW;
    {$ELSE}
      BFFM_SETSTATUSTEXT    =      BFFM_SETSTATUSTEXTA;
      BFFM_SETSELECTION     =      BFFM_SETSELECTIONA;
    {$ENDIF}
    ISHCUTCMDID_DOWNLOADICON  = 0;
    ISHCUTCMDID_INTSHORTCUTCREATE = 1;
    ACLO_NONE            = 0;    // don't enumerate anything
    ACLO_CURRENTDIR      = 1;    // enumerate current directory
    ACLO_MYCOMPUTER      = 2;    // enumerate MyComputer
    ACLO_DESKTOP         = 4;    // enumerate Desktop Folder
    ACLO_FAVORITES       = 8;    // enumerate Favorites Folder
    ACLO_FILESYSONLY     = 16;   // enumerate only the file system
    ACLO_FILESYSDIRS     = 32;   // enumerate only the file system dirs, UNC shares, and UNC servers.
// Flags for IProgressDialog::StartProgressDialog() (dwFlags)

    PROGDLG_NORMAL          = $00000000;      // default normal progress dlg behavior
    PROGDLG_MODAL           = $00000001;      // the dialog is modal to its hwndParent (default is modeless)
    PROGDLG_AUTOTIME        = $00000002;      // automatically updates the "Line3" text with the "time remaining" (you cant call SetLine3 if you passs this!)
    PROGDLG_NOTIME          = $00000004;      // we dont show the "time remaining" if this is set. We need this if dwTotal < dwCompleted for sparse files
    PROGDLG_NOMINIMIZE      = $00000008;      // Do not have a minimize button in the caption bar.
    PROGDLG_NOPROGRESSBAR   = $00000010;      // Don't display the progress bar

// Time Actions (dwTimerAction)
    PDTIMER_RESET           = $00000001;       // Reset the timer so the progress will be calculated from now until the first ::SetProgress() is called so
                                             // those this time will correspond to the values passed to ::SetProgress().  Only do this before ::SetProgress() is called.
// flags for RemoveToolbar
    DWFRF_NORMAL            = $0000;
    DWFRF_DELETECONFIGDATA  = $0001;

// flags for AddToolbar
    DWFAF_HIDDEN            = $0001;   // add hidden

// Convenient state values
    IRTIR_TASK_NOT_RUNNING  = 0;
    IRTIR_TASK_RUNNING      = 1;
    IRTIR_TASK_SUSPENDED    = 2;
    IRTIR_TASK_PENDING      = 3;
    IRTIR_TASK_FINISHED     = 4;

// Status() flags,
// wait for the current task to complete before deleting the scheduler
    ITSSFLAG_COMPLETE_ON_DESTROY        = $0000;

// kill the current task (if there is one) when the task scheduler is deleted
    ITSSFLAG_KILL_ON_DESTROY            = $0001;
    ITSSFLAG_SUPPORTS_TERMINATE         = $0002;
    ITSSFLAG_FLAGS_MASK                 = $0003;

// set the timeout for killing the thread when the object is terminated.
// this timeout can be used to stop the object from blocking the system
// indefinitely.
    ITSSFLAG_THREAD_TERMINATE_TIMEOUT   = $0010;

// set the timeout for threads that are idle in the thread pool
    ITSSFLAG_THREAD_POOL_TIMEOUT        = $0020;

// The default timeout passed to release Status to determine how long the thread
// can be asleep before the thread is expired
    ITSS_THREAD_DESTROY_DEFAULT_TIMEOUT     = (60*1000);

// default, we won't kill it...
    ITSS_THREAD_TERMINATE_TIMEOUT           = (INFINITE);

// there is no change to the thread timeout
    ITSS_THREAD_TIMEOUT_NO_CHANGE           = (INFINITE - 1);

// the LPARAM allows task to be associated with items thus all tasks owned by a
// particular item can be accessed by passing a non default value for this parameter
    ITSAT_DEFAULT_LPARAM        = $ffffffff;

// Task priorities
// ---------------
// This depends on the cooperation of tasks currently under execution. New tasks will
// be inserted in the queue in priority order. If a task of a low priority is currently
// under execution when a higher priority task is added, the scheduler will attempt
// to suspend the task currently under execution. It will be resumed when the other tasks
// have been completed.
    ITSAT_DEFAULT_PRIORITY      = $10000000;
    ITSAT_MAX_PRIORITY          = $7fffffff;
    ITSAT_MIN_PRIORITY          = $00000000;
    ITSSFLAG_TASK_PLACEINFRONT  = $00000001;
    ITSSFLAG_TASK_PLACEINBACK   = $00000002;
    SHIMSTCAPFLAG_LOCKABLE      = $0001;       // does the store require/support locking
    SHIMSTCAPFLAG_PURGEABLE     = $0002;       // does the store require dead items purging externally ?
////  IShellFolderBand

// Field mask
    ISFB_MASK_STATE          = $00000001; // TRUE if dwStateMask and dwState is valid
    ISFB_MASK_BKCOLOR        = $00000002; // TRUE if crBkgnd field is valid
    ISFB_MASK_VIEWMODE       = $00000004; // TRUE if wViewMode field is valid
    ISFB_MASK_SHELLFOLDER    = $00000008;
    ISFB_MASK_IDLIST         = $00000010;
    ISFB_MASK_COLORS         = $00000020; // TRUE if crXXXX fields are valid (except bkgnd)

    ISFB_STATE_DEFAULT       = $00000000;
    ISFB_STATE_DEBOSSED      = $00000001;
    ISFB_STATE_ALLOWRENAME   = $00000002;
    ISFB_STATE_NOSHOWTEXT    = $00000004; // TRUE if _fNoShowText
    ISFB_STATE_CHANNELBAR    = $00000010; // TRUE if we want NavigateTarget support
    ISFB_STATE_QLINKSMODE    = $00000020; // TRUE if we want to turn off drag & drop onto content items
    ISFB_STATE_FULLOPEN      = $00000040; // TRUE if band should maximize when opened
    ISFB_STATE_NONAMESORT    = $00000080; // TRUE if band should _not_ sort icons by name
    ISFB_STATE_BTNMINSIZE    = $00000100; // TRUE if band should report min thickness of button

    ISFBVIEWMODE_SMALLICONS  = $0001;
    ISFBVIEWMODE_LARGEICONS  = $0002;
    ISFBVIEWMODE_LOGOS       = $0003;

    DBC_GS_IDEAL             = 0;  // get the ideal size
    DBC_GS_SIZEDOWN          = 1;  // clip the height of a rect to a multiple of the rebar's integral size


    DBC_HIDE                 = 0; // Band is hidden (being destroyed)
    DBC_SHOW                 = 1; // Band is visible
    DBC_SHOWOBSCURE          = 2; // Band is completely obscured
// iCompType values
    COMP_TYPE_HTMLDOC        = 0;
    COMP_TYPE_PICTURE        = 1;
    COMP_TYPE_WEBSITE        = 2;
    COMP_TYPE_CONTROL        = 3;
    COMP_TYPE_CFHTML         = 4;
    COMP_TYPE_MAX            = 4;
// Defines for dwCurItemState
    IS_NORMAL               = $00000001;
    IS_FULLSCREEN           = $00000002;
    IS_SPLIT                = $00000004;
    IS_VALIDSIZESTATEBITS   = (IS_NORMAL  or  IS_SPLIT  or  IS_FULLSCREEN);  // The set of IS_* state bits which define the "size" of the component - these bits are mutually exclusive.
    IS_VALIDSTATEBITS       = (IS_NORMAL  or  IS_SPLIT  or  IS_FULLSCREEN or $80000000 or $40000000);  // All of the currently defined IS_* bits.

////////////////////////////////////////////
// Flags for IActiveDesktop::ApplyChanges()
    AD_APPLY_SAVE             = $00000001;
    AD_APPLY_HTMLGEN          = $00000002;
    AD_APPLY_REFRESH          = $00000004;
    AD_APPLY_ALL              = (AD_APPLY_SAVE or AD_APPLY_HTMLGEN  or  AD_APPLY_REFRESH);
    AD_APPLY_FORCE            = $00000008;
    AD_APPLY_BUFFERED_REFRESH = $00000010;
    AD_APPLY_DYNAMICREFRESH   = $00000020;

////////////////////////////////////////////
// Flags for IActiveDesktop::GetWallpaperOptions()
//           IActiveDesktop::SetWallpaperOptions()
    WPSTYLE_CENTER      = 0;
    WPSTYLE_TILE        = 1;
    WPSTYLE_STRETCH     = 2;
    WPSTYLE_MAX         = 3;

////////////////////////////////////////////
// Flags for IActiveDesktop::ModifyComponent()

    COMP_ELEM_TYPE          = $00000001;
    COMP_ELEM_CHECKED       = $00000002;
    COMP_ELEM_DIRTY         = $00000004;
    COMP_ELEM_NOSCROLL      = $00000008;
    COMP_ELEM_POS_LEFT      = $00000010;
    COMP_ELEM_POS_TOP       = $00000020;
    COMP_ELEM_SIZE_WIDTH    = $00000040;
    COMP_ELEM_SIZE_HEIGHT   = $00000080;
    COMP_ELEM_POS_ZINDEX    = $00000100;
    COMP_ELEM_SOURCE        = $00000200;
    COMP_ELEM_FRIENDLYNAME  = $00000400;
    COMP_ELEM_SUBSCRIBEDURL = $00000800;
    COMP_ELEM_ORIGINAL_CSI  = $00001000;
    COMP_ELEM_RESTORED_CSI  = $00002000;
    COMP_ELEM_CURITEMSTATE  = $00004000;

    COMP_ELEM_ALL  =   (COMP_ELEM_TYPE or COMP_ELEM_CHECKED or COMP_ELEM_DIRTY or                     
                         COMP_ELEM_NOSCROLL or COMP_ELEM_POS_LEFT or COMP_ELEM_SIZE_WIDTH  or          
                         COMP_ELEM_SIZE_HEIGHT or COMP_ELEM_POS_ZINDEX or COMP_ELEM_SOURCE or          
                         COMP_ELEM_FRIENDLYNAME or COMP_ELEM_POS_TOP or COMP_ELEM_SUBSCRIBEDURL or     
                         COMP_ELEM_ORIGINAL_CSI or COMP_ELEM_RESTORED_CSI or COMP_ELEM_CURITEMSTATE);


////////////////////////////////////////////
// Flags for IActiveDesktop::AddDesktopItemWithUI()
    DTI_ADDUI_DEFAULT               = $00000000;
    DTI_ADDUI_DISPSUBWIZARD         = $00000001;
    DTI_ADDUI_POSITIONITEM          = $00000002;


////////////////////////////////////////////
// Flags for IActiveDesktop::AddUrl()
    ADDURL_SILENT            = $0001;

// Default positions for ADI
    COMPONENT_DEFAULT_LEFT    = $FFFF;
    COMPONENT_DEFAULT_TOP     = $FFFF;
// Flags for SetSafeMode
    SSM_CLEAR   = $0000;
    SSM_SET     = $0001;
    SSM_REFRESH = $0002;
    SSM_UPDATE  = $0004;

// Flags for Set/GetScheme
    SCHEME_DISPLAY          = $0001;
    SCHEME_EDIT             = $0002;
    SCHEME_LOCAL            = $0004;
    SCHEME_GLOBAL           = $0008;
    SCHEME_REFRESH          = $0010;
    SCHEME_UPDATE           = $0020;
    SCHEME_DONOTUSE         = $0040; // used to be SCHEME_ENUMERATE; no longer supported
    SCHEME_CREATE           = $0080;

    SHCDF_UPDATEITEM        = $00000001;      // this flag is a hint that the file has changed since the last call to GetItemData

    DVASPECT_SHORTNAME      = 2; // use for CF_HDROP to get short name version of file paths
    DVASPECT_COPY           = 3; // use to indicate format is a "Copy" of the data (FILECONTENTS, FILEDESCRIPTOR, etc)
    DVASPECT_LINK           = 4; // use to indicate format is a "Shortcut" to the data (FILECONTENTS, FILEDESCRIPTOR, etc)
    FD_CLSID                = $0001;
    FD_SIZEPOINT            = $0002;
    FD_ATTRIBUTES           = $0004;
    FD_CREATETIME           = $0008;
    FD_ACCESSTIME           = $0010;
    FD_WRITESTIME           = $0020;
    FD_FILESIZE             = $0040;
    FD_PROGRESSUI           = $4000;       // Show Progress UI w/Drag and Drop
    FD_LINKUI               = $8000;       // 'link' UI is prefered
    SHCNE_RENAMEITEM          = DWord($00000001);
    SHCNE_CREATE              = DWord($00000002);
    SHCNE_DELETE              = DWord($00000004);
    SHCNE_MKDIR               = DWord($00000008);
    SHCNE_RMDIR               = DWord($00000010);
    SHCNE_MEDIAINSERTED       = DWord($00000020);
    SHCNE_MEDIAREMOVED        = DWord($00000040);
    SHCNE_DRIVEREMOVED        = DWord($00000080);
    SHCNE_DRIVEADD            = DWord($00000100);
    SHCNE_NETSHARE            = DWord($00000200);
    SHCNE_NETUNSHARE          = DWord($00000400);
    SHCNE_ATTRIBUTES          = DWord($00000800);
    SHCNE_UPDATEDIR           = DWord($00001000);
    SHCNE_UPDATEITEM          = DWord($00002000);
    SHCNE_SERVERDISCONNECT    = DWord($00004000);
    SHCNE_UPDATEIMAGE         = DWord($00008000);
    SHCNE_DRIVEADDGUI         = DWord($00010000);
    SHCNE_RENAMEFOLDER        = DWord($00020000);
    SHCNE_FREESPACE           = DWord($00040000);
    SHCNE_EXTENDED_EVENT      = DWord($04000000);
    SHCNE_ASSOCCHANGED        = DWord($08000000);

    SHCNE_DISKEVENTS          = DWord($0002381F);
    SHCNE_GLOBALEVENTS        = DWord($0C0581E0); // Events that dont match pidls first
    SHCNE_ALLEVENTS           = DWord($7FFFFFFF);
    SHCNE_INTERRUPT           = DWord($80000000); // The presence of this flag indicates
                                            // that the event was generated by an
                                            // interrupt.  It is stripped out before
                                            // the clients of SHCNNotify_ see it.

//#if (_WIN32_IE >= $0400)
// SHCNE_EXTENDED_EVENT extended events.  These events are ordinals.
// This is not a bitfield.

    SHCNEE_ORDERCHANGED         = 2;  // pidl2 is the changed folder
    SHCNEE_MSI_CHANGE           = 4;  // pidl2 is a SHChangeProductKeyAsIDList
    SHCNEE_MSI_UNINSTALL        = 5;  // pidl2 is a SHChangeProductKeyAsIDList
//#endif


// Flags
// uFlags & SHCNF_TYPE is an ID which indicates what dwItem1 and dwItem2 mean
    SHCNF_IDLIST      = $0000;        // LPITEMIDLIST
    SHCNF_PATHA       = $0001;        // path name
    SHCNF_PRINTERA    = $0002;        // printer friendly name
    SHCNF_DWORD       = $0003;        // DWORD
    SHCNF_PATHW       = $0005;        // path name
    SHCNF_PRINTERW    = $0006;        // printer friendly name
    SHCNF_TYPE        = $00FF;
    SHCNF_FLUSH       = $1000;
    SHCNF_FLUSHNOWAIT = $2000;
{$ifdef UNICODE}
    SHCNF_PATH      = SHCNF_PATHW;
    SHCNF_PRINTER   = SHCNF_PRINTERW;
{$else}
    SHCNF_PATH      = SHCNF_PATHA;
    SHCNF_PRINTER   = SHCNF_PRINTERA;
{$endif}
    QITIPF_DEFAULT          = $00000000;
    QITIPF_USENAME          = $00000001;
    QITIPF_LINKNOTARGET     = $00000002;
    QITIPF_LINKUSETARGET    = $00000004;
    QITIPF_USESLOWTIP       = $00000008;  // Flag says it's OK to take a long time generating tip

    QIF_CACHED              = $00000001;
    QIF_DONTEXPANDFOLDER    = $00000002;


//
// SHAddToRecentDocs
//
    SHARD_PIDL      = DWord($00000001);
    SHARD_PATHA     = DWord($00000002);
    SHARD_PATHW     = DWord($00000003);

{$ifdef UNICODE}
    SHARD_PATH  = SHARD_PATHW;
{$else}
    SHARD_PATH  = SHARD_PATHA;
{$endif}

//
// SHGetDataFromIDListA/W
//
// SHGetDataFromIDList nFormat values TCHAR
    SHGDFIL_FINDDATA           = 1;
    SHGDFIL_NETRESOURCE        = 2;
    SHGDFIL_DESCRIPTIONID      = 3;

    SHDID_ROOT_REGITEM         = 1 ;
    SHDID_FS_FILE              = 2 ;
    SHDID_FS_DIRECTORY         = 3 ;
    SHDID_FS_OTHER             = 4 ;
    SHDID_COMPUTER_DRIVE35     = 5 ;
    SHDID_COMPUTER_DRIVE525    = 6 ;
    SHDID_COMPUTER_REMOVABLE   = 7 ;
    SHDID_COMPUTER_FIXED       = 8 ;
    SHDID_COMPUTER_NETDRIVE    = 9 ;
    SHDID_COMPUTER_CDROM       = 10;
    SHDID_COMPUTER_RAMDISK     = 11;
    SHDID_COMPUTER_OTHER       = 12;
    SHDID_NET_DOMAIN           = 13;
    SHDID_NET_SERVER           = 14;
    SHDID_NET_SHARE            = 15;
    SHDID_NET_RESTOFNET        = 16;
    SHDID_NET_OTHER            = 17;
    SHDID_COMPUTER_IMAGING     = 18;
    SHDID_COMPUTER_AUDIO       = 19;
    SHDID_COMPUTER_SHAREDDOCS  = 20;
// PathResolve flags
    PRF_VERIFYEXISTS            = $0001;
    PRF_TRYPROGRAMEXTENSIONS    = ($0002  or  PRF_VERIFYEXISTS);
    PRF_FIRSTDIRDEF             = $0004;
    PRF_DONTFINDLNK             = $0008;      // if PRF_TRYPROGRAMEXTENSIONS is specified
    PCS_FATAL                   = $80000000;
    PCS_REPLACEDCHAR            = $00000001;
    PCS_REMOVEDCHAR             = $00000002;
    PCS_TRUNCATED               = $00000004;
    PCS_PATHTOOLONG             = $00000008;  // Always combined with FATA);
    MM_ADDSEPARATOR             = DWord($00000001);
    MM_SUBMENUSHAVEIDS          = DWord($00000002);
    MM_DONTREMOVESEPS           = DWord($00000004);

    SHOP_PRINTERNAME            = $00000001;  // lpObject points to a printer friendly name
    SHOP_FILEPATH               = $00000002;  // lpObject points to a fully qualified path+file name
    SHOP_VOLUMEGUID             = $00000004;  // lpObject points to a Volume GUID
    SHFMT_ID_DEFAULT            = $FFFF;
//
// Option bits for options parameter
//
    SHFMT_OPT_FULL             = $0001;
    SHFMT_OPT_SYSONLY          = $0002;

//
// Special return values. PLEASE NOTE that these are DWORD values.
//
    SHFMT_ERROR     = DWord($FFFFFFFF);     // Error on last format, drive may be formatable
    SHFMT_CANCEL    = DWord($FFFFFFFE);     // Last format was canceled
    SHFMT_NOFORMAT  = DWord($FFFFFFFD);     // Drive is not formatable
    REST_NONE                       = $00000000;
    REST_NORUN                      = $00000001;
    REST_NOCLOSE                    = $00000002;
    REST_NOSAVESET                  = $00000004;
    REST_NOFILEMENU                 = $00000008;
    REST_NOSETFOLDERS               = $00000010;
    REST_NOSETTASKBAR               = $00000020;
    REST_NODESKTOP                  = $00000040;
    REST_NOFIND                     = $00000080;
    REST_NODRIVES                   = $00000100;
    REST_NODRIVEAUTORUN             = $00000200;
    REST_NODRIVETYPEAUTORUN         = $00000400;
    REST_NONETHOOD                  = $00000800;
    REST_STARTBANNER                = $00001000;
    REST_RESTRICTRUN                = $00002000;
    REST_NOPRINTERTABS              = $00004000;
    REST_NOPRINTERDELETE            = $00008000;
    REST_NOPRINTERADD               = $00010000;
    REST_NOSTARTMENUSUBFOLDERS      = $00020000;
    REST_MYDOCSONNET                = $00040000;
    REST_NOEXITTODOS                = $00080000;
    REST_ENFORCESHELLEXTSECURITY    = $00100000;
    REST_LINKRESOLVEIGNORELINKINFO  = $00200000;
    REST_NOCOMMONGROUPS             = $00400000;
    REST_SEPARATEDESKTOPPROCESS     = $00800000;
    REST_NOWEB                      = $01000000;
    REST_NOTRAYCONTEXTMENU          = $02000000;
    REST_NOVIEWCONTEXTMENU          = $04000000;
    REST_NONETCONNECTDISCONNECT     = $08000000;
    REST_STARTMENULOGOFF            = $10000000;
    REST_NOSETTINGSASSIST           = $20000000;
    REST_NOINTERNETICON             = $40000001;
    REST_NORECENTDOCSHISTORY        = $40000002;
    REST_NORECENTDOCSMENU           = $40000003;
    REST_NOACTIVEDESKTOP            = $40000004;
    REST_NOACTIVEDESKTOPCHANGES     = $40000005;
    REST_NOFAVORITESMENU            = $40000006;
    REST_CLEARRECENTDOCSONEXIT      = $40000007;
    REST_CLASSICSHELL               = $40000008;
    REST_NOCUSTOMIZEWEBVIEW         = $40000009;
    REST_NOHTMLWALLPAPER            = $40000010;
    REST_NOCHANGINGWALLPAPER        = $40000011;
    REST_NODESKCOMP                 = $40000012;
    REST_NOADDDESKCOMP              = $40000013;
    REST_NODELDESKCOMP              = $40000014;
    REST_NOCLOSEDESKCOMP            = $40000015;
    REST_NOCLOSE_DRAGDROPBAND       = $40000016;   // Disable Close and Drag & Drop on ALL Bands
    REST_NOMOVINGBAND               = $40000017;   // Disable Moving ALL Bands
    REST_NOEDITDESKCOMP             = $40000018;
    REST_NORESOLVESEARCH            = $40000019;
    REST_NORESOLVETRACK             = $4000001A;
    REST_FORCECOPYACLWITHFILE       = $4000001B;
    REST_NOLOGO3CHANNELNOTIFY       = $4000001C;
    REST_NOFORGETSOFTWAREUPDATE     = $4000001D;
    REST_NOSETACTIVEDESKTOP         = $4000001E;   // No Active desktop on Settings Menu
    REST_NOUPDATEWINDOWS            = $4000001F;   // No Windows Update on Settings Menu
    REST_NOCHANGESTARMENU           = $40000020;   // No Context menu or Drag and Drop on Start menu
    REST_NOFOLDEROPTIONS            = $40000021;   // No Folder Options on Settings Menu
    REST_HASFINDCOMPUTERS           = $40000022;   // Show Start/Search/Computers
    REST_INTELLIMENUS               = $40000023;
    REST_RUNDLGMEMCHECKBOX          = $40000024;
    REST_ARP_ShowPostSetup          = $40000025;   // ARP: Show Post-Setup page
    REST_NOCSC                      = $40000026;   // Disable the ClientSide caching on SM
    REST_NOCONTROLPANEL             = $40000027;   // Remove the Control Panel only from SM or Settings
    REST_ENUMWORKGROUP              = $40000028;   // Enumerate workgroup in root of nethood
    REST_ARP_NOARP                  = $40000029;   // ARP: Don't Allow ARP to come up at all
    REST_ARP_NOREMOVEPAGE           = $4000002A;   // ARP: Don't allow Remove page
    REST_ARP_NOADDPAGE              = $4000002B;   // ARP: Don't allow Add page
    REST_ARP_NOWINSETUPPAGE         = $4000002C;   // ARP: Don't allow opt components page
    REST_GREYMSIADS                 = $4000002D;    // SM: Allow the greying of Darwin Ads in SM
    REST_NOCHANGEMAPPEDDRIVELABEL   = $4000002E;   // Don't enable the UI which allows users to rename mapped drive labels
    REST_NOCHANGEMAPPEDDRIVECOMMENT = $4000002F;   // Don't enable the UI which allows users to change mapped drive comments
    REST_MaxRecentDocs              = $40000030;
    REST_NONETWORKCONNECTIONS       = $40000031;   // No Start Menu  or  Settings  or Network Connections
    REST_FORCESTARTMENULOGOFF       = $40000032;   // Force logoff on the Start Menu
    REST_NOWEBVIEW                  = $40000033;   // Disable Web View
    REST_NOCUSTOMIZETHISFOLDER      = $40000034;   // Disable Customize This Folder
    REST_NOENCRYPTION               = $40000035;   // Don't allow file encryption
//  Do NOT use me                     = $40000036;
    REST_DONTSHOWSUPERHIDDEN        = $40000037;   // don't show super hidden files
    REST_NOSHELLSEARCHBUTTON        = $40000038;
    REST_NOHARDWARETAB              = $40000039;   // No Hardware tab on Drives or in control panel
    REST_NORUNASINSTALLPROMPT       = $4000003A;   // Don't bring up "Run As" prompt for install programs
    REST_PROMPTRUNASINSTALLNETPATH  = $4000003B;   // Force the  "Run As" prompt for install programs on unc/network shares
    REST_NOMANAGEMYCOMPUTERVERB     = $4000003C;   // No Manage verb on My Computer
    REST_NORECENTDOCSNETHOOD        = $4000003D;   // dont add the recent docs shares to nethood
    REST_DISALLOWRUN                = $4000003E;   // don't allow certain apps to be run
    REST_NOWELCOMESCREEN            = $4000003F;   // don't allow the welcome screen to be displayed.
    REST_RESTRICTCPL                = $40000040;   // only allow certain cpls to be run
    REST_DISALLOWCPL                = $40000041;   // don't allow certain cpls to be run
    REST_NOSMBALLOONTIP             = $40000042;   // No Start Menu Balloon Tip
    REST_NOSMHELP                   = $40000043;   // No Help on the Start Menu
    REST_NOWINKEYS                  = $40000044;   // No Windows-X Hot keys
    REST_NOENCRYPTONMOVE            = $40000045;   // Don't automatically try to encrypt files that are moved to encryped directories
    REST_NOLOCALMACHINERUN          = $40000046;   // ignore HKLM\sw\ms\win\cv\Run and all of it's sub keys
    REST_NOCURRENTUSERRUN           = $40000047;   // ignore HKCU\sw\ms\win\cv\Run and all of it's sub keys
    REST_NOLOCALMACHINERUNONCE      = $40000048;   // ignore HKLM\sw\ms\win\cv\RunOnce and all of it's sub keys
    REST_NOCURRENTUSERRUNONCE       = $40000049;   // ignore HKCU\sw\ms\win\cv\RunOnce and all of it's sub keys
    REST_FORCEACTIVEDESKTOPON       = $4000004A;   // Force ActiveDesktop to be turned ON all the time.
    REST_NOCOMPUTERSNEARME          = $4000004B;   // removes the "Computers near me" link
    REST_NOVIEWONDRIVE              = $4000004C;   // disallows CreateViewObject() on specified drives (CFSFolder only)
    REST_NONETCRAWL                 = $4000004D;   // disables the crawling of the WNet namespace.
    REST_NOSHAREDDOCUMENTS          = $4000004E;   // don't auto share the Shared Documents/create link
    REST_NOSMMYDOCS                 = $4000004F;   // Don't show the My Documents item on the Start Menu.
    REST_NOSMMYPICS                 = $40000050;   // Don't show the My Pictures item on the Start Menu
    REST_ALLOWBITBUCKDRIVES         = $40000051;   // Bit mask indicating which which drives have bit bucket support
    REST_NONLEGACYSHELLMODE         = $40000052;   // new consumer shell modes
    REST_NOCONTROLPANELBARRICADE    = $40000053;   // The webview barricade in Control Panel
    REST_NOSTARTPAGE                = $40000054;   // Whistler Start Page on desktop.
    REST_NOAUTOTRAYNOTIFY           = $40000055;   // Whistler auto-tray notify feature
    REST_NOTASKGROUPING             = $40000056;   // Whistler taskbar button grouping feature
    REST_NOCDBURNING                = $40000057;   // whistler cd burning feature
    REST_MYCOMPNOPROP               = $40000058;   // disables Properties on My Computer's context menu
    REST_MYDOCSNOPROP               = $40000059;   // disables Properties on My Documents' context menu
    REST_NOSTARTPANEL               = $4000005A;   // Windows start panel (New start menu) for Whistler.
    REST_NODISPLAYAPPEARANCEPAGE    = $4000005B;   // disable Themes and Appearance tabs in the Display Control Panel.
    REST_NOTHEMESTAB                = $4000005C;   // disable the Themes tab in the Display Control Panel.
    REST_NOVISUALSTYLECHOICE        = $4000005D;   // disable the visual style drop down in the Appearance tab of the Display Control Panel.
    REST_NOSIZECHOICE               = $4000005E;   // disable the size drop down in the Appearance tab of the Display Control Panel.
    REST_NOCOLORCHOICE              = $4000005F;   // disable the color drop down in the Appearance tab of the Display Control Panel.
    REST_SETVISUALSTYLE             = $40000060;   // Load the specified file as the visual style.
    REST_STARTRUNNOHOMEPATH         = $40000061;   // dont use the %HOMEPATH% env var for the Start-Run dialog
    REST_NOUSERNAMEINSTARTPANEL     = $40000062;   // don't show the username is the startpanel.
    REST_NOMYCOMPUTERICON           = $40000063;   // don't show my computer anywhere, hide its contents
    REST_NOSMNETWORKPLACES          = $40000064;   // don't show network places in startpanel.
    REST_NOSMPINNEDLIST             = $40000065;   // don't show the pinned list in startpanel.
    REST_NOSMMYMUSIC                = $40000066;   // don't show MyMusic folder in startpanel
    REST_NOSMEJECTPC                = $40000067;   // don't show "Undoc PC" command in startmenu
    REST_NOSMMOREPROGRAMS           = $40000068;   // don't show "More Programs" button in StartPanel.
    REST_NOSMMFUPROGRAMS            = $40000069;   // don't show the MFU programs list in StartPanel.
    REST_NOTRAYITEMSDISPLAY         = $4000006A;   // disables the display of the system tray
    REST_NOTOOLBARSONTASKBAR        = $4000006B;   // disables toolbar display on the taskbar
    REST_NOSMCONFIGUREPROGRAMS      = $4000006F;   // No Configure Programs on Settings Menu
    REST_HIDECLOCK                  = $40000070;   // don't show the clock
    REST_NOLOWDISKSPACECHECKS       = $40000071;   // disable the low disk space checking
    REST_NOENTIRENETWORK            = $40000072;   // removes the "Entire Network" link (i.e. from "My Network Places")
    REST_NODESKTOPCLEANUP           = $40000073;   // disable the desktop cleanup wizard
    REST_BITBUCKNUKEONDELETE        = $40000074;   // disables recycling of files
    REST_BITBUCKCONFIRMDELETE       = $40000075;   // always show the delete confirmation dialog when deleting files
    REST_BITBUCKNOPROP              = $40000076;   // disables Properties on Recycle Bin's context menu
    REST_NODISPBACKGROUND           = $40000077;   // disables the Desktop tab in the Display CP);
    REST_NODISPSCREENSAVEPG         = $40000078;   // disables the Screen Saver tab in the Display CP);
    REST_NODISPSETTINGSPG           = $40000079;   // disables the Settings tab in the Display CP);
    REST_NODISPSCREENSAVEPREVIEW    = $4000007A;   // disables the screen saver on the Screen Saver tab in the Display CP);
    REST_NODISPLAYCPL               = $4000007B;   // disables the Display CPL
    REST_HIDERUNASVERB              = $4000007C;   // hides the "Run As..." context menu item
    REST_NOTHUMBNAILCACHE           = $4000007D;   // disables use of the thumbnail cache
    REST_NOSTRCMPLOGICAL            = $4000007E;   // dont use StrCmpLogical() instead use default CompareString()
    REST_NOPUBLISHWIZARD            = $4000007F;   // disables publishing wizard (WPW)
    REST_NOONLINEPRINTSWIZARD       = $40000080;   // disables online prints wizard (OPW)
    REST_NOWEBSERVICES              = $40000081;   // disables the web specified services for both OPW and WPW
    REST_ALLOWUNHASHEDWEBVIEW       = $40000082;   // allow the user to be promted to accept web view templates that don't already have an md5 hash in the registry
    REST_ALLOWLEGACYWEBVIEW         = $40000083;   // allow legacy webview template to be shown.
    REST_REVERTWEBVIEWSECURITY      = $40000084;   // disable added webview security measures (revert to w2k functionality).
    REST_INHERITCONSOLEHANDLES      = $40000086;   // ShellExec() will check for the current process and target process being console processes to inherit handles
    REST_SORTMAXITEMCOUNT           = $40000087;   // Do not sort views with more items than this key. Useful for viewing big amount of files in one folder.
    REST_NOREMOTERECURSIVEEVENTS    = $40000089;   // Dont register network change events recursively to avoid network traffic
    REST_NOREMOTECHANGENOTIFY       = $40000091;   // Do not register for remote change notifies
    REST_NOSIMPLENETIDLIST          = $40000092;   // No simple network IDLists
    REST_NOENUMENTIRENETWORK        = $40000093;   // Don't enumerate entire network if we happen to get to it (in conjunction with REST_NOENTIRENETWORK)
    REST_NODETAILSTHUMBNAILONNETWORK= $40000094;   // Disable Thumbnail for Network files in DUI Details pane
    REST_NOINTERNETOPENWITH         = $40000095;   // dont allow looking on the internet for file associations
    REST_ALLOWLEGACYLMZBEHAVIOR     = $4000009A;   // allowable LMZ behavior for ActiveX objects changed in XPSP2, this policy gets the pre-XPSP2 behavior
    REST_DONTRETRYBADNETNAME        = $4000009B;   // In Network Places: if provider returns ERROR_BAD_NET_NAME, give up
    REST_ALLOWFILECLSIDJUNCTIONS    = $4000009C;   // re-enable legacy support for file.{guid} junctions in FileSystem Folder 
    REST_NOUPNPINSTALL              = $4000009D;   // disable "install UPnP" task in My Net Places
    
    REST_NODISCONNECT               = $41000001;   // No Disconnect option in Start menu
    REST_NOSECURITY                 = $41000002;   // No Security option in start menu
    REST_NOFILEASSOCIATE            = $41000003;   // Do not allow user to change file association
    REST_ALLOWCOMMENTTOGGLE         = $41000004;   // Allow the user to toggle the positions of the Comment and the Computer Name
    REST_USEDESKTOPINICACHE         = $41000005;   // Cache desktop.ini entries from network folders
//
// Path processing function
//
    PPCF_ADDQUOTES                  = $00000001;        // return a quoted name if required
    PPCF_ADDARGUMENTS               = $00000003;        // appends arguments (and wraps in quotes if required)
    PPCF_NODIRECTORIES              = $00000010;        // don't match to directories
    PPCF_FORCEQUALIFY               = $00000040;        // qualify even non-relative names
    PPCF_LONGESTPOSSIBLE            = $00000080;        // always find the longest possible name
    VALIDATEUNC_NOUI                = $0002;          // don't bring up UI
    VALIDATEUNC_CONNECT             = $0001;          // connect a drive letter
    VALIDATEUNC_PRINT               = $0004;          // validate as print share instead of disk share
    VALIDATEUNC_VALID               = $0007;          // valid flags

    OPENPROPS_NONE                  = $0000;
    OPENPROPS_INHIBITPIF            = $8000;
    GETPROPS_NONE                   = $0000;
    SETPROPS_NONE                   = $0000;
    CLOSEPROPS_NONE                 = $0000;		
    CLOSEPROPS_DISCARD              = $0001;		

    PIFNAMESIZE                     =  30;
    PIFSTARTLOCSIZE                 =  63;
    PIFDEFPATHSIZE                  =  64;
    PIFPARAMSSIZE                   =  64;
    PIFSHPROGSIZE                   =  64;
    PIFSHDATASIZE                   =  64;
    PIFDEFFILESIZE                  =  80;
    PIFMAXFILEPATH                  = 260;
    TBIF_APPEND                     =   0;
    TBIF_PREPEND                    =   1;
    TBIF_REPLACE                    =   2;
    TBIF_DEFAULT                    = $00000000;
    TBIF_INTERNETBAR                = $00010000;
    TBIF_STANDARDTOOLBAR   	    = $00020000;
    TBIF_NOTOOLBAR                  = $00030000;
//                                 uMsg    wParam             lParam
    SFVM_MERGEMENU           =  1;    // -                  LPQCMINFO
    SFVM_INVOKECOMMAND       =  2;    // idCmd              -
    SFVM_GETHELPTEXT         =  3;    // idCmd,cchMax       pszText
    SFVM_GETTOOLTIPTEXT      =  4;    // idCmd,cchMax       pszText
    SFVM_GETBUTTONINFO       =  5;    // -                  LPTBINFO
    SFVM_GETBUTTONS          =  6;    // idCmdFirst,cbtnMax LPTBBUTTON
    SFVM_INITMENUPOPUP       =  7;    // idCmdFirst,nIndex  hmenu
    SFVM_FSNOTIFY            = 14;    // LPCITEMIDLIST*     lEvent
    SFVM_WINDOWCREATED       = 15;    // hwnd               -
    SFVM_GETDETAILSOF        = 23;    // iColumn            DETAILSINFO*
    SFVM_COLUMNCLICK         = 24;    // iColumn            -
    SFVM_QUERYFSNOTIFY       = 25;    // -                  SHChangeNotifyEntry *
    SFVM_DEFITEMCOUNT        = 26;    // -                  UINT*
    SFVM_DEFVIEWMODE         = 27;    // -                  FOLDERVIEWMODE*
    SFVM_UNMERGEMENU         = 28;    // -                  hmenu
    SFVM_UPDATESTATUSBAR     = 31;    // fInitialize        -
    SFVM_BACKGROUNDENUM      = 32;    // -                  -
    SFVM_DIDDRAGDROP         = 36;    // dwEffect           IDataObject *
    SFVM_SETISFV             = 39;    // -                  IShellFolderView*
    SFVM_THISIDLIST          = 41;    // -                  LPITMIDLIST*
    SFVM_ADDPROPERTYPAGES    = 47;    // -                  SFVM_PROPPAGE_DATA *
    SFVM_BACKGROUNDENUMDONE  = 48;    // -                  -
    SFVM_GETNOTIFY           = 49;    // LPITEMIDLIST*      LONG*
    SFVM_GETSORTDEFAULTS     = 53;    // iDirection         iParamSort
    SFVM_SIZE                = 57;    // -                  -
    SFVM_GETZONE             = 58;    // -                  DWORD*
    SFVM_GETPANE             = 59;    // Pane ID            DWORD*
    SFVM_GETHELPTOPIC        = 63;    // -                  SFVM_HELPTOPIC_DATA *
    SFVM_GETANIMATION        = 68;    // HINSTANCE *        WCHAR *
//                                  uMsg       wParam       lParam
    DFM_MERGECONTEXTMENU     =    1;      // uFlags       LPQCMINFO
    DFM_INVOKECOMMAND        =    2;      // idCmd        pszArgs
    DFM_GETDEFSTATICID       =    14;     // idCmd *      0
// Commands from DFM_INVOKECOMMAND when strings are passed in
    DFM_CMD_PROPERTIES      = dword(-5);
    SFVM_REARRANGE          = $00000001;
    SFVM_ADDOBJECT          = $00000003;
    SFVM_REMOVEOBJECT       = $00000006;
    SFVM_UPDATEOBJECT       = $00000007;
    SFVM_GETSELECTEDOBJECTS = $00000009;
    SFVM_SETITEMPOS         = $0000000e;
    SFVM_SETCLIPBOARD       = $00000010;
    SFVM_SETPOINTS          = $00000017;
    PID_IS_URL              = 2;
    PID_IS_NAME             = 4;
    PID_IS_WORKINGDIR       = 5;
    PID_IS_HOTKEY           = 6;
    PID_IS_SHOWCMD          = 7;
    PID_IS_ICONINDEX        = 8;
    PID_IS_ICONFILE         = 9;
    PID_IS_WHATSNEW         = 10;
    PID_IS_AUTHOR           = 11;
    PID_IS_DESCRIPTION      = 12;
    PID_IS_COMMENT          = 13;
    PID_INTSITE_WHATSNEW      = 2;
    PID_INTSITE_AUTHOR        = 3;
    PID_INTSITE_LASTVISIT     = 4;
    PID_INTSITE_LASTMOD       = 5;
    PID_INTSITE_VISITCOUNT    = 6;
    PID_INTSITE_DESCRIPTION   = 7;
    PID_INTSITE_COMMENT       = 8;
    PID_INTSITE_FLAGS         = 9;
    PID_INTSITE_CONTENTLEN    = 10;
    PID_INTSITE_CONTENTCODE   = 11;
    PID_INTSITE_RECURSE       = 12;
    PID_INTSITE_WATCH         = 13;
    PID_INTSITE_SUBSCRIPTION  = 14;
    PID_INTSITE_URL           = 15;
    PID_INTSITE_TITLE         = 16;
    PID_INTSITE_CODEPAGE      = 18;
    PID_INTSITE_TRACKING      = 19;
    PID_INTSITE_ICONINDEX     = 20;
    PID_INTSITE_ICONFILE      = 21;


// Flags for PID_IS_FLAGS
    PIDISF_RECENTLYCHANGED  = $00000001;
    PIDISF_CACHEDSTICKY     = $00000002;
    PIDISF_CACHEIMAGES      = $00000010;
    PIDISF_FOLLOWALLLINKS   = $00000020;

// Values for PID_INTSITE_WATCH
    PIDISM_GLOBAL           = 0;       // Monitor based on global setting
    PIDISM_WATCH            = 1;       // User says watch
    PIDISM_DONTWATCH        = 2;       // User says don't watch

    SSF_SHOWALLOBJECTS          = $00000001;
    SSF_SHOWEXTENSIONS          = $00000002;
    SSF_HIDDENFILEEXTS          = $00000004;
    SSF_SERVERADMINUI           = $00000004;
    SSF_SHOWCOMPCOLOR           = $00000008;
    SSF_SORTCOLUMNS             = $00000010;
    SSF_SHOWSYSFILES            = $00000020;
    SSF_DOUBLECLICKINWEBVIEW    = $00000080;
    SSF_SHOWATTRIBCOL           = $00000100;
    SSF_DESKTOPHTML             = $00000200;
    SSF_WIN95CLASSIC            = $00000400;
    SSF_DONTPRETTYPATH          = $00000800;
    SSF_SHOWINFOTIP             = $00002000;
    SSF_MAPNETDRVBUTTON         = $00001000;
    SSF_NOCONFIRMRECYCLE        = $00008000;
    SSF_HIDEICONS               = $00004000;
    SSF_FILTER                  = $00010000;
    SSF_WEBVIEW                 = $00020000;
    SSF_SHOWSUPERHIDDEN         = $00040000;
    SSF_SEPPROCESS              = $00080000;
    SSF_NONETCRAWLING           = $00100000;
    SSF_STARTPANELON            = $00200000;
    SSF_SHOWSTARTPAGE           = $00400000;

    SHPPFW_NONE             = $00000000;
    SHPPFW_DIRCREATE        = $00000001;              // Create the directory if it doesn't exist without asking the user.
    SHPPFW_DEFAULT          = SHPPFW_DIRCREATE;        // May change
    SHPPFW_ASKDIRCREATE     = $00000002;              // Create the directory if it doesn't exist after asking the user.
    SHPPFW_IGNOREFILENAME   = $00000004;              // Ignore the last item in pszPath because it's a file.  Example: pszPath="C:\DirA\DirB", only use "C:\DirA".
    SHPPFW_NOWRITECHECK     = $00000008;              // Caller only needs to read from the drive, so don't check if it's READ ONLY.
    SHPPFW_MEDIACHECKONLY   = $00000010;              // do the retrys on the media (or net path), return errors if the file can't be found

Type                            
     HDROP    = THandle;
     PHIcon   = ^HIcon;

     STARTUPINFOW = record  // a guess. Omission should get fixed in Windows.
          cb : DWORD;
          lpReserved : LPTSTR;
          lpDesktop : LPTSTR;
          lpTitle : LPTSTR;
          dwX : DWORD;
          dwY : DWORD;
          dwXSize : DWORD;
          dwYSize : DWORD;
          dwXCountChars : DWORD;
          dwYCountChars : DWORD;
          dwFillAttribute : DWORD;
          dwFlags : DWORD;
          wShowWindow : WORD;
          cbReserved2 : WORD;
          lpReserved2 : LPBYTE;
          hStdInput : HANDLE;
          hStdOutput : HANDLE;
          hStdError : HANDLE;
       end;
     LPSTARTUPINFOW = ^STARTUPINFOW;
     _STARTUPINFOW = STARTUPINFOW;
     TSTARTUPINFOW = STARTUPINFOW;
     PSTARTUPINFOW = ^STARTUPINFOW;


{unicode}
<<<<<<< HEAD
<<<<<<< HEAD
Function DragQueryFileA(arg1 : HDROP; arg2 : UINT;arg3 : LPSTR ; arg4 : UINT):UINT;external shell32 name 'DragQueryFileA';
Function DragQueryFileW(arg1 : HDROP; arg2 : UINT;arg3 : LPWSTR; arg4 : UINT):UINT;external shell32 name 'DragQueryFileW';
Function DragQueryFile(arg1 : HDROP; arg2 : UINT;arg3 : LPSTR ; arg4 : UINT):UINT;external shell32 name 'DragQueryFileA';
Function DragQueryFile(arg1 : HDROP; arg2 : UINT;arg3 : LPWSTR; arg4 : UINT):UINT;external shell32 name 'DragQueryFileW';

Function DragQueryPoint(arg1 : HDROP; arg2 :LPPOINT):BOOL; external shell32 name 'DragQueryPoint';
Procedure DragFinish(arg1 : HDROP);                     external shell32 name 'DragFinish';
Procedure DragAcceptFiles(hwnd : HWND;arg2: BOOL);      external shell32 name 'DragAcceptFiles';

Function ShellExecuteA(HWND: hwnd;lpOperation : LPCSTR ; lpFile : LPCSTR ; lpParameters : LPCSTR; lpDirectory:  LPCSTR; nShowCmd:LONGINT):HInst; external shell32 name 'ShellExecuteA';
Function ShellExecuteW(hwnd: HWND;lpOperation : LPCWSTR ; lpFile : LPCWSTR ; lpParameters : LPCWSTR; lpDirectory:  LPCWSTR; nShowCmd:LONGINT):HInst; external shell32 name 'ShellExecuteW';
Function ShellExecute(HWND: hwnd;lpOperation : LPCSTR ; lpFile : LPCSTR ; lpParameters : LPCSTR; lpDirectory:  LPCSTR; nShowCmd:LONGINT):HInst; external shell32 name 'ShellExecuteA';
Function ShellExecute(hwnd: HWND;lpOperation : LPCWSTR ; lpFile : LPCWSTR ; lpParameters : LPCWSTR; lpDirectory:  LPCWSTR; nShowCmd:LONGINT):HInst; external shell32 name 'ShellExecuteW';

Function FindExecutableA(lpFile : LPCSTR ;lpDirectory : LPCSTR ; lpResult : LPSTR):HInst;external shell32 name 'FindExecutableA';
Function FindExecutableW(lpFile : LPCWSTR;lpDirectory : LPCWSTR; lpResult : LPWSTR):HInst;external shell32 name 'FindExecutableW';
Function FindExecutable(lpFile : LPCSTR ;lpDirectory : LPCSTR ; lpResult : LPSTR):HInst;external shell32 name 'FindExecutableA';
Function FindExecutable(lpFile : LPCWSTR;lpDirectory : LPCWSTR; lpResult : LPWSTR):HInst;external shell32 name 'FindExecutableW';

Function CommandLineToArgvW(lpCmdLine : LPCWSTR;pNumArgs : plongint):pLPWSTR;external shell32 name 'CommandLineToArgvW';

Function ShellAboutA(HWND: hWnd; szApp : LPCSTR; szOtherStuff : LPCSTR; HICON : hIcon):Longint; external shell32 name 'ShellAboutA';
Function ShellAboutW(HWND: hWnd; szApp : LPCWSTR; szOtherStuff : LPCWSTR; HICON : hIcon):Longint; external shell32 name 'ShellAboutW';
Function ShellAbout(HWND: hWnd; szApp : LPCSTR; szOtherStuff : LPCSTR; HICON : hIcon):Longint; external shell32 name 'ShellAboutA';
Function ShellAbout(HWND: hWnd; szApp : LPCWSTR; szOtherStuff : LPCWSTR; HICON : hIcon):Longint; external shell32 name 'ShellAboutW';

Function DuplicateIcon(hinst : HINST; HICON: hIcon):HIcon; external shell32 name 'DuplicateIcon';

Function  ExtractAssociatedIconA(hInst : HINST; lpIconPath : LPSTR; lpiIcon : LPWORD):HICON;external shell32 name 'ExtractAssociatedIconA';
Function  ExtractAssociatedIconW(hInst : HINST; lpIconPath : LPWSTR; lpiIcon : LPWORD):HICON;external shell32 name 'ExtractAssociatedIconW';
Function  ExtractAssociatedIcon(hInst : HINST; lpIconPath : LPSTR; lpiIcon : LPWORD):HICON;external shell32 name 'ExtractAssociatedIconA';
Function  ExtractAssociatedIcon(hInst : HINST; lpIconPath : LPWSTR; lpiIcon : LPWORD):HICON;external shell32 name 'ExtractAssociatedIconW';

Function ExtractIconA(hInst: HINST; lpszExeFileName :LPCSTR ; nIconIndex : UINT):HICON;external shell32 name 'ExtractIconA';
Function ExtractIconW(hInst: HINST; lpszExeFileName :LPCWSTR ; nIconIndex : UINT):HICON;external shell32 name 'ExtractIconW';

Function ExtractIcon(hInst: HINST; lpszExeFileName :LPCSTR ; nIconIndex : UINT):HICON;external shell32 name 'ExtractIconA';
Function ExtractIcon(hInst: HINST; lpszExeFileName :LPCWSTR ; nIconIndex : UINT):HICON;external shell32 name 'ExtractIconW';
=======
=======
>>>>>>> origin/fixes_2_2
Function DragQueryFileA(arg1 : HDROP; arg2 : UINT;arg3 : LPSTR ; arg4 : UINT):UINT;external 'shell32.dll' name 'DragQueryFileA';
Function DragQueryFileW(arg1 : HDROP; arg2 : UINT;arg3 : LPWSTR; arg4 : UINT):UINT;external 'shell32.dll' name 'DragQueryFileW';
Function DragQueryFile(arg1 : HDROP; arg2 : UINT;arg3 : LPSTR ; arg4 : UINT):UINT;external 'shell32.dll' name 'DragQueryFileA';
Function DragQueryFile(arg1 : HDROP; arg2 : UINT;arg3 : LPWSTR; arg4 : UINT):UINT;external 'shell32.dll' name 'DragQueryFileW';

Function DragQueryPoint(arg1 : HDROP; arg2 :LPPOINT):BOOL; external 'shell32.dll' name 'DragQueryPoint';
Procedure DragFinish(arg1 : HDROP);                     external 'shell32.dll' name 'DragFinish';
Procedure DragAcceptFiles(hwnd : HWND;arg2: BOOL);      external 'shell32.dll' name 'DragAcceptFiles';

Function ShellExecuteA(HWND: hwnd;lpOperation : LPCSTR ; lpFile : LPCSTR ; lpParameters : LPCSTR; lpDirectory:  LPCSTR; nShowCmd:LONGINT):HInst; external 'shell32.dll' name 'ShellExecuteA';
Function ShellExecuteW(hwnd: HWND;lpOperation : LPCWSTR ; lpFile : LPCWSTR ; lpParameters : LPCWSTR; lpDirectory:  LPCWSTR; nShowCmd:LONGINT):HInst; external 'shell32.dll' name 'ShellExecuteW';
Function ShellExecute(HWND: hwnd;lpOperation : LPCSTR ; lpFile : LPCSTR ; lpParameters : LPCSTR; lpDirectory:  LPCSTR; nShowCmd:LONGINT):HInst; external 'shell32.dll' name 'ShellExecuteA';
Function ShellExecute(hwnd: HWND;lpOperation : LPCWSTR ; lpFile : LPCWSTR ; lpParameters : LPCWSTR; lpDirectory:  LPCWSTR; nShowCmd:LONGINT):HInst; external 'shell32.dll' name 'ShellExecuteW';

Function FindExecutableA(lpFile : LPCSTR ;lpDirectory : LPCSTR ; lpResult : LPSTR):HInst;external 'shell32.dll' name 'FindExecutableA';
Function FindExecutableW(lpFile : LPCWSTR;lpDirectory : LPCWSTR; lpResult : LPWSTR):HInst;external 'shell32.dll' name 'FindExecutableW';
Function FindExecutable(lpFile : LPCSTR ;lpDirectory : LPCSTR ; lpResult : LPSTR):HInst;external 'shell32.dll' name 'FindExecutableA';
Function FindExecutable(lpFile : LPCWSTR;lpDirectory : LPCWSTR; lpResult : LPWSTR):HInst;external 'shell32.dll' name 'FindExecutableW';

Function CommandLineToArgvW(lpCmdLine : LPCWSTR;pNumArgs : plongint):pLPWSTR;external 'shell32.dll' name 'CommandLineToArgvW';

Function ShellAboutA(HWND: hWnd; szApp : LPCSTR; szOtherStuff : LPCSTR; HICON : hIcon):Longint; external 'shell32.dll' name 'ShellAboutA';
Function ShellAboutW(HWND: hWnd; szApp : LPCWSTR; szOtherStuff : LPCWSTR; HICON : hIcon):Longint; external 'shell32.dll' name 'ShellAboutW';
Function ShellAbout(HWND: hWnd; szApp : LPCSTR; szOtherStuff : LPCSTR; HICON : hIcon):Longint; external 'shell32.dll' name 'ShellAboutA';
Function ShellAbout(HWND: hWnd; szApp : LPCWSTR; szOtherStuff : LPCWSTR; HICON : hIcon):Longint; external 'shell32.dll' name 'ShellAboutW';

Function DuplicateIcon(hinst : HINST; HICON: hIcon):HIcon; external 'shell32.dll' name 'DuplicateIcon';

Function  ExtractAssociatedIconA(hInst : HINST; lpIconPath : LPSTR; lpiIcon : LPWORD):HICON;external 'shell32.dll' name 'ExtractAssociatedIconA';
Function  ExtractAssociatedIconW(hInst : HINST; lpIconPath : LPWSTR; lpiIcon : LPWORD):HICON;external 'shell32.dll' name 'ExtractAssociatedIconW';
Function  ExtractAssociatedIcon(hInst : HINST; lpIconPath : LPSTR; lpiIcon : LPWORD):HICON;external 'shell32.dll' name 'ExtractAssociatedIconA';
Function  ExtractAssociatedIcon(hInst : HINST; lpIconPath : LPWSTR; lpiIcon : LPWORD):HICON;external 'shell32.dll' name 'ExtractAssociatedIconW';

Function ExtractIconA(hInst: HINST; lpszExeFileName :LPCSTR ; nIconIndex : UINT):HICON;external 'shell32.dll' name 'ExtractIconA';
Function ExtractIconW(hInst: HINST; lpszExeFileName :LPCWSTR ; nIconIndex : UINT):HICON;external 'shell32.dll' name 'ExtractIconW';

Function ExtractIcon(hInst: HINST; lpszExeFileName :LPCSTR ; nIconIndex : UINT):HICON;external 'shell32.dll' name 'ExtractIconA';
Function ExtractIcon(hInst: HINST; lpszExeFileName :LPCWSTR ; nIconIndex : UINT):HICON;external 'shell32.dll' name 'ExtractIconW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

// if(WINVER >= 0x0400)

Type
    { init with sizeof(DRAGINFO)  }

       _DRAGINFOA = Record
                     uSize       : UINT;
                     pt          : POINT;
                     fNC         : BOOL;
                     lpFileList  : LPSTR;
                     grfKeyState : DWORD;
                   end;
       DRAGINFOA   = _DRAGINFOA;
       TDRAGINFOA  = _DRAGINFOA;
       LPDRAGINFOA = ^_DRAGINFOA;

    { init with sizeof(DRAGINFO)  }

       _DRAGINFOW = Record
                     uSize       : UINT;
                     pt          : POINT;
                     fNC         : BOOL;
                     lpFileList  : LPWSTR;
                     grfKeyState : DWORD;
                    end;
       DRAGINFOW   = _DRAGINFOW;
       TDRAGINFOW  = _DRAGINFOW;
       LPDRAGINFOW = ^_DRAGINFOW;

{$ifdef UNICODE}
       DRAGINFO         = DRAGINFOW;
       TDRAGINFO        = DRAGINFOW;
       LPDRAGINFO       = LPDRAGINFOW;
{$else}
       DRAGINFO         = DRAGINFOA;
       TDRAGINFO        = DRAGINFOW;
       LPDRAGINFO       = LPDRAGINFOA;
{$endif}

Const
       ABM_NEW                  = $00000000;
       ABM_REMOVE               = $00000001;
       ABM_QUERYPOS             = $00000002;
       ABM_SETPOS               = $00000003;
       ABM_GETSTATE             = $00000004;
       ABM_GETTASKBARPOS        = $00000005;
       ABM_ACTIVATE             = $00000006;     { lParam == TRUE/FALSE means activate/deactivate }
       ABM_GETAUTOHIDEBAR       = $00000007;
       ABM_SETAUTOHIDEBAR       = $00000008;     { this can fail at any time.  MUST check the result }
                                                 { lParam = TRUE/FALSE  Set/Unset }
                                                 { uEdge = what edge }
       ABM_WINDOWPOSCHANGED     = $0000009;
       ABM_SETSTATE             = $0000000a;
       ABN_STATECHANGE          = $0000000;      { these are put in the wparam of callback messages }
       ABN_POSCHANGED           = $0000001;
       ABN_FULLSCREENAPP        = $0000002;
       ABN_WINDOWARRANGE        = $0000003;      { lParam == TRUE means hide }

       { flags for get state }
       ABS_AUTOHIDE             = $0000001;
       ABS_ALWAYSONTOP          = $0000002;
       ABE_LEFT                 = 0;
       ABE_TOP                  = 1;
       ABE_RIGHT                = 2;
       ABE_BOTTOM               = 3;


Type

       _AppBarData        = Record
                             cbSize             : DWORD;
                             hWnd               : HWND;
                             uCallbackMessage   : UINT;
                             uEdge              : UINT;
                             rc                 : RECT;
                             lParam             : LPARAM; { message specific }
                            end;
       APPBARDATA         = _AppBarData;
       TAPPBARDATA        = _AppBarData;
       PAPPBARDATA        = ^_AppBarData;


<<<<<<< HEAD
<<<<<<< HEAD
Function SHAppBarMessage(dwMessage : DWORD; pData : pAPPBARDATA):UINT_PTR;external shell32 name 'SHAppBarMessage';
=======
Function SHAppBarMessage(dwMessage : DWORD; pData : pAPPBARDATA):UINT_PTR;external 'shell32.dll' name 'SHAppBarMessage';
>>>>>>> graemeg/fixes_2_2
=======
Function SHAppBarMessage(dwMessage : DWORD; pData : pAPPBARDATA):UINT_PTR;external 'shell32.dll' name 'SHAppBarMessage';
>>>>>>> origin/fixes_2_2

    //
    //  EndAppBar
    //

<<<<<<< HEAD
<<<<<<< HEAD
Function   DoEnvironmentSubstA(szString: LPSTR; cchString:UINT):DWORD;external shell32 name 'DoEnvironmentSubstA';
Function   DoEnvironmentSubstW(szString: LPWSTR; cchString:UINT):DWORD;external shell32 name 'DoEnvironmentSubstW';
Function   DoEnvironmentSubst(szString: LPSTR; cchString:UINT):DWORD;external shell32 name 'DoEnvironmentSubstA';
Function   DoEnvironmentSubst(szString: LPWSTR; cchString:UINT):DWORD;external shell32 name 'DoEnvironmentSubstW';
=======
=======
>>>>>>> origin/fixes_2_2
Function   DoEnvironmentSubstA(szString: LPSTR; cchString:UINT):DWORD;external 'shell32.dll' name 'DoEnvironmentSubstA';
Function   DoEnvironmentSubstW(szString: LPWSTR; cchString:UINT):DWORD;external 'shell32.dll' name 'DoEnvironmentSubstW';
Function   DoEnvironmentSubst(szString: LPSTR; cchString:UINT):DWORD;external 'shell32.dll' name 'DoEnvironmentSubstA';
Function   DoEnvironmentSubst(szString: LPWSTR; cchString:UINT):DWORD;external 'shell32.dll' name 'DoEnvironmentSubstW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{Macro}
function EIRESID(x : longint) : longint;

<<<<<<< HEAD
<<<<<<< HEAD
Function ExtractIconExA(lpszFile : LPCSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT;   external shell32 name 'ExtractIconExA';
Function ExtractIconExW(lpszFile : LPCWSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT;  external shell32 name 'ExtractIconExW';
Function ExtractIconExA(lpszFile : LPCSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT;   external shell32 name 'ExtractIconExA';
Function ExtractIconExW(lpszFile : LPCWSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT;  external shell32 name 'ExtractIconExW';

Function ExtractIconEx (lpszFile : LPCSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT; external shell32 name 'ExtractIconExA';
Function ExtractIconEx (lpszFile : LPCWSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT; external shell32 name 'ExtractIconExW';
Function ExtractIconEx (lpszFile : LPCSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT; external shell32 name 'ExtractIconExA';
Function ExtractIconEx (lpszFile : LPCWSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT; external shell32 name 'ExtractIconExW';
=======
=======
>>>>>>> origin/fixes_2_2
Function ExtractIconExA(lpszFile : LPCSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT;   external 'shell32.dll' name 'ExtractIconExA';
Function ExtractIconExW(lpszFile : LPCWSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT;  external 'shell32.dll' name 'ExtractIconExW';
Function ExtractIconExA(lpszFile : LPCSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT;   external 'shell32.dll' name 'ExtractIconExA';
Function ExtractIconExW(lpszFile : LPCWSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT;  external 'shell32.dll' name 'ExtractIconExW';

Function ExtractIconEx (lpszFile : LPCSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT; external 'shell32.dll' name 'ExtractIconExA';
Function ExtractIconEx (lpszFile : LPCWSTR; nIconIndex:Longint; phiconLarge:pHICON; phiconSmall:pHIcon; nIcons:UINT):UINT; external 'shell32.dll' name 'ExtractIconExW';
Function ExtractIconEx (lpszFile : LPCSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT; external 'shell32.dll' name 'ExtractIconExA';
Function ExtractIconEx (lpszFile : LPCWSTR; nIconIndex:Longint; var phiconLarge:HICON;var phiconSmall:HIcon; nIcons:UINT):UINT; external 'shell32.dll' name 'ExtractIconExW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

//
// Shell File Operations
//

//ifndef FO_MOVE  //these need to be kept in sync with the ones in shlobj.h}
Const
       FO_MOVE                  = $0001;
       FO_COPY                  = $0002;
       FO_DELETE                = $0003;
       FO_RENAME                = $0004;
       FOF_MULTIDESTFILES       = $0001;
       FOF_CONFIRMMOUSE         = $0002;
       FOF_SILENT               = $0004;    { don't create progress/report }
       FOF_RENAMEONCOLLISION    = $0008;
       FOF_NOCONFIRMATION       = $0010;    { Don't prompt the user. }
       FOF_WANTMAPPINGHANDLE    = $0020;    { Fill in SHFILEOPSTRUCT.hNameMappings }
       FOF_ALLOWUNDO            = $0040;    { Must be freed using SHFreeNameMappings }
       FOF_FILESONLY            = $0080;    { on *.*, do only files }
       FOF_SIMPLEPROGRESS       = $0100;    { means don't show names of files }
       FOF_NOCONFIRMMKDIR       = $0200;    { don't confirm making any needed dirs }
       FOF_NOERRORUI            = $0400;    { don't put up error UI }
       FOF_NOCOPYSECURITYATTRIBS= $0800;    { dont copy NT file Security Attributes }
       FOF_NORECURSION          = $1000;    { don't recurse into directories. }

//if (_WIN32_IE >= 0x0500)
       FOF_NO_CONNECTED_ELEMENTS= $2000;    { don't operate on connected elements. }
       FOF_WANTNUKEWARNING      = $4000;    { during delete operation, warn if nuking instead of recycling (partially overrides FOF_NOCONFIRMATION) }
//endif

//if (_WIN32_WINNT >= 0x0501)
       FOF_NORECURSEREPARSE     = $8000;    { treat reparse points as objects, not containers }
//endif

Type
       FILEOP_FLAGS             = WORD;

Const
       PO_DELETE                = $0013;    { printer is being deleted }
       PO_RENAME                = $0014;    { printer is being renamed }

       PO_PORTCHANGE            = $0020;    { port this printer connected to is being changed }
                                            { if this id is set, the strings received by }
                                            { the copyhook are a doubly-null terminated }
                                            { list of strings.  The first is the printer }
                                            { name and the second is the printer port. }

       PO_REN_PORT              = $0034;    { PO_RENAME and PO_PORTCHANGE at same time. }

{ no POF_ flags currently defined }

Type

       PRINTEROP_FLAGS = WORD;
//endif}

    { FO_MOVE }
    { implicit parameters are: }
    {      if pFrom or pTo are unqualified names the current directories are }
    {      taken from the global current drive/directory settings managed }
    {      by Get/SetCurrentDrive/Directory }
    { }
    {      the global confirmation settings }
    { only used if FOF_SIMPLEPROGRESS }

Type

       _SHFILEOPSTRUCTA         = Record
                                   wnd               : HWND;
                                   wFunc             : UINT;
                                   pFrom             : LPCSTR;
                                   pTo               : LPCSTR;
                                   fFlags            : FILEOP_FLAGS;
                                   fAnyOperationsAborted : BOOL;
                                   hNameMappings     : LPVOID;
                                   lpszProgressTitle : LPCSTR;     { only used if FOF_SIMPLEPROGRESS }
                                  end;
       SHFILEOPSTRUCTA          = _SHFILEOPSTRUCTA;
       TSHFILEOPSTRUCTA         = _SHFILEOPSTRUCTA;
       LPSHFILEOPSTRUCTA        = ^_SHFILEOPSTRUCTA;


       _SHFILEOPSTRUCTW         = record
                                   wnd               : HWND;
                                   wFunc             : UINT;
                                   pFrom             : LPCWSTR;
                                   pTo               : LPCWSTR;
                                   fFlags            : FILEOP_FLAGS;
                                   fAnyOperationsAborted : BOOL;
                                   hNameMappings     : LPVOID;
                                   lpszProgressTitle : LPCWSTR;
                                  end;
       SHFILEOPSTRUCTW          = _SHFILEOPSTRUCTW;
       TSHFILEOPSTRUCTW         = _SHFILEOPSTRUCTW;
       LPSHFILEOPSTRUCTW        = ^_SHFILEOPSTRUCTW;
{$ifdef UNICODE}
       SHFILEOPSTRUCT           = SHFILEOPSTRUCTW;
       TSHFILEOPSTRUCT          = SHFILEOPSTRUCTW;
       LPSHFILEOPSTRUCT         = LPSHFILEOPSTRUCTW;
{$else}
       SHFILEOPSTRUCT           = SHFILEOPSTRUCTA;
       TSHFILEOPSTRUCT          = SHFILEOPSTRUCTA;
       LPSHFILEOPSTRUCT         = LPSHFILEOPSTRUCTA;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
Function SHFileOperationA(lpFileOp:LPSHFILEOPSTRUCTA ):Longint;external shell32 name 'SHFileOperationA';

Function SHFileOperationW(lpFileOp:LPSHFILEOPSTRUCTW ):Longint;external shell32 name 'SHFileOperationW';

Function SHFileOperation(lpFileOp:LPSHFILEOPSTRUCTA ):Longint;external shell32 name 'SHFileOperationA';
Function SHFileOperation(var lpFileOp:SHFILEOPSTRUCTA ):Longint;external shell32 name 'SHFileOperationA';
Function SHFileOperation(lpFileOp:LPSHFILEOPSTRUCTW ):Longint;external shell32 name 'SHFileOperationW';

Procedure SHFreeNameMappings(hNameMappings : THandle);external shell32 name 'SHFreeNameMappings';
=======
=======
>>>>>>> origin/fixes_2_2
Function SHFileOperationA(lpFileOp:LPSHFILEOPSTRUCTA ):Longint;external 'shell32.dll' name 'SHFileOperationA';

Function SHFileOperationW(lpFileOp:LPSHFILEOPSTRUCTW ):Longint;external 'shell32.dll' name 'SHFileOperationW';

Function SHFileOperation(lpFileOp:LPSHFILEOPSTRUCTA ):Longint;external 'shell32.dll' name 'SHFileOperationA';
Function SHFileOperation(var lpFileOp:SHFILEOPSTRUCTA ):Longint;external 'shell32.dll' name 'SHFileOperationA';
Function SHFileOperation(lpFileOp:LPSHFILEOPSTRUCTW ):Longint;external 'shell32.dll' name 'SHFileOperationW';

Procedure SHFreeNameMappings(hNameMappings : THandle);external 'shell32.dll' name 'SHFreeNameMappings';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

Type

       _SHNAMEMAPPINGA          = Record
                                   pszOldPath : LPSTR;
                                   pszNewPath : LPSTR;
                                   cchOldPath : longint;
                                   cchNewPath : longint;
                                  end;
       SHNAMEMAPPINGA           = _SHNAMEMAPPINGA;
       TSHNAMEMAPPINGA          = _SHNAMEMAPPINGA;
       LPSHNAMEMAPPINGA         = ^_SHNAMEMAPPINGA;

       _SHNAMEMAPPINGW          = Record
                                   pszOldPath : LPWSTR;
                                   pszNewPath : LPWSTR;
                                   cchOldPath : longint;
                                   cchNewPath : longint;
                                  end;
       SHNAMEMAPPINGW           = _SHNAMEMAPPINGW;
       TSHNAMEMAPPINGW          = _SHNAMEMAPPINGW;
       LPSHNAMEMAPPINGW         = ^_SHNAMEMAPPINGW;
{$ifndef UNICODE}
       SHNAMEMAPPING            = SHNAMEMAPPINGW;
       TSHNAMEMAPPING           = SHNAMEMAPPINGW;
       LPSHNAMEMAPPING          = LPSHNAMEMAPPINGW;
{$else}
       SHNAMEMAPPING            = SHNAMEMAPPINGA;
       TSHNAMEMAPPING           = SHNAMEMAPPINGA;
       LPSHNAMEMAPPING          = LPSHNAMEMAPPINGA;
{$endif}

    //
    // End Shell File Operations
    //
    //
    //  Begin ShellExecuteEx and family
    //


    { ShellExecute() and ShellExecuteEx() error codes  }
    { regular WinExec() codes  }


    const
       SE_ERR_FNF               = 2;    { file not found }
       SE_ERR_PNF               = 3;    { path not found }
       SE_ERR_ACCESSDENIED      = 5;    { access denied }
       SE_ERR_OOM               = 8;    { out of memory }
       SE_ERR_DLLNOTFOUND       = 32;
// endif   WINVER >= 0x0400

    { error values for ShellExecute() beyond the regular WinExec() codes  }
       SE_ERR_SHARE             = 26;
       SE_ERR_ASSOCINCOMPLETE   = 27;
       SE_ERR_DDETIMEOUT        = 28;
       SE_ERR_DDEFAIL           = 29;
       SE_ERR_DDEBUSY           = 30;
       SE_ERR_NOASSOC           = 31;

//if(WINVER >= 0x0400)}

    { Note CLASSKEY overrides CLASSNAME }

       SEE_MASK_CLASSNAME       = $00000001;
       SEE_MASK_CLASSKEY        = $00000003;
    { Note INVOKEIDLIST overrides IDLIST }
       SEE_MASK_IDLIST          = $00000004;
       SEE_MASK_INVOKEIDLIST    = $0000000c;
       SEE_MASK_ICON            = $00000010;
       SEE_MASK_HOTKEY          = $00000020;
       SEE_MASK_NOCLOSEPROCESS  = $00000040;
       SEE_MASK_CONNECTNETDRV   = $00000080;
       SEE_MASK_FLAG_DDEWAIT    = $00000100;
       SEE_MASK_DOENVSUBST      = $00000200;
       SEE_MASK_FLAG_NO_UI      = $00000400;
       SEE_MASK_UNICODE         = $00004000;
       SEE_MASK_NO_CONSOLE      = $00008000;
       SEE_MASK_ASYNCOK         = $00100000;
       SEE_MASK_HMONITOR        = $00200000;
//if (_WIN32_IE >= 0x0500)
       SEE_MASK_NOQUERYCLASSSTORE= $01000000;
       SEE_MASK_WAITFORINPUTIDLE= $02000000;
//endif  (_WIN32_IE >= 0x500)
//if (_WIN32_IE >= 0x0560)
       SEE_MASK_FLAG_LOG_USAGE  = $04000000;
//endif
    { (_WIN32_IE >= 0x560) }

    type

       _SHELLEXECUTEINFOA       = record
                                   cbSize : DWORD;
                                   fMask : ULONG;
                                   wnd  : HWND;
                                   lpVerb : LPCSTR;
                                   lpFile : LPCSTR;
                                   lpParameters : LPCSTR;
                                   lpDirectory : LPCSTR;
                                   nShow : longint;
                                   hInstApp : HINST;
                                   lpIDList : LPVOID;
                                   lpClass : LPCSTR;
                                   hkeyClass : HKEY;
                                   dwHotKey : DWORD;
                                   DUMMYUNIONNAME : record
                                                      case longint of
                                                       0 : ( hIcon : HANDLE );
                                                       1 : ( hMonitor : HANDLE );
                                                      end;
                                   hProcess : HANDLE;
                                  end;

       SHELLEXECUTEINFOA        = _SHELLEXECUTEINFOA;
       TSHELLEXECUTEINFOA       = _SHELLEXECUTEINFOA;
       LPSHELLEXECUTEINFOA      = ^_SHELLEXECUTEINFOA;


       _SHELLEXECUTEINFOW       = record
                                   cbSize : DWORD;
                                   fMask : ULONG;
                                   wnd : HWND;
                                   lpVerb : lpcwstr;
                                   lpFile : lpcwstr;
                                   lpParameters : lpcwstr;
                                   lpDirectory : lpcwstr;
                                   nShow : longint;
                                   hInstApp : HINST;
                                   lpIDList : LPVOID;
                                   lpClass : LPCWSTR;
                                   hkeyClass : HKEY;
                                   dwHotKey : DWORD;
                                   DUMMYUNIONNAME : record
                                                      case longint of
                                                       0 : ( hIcon : HANDLE );
                                                       1 : ( hMonitor : HANDLE );
                                                      end;
                                   hProcess : HANDLE;
                                  end;

       SHELLEXECUTEINFOW        = _SHELLEXECUTEINFOW;
       TSHELLEXECUTEINFOW       = _SHELLEXECUTEINFOW;
       LPSHELLEXECUTEINFOW      = ^_SHELLEXECUTEINFOW;

{$ifdef UNICODE}
       SHELLEXECUTEINFO         = SHELLEXECUTEINFOW;
       TSHELLEXECUTEINFO        = SHELLEXECUTEINFOW;
       LPSHELLEXECUTEINFO       = LPSHELLEXECUTEINFOW;
{$else}
       SHELLEXECUTEINFO         = SHELLEXECUTEINFOA;
       TSHELLEXECUTEINFO        = SHELLEXECUTEINFOA;
       LPSHELLEXECUTEINFO       = LPSHELLEXECUTEINFOA;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
Function ShellExecuteExA(lpExecInfo: LPSHELLEXECUTEINFOA):Bool;external shell32 name 'ShellExecuteExA';
Function ShellExecuteExW(lpExecInfo: LPSHELLEXECUTEINFOW):Bool;external shell32 name 'ShellExecuteExW';
Function ShellExecuteEx(lpExecInfo: LPSHELLEXECUTEINFOA):Bool;external shell32 name 'ShellExecuteExA';
Function ShellExecuteEx(lpExecInfo: LPSHELLEXECUTEINFOW):Bool;external shell32 name 'ShellExecuteExW';

Procedure WinExecErrorA(HWND : hwnd; error : Longint;lpstrFileName:LPCSTR; lpstrTitle:LPCSTR);   external shell32 name 'WinExecErrorA';
Procedure WinExecErrorW(HWND : hwnd; error : Longint;lpstrFileName:LPCWSTR; lpstrTitle:LPCWSTR); external shell32 name 'WinExecErrorW';
Procedure WinExecError(HWND : hwnd; error : Longint;lpstrFileName:LPCSTR; lpstrTitle:LPCSTR); external shell32 name 'WinExecErrorA';
Procedure WinExecError(HWND : hwnd; error : Longint;lpstrFileName:LPCWSTR; lpstrTitle:LPCWSTR); external shell32 name 'WinExecErrorW';
=======
=======
>>>>>>> origin/fixes_2_2
Function ShellExecuteExA(lpExecInfo: LPSHELLEXECUTEINFOA):Bool;external 'shell32.dll' name 'ShellExecuteExA';
Function ShellExecuteExW(lpExecInfo: LPSHELLEXECUTEINFOW):Bool;external 'shell32.dll' name 'ShellExecuteExW';
Function ShellExecuteEx(lpExecInfo: LPSHELLEXECUTEINFOA):Bool;external 'shell32.dll' name 'ShellExecuteExA';
Function ShellExecuteEx(lpExecInfo: LPSHELLEXECUTEINFOW):Bool;external 'shell32.dll' name 'ShellExecuteExW';

Procedure WinExecErrorA(HWND : hwnd; error : Longint;lpstrFileName:LPCSTR; lpstrTitle:LPCSTR);   external 'shell32.dll' name 'WinExecErrorA';
Procedure WinExecErrorW(HWND : hwnd; error : Longint;lpstrFileName:LPCWSTR; lpstrTitle:LPCWSTR); external 'shell32.dll' name 'WinExecErrorW';
Procedure WinExecError(HWND : hwnd; error : Longint;lpstrFileName:LPCSTR; lpstrTitle:LPCSTR); external 'shell32.dll' name 'WinExecErrorA';
Procedure WinExecError(HWND : hwnd; error : Longint;lpstrFileName:LPCWSTR; lpstrTitle:LPCWSTR); external 'shell32.dll' name 'WinExecErrorW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

type

     _SHCREATEPROCESSINFOW      = record
                                   cbSize               : DWORD;
                                   fMask                : ULONG;
                                   hwnd                 : HWND;
                                   pszFile              : LPCWSTR;
                                   pszParameters        : LPCWSTR;
                                   pszCurrentDirectory  : LPCWSTR;
                             {in}  hUserToken           : HANDLE;
                             {in}  lpProcessAttributes  : LPSECURITY_ATTRIBUTES;
                             {in}  lpThreadAttributes   : LPSECURITY_ATTRIBUTES;
                             {in}  bInheritHandles      : BOOL;
                             {in}  dwCreationFlags      : DWORD;
                             {in}  lpStartupInfo        : LPSTARTUPINFOW;
                             {out} lpProcessInformation : LPPROCESS_INFORMATION;
                                  end;
     SHCREATEPROCESSINFOW       = _SHCREATEPROCESSINFOW;
     TSHCREATEPROCESSINFOW      = _SHCREATEPROCESSINFOW;
     PSHCREATEPROCESSINFOW      = ^_SHCREATEPROCESSINFOW;

<<<<<<< HEAD
<<<<<<< HEAD
Function SHCreateProcessAsUserW(pscpi : PSHCREATEPROCESSINFOW):Bool;external shell32 name 'SHCreateProcessAsUserW';
=======
Function SHCreateProcessAsUserW(pscpi : PSHCREATEPROCESSINFOW):Bool;external 'shell32.dll' name 'SHCreateProcessAsUserW';
>>>>>>> graemeg/fixes_2_2
=======
Function SHCreateProcessAsUserW(pscpi : PSHCREATEPROCESSINFOW):Bool;external 'shell32.dll' name 'SHCreateProcessAsUserW';
>>>>>>> origin/fixes_2_2

    //
    //  End ShellExecuteEx and family }
    //

    //
    // RecycleBin
    //

    { struct for query recycle bin info }

Type
       _SHQUERYRBINFO           = record
                                   cbSize       : DWORD;
                                   i64Size      : int64;
                                   i64NumItems  : int64;
                                  end;
       SHQUERYRBINFO            = _SHQUERYRBINFO;
       TSHQUERYRBINFO           = _SHQUERYRBINFO;
       LPSHQUERYRBINFO          = ^_SHQUERYRBINFO;

       { flags for SHEmptyRecycleBin }

const
       SHERB_NOCONFIRMATION     = $00000001;
       SHERB_NOPROGRESSUI       = $00000002;
       SHERB_NOSOUND            = $00000004;

<<<<<<< HEAD
<<<<<<< HEAD
function SHQueryRecycleBinA(pszRootPath:LPCSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external shell32 name 'SHQueryRecycleBinA';
function SHQueryRecycleBinW(pszRootPath:LPCWSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external shell32 name 'SHQueryRecycleBinW';
function SHQueryRecycleBin(pszRootPath:LPCSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external shell32 name 'SHQueryRecycleBinA';
function SHQueryRecycleBin(pszRootPath:LPCWSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external shell32 name 'SHQueryRecycleBinW';

function SHEmptyRecycleBinA(hwnd:HWND; pszRootPath:LPCSTR; dwFlags:DWORD):HRESULT;external shell32 name 'SHEmptyRecycleBinA';
function SHEmptyRecycleBinW(hwnd:HWND; pszRootPath:LPCWSTR; dwFlags:DWORD):HRESULT;external shell32 name 'SHEmptyRecycleBinW';
function SHEmptyRecycleBin(hwnd:HWND; pszRootPath:LPCSTR; dwFlags:DWORD):HRESULT;external shell32 name 'SHEmptyRecycleBinA';
function SHEmptyRecycleBin(hwnd:HWND; pszRootPath:LPCWSTR; dwFlags:DWORD):HRESULT;external shell32 name 'SHEmptyRecycleBinW';
=======
=======
>>>>>>> origin/fixes_2_2
function SHQueryRecycleBinA(pszRootPath:LPCSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external 'shell32.dll' name 'SHQueryRecycleBinA';
function SHQueryRecycleBinW(pszRootPath:LPCWSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external 'shell32.dll' name 'SHQueryRecycleBinW';
function SHQueryRecycleBin(pszRootPath:LPCSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external 'shell32.dll' name 'SHQueryRecycleBinA';
function SHQueryRecycleBin(pszRootPath:LPCWSTR; pSHQueryRBInfo:LPSHQUERYRBINFO):HRESULT;external 'shell32.dll' name 'SHQueryRecycleBinW';

function SHEmptyRecycleBinA(hwnd:HWND; pszRootPath:LPCSTR; dwFlags:DWORD):HRESULT;external 'shell32.dll' name 'SHEmptyRecycleBinA';
function SHEmptyRecycleBinW(hwnd:HWND; pszRootPath:LPCWSTR; dwFlags:DWORD):HRESULT;external 'shell32.dll' name 'SHEmptyRecycleBinW';
function SHEmptyRecycleBin(hwnd:HWND; pszRootPath:LPCSTR; dwFlags:DWORD):HRESULT;external 'shell32.dll' name 'SHEmptyRecycleBinA';
function SHEmptyRecycleBin(hwnd:HWND; pszRootPath:LPCWSTR; dwFlags:DWORD):HRESULT;external 'shell32.dll' name 'SHEmptyRecycleBinW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

//
// end of RecycleBin
//

//
// Tray notification definitions
//

Type

       _NOTIFYICONDATAA         = record
                                   cbSize               : DWORD;
                                   hWnd                 : HWND;
                                   uID                  : UINT;
                                   uFlags               : UINT;
                                   uCallbackMessage     : UINT;
                                   hIcon                : HICON;
                                   {$ifdef IELower5}
                                    szTip               : array[0..63] of CHAR;
                                   {$else}
                                    szTip               : array[0..127] of CHAR;
                                   {$endif}
                                   {$ifdef IEhigherEqual5}
                                    dwState             : DWORD;
                                    dwStateMask         : DWORD;
                                    szInfo              : array[0..255] of CHAR;
                                    DUMMYUNIONNAME      : record
                                                           case longint of
                                                               0 : ( uTimeout : UINT );
                                                               1 : ( uVersion : UINT );
                                                              end;
                                    szInfoTitle : array[0..63] of CHAR;
                                    dwInfoFlags : DWORD;
                                   {$endif}
                                   {$ifdef IEHighEq6}
                                    guidItem : GUID;
                                   {$endif}
                                   end;
       NOTIFYICONDATAA          = _NOTIFYICONDATAA;
       TNOTIFYICONDATAA         = _NOTIFYICONDATAA;
       PNOTIFYICONDATAA         = ^_NOTIFYICONDATAA;


       _NOTIFYICONDATAW         = record
                                   cbSize               : DWORD;
                                   hWnd                 : HWND;
                                   uID                  : UINT;
                                   uFlags               : UINT;
                                   uCallbackMessage     : UINT;
                                   hIcon                : HICON;
                                   {$ifdef IELower5}
                                    szTip               : array[0..63] of WCHAR;
                                   {$else}
                                    szTip               : array[0..127] of WCHAR;
                                   {$endif}
                                   {$ifdef IEhigherEqual5}
                                    dwState             : DWORD;
                                    dwStateMask         : DWORD;
                                    szInfo              : array[0..255] of WCHAR;
                                    DUMMYUNIONNAME      : record
                                                           case longint of
                                                               0 : ( uTimeout : UINT );
                                                               1 : ( uVersion : UINT );
                                                              end;
                                    szInfoTitle : array[0..63] of CHAR;
                                    dwInfoFlags : DWORD;
                                   {$endif}
                                   {$ifdef IEHighEq6}
                                    guidItem : GUID;
                                   {$endif}
                                   end;
       NOTIFYICONDATAW          = _NOTIFYICONDATAW;
       TNOTIFYICONDATAW         = _NOTIFYICONDATAW;
       PNOTIFYICONDATAW         = ^_NOTIFYICONDATAW;
{$ifdef UNICODE}
       NOTIFYICONDATA           = NOTIFYICONDATAW;
       TNOTIFYICONDATA          = NOTIFYICONDATAW;
       PNOTIFYICONDATA          = PNOTIFYICONDATAW;
{$else}
       NOTIFYICONDATA           = NOTIFYICONDATAA;
       TNOTIFYICONDATA          = NOTIFYICONDATAA;
       PNOTIFYICONDATA          = PNOTIFYICONDATAA;
{$endif}
    { UNICODE }

    {

#define NOTIFYICONDATAA_V1_SIZE     FIELD_OFFSET(NOTIFYICONDATAA, szTip[64])
#define NOTIFYICONDATAW_V1_SIZE     FIELD_OFFSET(NOTIFYICONDATAW, szTip[64])
#ifdef UNICODE
#define NOTIFYICONDATA_V1_SIZE      NOTIFYICONDATAW_V1_SIZE
#else
#define NOTIFYICONDATA_V1_SIZE      NOTIFYICONDATAA_V1_SIZE
#endif

#define NOTIFYICONDATAA_V2_SIZE     FIELD_OFFSET(NOTIFYICONDATAA, guidItem)
#define NOTIFYICONDATAW_V2_SIZE     FIELD_OFFSET(NOTIFYICONDATAW, guidItem)
#ifdef UNICODE
#define NOTIFYICONDATA_V2_SIZE      NOTIFYICONDATAW_V2_SIZE
#else
#define NOTIFYICONDATA_V2_SIZE      NOTIFYICONDATAA_V2_SIZE
#endif
}


    const
       NIN_SELECT               = WM_USER + 0;
       NINF_KEY                 = $1;
       NIN_KEYSELECT            = NIN_SELECT or NINF_KEY;
// if (_WIN32_IE >= 0x0501)}

       NIN_BALLOONSHOW          = WM_USER + 2;
       NIN_BALLOONHIDE          = WM_USER + 3;
       NIN_BALLOONTIMEOUT       = WM_USER + 4;
       NIN_BALLOONUSERCLICK     = WM_USER + 5;
       NIM_ADD                  = $00000000;
       NIM_MODIFY               = $00000001;
       NIM_DELETE               = $00000002;
//if (_WIN32_IE >= 0x0500)}

       NIM_SETFOCUS             = $00000003;
       NIM_SETVERSION           = $00000004;
       NOTIFYICON_VERSION       = 3;

       NIF_MESSAGE              = $00000001;
       NIF_ICON                 = $00000002;
       NIF_TIP                  = $00000004;
// if (_WIN32_IE >= 0x0500)}
       NIF_STATE                = $00000008;
       NIF_INFO                 = $00000010;
//if (_WIN32_IE >= 0x600)}

       NIF_GUID                 = $00000020;
//if (_WIN32_IE >= 0x0500)}

       NIS_HIDDEN               = $00000001;
       NIS_SHAREDICON           = $00000002;
    { says this is the source of a shared icon }
    { Notify Icon Infotip flags }
       NIIF_NONE                = $00000000;
    { icon flags are mutually exclusive }
    { and take only the lowest 2 bits }
       NIIF_INFO                = $00000001;
       NIIF_WARNING             = $00000002;
       NIIF_ERROR               = $00000003;
       NIIF_ICON_MASK           = $0000000F;
//if (_WIN32_IE >= 0x0501)}

       NIIF_NOSOUND             = $00000010;

<<<<<<< HEAD
<<<<<<< HEAD
Function Shell_NotifyIconA( dwMessage: Dword;lpData: PNOTIFYICONDATAA):Bool;external shell32 name 'Shell_NotifyIconA';
Function Shell_NotifyIconW( dwMessage: Dword;lpData: PNOTIFYICONDATAW):Bool;external shell32 name 'Shell_NotifyIconW';

Function Shell_NotifyIcon( dwMessage: Dword;lpData: PNOTIFYICONDATAA):Bool;external shell32 name 'Shell_NotifyIconA';
Function Shell_NotifyIcon( dwMessage: Dword;lpData: PNOTIFYICONDATAW):Bool;external shell32 name 'Shell_NotifyIconW';
=======
=======
>>>>>>> origin/fixes_2_2
Function Shell_NotifyIconA( dwMessage: Dword;lpData: PNOTIFYICONDATAA):Bool;external 'shell32.dll' name 'Shell_NotifyIconA';
Function Shell_NotifyIconW( dwMessage: Dword;lpData: PNOTIFYICONDATAW):Bool;external 'shell32.dll' name 'Shell_NotifyIconW';

Function Shell_NotifyIcon( dwMessage: Dword;lpData: PNOTIFYICONDATAA):Bool;external 'shell32.dll' name 'Shell_NotifyIconA';
Function Shell_NotifyIcon( dwMessage: Dword;lpData: PNOTIFYICONDATAW):Bool;external 'shell32.dll' name 'Shell_NotifyIconW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//
// End Tray Notification Icons
//

//
// Begin SHGetFileInfo
//
    {
       The SHGetFileInfo API provides an easy way to get attributes
       for a file given a pathname.

         PARAMETERS

           pszPath              file name to get info about
           dwFileAttributes     file attribs, only used with SHGFI_USEFILEATTRIBUTES
           psfi                 place to return file info
           cbFileInfo           size of structure
           uFlags               flags

         RETURN
           TRUE if things worked
      }
    { out: icon }
    { out: icon index }
    { out: SFGAO_ flags }
    { out: display name (or path) }
    { out: type name }

    type

       _SHFILEINFOA                     = record
                                            hIcon         : HICON;                          { out: icon }
                                            iIcon         : longint;                        { out: icon index }
                                            dwAttributes  : DWORD;                          { out: SFGAO_ flags }
                                            szDisplayName : array[0..(MAX_PATH)-1] of CHAR; { out: display name (or path) }
                                            szTypeName    : array[0..79] of CHAR;           { out: type name }
                                           end;
       SHFILEINFOA                      = _SHFILEINFOA;
       TSHFILEINFOA                     = _SHFILEINFOA;
       pSHFILEINFOA                     = ^_SHFILEINFOA;

       _SHFILEINFOW                     = record
                                            hIcon         : HICON;                          { out: icon }
                                            iIcon         : longint;                        { out: icon index }
                                            dwAttributes  : DWORD;                          { out: SFGAO_ flags }
                                            szDisplayName : array[0..(MAX_PATH)-1] of WCHAR;{ out: display name (or path) }
                                            szTypeName    : array[0..79] of WCHAR;          { out: type name }
                                           end;
       SHFILEINFOW                      = _SHFILEINFOW;
       TSHFILEINFOW                     = _SHFILEINFOW;
       pSHFILEINFOW                     = ^_SHFILEINFOW;

{$ifdef UNICODE}
       SHFILEINFO                       = SHFILEINFOW;
       TSHFILEINFO                      = SHFILEINFOW;
       pFILEINFO                        = SHFILEINFOW;
{$else}
       SHFILEINFO                       = SHFILEINFOA;
       TSHFILEINFO                      = SHFILEINFOA;
       pFILEINFO                        = SHFILEINFOA;
{$endif}
    { NOTE: This is also in shlwapi.h.  Please keep in synch. }

    const
       SHGFI_ICON               = $000000100;    { get Icon}
       SHGFI_DISPLAYNAME        = $000000200;    { get display name }
       SHGFI_TYPENAME           = $000000400;    { get type name }
       SHGFI_ATTRIBUTES         = $000000800;    { get attributes }
       SHGFI_ICONLOCATION       = $000001000;    { get icon location}
       SHGFI_EXETYPE            = $000002000;    { return exe type }
       SHGFI_SYSICONINDEX       = $000004000;    { get system icon index }
       SHGFI_LINKOVERLAY        = $000008000;    { put a link overlay on icon }
       SHGFI_SELECTED           = $000010000;    { show icon in selected state }
       SHGFI_ATTR_SPECIFIED     = $000020000;    { get only specified attributes }
       SHGFI_LARGEICON          = $000000000;    { get large icon }
       SHGFI_SMALLICON          = $000000001;    { get small icon }
       SHGFI_OPENICON           = $000000002;    { get open icon }
       SHGFI_SHELLICONSIZE      = $000000004;    { get shell size icon }
       SHGFI_PIDL               = $000000008;    { pszPath is a pidl }
       SHGFI_USEFILEATTRIBUTES  = $000000010;    { use passed dwFileAttribute }
//if (_WIN32_IE >= 0x0500)}
       SHGFI_ADDOVERLAYS        = $000000020;    { apply the appropriate overlays }
       SHGFI_OVERLAYINDEX       = $000000040;    { Get the index of the overlay }
                                                 { in the upper 8 bits of the iIcon  }
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
Function SHGetFileInfoA(pszPath: LPCSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD_PTR;external shell32 name 'SHGetFileInfoA';
Function SHGetFileInfoW(pszPath: LPCWSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD_PTR;external shell32 name 'SHGetFileInfoW';
Function SHGetFileInfo(pszPath: LPCSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD_PTR;external shell32 name 'SHGetFileInfoA';

Function SHGetFileInfoA(pszPath: LPCSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD_PTR;external shell32 name 'SHGetFileInfoA';
Function SHGetFileInfoW(pszPath: LPCWSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD_PTR;external shell32 name 'SHGetFileInfoW';
Function SHGetFileInfo(pszPath: LPCSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD_PTR;external shell32 name 'SHGetFileInfoA';
Function SHGetFileInfo(pszPath: LPCWSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD_PTR;external shell32 name 'SHGetFileInfoW';
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
Function SHGetFileInfoA(pszPath: LPCSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external shell32 name 'SHGetFileInfoA';
Function SHGetFileInfoW(pszPath: LPCWSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD;external shell32 name 'SHGetFileInfoW';
Function SHGetFileInfo(pszPath: LPCSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external shell32 name 'SHGetFileInfoA';

Function SHGetFileInfoA(pszPath: LPCSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external shell32 name 'SHGetFileInfoA';
Function SHGetFileInfoW(pszPath: LPCWSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD;external shell32 name 'SHGetFileInfoW';
Function SHGetFileInfo(pszPath: LPCSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external shell32 name 'SHGetFileInfoA';
Function SHGetFileInfo(pszPath: LPCWSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD;external shell32 name 'SHGetFileInfoW';
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew

Function SHGetDiskFreeSpaceExA( pszDirectoryName : LPCSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external shell32 name 'SHGetDiskFreeSpaceExA';
Function SHGetDiskFreeSpaceExW( pszDirectoryName : LPCWSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external shell32 name 'SHGetDiskFreeSpaceExW';
Function SHGetDiskFreeSpaceEx( pszDirectoryName : LPCSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external shell32 name 'SHGetDiskFreeSpaceExA';
Function SHGetDiskFreeSpace( pszDirectoryName : LPCSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external shell32 name 'SHGetDiskFreeSpaceExA';
Function SHGetDiskFreeSpaceEx( pszDirectoryName : LPCWSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external shell32 name 'SHGetDiskFreeSpaceExW';
Function SHGetDiskFreeSpace( pszDirectoryName : LPCWSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external shell32 name 'SHGetDiskFreeSpaceExW';

Function SHGetNewLinkInfoA(pszLinkTo:LPCSTR;pszDir:LPCSTR; pszName:LPSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external shell32 name 'SHGetNewLinkInfoA';
Function SHGetNewLinkInfoW(pszLinkTo:LPCWSTR;pszDir:LPCWSTR; pszName:LPWSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external shell32 name 'SHGetNewLinkInfoW';

Function SHGetNewLinkInfo (pszLinkTo:LPCSTR;pszDir:LPCSTR; pszName:LPSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external shell32 name 'SHGetNewLinkInfoA';
Function SHGetNewLinkInfo (pszLinkTo:LPCWSTR;pszDir:LPCWSTR; pszName:LPWSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external shell32 name 'SHGetNewLinkInfoW';
=======
=======
>>>>>>> origin/fixes_2_2
Function SHGetFileInfoA(pszPath: LPCSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external 'shell32.dll' name 'SHGetFileInfoA';
Function SHGetFileInfoW(pszPath: LPCWSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD;external 'shell32.dll' name 'SHGetFileInfoW';
Function SHGetFileInfo(pszPath: LPCSTR; dwFileAttributes : DWORD; psfi: pSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external 'shell32.dll' name 'SHGetFileInfoA';

Function SHGetFileInfoA(pszPath: LPCSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external 'shell32.dll' name 'SHGetFileInfoA';
Function SHGetFileInfoW(pszPath: LPCWSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD;external 'shell32.dll' name 'SHGetFileInfoW';
Function SHGetFileInfo(pszPath: LPCSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOA; cbFileInfo,UFlags: UINT):DWORD;external 'shell32.dll' name 'SHGetFileInfoA';
Function SHGetFileInfo(pszPath: LPCWSTR; dwFileAttributes : DWORD; var psfi: TSHFILEINFOW; cbFileInfo,UFlags: UINT):DWORD;external 'shell32.dll' name 'SHGetFileInfoW';

Function SHGetDiskFreeSpaceExA( pszDirectoryName : LPCSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external 'shell32.dll' name 'SHGetDiskFreeSpaceExA';
Function SHGetDiskFreeSpaceExW( pszDirectoryName : LPCWSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external 'shell32.dll' name 'SHGetDiskFreeSpaceExW';
Function SHGetDiskFreeSpaceEx( pszDirectoryName : LPCSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external 'shell32.dll' name 'SHGetDiskFreeSpaceExA';
Function SHGetDiskFreeSpace( pszDirectoryName : LPCSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external 'shell32.dll' name 'SHGetDiskFreeSpaceExA';
Function SHGetDiskFreeSpaceEx( pszDirectoryName : LPCWSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external 'shell32.dll' name 'SHGetDiskFreeSpaceExW';
Function SHGetDiskFreeSpace( pszDirectoryName : LPCWSTR; pulFreeBytesAvailableToCaller : pULARGE_INTEGER; pulTotalNumberOfBytes : pULARGE_INTEGER;pulTotalNumberOfFreeBytes: pULARGE_INTEGER):Bool;external 'shell32.dll' name 'SHGetDiskFreeSpaceExW';

Function SHGetNewLinkInfoA(pszLinkTo:LPCSTR;pszDir:LPCSTR; pszName:LPSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external 'shell32.dll' name 'SHGetNewLinkInfoA';
Function SHGetNewLinkInfoW(pszLinkTo:LPCWSTR;pszDir:LPCWSTR; pszName:LPWSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external 'shell32.dll' name 'SHGetNewLinkInfoW';

Function SHGetNewLinkInfo (pszLinkTo:LPCSTR;pszDir:LPCSTR; pszName:LPSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external 'shell32.dll' name 'SHGetNewLinkInfoA';
Function SHGetNewLinkInfo (pszLinkTo:LPCWSTR;pszDir:LPCWSTR; pszName:LPWSTR; pfMustCopy: pBool; uFlags:UINT):Bool;external 'shell32.dll' name 'SHGetNewLinkInfoW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

    const
       SHGNLI_PIDL              = $000000001;    { pszLinkTo is a pidl }
       SHGNLI_PREFIXNAME        = $000000002;    { Make name "Shortcut to xxx" }
       SHGNLI_NOUNIQUE          = $000000004;    { don't do the unique name generation }
// {if (_WIN32_IE >= 0x0501)}
       SHGNLI_NOLNK             = $000000008;    { don't add ".lnk" extension }
// {$endif}
    { _WIN2_IE >= 0x0501 }
//
// End SHGetFileInfo
//

    { Printer stuff }
       PRINTACTION_OPEN             = 0;
       PRINTACTION_PROPERTIES       = 1;
       PRINTACTION_NETINSTALL       = 2;
       PRINTACTION_NETINSTALLLINK   = 3;
       PRINTACTION_TESTPAGE         = 4;
       PRINTACTION_OPENNETPRN       = 5;
{$ifdef WINNT}
       PRINTACTION_DOCUMENTDEFAULTS = 6;
       PRINTACTION_SERVERPROPERTIES = 7;
{$endif}

<<<<<<< HEAD
<<<<<<< HEAD
Function SHInvokePrinterCommandA(HWND: hwnd; uAction:UINT; lpBuf1: LPCSTR; lpBuf2: LPCSTR; fModal:Bool):Bool;external shell32 name 'SHInvokePrinterCommandA';
Function SHInvokePrinterCommandW(HWND: hwnd; uAction:UINT; lpBuf1: LPCWSTR; lpBuf2: LPCWSTR; fModal:Bool):Bool;external shell32 name 'SHInvokePrinterCommandW';
Function SHInvokePrinterCommand(HWND: hwnd; uAction:UINT; lpBuf1: LPCSTR; lpBuf2: LPCSTR; fModal:Bool):Bool;external shell32 name 'SHInvokePrinterCommandA';
Function SHInvokePrinterCommand(HWND: hwnd; uAction:UINT; lpBuf1: LPCWSTR; lpBuf2: LPCWSTR; fModal:Bool):Bool;external shell32 name 'SHInvokePrinterCommandW';
=======
=======
>>>>>>> origin/fixes_2_2
Function SHInvokePrinterCommandA(HWND: hwnd; uAction:UINT; lpBuf1: LPCSTR; lpBuf2: LPCSTR; fModal:Bool):Bool;external 'shell32.dll' name 'SHInvokePrinterCommandA';
Function SHInvokePrinterCommandW(HWND: hwnd; uAction:UINT; lpBuf1: LPCWSTR; lpBuf2: LPCWSTR; fModal:Bool):Bool;external 'shell32.dll' name 'SHInvokePrinterCommandW';
Function SHInvokePrinterCommand(HWND: hwnd; uAction:UINT; lpBuf1: LPCSTR; lpBuf2: LPCSTR; fModal:Bool):Bool;external 'shell32.dll' name 'SHInvokePrinterCommandA';
Function SHInvokePrinterCommand(HWND: hwnd; uAction:UINT; lpBuf1: LPCWSTR; lpBuf2: LPCWSTR; fModal:Bool):Bool;external 'shell32.dll' name 'SHInvokePrinterCommandW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

// WINVER >= 0x0400
//if (_WIN32_WINNT >= 0x0500) || (_WIN32_WINDOWS >= 0x0500)
    //
    // The SHLoadNonloadedIconOverlayIdentifiers API causes the shell's
    // icon overlay manager to load any registered icon overlay
    // identifers that are not currently loaded.  This is useful if an
    // overlay identifier did not load at shell startup but is needed
    // and can be loaded at a later time.  Identifiers already loaded
    // are not affected.  Overlay identifiers implement the
    // IShellIconOverlayIdentifier interface.
    //
    // Returns:
    //      S_OK
    //

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function SHLoadNonloadedIconOverlayIdentifiers:HResult; external shell32 name 'SHLoadNonloadedIconOverlayIdentifiers';
=======
function SHLoadNonloadedIconOverlayIdentifiers:HResult; external shell32 name 'SHInvokePrinterCommandW';
>>>>>>> graemeg/cpstrnew
=======
function SHLoadNonloadedIconOverlayIdentifiers:HResult; external shell32 name 'SHInvokePrinterCommandW';
>>>>>>> graemeg/cpstrnew
=======
function SHLoadNonloadedIconOverlayIdentifiers:HResult; external shell32 name 'SHInvokePrinterCommandW';
>>>>>>> graemeg/cpstrnew
=======
function SHLoadNonloadedIconOverlayIdentifiers:HResult; external shell32 name 'SHInvokePrinterCommandW';
>>>>>>> origin/cpstrnew
=======
function SHLoadNonloadedIconOverlayIdentifiers:HResult; external 'shell32.dll' name 'SHInvokePrinterCommandW';
>>>>>>> graemeg/fixes_2_2
=======
function SHLoadNonloadedIconOverlayIdentifiers:HResult; external 'shell32.dll' name 'SHInvokePrinterCommandW';
>>>>>>> origin/fixes_2_2

    //
    // The SHIsFileAvailableOffline API determines whether a file
    // or folder is available for offline use.
    //
    // Parameters:
    //     pwszPath             file name to get info about
    //     pdwStatus            (optional) OFFLINE_STATUS_* flags returned here
    //
    // Returns:
    //     S_OK                 File/directory is available offline, unless
    //                            OFFLINE_STATUS_INCOMPLETE is returned.
    //     E_INVALIDARG         Path is invalid, or not a net path
    //     E_FAIL               File/directory is not available offline
    //
    // Notes:
    //     OFFLINE_STATUS_INCOMPLETE is never returned for directories.
    //     Both OFFLINE_STATUS_LOCAL and OFFLINE_STATUS_REMOTE may be returned,
    //     indicating "open in both places." This is common when the server is online.
    //
<<<<<<< HEAD
<<<<<<< HEAD
function SHIsFileAvailableOffline(pwszPath:LPCWSTR; pdwStatus:LPDWORD):HRESULT; external shell32 name 'SHIsFileAvailableOffline';
=======
function SHIsFileAvailableOffline(pwszPath:LPCWSTR; pdwStatus:LPDWORD):HRESULT; external 'shell32.dll' name 'SHIsFileAvailableOffline';
>>>>>>> graemeg/fixes_2_2
=======
function SHIsFileAvailableOffline(pwszPath:LPCWSTR; pdwStatus:LPDWORD):HRESULT; external 'shell32.dll' name 'SHIsFileAvailableOffline';
>>>>>>> origin/fixes_2_2

const
       OFFLINE_STATUS_LOCAL         = $0001;    { If open, it's open locally }
       OFFLINE_STATUS_REMOTE        = $0002;    { If open, it's open remotely }
       OFFLINE_STATUS_INCOMPLETE    = $0004;    { The local copy is currently incomplete. }
                                                { The file will not be available offline }
                                                { until it has been synchronized. }
    {  sets the specified path to use the string resource }
    {  as the UI instead of the file system name }

<<<<<<< HEAD
<<<<<<< HEAD
function SHSetLocalizedName(pszPath:LPWSTR; pszResModule:LPCWSTR; idsRes:longint):HRESULT;external shell32 name 'SHSetLocalizedName';

//if         _WIN32_IE >= 0x0600}

function SHEnumerateUnreadMailAccountsA(hKeyUser:HKEY; dwIndex:DWORD; pszMailAddress:LPSTR; cchMailAddress:longint):HRESULT;external shell32 name 'SHEnumerateUnreadMailAccountsA';
function SHEnumerateUnreadMailAccountsW(hKeyUser:HKEY; dwIndex:DWORD; pszMailAddress:LPWSTR; cchMailAddress:longint):HRESULT;external shell32 name 'SHEnumerateUnreadMailAccountsW';

function SHEnumerateUnreadMailAccounts(hKeyUser:HKEY; dwIndex:DWORD; pszMailAddress:LPWSTR; cchMailAddress:longint):HRESULT;external shell32 name 'SHEnumerateUnreadMailAccountsW';

function SHGetUnreadMailCountA(hKeyUser:HKEY; pszMailAddress:LPCSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPSTR;cchShellExecuteCommand:longint):HRESULT;external shell32 name 'SHGetUnreadMailCountA';
function SHGetUnreadMailCountW(hKeyUser:HKEY; pszMailAddress:LPCWSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPWSTR;cchShellExecuteCommand:longint):HRESULT;external shell32 name 'SHGetUnreadMailCountW';
function SHGetUnreadMailCount(hKeyUser:HKEY; pszMailAddress:LPCSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPSTR;cchShellExecuteCommand:longint):HRESULT;external shell32 name 'SHGetUnreadMailCountA';
function SHGetUnreadMailCount(hKeyUser:HKEY; pszMailAddress:LPCWSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPWSTR;cchShellExecuteCommand:longint):HRESULT;external shell32 name 'SHGetUnreadMailCountW';

function SHSetUnreadMailCountA(pszMailAddress:LPCSTR; dwCount:DWORD; pszShellExecuteCommand:LPCSTR):HRESULT;external shell32 name 'SHSetUnreadMailCountA';
function SHSetUnreadMailCountW(pszMailAddress:LPCWSTR; dwCount:DWORD; pszShellExecuteCommand:LPCWSTR):HRESULT;external shell32 name 'SHSetUnreadMailCountW';
function SHSetUnreadMailCount(pszMailAddress:LPCSTR; dwCount:DWORD; pszShellExecuteCommand:LPCSTR):HRESULT;external shell32 name 'SHSetUnreadMailCountA';
function SHSetUnreadMailCount(pszMailAddress:LPCWSTR; dwCount:DWORD; pszShellExecuteCommand:LPCWSTR):HRESULT;external shell32 name 'SHSetUnreadMailCountW';
=======
=======
>>>>>>> origin/fixes_2_2
function SHSetLocalizedName(pszPath:LPWSTR; pszResModule:LPCWSTR; idsRes:longint):HRESULT;external 'shell32.dll' name 'SHSetLocalizedName';

//if         _WIN32_IE >= 0x0600}

function SHEnumerateUnreadMailAccountsA(hKeyUser:HKEY; dwIndex:DWORD; pszMailAddress:LPSTR; cchMailAddress:longint):HRESULT;external 'shell32.dll' name 'SHEnumerateUnreadMailAccountsA';
function SHEnumerateUnreadMailAccountsW(hKeyUser:HKEY; dwIndex:DWORD; pszMailAddress:LPWSTR; cchMailAddress:longint):HRESULT;external 'shell32.dll' name 'SHEnumerateUnreadMailAccountsW';

function SHEnumerateUnreadMailAccounts(hKeyUser:HKEY; dwIndex:DWORD; pszMailAddress:LPWSTR; cchMailAddress:longint):HRESULT;external 'shell32.dll' name 'SHEnumerateUnreadMailAccountsW';

function SHGetUnreadMailCountA(hKeyUser:HKEY; pszMailAddress:LPCSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPSTR;cchShellExecuteCommand:longint):HRESULT;external 'shell32.dll' name 'SHGetUnreadMailCountA';
function SHGetUnreadMailCountW(hKeyUser:HKEY; pszMailAddress:LPCWSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPWSTR;cchShellExecuteCommand:longint):HRESULT;external 'shell32.dll' name 'SHGetUnreadMailCountW';
function SHGetUnreadMailCount(hKeyUser:HKEY; pszMailAddress:LPCSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPSTR;cchShellExecuteCommand:longint):HRESULT;external 'shell32.dll' name 'SHGetUnreadMailCountA';
function SHGetUnreadMailCount(hKeyUser:HKEY; pszMailAddress:LPCWSTR; pdwCount:PDWORD; pFileTime:PFILETIME; pszShellExecuteCommand:LPWSTR;cchShellExecuteCommand:longint):HRESULT;external 'shell32.dll' name 'SHGetUnreadMailCountW';

function SHSetUnreadMailCountA(pszMailAddress:LPCSTR; dwCount:DWORD; pszShellExecuteCommand:LPCSTR):HRESULT;external 'shell32.dll' name 'SHSetUnreadMailCountA';
function SHSetUnreadMailCountW(pszMailAddress:LPCWSTR; dwCount:DWORD; pszShellExecuteCommand:LPCWSTR):HRESULT;external 'shell32.dll' name 'SHSetUnreadMailCountW';
function SHSetUnreadMailCount(pszMailAddress:LPCSTR; dwCount:DWORD; pszShellExecuteCommand:LPCSTR):HRESULT;external 'shell32.dll' name 'SHSetUnreadMailCountA';
function SHSetUnreadMailCount(pszMailAddress:LPCWSTR; dwCount:DWORD; pszShellExecuteCommand:LPCWSTR):HRESULT;external 'shell32.dll' name 'SHSetUnreadMailCountW';
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

//  _WIN32_IE >= 0x0600      }
//  if         _WIN32_IE >= 0x0600}

<<<<<<< HEAD
<<<<<<< HEAD
function SHGetImageList(iImageList:longint;CONST riid:TIID; ppvObj:Ppointer):HRESULT;external shell32 name 'SHGetImageList';
=======
function SHGetImageList(iImageList:longint;CONST riid:TIID; ppvObj:Ppointer):HRESULT;external 'shell32.dll' name 'SHGetImageList';
>>>>>>> graemeg/fixes_2_2
=======
function SHGetImageList(iImageList:longint;CONST riid:TIID; ppvObj:Ppointer):HRESULT;external 'shell32.dll' name 'SHGetImageList';
>>>>>>> origin/fixes_2_2

Const
       SHIL_LARGE                   = 0;    { normally 32x32 }
       SHIL_SMALL                   = 1;    { normally 16x16 }
       SHIL_EXTRALARGE              = 2;
       SHIL_SYSSMALL                = 3;    { like SHIL_SMALL, but tracks system small icon metric correctly }
       SHIL_LAST                    = SHIL_SYSSMALL;

    { Function call types for ntshrui folder sharing helpers }

//typedef HRESULT (STDMETHODCALLTYPE *PFNSHOWSHAREFOLDERUIW)(IN HWND hwndParent, IN LPCSTR pszPath);
//typedef HRESULT (STDMETHODCALLTYPE *PFNSHOWSHAREFOLDERUIW)(IN HWND hwndParent, IN LPCWSTR pszPath);

implementation

function EIRESID(x : longint) : longint;
Begin
  EIRESID:=-x;
End;

end.
