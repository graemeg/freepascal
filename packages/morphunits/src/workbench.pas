{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}

unit workbench;

interface

uses
  exec, amigados, utility, intuition, agraphics;

// wb startup defines
type
  PWBArg = ^TWBArg;
  TWBArg = record
    wa_Lock: BPTR;   // a lock descriptor
    wa_Name: STRPTR; // a string relative to that lock
  end;

  WBArgList = array[1..100] of TWBArg; // Only 1..smNumArgs are valid
  PWBArgList = ^WBArgList;

  PWBStartup = ^TWBStartup;
  TWBStartup = record
    sm_Message: TMessage;   // a standard message structure
    sm_Process: PMsgPort;   // the process descriptor for you
    sm_Segment: BPTR;       // a descriptor for your code
    sm_NumArgs: Longint;    // the number of elements in ArgList
    sm_ToolWindow: STRPTR;  // description of window
    sm_ArgList: PWBArgList; // the arguments themselves
  end;

const
  WBDISK    = 1;
  WBDRAWER  = 2;
  WBTOOL    = 3;
  WBPROJECT = 4;
  WBGARBAGE = 5;
  WBDEVICE  = 6;
  WBKICK    = 7;
  WBAPPICON = 8;

type
  POldDrawerData = ^TOldDrawerData;
  TOldDrawerData = record
    dd_NewWindow: TNewWindow; // args to open window
    dd_CurrentX: Longint;     // current x coordinate of origin
    dd_CurrentY: Longint;     // current y coordinate of origin
  end;

const
  // the amount of DrawerData actually written to disk
  OLDDRAWERDATAFILESIZE  = SizeOf(TOldDrawerData);

type
  PDrawerData = ^TDrawerData;
  TDrawerData = record
    dd_NewWindow: TNewWindow; // args to open window
    dd_CurrentX: Longint;     // current x coordinate of origin
    dd_CurrentY: Longint;     // current y coordinate of origin
    dd_Flags: Longint;        // flags for drawer DDFLAGS_*
    dd_ViewModes: Word;       // view mode for drawer DDVM_*
  end;

const
  // the amount of DrawerData actually written to disk
  DRAWERDATAFILESIZE  = SizeOf(TDrawerData);

  // dd_ViewModes
  DDVM_BYDEFAULT = 0;
  DDVM_BYICON    = 1; // view as icons
  DDVM_BYNAME    = 2; // view as text, sorted by name
  DDVM_BYDATE    = 3; // view as text, sorted by date
  DDVM_BYSIZE    = 4; // view as text, sorted by size
  DDVM_BYTYPE    = 5; // view as text, sorted by type

  // definitions for dd_Flags
  DDFLAGS_SHOWDEFAULT = 0; // default (show only icons)
  DDFLAGS_SHOWICONS   = 1; // show only icons
  DDFLAGS_SHOWALL     = 2; // show all files

type
  PDiskObject = ^TDiskObject;
  TDiskObject = record
    do_Magic: Word;         // a magic number at the start of the file
    do_Version: Word;       // a version number, so we can change it
    do_Gadget: TGadget;     // a copy of in core gadget
    do_Type: Byte;
    do_DefaultTool: STRPTR;
    do_ToolTypes: PPChar;
    do_CurrentX: LongInt;
    do_CurrentY: LongInt;
    do_DrawerData: PDrawerData;
    do_ToolWindow: STRPTR;  // only applies to tools
    do_StackSize: Longint;  // only applies to tools
  end;

const
  WB_DISKMAGIC        = $e310; // a magic number, not easily impersonated
  WB_DISKVERSION      = 1;     // our current version number
  WB_DISKREVISION     = 1;     // our current revision number
  WB_DISKREVISIONMASK = 255;   // I only use the lower 8 bits of Gadget.UserData for the revision #

type
  PFreeList = ^TFreeList;
  TFreeList = record
    fl_NumFree: SmallInt;
    fl_MemList: TList;
  end;

const
{ workbench does different complement modes for its gadgets. It supports separate images, complement mode, and backfill mode.
  The first two are identical to intuitions GADGIMAGE and GADGHCOMP. backfill is similar to GADGHCOMP, but the region outside of the
  image (which normally would be color three when complemented) is flood-filled to color zero. }
  GFLG_GADGBACKFILL = $0001;
  GADGBACKFILL      = GFLG_GADGBACKFILL; // an old synonym

  NO_ICON_POSITION  = $80000000; // if an icon does not really live anywhere, set its current position to here

  AM_VERSION = 1; // If you find am_Version >= AM_VERSION, you know this structure has at least the fields defined in this version of the include file

type
  PAppMessage = ^TAppMessage;
  TAppMessage = record
    am_Message: TMessage;   // standard message structure
    am_Type: Word;          // message type AMTYPE_*
    am_UserData: LongWord;  // application specific
    am_ID: LongWord;        // application definable ID
    am_NumArgs: LongInt;    // # of elements in arglist
    am_ArgList: PWBArgList; // the arguements themselves
    am_Version: Word;       // will be AM_VERSION
    am_Class: Word;         // message class AMCLASSICON_*
    am_MouseX: SmallInt;    // mouse x position of event
    am_MouseY: SmallInt;    // mouse y position of event
    am_Seconds: LongWord;   // current system clock time
    am_Micros: LongWord;    // current system clock time
    am_Reserved: array[0..7] of LongWord; // avoid recompilation
  end;

// types of app messages
const
  // am_Type
  AMTYPE_APPWINDOW     = 7;  // app window message
  AMTYPE_APPICON       = 8;  // app icon message
  AMTYPE_APPMENUITEM   = 9;  // app menu item message
  AMTYPE_APPWINDOWZONE = 10; // app menu item message

  //am_Class Classes of AppIcon messages (V44)
  AMCLASSICON_Open        = 0;  // The "Open" menu item was invoked, the icon got double-clicked or an icon got dropped on it.
  AMCLASSICON_Copy        = 1;  // The "Copy" menu item was invoked
  AMCLASSICON_Rename      = 2;  // The "Rename" menu item was invoked
  AMCLASSICON_Information = 3;  // The "Information" menu item was invoked
  AMCLASSICON_Snapshot    = 4;  // The "Snapshot" menu item was invoked
  AMCLASSICON_UnSnapshot  = 5;  // The "UnSnapshot" menu item was invoked
  AMCLASSICON_LeaveOut    = 6;  // The "Leave Out" menu item was invoked
  AMCLASSICON_PutAway     = 7;  // The "Put Away" menu item was invoked
  AMCLASSICON_Delete      = 8;  // The "Delete" menu item was invoked
  AMCLASSICON_FormatDisk  = 9;  // The "Format Disk" menu item was invoked
  AMCLASSICON_EmptyTrash  = 10; // The "Empty Trash" menu item was invoked

  AMCLASSICON_Selected    = 11; // The icon is now selected
  AMCLASSICON_Unselected  = 12; // The icon is now unselected

// The following structures are private.  These are just stub structures for code compatibility...
type
  PAppWindow = ^TAppWindow;
  TAppWindow = record
    aw_PRIVATE: APTR;
  end;

  PAppWindowDropZone = ^TAppWindowDropZone;
  TAppWindowDropZone = record
    awdz_PRIVATE: APTR;
  end;

  PAppIcon = ^TAppIcon;
  TAppIcon = record
    ai_PRIVATE: APTR;
  end;

  PAppMenuItem = ^TAppMenuItem;
  TAppMenuItem = record
    ami_PRIVATE: APTR;
  end;

  PAppMenu = ^TAppMenu;
  TAppMenu = record
    am_PRIVATE: APTR;
  end;

const
  // Tags for use with AddAppIconA()
  WBA_Dummy = TAG_USER + $A000;

  WBAPPICONA_SupportsOpen        = WBA_Dummy + 1;  // (Boolean) AppIcon responds to the "Open" menu item
  WBAPPICONA_SupportsCopy        = WBA_Dummy + 2;  // (Boolean) AppIcon responds to the "Copy" menu item
  WBAPPICONA_SupportsRename      = WBA_Dummy + 3;  // (Boolean) AppIcon responds to the "Rename" menu item
  WBAPPICONA_SupportsInformation = WBA_Dummy + 4;  // (Boolean) AppIcon responds to the "Information" menu item
  WBAPPICONA_SupportsSnapshot    = WBA_Dummy + 5;  // (Boolean) AppIcon responds to the "Snapshot" menu item
  WBAPPICONA_SupportsUnSnapshot  = WBA_Dummy + 6;  // (Boolean) AppIcon responds to the "UnSnapshot" menu item
  WBAPPICONA_SupportsLeaveOut    = WBA_Dummy + 7;  // (Boolean) AppIcon responds to the "LeaveOut" menu item
  WBAPPICONA_SupportsPutAway     = WBA_Dummy + 8;  // (Boolean) AppIcon responds to the "PutAway" menu item
  WBAPPICONA_SupportsDelete      = WBA_Dummy + 9;  // (Boolean) AppIcon responds to the "Delete" menu item
  WBAPPICONA_SupportsFormatDisk  = WBA_Dummy + 10; // (Boolean) AppIcon responds to the "FormatDisk" menu item
  WBAPPICONA_SupportsEmptyTrash  = WBA_Dummy + 11; // (Boolean) AppIcon responds to the "EmptyTrash" menu item
  WBAPPICONA_PropagatePosition   = WBA_Dummy + 12; // (Boolean) AppIcon position should be propagated back to original DiskObject
  WBAPPICONA_RenderHook          = WBA_Dummy + 13; // (PHook) Callback hook to be invoked when rendering this icon
  WBAPPICONA_NotifySelectState   = WBA_Dummy + 14; // (Boolean) AppIcon wants to be notified when its select state changes
  // Tags for use with AddAppMenuA()
  WBAPPMENUA_CommandKeyString = WBA_Dummy + 15; // (STRPTR) Command key string for this AppMenu
  // Tags for use with OpenWorkbenchObjectA()
  WBOPENA_ArgLock = WBA_Dummy + 16; // Corresponds to the wa_Lock member of a TWBArg
  WBOPENA_ArgName = WBA_Dummy + 17; // Corresponds to the wa_Name member of a TWBArg
  // Tags for use with WorkbenchControlA()
  WBCTRLA_IsOpen              = WBA_Dummy + 18; // (PLongWord) Check if the named drawer is currently open
  WBCTRLA_DuplicateSearchPath = WBA_Dummy + 19; // (^BPTR) Create a duplicate of the Workbench private search path list
  WBCTRLA_FreeSearchPath      = WBA_Dummy + 20; // (BPTR) Free the duplicated search path list
  WBCTRLA_GetDefaultStackSize = WBA_Dummy + 21; // (PLongWord) Get the default stack size for launching programs with
  WBCTRLA_SetDefaultStackSize = WBA_Dummy + 22; // (LongWord) Set the default stack size for launching programs with
  WBCTRLA_RedrawAppIcon       = WBA_Dummy + 23; // (PAppIcon) Cause an AppIcon to be redrawn
  WBCTRLA_GetProgramList      = WBA_Dummy + 24; // (PList) Get a list of currently running Workbench programs
  WBCTRLA_FreeProgramList     = WBA_Dummy + 25; // (PList) Release the list of currently running Workbench programs
  // Tags for use with AddAppWindowDropZoneA()
  WBDZA_Left      = WBA_Dummy + 26; // (SmallInt) Zone left edge
  WBDZA_RelRight  = WBA_Dummy + 27; // (SmallInt) Zone left edge, if relative to the right edge of the window
  WBDZA_Top       = WBA_Dummy + 28; // (SmallInt) Zone top edge
  WBDZA_RelBottom = WBA_Dummy + 29; // (SmallInt) Zone top edge, if relative to the bottom edge of the window
  WBDZA_Width     = WBA_Dummy + 30; // (SmallInt) Zone width
  WBDZA_RelWidth  = WBA_Dummy + 31; // (SmallInt) Zone width, if relative to the window width
  WBDZA_Height    = WBA_Dummy + 32; // (SmallInt) Zone height
  WBDZA_RelHeight = WBA_Dummy + 33; // (SmallInt) Zone height, if relative to the window height
  WBDZA_Box       = WBA_Dummy + 34; // (PIBox) Zone position and size
  WBDZA_Hook      = WBA_Dummy + 35; // (PHook) Hook to invoke when the mouse enters or leave a drop zone
  // Tags for use with WorkbenchControlA()

  WBCTRLA_GetSelectedIconList  = WBA_Dummy + 36; // (PList) Get a list of currently selected icons
  WBCTRLA_FreeSelectedIconList = WBA_Dummy + 37; // (PList) Release the list of currently selected icons
  WBCTRLA_GetOpenDrawerList    = WBA_Dummy + 38; // (PList) Get a list of currently open drawers
  WBCTRLA_FreeOpenDrawerList   = WBA_Dummy + 39; // (PList) Release the list of currently open icons

  WBA_Reserved1 = WBA_Dummy + 40;
  WBA_Reserved2 = WBA_Dummy + 41;

  WBCTRLA_GetHiddenDeviceList    = WBA_Dummy + 42; // (PList) Get the list of hidden devices
  WBCTRLA_FreeHiddenDeviceList   = WBA_Dummy + 43; // (PList) Release the list of hidden devices
  WBCTRLA_AddHiddenDeviceName    = WBA_Dummy + 44; // (STRPTR) Add the name of a device which Workbench should never try to read a disk icon from
  WBCTRLA_RemoveHiddenDeviceName = WBA_Dummy + 45; // (STRPTR) Remove a name from list of hidden devices

  WBA_Reserved3 = WBA_Dummy + 46;

  WBCTRLA_GetTypeRestartTime = WBA_Dummy + 47; // (PLongWord) Get the number of seconds that have to pass before typing the next character in a drawer window will restart with a new file name
  WBCTRLA_SetTypeRestartTime = WBA_Dummy + 48; // (LongWord) Set the number of seconds that have to pass before typing the next character in a drawer window will restart with a new file name

  WBA_Reserved4 = WBA_Dummy + 49;

  WBA_Reserved5  = WBA_Dummy + 50;
  WBA_Reserved6  = WBA_Dummy + 51;
  WBA_Reserved7  = WBA_Dummy + 52;
  WBA_Reserved8  = WBA_Dummy + 53;
  WBA_Reserved9  = WBA_Dummy + 54;
  WBA_Reserved10 = WBA_Dummy + 55;
  WBA_Reserved11 = WBA_Dummy + 56;
  WBA_Reserved12 = WBA_Dummy + 57;
  WBA_Reserved13 = WBA_Dummy + 58;
  WBA_Reserved14 = WBA_Dummy + 59;
  WBA_Reserved15 = WBA_Dummy + 60;
  WBA_Reserved16 = WBA_Dummy + 61;
  WBA_Reserved17 = WBA_Dummy + 62;
  WBA_Reserved18 = WBA_Dummy + 63;
  WBA_Reserved19 = WBA_Dummy + 64;

  WBAPPMENUA_GetKey = WBA_Dummy + 65; // (PLongWord) Item to be added should get sub menu items attached to; make room for it, then return the key to use later for attaching the items
  WBAPPMENUA_UseKey = WBA_Dummy + 66; // (LongWord) This item should be attached to a sub menu; the key provided refers to the sub menu it should be attached to

  // V45
  WBCTRLA_GetCopyHook      = WBA_Dummy + 69; // (PPHook) Obtain the hook that will be invoked when Workbench starts to copy files and data
  WBCTRLA_SetCopyHook      = WBA_Dummy + 70; // (PHook) Install the hook that will be invoked when Workbench starts to copy files and data
  WBCTRLA_GetDeleteHook    = WBA_Dummy + 71; // (PPHook) Obtain the hook that will be invoked when Workbench discards files and drawers or empties the trashcan
  WBCTRLA_SetDeleteHook    = WBA_Dummy + 72; // (PHook) Install the hook that will be invoked when Workbench discards files and drawers or empties the trashcan
  WBCTRLA_GetTextInputHook = WBA_Dummy + 73; // (PPHook) Obtain the hook that will be invoked when Workbench requests that the user enters text, such as when a file is to be renamed or a new drawer is to be created
  WBCTRLA_SetTextInputHook = WBA_Dummy + 74; // (PHook) Install the hook that will be invoked when Workbench requests that the user enters text, such as when a file is to be renamed or a new drawer is to be created

  WBOPENA_Show   = WBA_Dummy + 75; // (Byte) When opening a drawer, show all files or only icons? This must be one out of DDFLAGS_SHOWICONS, or DDFLAGS_SHOWALL
  WBOPENA_ViewBy = WBA_Dummy + 76; // (Byte) When opening a drawer, view the contents by icon, name, date, size or type? This must be one out of DDVM_BYICON, DDVM_BYNAME, DDVM_BYDATE, DDVM_BYSIZE or DDVM_BYTYPE;

  WBAPPMENUA_GetTitleKey = WBA_Dummy + 77; // (PLongWord) Item to be added is in fact a new menu title; make room for it, then return the key to use later for attaching the items

  WBCTRLA_AddSetupCleanupHook = WBA_Dummy + 78; // (PHook) Add a hook that will be invoked when Workbench is about to shut down (cleanup), and when Workbench has returned to operational state (setup)
  WBCTRLA_RemSetupCleanupHook = WBA_Dummy + 79; // (PHook) Remove a hook that has been installed with the WBCTRLA_AddSetupCleanupHook tag

  // V50
  WBAPPICONA_Clone = WBA_Dummy + 80; // (Boolean) Clone appicon default: False

  WBA_LAST_TAG = WBAPPICONA_Clone;

  // The message your setup/cleanup hook gets invoked with.
type
  PSetupCleanupHookMsg = ^TSetupCleanupHookMsg;
  TSetupCleanupHookMsg = record
    schm_Length: LongWord;
    schm_State: LongInt;   // SCHMSTATE_*
  end;

const
  // schm_State
  SCHMSTATE_TryCleanup = 0; // Workbench will attempt to shut down now.
  SCHMSTATE_Cleanup    = 1; // Workbench will really shut down now.
  SCHMSTATE_Setup      = 2; // Workbench is operational again or could not be shut down.

type
// The message your AppIcon rendering hook gets invoked with.
  PAppIconRenderMsg = ^TAppIconRenderMsg;
  TAppIconRenderMsg = record
    arm_RastPort: PRastPort; // RastPort to render into
    arm_Icon: PDiskObject;   // The icon to be rendered
    arm_Label: STRPTR;       // The icon label txt
    arm_Tags: PTagItem;      // Further tags to be passed on to DrawIconStateA().
    arm_Left: SmallInt;      // \ Rendering origin, not taking the
    arm_Top: SmallInt;       // / button border into account.
    arm_Width: SmallInt;     // \ Limit your rendering to
    arm_Height: SmallInt;    // / this area.
    arm_State: LongWord;     // IDS_SELECTED, IDS_NORMAL, etc.
  end;

// The message your drop zone hook gets invoked with. }
  PAppWindowDropZoneMsg = ^TAppWindowDropZoneMsg;
  TAppWindowDropZoneMsg = record
    adzm_RastPort: PRastPort; // RastPort to render into.
    adzm_DropZoneBox: TIBox;  // Limit your rendering to this area.
    adzm_ID: LongWord;        // \ These come from straight
    adzm_UserData: LongWord;  // / from AddAppWindowDropZoneA().
    adzm_Action: LongInt;     // See below for a list of actions. ADZMACTION_*
  end;

const
  // adzm_Action
  ADZMACTION_Enter = 0;
  ADZMACTION_Leave = 1;

type
// The message your icon selection change hook is invoked with.
  PIconSelectMsg = ^TIconSelectMsg;
  TIconSelectMsg = record
    ism_Length: LongWord;      // Size of this data structure (in bytes)
    ism_Drawer: BPTR;          // Lock on the drawer this object resides in, NULL for Workbench backdrop (devices)
    ism_Name: STRPTR;          // Name of the object in question
    ism_Type: Word;            // One of WBDISK, WBDRAWER, WBTOOL, WBPROJECT, WBGARBAGE, WBDEVICE, WBKICK or WBAPPICON
    ism_Selected: WordBool;    // True if currently selected, False otherwise.
    ism_Tags: PTagItem;        // Pointer to the list of tag items passed to ChangeWorkbenchSelectionA()
    ism_DrawerWindow: PWindow; // Pointer to the window attached to this icon,if the icon is a drawer-like object.}
    ism_ParentWindow : PWindow;{ Pointer to the window the icon resides in. }

    ism_Left: SmallInt;   // Position and size of the icon;
    ism_Top: SmallInt;    // note that the icon may not entirely
    ism_Width: SmallInt;  // reside within the visible bounds of
    ism_Height: SmallInt; // the parent window
  end;

const
  // These are the values your hook code can return.
  ISMACTION_Unselect = 0; // Unselect the icon
  ISMACTION_Select   = 1; // Select the icon
  ISMACTION_Ignore   = 2; // Do not change the selection state.
  ISMACTION_Stop     = 3; // Do not invoke the hook code again, leave the icon as it is.

type
// The messages your copy hook is invoked with.
  PCopyBeginMsg = ^TCopyBeginMsg;
  TCopyBeginMsg = record
    cbm_Length: LongWord;        // Size of this data structure in bytes
    cbm_Action: LongInt;         // Will be set to CPACTION_Begin (see below)
    cbm_SourceDrawer: BPTR;      // A lock on the source drawer
    cbm_DestinationDrawer: BPTR; // A lock on the destination drawer
  end;

  PCopyDataMsg = ^TCopyDataMsg;
  TCopyDataMsg = record
    cdm_Length: LongWord;        // Size of this data structure in bytes.
    cdm_Action: LongInt;         // Will be set to CPACTION_Copy (see below).
    cdm_SourceLock: BPTR;        // A lock on the parent directory of the source file/drawer.
    cdm_SourceName: STRPTR;      // The name of the source file or drawer.
    cdm_DestinationLock: BPTR;   // A lock on the parent directory of the destination file/drawer.
    cdm_DestinationName: STRPTR; // The name of the destination file/drawer. This may or may not match the name of the source file/drawer in case the
                                 //   data is to be copied under a different name. For example, this is the case with the Workbench "Copy" command which
                                 //   creates duplicates of file/drawers by prefixing the duplicate's name with "Copy_XXX_of".
    cdm_DestinationX: LongInt;   // When the icon corresponding to the destination is written to disk, this is the position (put into its
    cdm_DestinationY: LongInt;   // DiskObject^.do_CurrentX/DiskObject^.do_CurrentY fields) it should be placed at
  end;

  PCopyEndMsg = ^TCopyEndMsg;
  TCopyEndMsg = record
    cem_Length: LongWord; // Size of this data structure in bytes
    cem_Action: LongInt;  // Will be set to CPACTION_End (see below)
  end;

const
  CPACTION_Begin = 0; // This message arrives for each file or drawer to be copied.
  CPACTION_Copy  = 1; // This message arrives when all files/drawers have been copied.
  CPACTION_End   = 2;

type
// The messages your delete hook is invoked with.
  PDeleteBeginMsg = ^TDeleteBeginMsg;
  TDeleteBeginMsg = record
    dbm_Length: LongWord; // Size of this data structure in bytes.
    dbm_Action: LongInt;  // Will be set to either DLACTION_BeginDiscard or DLACTION_BeginEmptyTrash (see below).
  end;

  PDeleteDataMsg = ^TDeleteDataMsg;
  TDeleteDataMsg = record
    ddm_Length: LongWord; // Size of this data structure in bytes.
    ddm_Action: LongInt;  // Will be set to either DLACTION_DeleteContents or DLACTION_DeleteObject (see below).
    ddm_Lock: BPTR;       // A Lock on the parent directory of the object whose contents or which itself should be deleted.
    ddm_Name: STRPTR;     // The name of the object whose contents or which itself should be deleted.
  end;

  PDeleteEndMsg = ^TDeleteEndMsg;
  TDeleteEndMsg = record
    dem_Length: LongWord; // Size of this data structure in bytes
    dem_Action: LongInt;   // Will be set to DLACTION_End (see below)
  end;

const
  DLACTION_BeginDiscard    = 0; // This indicates that the following delete operations are intended to empty the trashcan.
  DLACTION_BeginEmptyTrash = 1; // This indicates that the object described by lock and name refers to a drawer; you should empty its contents but  DO NOT  delete the drawer itself!
  DLACTION_DeleteContents  = 3; // This indicates that the object described by lock and name should be deleted; this could be a file or an empty drawer.
  DLACTION_DeleteObject    = 4; // This indicates that the deletion process is finished.
  DLACTION_End             = 5;

type
// The messages your text input hook is invoked with.
  PTextInputMsg = ^TTextInputMsg;
  TTextInputMsg = record
    tim_Length: LongWord; // Size of this data structure in bytes.
    tim_Action: LongInt;  // One of the TIACTION_* values listed below.
    tim_Prompt: STRPTR;   // The Workbench suggested result, depending on what kind of input is requested (as indicated by the tim_Action member).
  end;

const
  TIACTION_Rename        = 0; // A file or drawer is to be renamed.
  TIACTION_RelabelVolume = 1; // A volume is to be relabeled.
  TIACTION_NewDrawer     = 2; // A new drawer is to be created.
  TIACTION_Execute       = 3; // A program or script is to be executed.

// Parameters for the UpdateWorkbench() function.
  UPDATEWB_ObjectRemoved = 0; // Object has been deleted.
  UPDATEWB_ObjectAdded   = 1; // Object is new or has changed.

  WORKBENCHNAME : PChar  = 'workbench.library';

var
  WorkbenchBase: PLibrary = nil;

function AddAppIconA(Id: LongWord location 'd0'; UserData: LongWord location 'd1'; Text_: PChar location 'a0'; MsgPort: PMsgPort location 'a1'; Lock: BPTR location 'a2'; DiskObj: PDiskObject location 'a3'; const TagList: PTagItem location 'a4'): PAppIcon; syscall WorkbenchBase 060;
function AddAppMenuItemA(Id: LongWord location 'd0'; UserData: LongWord location 'd1'; Text_: PChar location 'a0'; MsgPort: PMsgPort location 'a1'; const TagList: PTagItem location 'a2'): PAppMenuItem; syscall WorkbenchBase 072;
function AddAppWindowA(Id: LongWord location 'd0'; UserData: LongWord location 'd1'; Window: PWindow location 'a0'; MsgPort: PMsgPort location 'a1'; const TagList: PTagItem location 'a2'): PAppWindow; syscall WorkbenchBase 048;
function RemoveAppIcon(AppIcon: PAppIcon location 'a0'): LongBool; syscall WorkbenchBase 066;
function RemoveAppMenuItem(AppMenuItem: PAppMenuItem location 'a0'): LongBool; syscall WorkbenchBase 078;
function RemoveAppWindow(AppWindow: PAppWindow location 'a0'): LongBool; syscall WorkbenchBase 054;
procedure WBInfo(Lock: BPTR location 'a0'; Name: STRPTR location 'a1'; Screen: PScreen location 'a2'); syscall WorkbenchBase 090;

function AddAppWindowDropZoneA(Aw: PAppWindow location 'a0'; Id: LongWord location 'd0'; UserData: LongWord location 'd1'; const Tags: PTagItem location 'a1'): PAppWindowDropZone; syscall WorkbenchBase 114;
function ChangeWorkbenchSelectionA(Name: STRPTR location 'a0'; Hook: PHook location 'a1'; const Tags: PTagItem location 'a2'): LongBool; syscall WorkbenchBase 126;
function CloseWorkbenchObjectA(Name: STRPTR location 'a0'; const Tags: PTagItem location 'a1'): LongBool; syscall WorkbenchBase 102;
function MakeWorkbenchObjectVisibleA(Name: STRPTR location 'a0'; const Tags: PTagItem location 'a1'): LongBool; syscall WorkbenchBase 132;
function OpenWorkbenchObjectA(Name: STRPTR location 'a0'; const Tags: PTagItem location 'a1'): LongBool; syscall WorkbenchBase 096;
function RemoveAppWindowDropZone(Aw: PAppWindow location 'a0'; DropZone: PAppWindowDropZone location 'a1'): LongBool; syscall WorkbenchBase 120;
function WorkbenchControlA(Name: STRPTR location 'a0'; const Tags: PTagItem location 'a1'): LongBool; syscall WorkbenchBase 108;
// V51
function ManageDesktopObjectA(Name: PChar; Action: LongInt; Tags: PTagItem): Boolean; syscall BaseSysV WorkbenchBase 148;
function CreateDrawerA(Drawer: PChar; Tags: PTagItem): Boolean; syscall BaseSysV WorkbenchBase 154;
function CreateIconA(Name: PChar; Tags: PTagItem): Boolean; syscall BaseSysV WorkbenchBase 160;

function AddAppIcon(Id: LongWord; UserData: LongWord; Text_: PChar; MsgPort: PMsgPort; Lock: BPTR; DiskObj: PDiskObject; const TagList: array of PtrUInt): PAppIcon; inline;
function AddAppMenuItem(Id: LongWord; UserData: LongWord; Text_: PChar; MsgPort: PMsgPort; const Tags: array of PtrUInt): PAppMenuItem; inline;
function AddAppWindow(Id: LongWord; UserData: LongWord; Window: PWindow; MsgPort: PMsgPort; const Tags: array of PtrUInt): PAppWindow; inline;

function OpenWorkbenchObject(Name: PChar; const Tags: array of PtrUInt): LongBool; inline;
function CloseWorkbenchObject(Name: PChar; const Tags: array of PtrUInt): LongBool; inline;
function WorkbenchControl(Name: PChar; const Tags: array of PtrUInt): LongBool; inline;
function AddAppWindowDropZone(Aw: PAppWindow; Id: LongWord; UserData: LongWord; const Tags: array of PtrUInt): PAppWindowDropZone; inline;
function ChangeWorkbenchSelection(Name: STRPTR; Hook: PHook; const Tags: array of PtrUInt): LongBool; inline;
function MakeWorkbenchObjectVisible(Name: STRPTR; const Tags: array of PtrUInt): LongBool; inline;
// V51
function ManageDesktopObjectTags(Name: PChar; Action: LongInt; const Tags: array of PtrUInt): Boolean; inline;
function CreateDrawerTags(Drawer: PChar; const Tags: array of PtrUInt): Boolean; inline;
function CreateIconTags(Name: PChar; const Tags: array of PtrUInt): Boolean; inline;

implementation

function AddAppIcon(Id: LongWord; UserData: LongWord; Text_: PChar; MsgPort: PMsgPort; Lock: BPTR; DiskObj: PDiskObject; const TagList: array of PtrUInt): PAppIcon;
begin
  AddAppIcon := AddAppIconA(Id, UserData, Text_, MsgPort, Lock, Diskobj, @TagList);
end;

function AddAppMenuItem(Id: LongWord; UserData: LongWord; Text_: PChar; MsgPort: PMsgPort; const Tags: array of PtrUInt): PAppMenuItem;
begin
  AddAppMenuItem := AddAppMenuItemA(Id, UserData, Text_, MsgPort, @Tags);
end;

function AddAppWindow(Id: LongWord; UserData: LongWord; Window: PWindow; MsgPort: PMsgPort; const Tags: array of PtrUInt): PAppWindow;
begin
  AddAppWindow := AddAppWindowA(Id, UserData, Window, MsgPort, @Tags);
end;

function OpenWorkbenchObject(Name: PChar; const Tags: array of PtrUInt): LongBool;
begin
  OpenWorkbenchObject := OpenWorkbenchObjectA(Name, @Tags);
end;

function CloseWorkbenchObject(Name: PChar; const Tags: array of PtrUInt): LongBool;
begin
  CloseWorkbenchObject := CloseWorkbenchObjectA(Name, @Tags);
end;

function WorkbenchControl(Name: PChar; const Tags: array of PtrUInt): LongBool;
begin
  WorkbenchControl := WorkbenchControlA(Name, @Tags);
end;

function AddAppWindowDropZone(Aw: PAppWindow; Id: LongWord; UserData: LongWord; const Tags: array of PtrUInt): PAppWindowDropZone;
begin
  AddAppWindowDropZone := AddAppWindowDropZoneA(Aw, Id, UserData, @Tags);
end;

function ChangeWorkbenchSelection(Name: STRPTR; Hook: PHook; const Tags: array of PtrUInt): LongBool;
begin
  ChangeWorkbenchSelection := ChangeWorkbenchSelectionA(Name, Hook, @Tags);
end;

function MakeWorkbenchObjectVisible(Name: STRPTR; const Tags: array of PtrUInt): LongBool;
begin
  MakeWorkbenchObjectVisible := MakeWorkbenchObjectVisibleA(Name, @Tags);
end;

function ManageDesktopObjectTags(Name: PChar; Action: LongInt; const Tags: array of PtrUInt): Boolean;
begin
  ManageDesktopObjectTags := ManageDesktopObjectA(Name, Action, @Tags);
end;

function CreateDrawerTags(Drawer: PChar; const Tags: array of PtrUInt): Boolean;
begin
  CreateDrawerTags := CreateDrawerA(Drawer, @Tags);
end;

function CreateIconTags(Name: PChar; const Tags: array of PtrUInt): Boolean;
begin
  CreateIconTags := CreateIconA(Name, @Tags);
end;

const
  LIBVERSION: LongWord = 0;

initialization
  WorkbenchBase := OpenLibrary(WORKBENCHNAME, LIBVERSION);
finalization
  if Assigned(WorkbenchBase) then
  CloseLibrary(WorkbenchBase);
end.
