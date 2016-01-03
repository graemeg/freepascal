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
{
    History:
    Added overlay functions for Pchar->Strings, functions
    and procedures.
    14 Jul 2000.

    Changed tWBArg.wa_Lock from a pointer to a longint.
    15 Aug 2000.

    Fixed tDiskObject.
    Member tDiskObject.do_CurrentX was defined as a pointer,
    should be longint.
    17 Aug 2000.

    Added functions and procedures with array of const.
    For use with fpc 1.0.7. Thay are in systemvartags.
    05 Nov 2002.

    Removed amigaoverlays, use smartlink instead.
    05 Nov 2002.

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening of
    the library.
    14 Jan 2003.

    Update for AmigaOS 3.9.
    A lot of new const and new records.
    New functions
            FUNCTION AddAppWindowDropZoneA
            FUNCTION ChangeWorkbenchSelectionA
            FUNCTION CloseWorkbenchObjectA
            FUNCTION MakeWorkbenchObjectVisibleA
            FUNCTION OpenWorkbenchObjectA
            FUNCTION RemoveAppWindowDropZone
            FUNCTION WorkbenchControlA
    Varargs functions are in systemvartags.
    02 Feb 2003.

    Changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

unit workbench;

INTERFACE

uses exec,amigados,utility, intuition,graphics;



Type

    pWBArg = ^tWBArg;
    tWBArg = record
        wa_Lock         : longint;      { a lock descriptor }
        wa_Name         : STRPTR;       { a string relative to that lock }
    end;

    WBArgList = Array [1..100] of tWBArg; { Only 1..smNumArgs are valid }
    pWBArgList = ^WBArgList;


    pWBStartup = ^tWBStartup;
    tWBStartup = record
        sm_Message      : tMessage;      { a standard message structure }
        sm_Process      : pMsgPort;   { the process descriptor for you }
        sm_Segment      : BPTR;         { a descriptor for your code }
        sm_NumArgs      : Longint;      { the number of elements in ArgList }
        sm_ToolWindow   : STRPTR;       { description of window }
        sm_ArgList      : pWBArgList; { the arguments themselves }
    end;


Const

    WBDISK              = 1;
    WBDRAWER            = 2;
    WBTOOL              = 3;
    WBPROJECT           = 4;
    WBGARBAGE           = 5;
    WBDEVICE            = 6;
    WBKICK              = 7;
    WBAPPICON           = 8;

Type

    pOldDrawerData = ^tOldDrawerData;
    tOldDrawerData = record
        dd_NewWindow    : tNewWindow;    { args to open window }
        dd_CurrentX     : Longint;      { current x coordinate of origin }
        dd_CurrentY     : Longint;      { current y coordinate of origin }
    end;

Const

{ the amount of DrawerData actually written to disk }

    OLDDRAWERDATAFILESIZE  = 56;  { sizeof(OldDrawerData) }

Type
    pDrawerData = ^tDrawerData;
    tDrawerData = record
        dd_NewWindow    : tNewWindow;    { args to open window }
        dd_CurrentX     : Longint;      { current x coordinate of origin }
        dd_CurrentY     : Longint;      { current y coordinate of origin }
        dd_Flags        : Longint;      { flags for drawer }
        dd_ViewModes    : Word;        { view mode for drawer }
    end;

Const

{ the amount of DrawerData actually written to disk }

    DRAWERDATAFILESIZE  = 62;  { sizeof(DrawerData) }

       DDVM_BYDEFAULT = 0;
    { view as icons  }
       DDVM_BYICON = 1;
    { view as text, sorted by name  }
       DDVM_BYNAME = 2;
    { view as text, sorted by date  }
       DDVM_BYDATE = 3;
    { view as text, sorted by size  }
       DDVM_BYSIZE = 4;
    { view as text, sorted by type  }
       DDVM_BYTYPE = 5;

    { definitions for dd_Flags  }
    { default (show only icons)  }
       DDFLAGS_SHOWDEFAULT = 0;
    { show only icons  }
       DDFLAGS_SHOWICONS = 1;
    { show all files  }
       DDFLAGS_SHOWALL = 2;

Type

    pDiskObject = ^tDiskObject;
    tDiskObject = record
        do_Magic        : Word;        { a magic number at the start of the file }
        do_Version      : Word;        { a version number, so we can change it }
        do_Gadget       : tGadget;       { a copy of in core gadget }
        do_Type         : Byte;
        do_DefaultTool  : STRPTR;
        do_ToolTypes    : Pointer;
        do_CurrentX     : Longint;
        do_CurrentY     : Longint;
        do_DrawerData   : pDrawerData;
        do_ToolWindow   : STRPTR;       { only applies to tools }
        do_StackSize    : Longint;      { only applies to tools }
    end;

Const

    WB_DISKMAGIC        = $e310;        { a magic number, not easily impersonated }
    WB_DISKVERSION      = 1;            { our current version number }
    WB_DISKREVISION     = 1;            { our current revision number }
  {I only use the lower 8 bits of Gadget.UserData for the revision # }
    WB_DISKREVISIONMASK = 255;
Type
    pFreeList = ^tFreeList;
    tFreeList = record
        fl_NumFree      : smallint;
        fl_MemList      : tList;
    end;

Const

{ each message that comes into the WorkBenchPort must have a type field
 * in the preceeding short.  These are the defines for this type
 }

    MTYPE_PSTD          = 1;    { a "standard Potion" message }
    MTYPE_TOOLEXIT      = 2;    { exit message from our tools }
    MTYPE_DISKCHANGE    = 3;    { dos telling us of a disk change }
    MTYPE_TIMER         = 4;    { we got a timer tick }
    MTYPE_CLOSEDOWN     = 5;    { <unimplemented> }
    MTYPE_IOPROC        = 6;    { <unimplemented> }
    MTYPE_APPWINDOW     = 7;    {     msg from an app window }
    MTYPE_APPICON       = 8;    {     msg from an app icon }
    MTYPE_APPMENUITEM   = 9;    {     msg from an app menuitem }
    MTYPE_COPYEXIT      = 10;   {     exit msg from copy process }
    MTYPE_ICONPUT       = 11;   {     msg from PutDiskObject in icon.library }


{ workbench does different complement modes for its gadgets.
 * It supports separate images, complement mode, and backfill mode.
 * The first two are identical to intuitions GADGIMAGE and GADGHCOMP.
 * backfill is similar to GADGHCOMP, but the region outside of the
 * image (which normally would be color three when complemented)
 * is flood-filled to color zero.
 }

    GFLG_GADGBACKFILL   = $0001;
    GADGBACKFILL        = $0001;   { an old synonym }

{ if an icon does not really live anywhere, set its current position
 * to here
 }

    NO_ICON_POSITION    = $80000000;

{    If you find am_Version >= AM_VERSION, you know this structure has
 * at least the fields defined in this version of the include file
 }
 AM_VERSION   =   1;

Type
   pAppMessage = ^tAppMessage;
   tAppMessage = record
    am_Message       : tMessage;            {    standard message structure }
    am_Type          : Word;              {    message type }
    am_UserData      : ULONG;            {    application specific }
    am_ID            : ULONG;            {    application definable ID }
    am_NumArgs       : ULONG;            {    # of elements in arglist }
    am_ArgList       : pWBArgList;       {    the arguements themselves }
    am_Version       : Word;              {    will be AM_VERSION }
    am_Class         : Word;              {    message class }
    am_MouseX        : smallint;              {    mouse x position of event }
    am_MouseY        : smallint;              {    mouse y position of event }
    am_Seconds       : ULONG;            {    current system clock time }
    am_Micros        : ULONG;            {    current system clock time }
    am_Reserved      : Array[0..7] of ULONG;       {    avoid recompilation }
   END;

{* types of app messages *}
const
    AMTYPE_APPWINDOW   = 7;    {* app window message    *}
    AMTYPE_APPICON     = 8;    {* app icon message  *}
    AMTYPE_APPMENUITEM = 9;    {* app menu item message *}

  { Classes of AppIcon messages (V44)  }
    { The "Open" menu item was invoked,
                                           the icon got double-clicked or an
                                           icon got dropped on it.
                                          }
       AMCLASSICON_Open = 0;
    { The "Copy" menu item was invoked  }
       AMCLASSICON_Copy = 1;
    { The "Rename" menu item was invoked  }
       AMCLASSICON_Rename = 2;
    { The "Information" menu item was invoked  }
       AMCLASSICON_Information = 3;
    { The "Snapshot" menu item was invoked  }
       AMCLASSICON_Snapshot = 4;
    { The "UnSnapshot" menu item was invoked  }
       AMCLASSICON_UnSnapshot = 5;
    { The "Leave Out" menu item was invoked  }
       AMCLASSICON_LeaveOut = 6;
    { The "Put Away" menu item was invoked  }
       AMCLASSICON_PutAway = 7;
    { The "Delete" menu item was invoked  }
       AMCLASSICON_Delete = 8;
    { The "Format Disk" menu item was invoked  }
       AMCLASSICON_FormatDisk = 9;
    { The "Empty Trash" menu item was invoked  }
       AMCLASSICON_EmptyTrash = 10;
    { The icon is now selected  }
       AMCLASSICON_Selected = 11;
    { The icon is now unselected  }
       AMCLASSICON_Unselected = 12;

{
 * The following structures are private.  These are just stub
 * structures for code compatibility...
 }
type

 tAppWindow = record
   aw_PRIVATE : Pointer;
 END;
 pAppWindow = ^tAppWindow;

 tAppIcon = record
   ai_PRIVATE : Pointer;
 END;
 pAppIcon = ^tAppIcon;

 tAppMenuItem = record
   ami_PRIVATE : Pointer;
 END;
 pAppMenuItem = ^tAppMenuItem;

 PAppWindowDropZone = ^tAppWindowDropZone;
       tAppWindowDropZone = record
            awdz_PRIVATE : pointer;
         end;

 PAppMenu = ^tAppMenu;
       tAppMenu = record
            am_PRIVATE : pointer;
         end;

   const
       WBA_Dummy = TAG_USER + $A000;
    {                                                                           }
    { Tags for use with AddAppIconA()  }
    { AppIcon responds to the "Open" menu item (BOOL).  }
       WBAPPICONA_SupportsOpen = WBA_Dummy + 1;

    { AppIcon responds to the "Copy" menu item (BOOL).  }
       WBAPPICONA_SupportsCopy = WBA_Dummy + 2;

    { AppIcon responds to the "Rename" menu item (BOOL).  }
       WBAPPICONA_SupportsRename = WBA_Dummy + 3;

    { AppIcon responds to the "Information" menu item (BOOL).  }
       WBAPPICONA_SupportsInformation = WBA_Dummy + 4;

    { AppIcon responds to the "Snapshot" menu item (BOOL).  }
       WBAPPICONA_SupportsSnapshot = WBA_Dummy + 5;

    { AppIcon responds to the "UnSnapshot" menu item (BOOL).  }
       WBAPPICONA_SupportsUnSnapshot = WBA_Dummy + 6;

    { AppIcon responds to the "LeaveOut" menu item (BOOL).  }
       WBAPPICONA_SupportsLeaveOut = WBA_Dummy + 7;

    { AppIcon responds to the "PutAway" menu item (BOOL).  }
       WBAPPICONA_SupportsPutAway = WBA_Dummy + 8;

    { AppIcon responds to the "Delete" menu item (BOOL).  }
       WBAPPICONA_SupportsDelete = WBA_Dummy + 9;

    { AppIcon responds to the "FormatDisk" menu item (BOOL).  }
       WBAPPICONA_SupportsFormatDisk = WBA_Dummy + 10;

    { AppIcon responds to the "EmptyTrash" menu item (BOOL).  }
       WBAPPICONA_SupportsEmptyTrash = WBA_Dummy + 11;

    { AppIcon position should be propagated back to original DiskObject (BOOL).  }
       WBAPPICONA_PropagatePosition = WBA_Dummy + 12;

    { Callback hook to be invoked when rendering this icon (struct Hook  ).  }
       WBAPPICONA_RenderHook = WBA_Dummy + 13;

    { AppIcon wants to be notified when its select state changes (BOOL).  }
       WBAPPICONA_NotifySelectState = WBA_Dummy + 14;

   {**************************************************************************}

    { Tags for use with AddAppMenuA()  }
    { Command key string for this AppMenu (STRPTR).  }
       WBAPPMENUA_CommandKeyString = WBA_Dummy + 15;
    { Item to be added should get sub menu items attached to; make room for it,
       then return the key to use later for attaching the items (ULONG  ).
      }
       WBAPPMENUA_GetKey = WBA_Dummy + 65;
    { This item should be attached to a sub menu; the key provided refers to
       the sub menu it should be attached to (ULONG).
      }
       WBAPPMENUA_UseKey = WBA_Dummy + 66;
    { Item to be added is in fact a new menu title; make room for it, then
       return the key to use later for attaching the items (ULONG  ).
      }
       WBAPPMENUA_GetTitleKey = WBA_Dummy + 77;

    {**************************************************************************}

    { Tags for use with OpenWorkbenchObjectA()  }
    { Corresponds to the wa_Lock member of a struct WBArg  }
       WBOPENA_ArgLock = WBA_Dummy + 16;

    { Corresponds to the wa_Name member of a struct WBArg  }
       WBOPENA_ArgName = WBA_Dummy + 17;

    { When opening a drawer, show all files or only icons?
       This must be one out of DDFLAGS_SHOWICONS,
       or DDFLAGS_SHOWALL; (UBYTE); (V45)
      }
       WBOPENA_Show = WBA_Dummy + 75;

    { When opening a drawer, view the contents by icon, name,
       date, size or type? This must be one out of DDVM_BYICON,
       DDVM_BYNAME, DDVM_BYDATE, DDVM_BYSIZE or DDVM_BYTYPE;
       (UBYTE); (V45)
      }
       WBOPENA_ViewBy = WBA_Dummy + 76;

   {**************************************************************************}

    { Tags for use with WorkbenchControlA()  }
    { Check if the named drawer is currently open (LONG  ).  }
       WBCTRLA_IsOpen = WBA_Dummy + 18;

    { Create a duplicate of the Workbench private search path list (BPTR  ).  }
       WBCTRLA_DuplicateSearchPath = WBA_Dummy + 19;

    { Free the duplicated search path list (BPTR).  }
       WBCTRLA_FreeSearchPath = WBA_Dummy + 20;

    { Get the default stack size for launching programs with (ULONG  ).  }
       WBCTRLA_GetDefaultStackSize = WBA_Dummy + 21;

    { Set the default stack size for launching programs with (ULONG).  }
       WBCTRLA_SetDefaultStackSize = WBA_Dummy + 22;

    { Cause an AppIcon to be redrawn (struct AppIcon  ).  }
       WBCTRLA_RedrawAppIcon = WBA_Dummy + 23;

    { Get a list of currently running Workbench programs (struct List   ).  }
       WBCTRLA_GetProgramList = WBA_Dummy + 24;

    { Release the list of currently running Workbench programs (struct List  ).  }
       WBCTRLA_FreeProgramList = WBA_Dummy + 25;

    { Get a list of currently selected icons (struct List   ).  }
       WBCTRLA_GetSelectedIconList = WBA_Dummy + 36;

    { Release the list of currently selected icons (struct List  ).  }
       WBCTRLA_FreeSelectedIconList = WBA_Dummy + 37;

    { Get a list of currently open drawers (struct List   ).  }
       WBCTRLA_GetOpenDrawerList = WBA_Dummy + 38;

    { Release the list of currently open icons (struct List  ).  }
       WBCTRLA_FreeOpenDrawerList = WBA_Dummy + 39;

    { Get the list of hidden devices (struct List   ).  }
       WBCTRLA_GetHiddenDeviceList = WBA_Dummy + 42;

    { Release the list of hidden devices (struct List  ).  }
       WBCTRLA_FreeHiddenDeviceList = WBA_Dummy + 43;

    { Add the name of a device which Workbench should never try to
       read a disk icon from (STRPTR).
      }
       WBCTRLA_AddHiddenDeviceName = WBA_Dummy + 44;

    { Remove a name from list of hidden devices (STRPTR).  }
       WBCTRLA_RemoveHiddenDeviceName = WBA_Dummy + 45;

    { Get the number of seconds that have to pass before typing
       the next character in a drawer window will restart
       with a new file name (ULONG  ).
      }
       WBCTRLA_GetTypeRestartTime = WBA_Dummy + 47;

    { Set the number of seconds that have to pass before typing
       the next character in a drawer window will restart
       with a new file name (ULONG).
      }
       WBCTRLA_SetTypeRestartTime = WBA_Dummy + 48;

    { Obtain the hook that will be invoked when Workbench starts
       to copy files and data (struct Hook   ); (V45)
      }
       WBCTRLA_GetCopyHook = WBA_Dummy + 69;

    { Install the hook that will be invoked when Workbench starts
       to copy files and data (struct Hook  ); (V45)
      }
       WBCTRLA_SetCopyHook = WBA_Dummy + 70;

    { Obtain the hook that will be invoked when Workbench discards
       files and drawers or empties the trashcan (struct Hook   );
       (V45).
      }
       WBCTRLA_GetDeleteHook = WBA_Dummy + 71;

    { Install the hook that will be invoked when Workbench discards
       files and drawers or empties the trashcan (struct Hook  );
       (V45).
      }
       WBCTRLA_SetDeleteHook = WBA_Dummy + 72;

    { Obtain the hook that will be invoked when Workbench requests
       that the user enters text, such as when a file is to be renamed
       or a new drawer is to be created (struct Hook   ); (V45)
      }
       WBCTRLA_GetTextInputHook = WBA_Dummy + 73;

    { Install the hook that will be invoked when Workbench requests
       that the user enters text, such as when a file is to be renamed
       or a new drawer is to be created (struct Hook  ); (V45)
      }
       WBCTRLA_SetTextInputHook = WBA_Dummy + 74;

    { Add a hook that will be invoked when Workbench is about
       to shut down (cleanup), and when Workbench has returned
       to operational state (setup) (struct Hook  ); (V45)
      }
       WBCTRLA_AddSetupCleanupHook = WBA_Dummy + 78;

    { Remove a hook that has been installed with the
       WBCTRLA_AddSetupCleanupHook tag (struct Hook  ); (V45)
      }
       WBCTRLA_RemSetupCleanupHook = WBA_Dummy + 79;

       {**************************************************************************}

{ The message your setup/cleanup hook gets invoked with. }
       type
       PSetupCleanupHookMsg = ^tSetupCleanupHookMsg;
       tSetupCleanupHookMsg = record
            schm_Length : ULONG;
            schm_State : LONG;
         end;

      const
    { Workbench will attempt to shut down now.  }
       SCHMSTATE_TryCleanup = 0;
    { Workbench will really shut down now.  }
       SCHMSTATE_Cleanup = 1;
    { Workbench is operational again or
       could not be shut down.
    }
       SCHMSTATE_Setup = 2;

{**************************************************************************}

    { Tags for use with AddAppWindowDropZoneA()  }
    { Zone left edge (WORD)  }
       WBDZA_Left = WBA_Dummy + 26;

    { Zone left edge, if relative to the right edge of the window (WORD)  }
       WBDZA_RelRight = WBA_Dummy + 27;

    { Zone top edge (WORD)  }
       WBDZA_Top = WBA_Dummy + 28;

    { Zone top edge, if relative to the bottom edge of the window (WORD)  }
       WBDZA_RelBottom = WBA_Dummy + 29;

    { Zone width (WORD)  }
       WBDZA_Width = WBA_Dummy + 30;

    { Zone width, if relative to the window width (WORD)  }
       WBDZA_RelWidth = WBA_Dummy + 31;

    { Zone height (WORD)  }
       WBDZA_Height = WBA_Dummy + 32;

    { Zone height, if relative to the window height (WORD)  }
       WBDZA_RelHeight = WBA_Dummy + 33;

    { Zone position and size (struct IBox  ).  }
       WBDZA_Box = WBA_Dummy + 34;

    { Hook to invoke when the mouse enters or leave a drop zone (struct Hook  ).  }
       WBDZA_Hook = WBA_Dummy + 35;

{**************************************************************************}

    { Reserved tags; don't use!  }
       WBA_Reserved1 = WBA_Dummy + 40;
       WBA_Reserved2 = WBA_Dummy + 41;
       WBA_Reserved3 = WBA_Dummy + 46;
       WBA_Reserved4 = WBA_Dummy + 49;
       WBA_Reserved5 = WBA_Dummy + 50;
       WBA_Reserved6 = WBA_Dummy + 51;
       WBA_Reserved7 = WBA_Dummy + 52;
       WBA_Reserved8 = WBA_Dummy + 53;
       WBA_Reserved9 = WBA_Dummy + 54;
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
       WBA_Reserved20 = WBA_Dummy + 67;
       WBA_Reserved21 = WBA_Dummy + 68;

{**************************************************************************}

       WBA_LAST_TAG = WBA_Dummy + 79;

{**************************************************************************}

{ The message your AppIcon rendering hook gets invoked with. }
   type
       PAppIconRenderMsg = ^tAppIconRenderMsg;
       tAppIconRenderMsg = record
            arm_RastPort : PRastPort;  { RastPort to render into }
            arm_Icon : PDiskObject;    { The icon to be rendered }
            arm_Label : STRPTR;        { The icon label txt }
            arm_Tags : PTagItem;       { Further tags to be passed on
                                                 * to DrawIconStateA().
                                                 }
            arm_Left : WORD;           { \ Rendering origin, not taking the }
            arm_Top : WORD;            { / button border into account. }
            arm_Width : WORD;          { \ Limit your rendering to }
            arm_Height : WORD;         { / this area. }
            arm_State : ULONG;         { IDS_SELECTED, IDS_NORMAL, etc. }
         end;

{ The message your drop zone hook gets invoked with. }
       PAppWindowDropZoneMsg = ^tAppWindowDropZoneMsg;
       tAppWindowDropZoneMsg = record
            adzm_RastPort : PRastPort;  { RastPort to render into. }
            adzm_DropZoneBox : tIBox;   { Limit your rendering to this area. }
            adzm_ID : ULONG;            { \ These come from straight }
            adzm_UserData : ULONG;      { / from AddAppWindowDropZoneA(). }
            adzm_Action : LONG;         { See below for a list of actions. }
         end;

   const
       ADZMACTION_Enter = 0;
       ADZMACTION_Leave = 1;

{**************************************************************************}

{ The message your icon selection change hook is invoked with. }
    type
       PIconSelectMsg = ^tIconSelectMsg;
       tIconSelectMsg = record
            { Size of this data structure (in bytes). }
            ism_Length : ULONG;
            { Lock on the drawer this object resides in,
             * NULL for Workbench backdrop (devices).
             }
            ism_Drawer : BPTR;
            { Name of the object in question. }
            ism_Name : STRPTR;
            { One of WBDISK, WBDRAWER, WBTOOL, WBPROJECT,
             * WBGARBAGE, WBDEVICE, WBKICK or WBAPPICON.
             }
            ism_Type : UWORD;
            { TRUE if currently selected, FALSE otherwise. }
            ism_Selected : BOOL;
            { Pointer to the list of tag items passed to
             * ChangeWorkbenchSelectionA().
             }
            ism_Tags : PTagItem;
            { Pointer to the window attached to this icon,
             * if the icon is a drawer-like object.
             }
            ism_DrawerWindow : PWindow;
            { Pointer to the window the icon resides in. }
            ism_ParentWindow : PWindow;
            { Position and size of the icon; note that the
             * icon may not entirely reside within the visible
             * bounds of the parent window.
             }
            ism_Left : WORD;
            ism_Top : WORD;
            ism_Width : WORD;
            ism_Height : WORD;
         end;

    { These are the values your hook code can return.  }
    const
    { Unselect the icon  }
       ISMACTION_Unselect = 0;
    { Select the icon  }
       ISMACTION_Select = 1;
    { Do not change the selection state.  }
       ISMACTION_Ignore = 2;
    { Do not invoke the hook code again,
      leave the icon as it is.
    }
       ISMACTION_Stop = 3;

{**************************************************************************}

{ The messages your copy hook is invoked with. }

   type
       PCopyBeginMsg = ^tCopyBeginMsg;
       tCopyBeginMsg = record
            cbm_Length : ULONG;           { Size of this data structure in bytes. }
            cbm_Action : LONG;            { Will be set to CPACTION_Begin (see below). }
            cbm_SourceDrawer : BPTR;      { A lock on the source drawer. }
            cbm_DestinationDrawer : BPTR; { A lock on the destination drawer. }
         end;

       PCopyDataMsg = ^tCopyDataMsg;
       tCopyDataMsg = record
            cdm_Length : ULONG;           { Size of this data structure in bytes. }
            cdm_Action : LONG;            { Will be set to CPACTION_Copy (see below). }
            cdm_SourceLock : BPTR;        { A lock on the parent directory of the
                                           * source file/drawer.
                                           }
            cdm_SourceName : STRPTR;      { The name of the source file or drawer. }
            cdm_DestinationLock : BPTR;   { A lock on the parent directory of the
                                           * destination file/drawer.
                                           }
            cdm_DestinationName : STRPTR; { The name of the destination file/drawer.
                                           * This may or may not match the name of
                                           * the source file/drawer in case the
                                           * data is to be copied under a different
                                           * name. For example, this is the case
                                           * with the Workbench "Copy" command which
                                           * creates duplicates of file/drawers by
                                           * prefixing the duplicate's name with
                                           * "Copy_XXX_of".
                                           }
            cdm_DestinationX : LONG;      { When the icon corresponding to the
                                           * destination is written to disk, this
                                           * is the position (put into its
                                           * DiskObject->do_CurrentX/DiskObject->do_CurrentY
                                           * fields) it should be placed at.
                                           }
            cdm_DestinationY : LONG;
         end;

     PCopyEndMsg = ^tCopyEndMsg;
       tCopyEndMsg = record
            cem_Length : ULONG;           { Size of this data structure in bytes. }
            cem_Action : LONG;            { Will be set to CPACTION_End (see below). }
         end;

    const
       CPACTION_Begin = 0;
    { This message arrives for each file or
      drawer to be copied.
    }
       CPACTION_Copy = 1;
    { This message arrives when all files/drawers
      have been copied.
    }
       CPACTION_End = 2;

{**************************************************************************}

{ The messages your delete hook is invoked with. }
    type
       PDeleteBeginMsg = ^tDeleteBeginMsg;
       tDeleteBeginMsg = record
            dbm_Length : ULONG;   { Size of this data structure in bytes. }
            dbm_Action : LONG;    { Will be set to either DLACTION_BeginDiscard
                                   * or DLACTION_BeginEmptyTrash (see below).
                                   }
         end;


       PDeleteDataMsg = ^tDeleteDataMsg;
       tDeleteDataMsg = record
            ddm_Length : ULONG;   { Size of this data structure in bytes. }
            ddm_Action : LONG;    { Will be set to either DLACTION_DeleteContents
                                   * or DLACTION_DeleteObject (see below).
                                   }
            ddm_Lock : BPTR;      { A Lock on the parent directory of the object
                                   * whose contents or which itself should be
                                   * deleted.
                                   }
            ddm_Name : STRPTR;    { The name of the object whose contents or
                                   * which itself should be deleted.
                                   }
         end;


       PDeleteEndMsg = ^tDeleteEndMsg;
       tDeleteEndMsg = record
            dem_Length : ULONG;   { Size of this data structure in bytes. }
            dem_Action : LONG;    { Will be set to DLACTION_End (see below). }
         end;

 const
       DLACTION_BeginDiscard = 0;
    { This indicates that the following
      delete operations are intended to
      empty the trashcan.
    }

       DLACTION_BeginEmptyTrash = 1;
    { This indicates that the object
      described by lock and name refers
      to a drawer; you should empty its
      contents but  DO NOT  delete the
      drawer itself!
    }

       DLACTION_DeleteContents = 3;
    { This indicates that the object
      described by lock and name should
      be deleted; this could be a file
      or an empty drawer.
    }

       DLACTION_DeleteObject = 4;
    { This indicates that the
      deletion process is finished.
    }

       DLACTION_End = 5;

{**************************************************************************}

{ The messages your text input hook is invoked with. }
    type
       PTextInputMsg = ^tTextInputMsg;
       tTextInputMsg = record
            tim_Length : ULONG;     { Size of this data structure
                                     * in bytes.
                                     }
            tim_Action : LONG;      { One of the TIACTION_...
                                     * values listed below.
                                     }
            tim_Prompt : STRPTR;    { The Workbench suggested
                                     * result, depending on what
                                     * kind of input is requested
                                     * (as indicated by the
                                     * tim_Action member).
                                     }
         end;

   const
   { A file or drawer is to be
    * renamed.
   }
       TIACTION_Rename = 0;

    { A volume is to be relabeled.  }
       TIACTION_RelabelVolume = 1;

    { A new drawer is to be created.  }
       TIACTION_NewDrawer = 2;

    { A program or script is to be
      executed.
    }
       TIACTION_Execute = 3;

{**************************************************************************}

{ Parameters for the UpdateWorkbench() function. }

    { Object has been deleted.  }
       UPDATEWB_ObjectRemoved = 0;

    { Object is new or has changed.  }
       UPDATEWB_ObjectAdded = 1;

    WORKBENCHNAME : PChar  = 'workbench.library';

VAR
    WorkbenchBase : pLibrary;

FUNCTION AddAppIconA(id : ULONG; userdata : ULONG; text_ : pCHAR; msgport : pMsgPort; lock : pFileLock; diskobj : pDiskObject;const taglist : pTagItem) : pAppIcon;
FUNCTION AddAppMenuItemA(id : ULONG; userdata : ULONG; text_ : pCHAR; msgport : pMsgPort;const taglist : pTagItem) : pAppMenuItem;
FUNCTION AddAppWindowA(id : ULONG; userdata : ULONG; window : pWindow; msgport : pMsgPort;const taglist : pTagItem) : pAppWindow;
FUNCTION RemoveAppIcon(appIcon : pAppIcon) : BOOLEAN;
FUNCTION RemoveAppMenuItem(appMenuItem : pAppMenuItem) : BOOLEAN;
FUNCTION RemoveAppWindow(appWindow : pAppWindow) : BOOLEAN;
PROCEDURE WBInfo(lock : BPTR; name : pCHAR; screen : pScreen);

FUNCTION AddAppWindowDropZoneA(aw : pAppWindow; id : longword; userdata : longword;const tags : pTagItem) : pAppWindowDropZone;
FUNCTION ChangeWorkbenchSelectionA(name : pCHAR; hook : pHook;const tags : pTagItem) : BOOLEAN;
FUNCTION CloseWorkbenchObjectA(name : pCHAR;const tags : pTagItem) : BOOLEAN;
FUNCTION MakeWorkbenchObjectVisibleA(name : pCHAR;const tags : pTagItem) : BOOLEAN;
FUNCTION OpenWorkbenchObjectA(name : pCHAR;const tags : pTagItem) : BOOLEAN;
FUNCTION RemoveAppWindowDropZone(aw : pAppWindow; dropZone : pAppWindowDropZone) : BOOLEAN;
FUNCTION WorkbenchControlA(name : pCHAR;const tags : pTagItem) : BOOLEAN;

{ overlays }
FUNCTION AddAppIconA(id : ULONG; userdata : ULONG; text_ : string; msgport : pMsgPort; lock : pFileLock; diskobj : pDiskObject;const taglist : pTagItem) : pAppIcon;
FUNCTION AddAppMenuItemA(id : ULONG; userdata : ULONG; text_ : string; msgport : pMsgPort;const taglist : pTagItem) : pAppMenuItem;
PROCEDURE WBInfo(lock : BPTR; name : string; screen : pScreen);

FUNCTION ChangeWorkbenchSelectionA(name : string; hook : pHook;const tags : pTagItem) : BOOLEAN;
FUNCTION CloseWorkbenchObjectA(name : string;const tags : pTagItem) : BOOLEAN;
FUNCTION MakeWorkbenchObjectVisibleA(name : string;const tags : pTagItem) : BOOLEAN;
FUNCTION OpenWorkbenchObjectA(name : string;const tags : pTagItem) : BOOLEAN;
FUNCTION WorkbenchControlA(name : string;const tags : pTagItem) : BOOLEAN;

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitWBLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    WBIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
pastoc;

FUNCTION AddAppIconA(id : ULONG; userdata : ULONG; text_ : pCHAR; msgport : pMsgPort; lock : pFileLock; diskobj : pDiskObject;const taglist : pTagItem) : pAppIcon;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  id,D0
    MOVE.L  userdata,D1
    MOVEA.L text_,A0
    MOVEA.L msgport,A1
    MOVEA.L lock,A2
    MOVEA.L diskobj,A3
    MOVEA.L taglist,A4
    MOVEA.L WorkbenchBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddAppMenuItemA(id : ULONG; userdata : ULONG; text_ : pCHAR; msgport : pMsgPort;const taglist : pTagItem) : pAppMenuItem;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  id,D0
    MOVE.L  userdata,D1
    MOVEA.L text_,A0
    MOVEA.L msgport,A1
    MOVEA.L taglist,A2
    MOVEA.L WorkbenchBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AddAppWindowA(id : ULONG; userdata : ULONG; window : pWindow; msgport : pMsgPort;const taglist : pTagItem) : pAppWindow;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  id,D0
    MOVE.L  userdata,D1
    MOVEA.L window,A0
    MOVEA.L msgport,A1
    MOVEA.L taglist,A2
    MOVEA.L WorkbenchBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION RemoveAppIcon(appIcon : pAppIcon) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L appIcon,A0
    MOVEA.L WorkbenchBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION RemoveAppMenuItem(appMenuItem : pAppMenuItem) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L appMenuItem,A0
    MOVEA.L WorkbenchBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION RemoveAppWindow(appWindow : pAppWindow) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L appWindow,A0
    MOVEA.L WorkbenchBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE WBInfo(lock : BPTR; name : pCHAR; screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L lock,A0
    MOVEA.L name,A1
    MOVEA.L screen,A2
    MOVEA.L WorkbenchBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AddAppWindowDropZoneA(aw : pAppWindow; id : longword; userdata : longword;const tags : pTagItem) : pAppWindowDropZone;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L aw,A0
        MOVE.L  id,D0
        MOVE.L  userdata,D1
        MOVEA.L tags,A1
        MOVEA.L WorkbenchBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION ChangeWorkbenchSelectionA(name : pCHAR; hook : pHook;const tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L hook,A1
        MOVEA.L tags,A2
        MOVEA.L WorkbenchBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION CloseWorkbenchObjectA(name : pCHAR;const tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L tags,A1
        MOVEA.L WorkbenchBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION MakeWorkbenchObjectVisibleA(name : pCHAR;const tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L tags,A1
        MOVEA.L WorkbenchBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION OpenWorkbenchObjectA(name : pCHAR;const tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L tags,A1
        MOVEA.L WorkbenchBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;


FUNCTION RemoveAppWindowDropZone(aw : pAppWindow; dropZone : pAppWindowDropZone) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L aw,A0
        MOVEA.L dropZone,A1
        MOVEA.L WorkbenchBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION WorkbenchControlA(name : pCHAR;const tags : pTagItem) : BOOLEAN;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L tags,A1
        MOVEA.L WorkbenchBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
        TST.W   D0
        BEQ.B   @end
        MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

FUNCTION AddAppIconA(id : ULONG; userdata : ULONG; text_ : string; msgport : pMsgPort; lock : pFileLock; diskobj : pDiskObject;const taglist : pTagItem) : pAppIcon;
begin
       AddAppIconA := AddAppIconA(id,userdata,pas2c(text_),msgport,lock,diskobj,taglist);
end;

FUNCTION AddAppMenuItemA(id : ULONG; userdata : ULONG; text_ : string; msgport : pMsgPort;const taglist : pTagItem) : pAppMenuItem;
begin
       AddAppMenuItemA := AddAppMenuItemA(id,userdata,pas2c(text_),msgport,taglist);
end;

PROCEDURE WBInfo(lock : BPTR; name : string; screen : pScreen);
begin
       WBInfo(lock,pas2c(name),screen);
end;

FUNCTION ChangeWorkbenchSelectionA(name : string; hook : pHook;const tags : pTagItem) : BOOLEAN;
begin
       ChangeWorkbenchSelectionA := ChangeWorkbenchSelectionA(pas2c(name),hook,tags);
end;

FUNCTION CloseWorkbenchObjectA(name : string;const tags : pTagItem) : BOOLEAN;
begin
       CloseWorkbenchObjectA := CloseWorkbenchObjectA(pas2c(name),tags);
end;

FUNCTION MakeWorkbenchObjectVisibleA(name : string;const tags : pTagItem) : BOOLEAN;
begin
       MakeWorkbenchObjectVisibleA := MakeWorkbenchObjectVisibleA(pas2c(name),tags);
end;

FUNCTION OpenWorkbenchObjectA(name : string;const tags : pTagItem) : BOOLEAN;
begin
       OpenWorkbenchObjectA := OpenWorkbenchObjectA(pas2c(name),tags);
end;

FUNCTION WorkbenchControlA(name : string;const tags : pTagItem) : BOOLEAN;
begin
       WorkbenchControlA := WorkbenchControlA(pas2c(name),tags);
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of workbench.library}
  {$Info don't forget to use InitWBLibrary in the beginning of your program}

var
    wb_exit : Pointer;

procedure ClosewbLibrary;
begin
    ExitProc := wb_exit;
    if WorkbenchBase <> nil then begin
        CloseLibrary(WorkbenchBase);
        WorkbenchBase := nil;
    end;
end;

procedure InitWBLibrary;
begin
    WorkbenchBase := nil;
    WorkbenchBase := OpenLibrary(WORKBENCHNAME,LIBVERSION);
    if WorkbenchBase <> nil then begin
        wb_exit := ExitProc;
        ExitProc := @ClosewbLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open workbench.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    WBIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of workbench.library}

var
    wb_exit : Pointer;

procedure ClosewbLibrary;
begin
    ExitProc := wb_exit;
    if WorkbenchBase <> nil then begin
        CloseLibrary(WorkbenchBase);
        WorkbenchBase := nil;
    end;
end;

begin
    WorkbenchBase := nil;
    WorkbenchBase := OpenLibrary(WORKBENCHNAME,LIBVERSION);
    if WorkbenchBase <> nil then begin
        wb_exit := ExitProc;
        ExitProc := @ClosewbLibrary;
        WBIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open workbench.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    WBIsCompiledHow := 3;
   {$Warning No autoopening of workbench.library compiled}
   {$Warning Make sure you open workbench.library yourself}
{$endif dont_use_openlib}


END. (* UNIT WB *)




