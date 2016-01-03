{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2006-2008 Free Pascal development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ Declarations for aygshell WinCE API

}

{exported functions list = to do,
 * please remove functions done *
 
     Exports

       ordinal    name

           146    ?ClearFontManager@@YAXXZ (void __cdecl ClearFontManager(void))
           166    ?CreateBackgroundSpec@@YAJPAPAVIBackgroundSpec@@@Z (long __cdecl CreateBackgroundSpec(class IBackgroundSpec * *))
           148    ?GetAppMetric@@YAHW4_enAppMetricID@@@Z (int __cdecl GetAppMetric(enum _enAppMetricID))
           145    ?GetStandardFont@@YAJW4eFontID@@PAPAUHFONT__@@@Z (long __cdecl GetStandardFont(enum eFontID,struct HFONT__ * *))
           348    ?IsScreenRotationSupported@@YAHXZ (int __cdecl IsScreenRotationSupported(void))
            63    ?MinPowerOff@@YAHXZ (int __cdecl MinPowerOff(void))
           147    ?OnSettingChange@@YAHIJ@Z (int __cdecl OnSettingChange(unsigned int,long))
           189    ?SHDrawGradientBubbleTitle@@YAXPAUtagGRADIENTTITLEINFO@@@Z (void __cdecl SHDrawGradientBubbleTitle(struct tagGRADIENTTITLEINFO *))
          2006    ?SHIdleTimerReset@@YAXXZ (void __cdecl SHIdleTimerReset(void))
           228    ?SHLoadMenuExtensions@@YAHPAUIUnknown@@PBG1PAPAX@Z (int __cdecl SHLoadMenuExtensions(struct IUnknown *,unsigned short const *,unsigned short const *,void * *))
            66    ?SHMenuBar_GetMenu@@YAPAUHMENU__@@PAUHWND__@@H@Z (struct HMENU__ * __cdecl SHMenuBar_GetMenu(struct HWND__ *,int))
           140    ADChgTaskList
           139    ADChgTrustedSrcList
           142    ADRegisterCallback
           141    ADTaskInfo
           143    AssociateNoteWithCall
            52    CancelSIPUp
            46    ComboBoxEditSubProc
            29    ComboEditAutoComplete
            45    ComboSubProc
           190    CreateImageCache
           330    DPI_ExtractIconEx
           311    DPI_LoadImageFile
           310    DPI_LoadLibraryRes
           237    DisplayNotRunnableAppDialog
           226    DoEditContextMenu
           191    DrawAlignedIcon
            44    EditSubProc
            13    FreeRegisteredAppInfo
           137    GetProtocol
            11    GetRegisteredAppInfo
           286    HIDPI_ImageList_LoadImage
           287    HIDPI_ImageList_ReplaceIcon
            92    IsFullScreenWindow
           347    IsModulePreWinCE421
            62    IsSANMessage
            70    LFHeightForPoint
           302    LoadHTML
           241    LoadStringEtcOver
           180    NotifyAppsOnEvent
<<<<<<< HEAD
<<<<<<< HEAD
=======
            24    PathAddBackslash
            26    PathCombine
>>>>>>> graemeg/fixes_2_2
=======
            24    PathAddBackslash
            26    PathCombine
>>>>>>> origin/fixes_2_2
            23    PathFindExtension
            27    PathFindFileName
           160    PathFindNextComponent
           116    PathIsPrefix
            28    PathIsRelative
            25    PathRemoveBackslash
            42    PathRemoveBlanks
           107    PhoneGetCallProperties
           106    PhoneGetCallPropertyBag
           159    PhoneShowCallLog
           158    SHAnimateListviewOpen
           117    SHAnimateRects
            22    SHAppNotifyDone
           130    SHBoldFontAllowed
           289    SHBorderPolyline
           288    SHBorderRectangle
           104    SHBox
           103    SHBoxEx
<<<<<<< HEAD
<<<<<<< HEAD
=======
           114    SHChangeNotifyDeregister
           115    SHChangeNotifyFree
           113    SHChangeNotifyRegister
>>>>>>> graemeg/fixes_2_2
=======
           114    SHChangeNotifyDeregister
           115    SHChangeNotifyFree
           113    SHChangeNotifyRegister
>>>>>>> origin/fixes_2_2
            48    SHCheckForContextMenu
            86    SHClearStartedBit
            55    SHColorDisplay
            37    SHCommandBar_EnableCommand
            38    SHCommandBar_GetClientRect
            36    SHCommandBar_GetCommandBarByID
           223    SHCopyBitmap
           306    SHCopyIcon
           164    SHCreateCOleWindow
            41    SHCreateContextMenu
            43    SHCreateMainWindow
            74    SHCreateMenuBarInternal
<<<<<<< HEAD
<<<<<<< HEAD
=======
           108    SHCreateNewItem
>>>>>>> graemeg/fixes_2_2
=======
           108    SHCreateNewItem
>>>>>>> origin/fixes_2_2
            53    SHCreateSystemFont
            35    SHCreateWorkerWindow
           329    SHDeleteTodayWallpaper
           109    SHDocManagerCreate
           112    SHDocManagerDestroy
           111    SHDocManagerQuery
           110    SHDocManagerRegister
           233    SHDrawBranding
           234    SHDrawClippedText
           331    SHDrawTextOverImage
            58    SHDrawUnderline
           136    SHDrawUnderlineColor
            49    SHEnableEditMenu
           192    SHEnableRadio
            81    SHEndProfileObj
          2013    SHEnumFiles
          2020    SHEnumFolders
            98    SHEnumPropSheetHandlers
           203    SHEscapeAccelerators
           201    SHEscapeBubbleHtml
           168    SHFadeImage
            54    SHFillRectClr
           132    SHFindForegroundMenuBar
           283    SHFindMenuBarInternal
           209    SHFindPreviousInstance
           212    SHFindPreviousInstanceEx
            71    SHFontMgrCreate
            73    SHFontMgrDestroy
            72    SHFontMgrManageFonts
            33    SHForceBaseState
            99    SHForceBaseStateEx
<<<<<<< HEAD
<<<<<<< HEAD
=======
           102    SHFreeContextMenuExtensions
>>>>>>> graemeg/fixes_2_2
=======
           102    SHFreeContextMenuExtensions
>>>>>>> origin/fixes_2_2
           120    SHFreeScanners
           213    SHGetActiveDialog
           292    SHGetBitmapDimensions
           285    SHGetBitmapLogPixels
           225    SHGetCarrierBranding
           224    SHGetCarrierBrandingFlag
            96    SHGetDeviceFeatureLevel
           299    SHGetDisplayRotation
<<<<<<< HEAD
<<<<<<< HEAD
           218    SHGetFontHeight
=======
           128    SHGetEmergencyCallList
           218    SHGetFontHeight
           232    SHGetInputContext
>>>>>>> graemeg/fixes_2_2
=======
           128    SHGetEmergencyCallList
           218    SHGetFontHeight
           232    SHGetInputContext
>>>>>>> origin/fixes_2_2
           133    SHGetKOBits
           305    SHGetLandscapeRotationSettings
           345    SHGetLegacySupportWindow
           242    SHGetLocaleInfo
           167    SHGetMessageBoxIcon
           281    SHGetMetric
           172    SHGetNavBarItemRect
           163    SHGetPowerOnTime
           153    SHGetPresetMessage
           282    SHGetScreenOrientation
           204    SHGetSimToolkitMenu
          2014    SHGetSoundFileList
           177    SHGetStyleBkColor
           178    SHGetStyleColor
           179    SHGetStyleFont
           217    SHGetSystemDefaultLCID
           144    SHGetTimeFormat
           312    SHGetUIMetrics
           219    SHGetUiInfo
           221    SHGradientDeInit
           222    SHGradientDraw
           220    SHGradientInit
            31    SHHandleActivate
            30    SHHandleSipChange
            76    SHHdrGrpSepLineDraw
             8    SHImListPopup
           149    SHInitPresetMessages
           181    SHInsertPresetMessage
           129    SHInvalidateScreen
<<<<<<< HEAD
<<<<<<< HEAD
           123    SHIsLocked
           295    SHIsPreOzoneUpdate
            94    SHIsPreRapierApp
=======
=======
>>>>>>> origin/fixes_2_2
           101    SHInvokeContextMenuCommand
           123    SHIsLocked
           295    SHIsPreOzoneUpdate
            94    SHIsPreRapierApp
           100    SHLoadContextMenuExtensions
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           230    SHLoadFileContextMenuExtensions
           313    SHLoadFontFromResource
            91    SHLoadMenuPopup
           216    SHLoadSKFromReg
           121    SHLock
           138    SHMakeCall
           235    SHMakeValidFilename
            32    SHMessageBox
            80    SHNewProfileObj
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
           155    SHNotificationAdd
           173    SHNotificationGetData
           157    SHNotificationRemove
           156    SHNotificationUpdate
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           208    SHNotifyAppsOnCallConnect
           210    SHNotifyAppsOnCarkit
           165    SHNotifyAppsOnDock
           195    SHNotifyAppsOnHeadset
           214    SHNotifyAppsOnIncomingCall
           211    SHNotifyAppsOnSpeakerPhone
           135    SHOnFullScreenAppActivate
           238    SHOnMissedCallCountChange
          2008    SHOnPluginDataChange
           227    SHOnVoiceMailCountChange
           162    SHPaintBubbleFrame
           294    SHPolyline
           152    SHPopulatePresetMessageMenu
           175    SHPreProcessLogFont
           229    SHQueryMenuExtensions
           297    SHRCMLDialogProc
           293    SHRectangle
          2022    SHRefreshStartMenu
           215    SHRegGetHLMDWValue
           176    SHRegSetValueEx
           346    SHReleaseLegacySupportWindow
           150    SHReleasePresetMessages
           202    SHReplaceString
           245    SHResizeDialogProc
            50    SHRunCpl
            82    SHRunFontManager
           151    SHRunPresetMessagesEdit
           239    SHRunSafeApplet
           174    SHSameWindowProcesses
            95    SHSavePWWarning
           118    SHScanBuffer
           119    SHScanFile
            97    SHSendBackToFocusWindow
           169    SHSetAsWatermark
<<<<<<< HEAD
<<<<<<< HEAD
           161    SHSetBubbleRegion
           298    SHSetDisplayRotation
           131    SHSetForegroundLastActivePopup
=======
=======
>>>>>>> origin/fixes_2_2
           184    SHSetBack
           161    SHSetBubbleRegion
           298    SHSetDisplayRotation
           131    SHSetForegroundLastActivePopup
           231    SHSetInputContext
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           134    SHSetKOBits
           154    SHSetPresetMessage
           170    SHSetSimToolkitMenu
          1003    SHSetSoftKey
           300    SHSetStretchMode
            59    SHSetWindowBits
            67    SHShowContextMenu
          1004    SHShowSoftKeys
            79    SHSignalDone
           314    SHSipMightBlockUI
           308    SHSkipDialogInitialFocus
           207    SHSoundManGetDisplayName
           205    SHSoundManGetDisplayNameList
           206    SHSoundManGetFileName
            93    SHStartAndBlock
            85    SHStartIfNeeded
            87    SHStartProfile
           284    SHStretchBitmap
           290    SHStretchBltBitmap
           291    SHStretchBltBitmapEx
           307    SHStretchIcon
           105    SHTextBox
           171    SHToolkitQueryShell
            57    SHTrackPopupMenu
          1000    SHTurnScreenOn
           240    SHUnEscapeAccelerators
           122    SHUnlock
          2003    SHVoiceTagDelete
          2002    SHVoiceTagPlayback
          2001    SHVoiceTagRecognize
          2000    SHVoiceTagTrain
           124    SHWriteLockState
           301    SetDialogAutoScrollBar
            12    SetRegisteredAppInfo
           296    SetWindowPosOnRotate
            15    Shell_Alloc
            17    Shell_AllocString
            18    Shell_CatStrAlloc
            16    Shell_Free
            14    Shell_HeapCreate
            19    Shell_LoadStringAlloc
            20    Shell_RegAllocString
            51    StrStrI
            47    SubClassThisWindow
           199    TZFindClose
           197    TZFindNext
           196    TZFindOpen
           198    TZGetData
          1005    UIHGetTextToStruct
          1007    UIHLimitTextControls
          1008    UIHSetHWNDToStruct
          1006    UIHSetTextFromStruct
           236    VerifyTrust
}

unit aygshell;

interface

<<<<<<< HEAD
<<<<<<< HEAD
{$MODE OBJFPC}

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
uses windows;

{$calling cdecl}

//*****************************************************************************
// consts
//*****************************************************************************
const
  UserDLLAyg    = 'aygshell';

  CEM_UPCASEALLWORDS     = (WM_USER + 1);
  CEM_ENABLEUPCASE       = (WM_USER + 2);

  {Gesture notifications}
  GN_CONTEXTMENU = 1000;

  IDC_COMMANDBANDS       = 100;
  {Shell Menubar support}
  // These defines MUST be < 100.  This is so apps can use these defines
  // to get strings from the shell.
  IDS_SHNEW       =  1;
  IDS_SHEDIT      =  2;
  IDS_SHTOOLS     =  3;
  IDS_SHVIEW      =  4;
  IDS_SHFILE      =  5;
  IDS_SHGO        =  6;
  IDS_SHFAVORITES =  7;
  IDS_SHOPEN      =  8;

  {Shared New menu support}
  IDM_SHAREDNEW        = 10;
  IDM_SHAREDNEWDEFAULT = 11;

  NOMENU                  = $FFFF;
  SHA_INPUTDIALOG         = $0001;
  SHACTI_FSIPUP           = $0001;
  SHACTI_FSIPONDEACTIVATE = $0002;
  SHACTI_FSIPRESERVED     = $FFFF xor SHACTI_FSIPUP xor SHACTI_FSIPUP;

  {Valid dwFlags}
  SHCMBF_EMPTYBAR      = $0001;
  SHCMBF_HIDDEN        = $0002; // create it hidden
  SHCMBF_HIDESIPBUTTON = $0004;
  SHCMBF_COLORBK       = $0008;
  SHCMBF_HMENU         = $0010; // specify an hmenu for resource rather than toolbar info

  SHCMBM_SETSUBMENU    = (WM_USER + 400); // wparam == id of button, lParam == hmenu, return is old hmenu
  SHCMBM_GETSUBMENU    = (WM_USER + 401); // lParam == ID
  SHCMBM_GETMENU       = (WM_USER + 402); // get the owning hmenu (as specified in the load resource)
  SHCMBM_OVERRIDEKEY   = (WM_USER + 403);
  SHCMBM_SETBKCOLOR    = (WM_USER + 406); // lParam == COLORREF


  {Valid mask values}
  SHIDIM_FLAGS         = $0001;
  {Valid flags}
  SHIDIF_DONEBUTTON          = $0001;
  SHIDIF_SIZEDLG             = $0002;
  SHIDIF_SIZEDLGFULLSCREEN   = $0004;
  SHIDIF_SIPDOWN             = $0008;
  SHIDIF_FULLSCREENNOMENUBAR = $0010;
  SHIDIF_EMPTYMENU           = $0020;
  SHIDIF_WANTSCROLLBAR       = $0040;

  SHMBOF_NODEFAULT  = $00000001; // do not do default handling of this key
  SHMBOF_NOTIFY     = $00000002; // send us the WM_* messages for this key

  {Gesture flags}
  SHRG_RETURNCMD      = $00000001;
  SHRG_NOTIFYPARENT   = $00000002;
  SHRG_LONGDELAY      = $00000008;
  SHRG_NOANIMATION    = $00000010;

  {Sip info}
  SIP_STATUS_UNAVAILABLE = 0;
  SIP_STATUS_AVAILABLE  = 1;

  SIPF_OFF    = $00000000;
  SIPF_ON     = $00000001;
  SIPF_DOCKED = $00000002;
  SIPF_LOCKED = $00000004;

  {Supported system parameters}
  SPI_SETCOMPLETIONINFO  = 223;
  SPI_SETSIPINFO         = 224;
  SPI_GETSIPINFO         = 225;
  SPI_SETCURRENTIM       = 226;
  SPI_GETCURRENTIM       = 227;
  SPI_APPBUTTONCHANGE    = 228;
  SPI_RESERVED           = 229;
  SPI_SYNCSETTINGSCHANGE = 230;

  WC_SIPPREF             = 'SIPPREF';

  {SHFullScreen - valide states}
  SHFS_SHOWTASKBAR     = $0001;
  SHFS_HIDETASKBAR     = $0002;
  SHFS_SHOWSIPBUTTON   = $0004;
  SHFS_HIDESIPBUTTON   = $0008;
  SHFS_SHOWSTARTICON   = $0010;
  SHFS_HIDESTARTICON   = $0020;

  { DoneButton - Valid state}
  SHDB_SHOW            = $0001;
  SHDB_HIDE            = $0002;
  SHDB_SHOWCANCEL      = $0004;   // added by Windows Mobile 5.0
  WS_NONAVDONEBUTTON   = WS_MINIMIZEBOX;

//*****************************************************************************
// types
//*****************************************************************************

type
  //Struct sent through WM_NOTIFY when SHRG_NOTIFYPARENT is used
  NMRGINFO = record
    hdr : NMHDR;
    ptAction : POINT;
    dwItemSpec : DWORD;
  end;
  PNMRGINFO=^NMRGINFO;

  SHACTIVATEINFO = record
    cbSize : DWORD;
    hwndLastFocus : HWND ;
    bits : Longint ;
  end;
  PSHACTIVATEINFO=^SHACTIVATEINFO;

  SHINITDLGINFO = record
    dwMask: DWORD;
    hDlg: HWND;
    dwFlags : DWORD ;
   end;
  PSHINITDLGINFO=^SHINITDLGINFO;

  SHMENUBARINFO = record
    cbSize : DWORD;        // IN  - Indicates which members of struct are valid
    hwndParent : HWND ;    // IN
    dwFlags : DWORD ;      // IN  - Some features we want
    nToolBarId : UINT ;    // IN  - Which toolbar are we using
    hInstRes : HINST;      // IN  - Instance that owns the resources
    nBmpId : longint;
    cBmpImages : longint;  // IN  - Count of bitmap images
    hwndMB : HWND ;        // OUT
    clrBk : COLORREF ;     // IN  - background color of the menu bar (excluding sip)
  end;
  PSHMENUBARINFO=^SHMENUBARINFO;

  {SHRecognizeGesture}
  SHRGI = record
    cbSize     : DWORD;
    hwndClient : HWND ;
    ptDown     : POINT;
    dwFlags    : DWORD;
  end;
  SHRGINFO=SHRGI;
  PSHRGINFO=^SHRGI;

  {Sip info}
  SIPINFO = record
    cbSize           : DWORD;
    fdwFlags         : DWORD;
    rcVisibleDesktop : RECT;
    rcSipRect        : RECT;
    dwImDataSize     : DWORD;
    pvImData         : pointer;
  end;
  TSIPINFO=SIPINFO;
  PSIPINFO=^SIPINFO;
  
  SIPSTATE= (SIP_UP= 0,SIP_DOWN,SIP_FORCEDOWN,SIP_UNCHANGED,SIP_INPUTDIALOG);

<<<<<<< HEAD
<<<<<<< HEAD
  CAMERACAPTURE_STILLQUALITY= (CAMERACAPTURE_STILLQUALITY_DEFAULT=0, CAMERACAPTURE_STILLQUALITY_LOW, CAMERACAPTURE_STILLQUALITY_NORMAL,
    CAMERACAPTURE_STILLQUALITY_HIGH);

  CAMERACAPTURE_VIDEOTYPE= (CAMERACAPTURE_VIDEOTYPE_ALL = $FFFF, CAMERACAPTURE_VIDEOTYPE_STANDARD = 1,
    CAMERACAPTURE_VIDEOTYPE_MESSAGING = 2);

  CAMERACAPTURE_MODE= (CAMERACAPTURE_MODE_STILL = 0, CAMERACAPTURE_MODE_VIDEOONLY, CAMERACAPTURE_MODE_VIDEOWITHAUDIO);

  TSHCAMERACAPTURE = record
    cbSize             : DWORD;
    hwndOwner          : HWND;
    szFile             : array[0..(MAX_PATH)-1] of WCHAR;
    pszInitialDir      : LPCTSTR;
    pszDefaultFileName : LPCTSTR;
    pszTitle           : LPCTSTR;
    StillQuality       : CAMERACAPTURE_STILLQUALITY;
    VideoTypes         : CAMERACAPTURE_VIDEOTYPE;
    nResolutionWidth   : DWORD;
    nResolutionHeight  : DWORD;
    nVideoTimeLimit    : DWORD;
    Mode               : CAMERACAPTURE_MODE;
  end;
  PSHCAMERACAPTURE=^TSHCAMERACAPTURE;



//====== File System Notification APIs ===============================
//
//
//  File System Notification flags
//
const
      SHCNE_RENAME              = $00000001;   // GOING AWAY
      SHCNE_RENAMEITEM          = $00000001;
      SHCNE_CREATE              = $00000002;
      SHCNE_DELETE              = $00000004;
      SHCNE_MKDIR               = $00000008;
      SHCNE_RMDIR               = $00000010;
      SHCNE_MEDIAINSERTED       = $00000020;
      SHCNE_MEDIAREMOVED        = $00000040;
      SHCNE_DRIVEREMOVED        = $00000080;
      SHCNE_DRIVEADD            = $00000100;
      SHCNE_NETSHARE            = $00000200;
      SHCNE_NETUNSHARE          = $00000400;
      SHCNE_ATTRIBUTES          = $00000800;
      SHCNE_UPDATEDIR           = $00001000;
      SHCNE_UPDATEITEM          = $00002000;
      SHCNE_SERVERDISCONNECT    = $00004000;
      SHCNE_UPDATEIMAGE         = $00008000;
      SHCNE_DRIVEADDGUI         = $00010000;
      SHCNE_RENAMEFOLDER        = $00020000;

      SHCNE_ASSOCCHANGED        = $08000000;

      SHCNE_DISKEVENTS          = $0002381F;
      SHCNE_GLOBALEVENTS        = $0C0181E0; // Events that dont match pidls first
      SHCNE_ALLEVENTS           = $7FFFFFFF;
      SHCNE_INTERRUPT           = $80000000; // The presence of this flag indicates
                                             // that the event was generated by an
                                             // interrupt.  It is stripped out before
                                             // the clients of SHCNNotify_ see it.

// Flags
// uFlags & SHCNF_TYPE is an ID which indicates what dwItem1 and dwItem2 mean
      SHCNF_IDLIST      = $0000;  // LPITEMIDLIST
      SHCNF_PATH        = $0001;  // path name
      SHCNF_PRINTER     = $0002;  // printer friendly name
      SHCNF_DWORD       = $0003;  // DWORD
      SHCNF_TYPE        = $00FF;
      SHCNF_FLUSH       = $1000;
      SHCNF_FLUSHNOWAIT = $2000;

const
      WM_FILECHANGEINFO   = WM_APP + $0101;

type
     _FILECHANGEINFO = record
       cbSize:DWORD;
       wEventId:LONG;
       uFlags:ULONG;
       dwItem1:DWORD;
       dwItem2:DWORD;
       dwAttributes:DWORD;
       ftModified:FILETIME;
       nFileSize:ULONG;
     end;
     FILECHANGEINFO = _FILECHANGEINFO;
     LPFILECHANGEINFO = ^_FILECHANGEINFO;
     LPCFILECHANGEINFO = ^FILECHANGEINFO;

     tagFILECHANGENOTIFY = record
       dwRefCount:DWORD;
       fci:FILECHANGEINFO;
     end;
     FILECHANGENOTIFY = tagFILECHANGENOTIFY;
     LPFILECHANGENOTIFY = ^tagFILECHANGENOTIFY;

     tagSHCHANGENOTIFYENTRY = record
       dwEventMask:DWORD;    // Events to watch
       pszWatchDir:LPTSTR;    // Directory or root for the events we want. NULL means all.
       fRecursive:BOOL;     // Indicates whether look just for pszWatchDir or recursively.
     end;
     SHCHANGENOTIFYENTRY = tagSHCHANGENOTIFYENTRY;
     LPSHCHANGENOTIFYENTRY = ^tagSHCHANGENOTIFYENTRY;

     
//////////////////////////////////////////////////////////////////////////////
//
// Input Context API
//
// These are definitions and APIs for the interacting with the input context
// properties of individual windows
//

{$PUSH}
{$PACKENUM 4}
// Word correct Options
type
     SHIC_FEATURE = (SHIC_FEATURE_RESTOREDEFAULT := 0,
                     SHIC_FEATURE_AUTOCORRECT    := $00000001,
                     SHIC_FEATURE_AUTOSUGGEST    := $00000002,
                     SHIC_FEATURE_HAVETRAILER    := $00000003,
                     SHIC_FEATURE_CLASS          := $00000004);
{$POP}

const
// Predefined input context classes
      SHIC_CLASS_DEFAULT              = '';
      SHIC_CLASS_EMAIL                = 'email';
      SHIC_CLASS_URL                  = 'url';
      SHIC_CLASS_PHONE                = 'phone';
      SHIC_CLASS_NAME                 = 'name';
      SHIC_CLASS_PHONE_AND_EMAIL      = 'phoneAndEmail';
      SHIC_CLASS_MAXLEN               = MAX_PATH - 11;

//@topic Input Context Features |
// The input context API supports the following features and their corresponding values:
//
//@flag   SHIC_FEATURE_RESTOREDEFAULT   | Restore original input context state. (no corresponding value)
//@flag   SHIC_FEATURE_AUTOCORRECT      | Turn auto-corrections on and off. (TRUE, FALSE)
//@flag   SHIC_FEATURE_AUTOCOMPLETE     | Turn dictionary suggestions on and off. (TRUE, FALSE)
//@flag   SHIC_FEATURE_HAVETRAILER      | Specify whether to append trailer characters after replacing words.
//                                      (TRUE, FALSE)
//@flag   SHIC_FEATURE_CLASS            | Make this control behave like a specific semantic type.
//                                      (SHIC_CLASS_DEFAULT, SHIC_CLASS_EMAIL, SHIC_CLASS_URL,
//                                      SHIC_CLASS_PHONE, SHIC_CLASS_NAME, SHIC_CLASS_PHONE_AND_EMAIL)
//
//@comm All SHIC_FEATUREs are inherited from parent if undefined. That is, if they are not defined in
//      a window or the window's SHIC class, the API looks at the parent chain to find the setting
//      that applies to the window.
//
//@xref <f SHSetInputContext> <f SHGetInputContext>
//
//
// end Input Context API
//
//////////////////////////////////////////////////////////////////////////////


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//*****************************************************************************
// functions
//*****************************************************************************

<<<<<<< HEAD
<<<<<<< HEAD
function PathAddBackslash(lpszPath:LPTSTR):LPTSTR; external UserDLLAyg name 'PathAddBackslash'; // index 24
function PathCombine(lpszDest:LPTSTR; lpszDir:LPCTSTR; lpszFile:LPCTSTR):LPTSTR; external UserDLLAyg name 'PathCombine'; // index 26

function ExitWindowsEx(uFlags:UINT; dwReserved:DWORD):WINBOOL; external UserDLLAyg name 'ExitWindowsEx';

function SHChangeNotifyDeregister(_hwnd:HWND):BOOL; external UserDLLAyg name 'SHChangeNotifyDeregister'; // index 114
procedure SHChangeNotifyFree(pfcn:LPFILECHANGENOTIFY); external UserDLLAyg name 'SHChangeNotifyFree'; // index 115
function SHChangeNotifyRegister(_hwnd:HWND; pshcne:LPSHCHANGENOTIFYENTRY):BOOL; external UserDLLAyg name 'SHChangeNotifyRegister'; // index 113
function SHCloseApps( dwMemSought : DWORD ): WINBOOL; external UserDLLAyg name 'SHCloseApps';
function SHCreateMenuBar(pmbi : PSHMENUBARINFO ): WINBOOL; external UserDLLAyg name 'SHCreateMenuBar';

// SHCreateNewItem
//    Creates a New item, as if an item were chosen from the
//    global New menu dropdown.
function SHCreateNewItem(hwndOwner:HWND; clsid:PCLSID{REFCLSID}):HRESULT; external UserDLLAyg name 'SHCreateNewItem'; // index 108

function SHDoneButton(hwndRequester: HWND ; dwState : DWORD ): WINBOOL; external UserDLLAyg name 'SHDoneButton';
function SHFindMenuBar(hwnd:HWND) : HWND; external UserDLLAyg name 'SHFindMenuBar';
function SHFreeContextMenuExtensions(hCMExtensions:HANDLE):BOOL; external UserDLLAyg name 'SHFreeContextMenuExtensions'; // index 102
function SHFullScreen(hwmdRequester: hWnd; dwState: DWord): WINBOOL; external UserDLLAyg name 'SHFullScreen';  {Pocket PC  special controls}
function SHGetAutoRunPath( pAutoRunPath : LPTSTR ): WINBOOL; external UserDLLAyg name 'SHGetAutoRunPath';

//  SHGetEmergencyCallList
//       Gets a list of emergency calls
function SHGetEmergencyCallList(pwszBuffer:PTCHAR; uLenBuf:UINT):HRESULT; external UserDLLAyg name 'SHGetEmergencyCallList'; // index 128    

=======
=======
>>>>>>> origin/fixes_2_2
function ExitWindowsEx(uFlags:UINT; dwReserved:DWORD):WINBOOL; external UserDLLAyg name 'ExitWindowsEx';
function SHCloseApps( dwMemSought : DWORD ): WINBOOL; external UserDLLAyg name 'SHCloseApps';
function SHCreateMenuBar(pmbi : PSHMENUBARINFO ): WINBOOL; external UserDLLAyg name 'SHCreateMenuBar';
function SHDoneButton(hwndRequester: HWND ; dwState : DWORD ): WINBOOL; external UserDLLAyg name 'SHDoneButton';
function SHFindMenuBar(hwnd:HWND) : HWND; external UserDLLAyg name 'SHFindMenuBar';
function SHFullScreen(hwmdRequester: hWnd; dwState: DWord): WINBOOL; external UserDLLAyg name 'SHFullScreen';  {Pocket PC  special controls}
function SHGetAutoRunPath( pAutoRunPath : LPTSTR ): WINBOOL; external UserDLLAyg name 'SHGetAutoRunPath';  
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function SHHandleWMActivate(hwnd:HWND; wParam:WPARAM; lParam:LPARAM; psai: PSHACTIVATEINFO; dwFlags:DWORD  ): WINBOOL; external UserDLLAyg index 84;
function SHHandleWMSettingChange(hwnd:HWND; wParam:WPARAM; lParam:LPARAM; psai: PSHACTIVATEINFO): WINBOOL; external UserDLLAyg index 83;
function SHInitDialog(pshidi: PSHINITDLGINFO): WINBOOL; external UserDLLAyg name 'SHInitDialog';
function SHInitExtraControls: WINBOOL; external UserDLLAyg name 'SHInitExtraControls';
procedure SHInputDialog(hwnd : HWND; uMsg : UINT; wParam: WPARAM ); external UserDLLAyg name 'SHInputDialog';
<<<<<<< HEAD
<<<<<<< HEAD

//    Invokes a command from a context menu.  Issues the command in the
//    extension that added it to the menu.
function SHInvokeContextMenuCommand(hwndOwner:HWND; idCmd:UINT; hCMExtensions:HANDLE):BOOL; external UserDLLAyg name 'SHInvokeContextMenuCommand'; // index 101

function SHLoadContextMenuExtensions(punkOwner:IUnknown;
                                     pszContext:LPCTSTR;
                                     pszClass:LPCTSTR;
                                     _hmenu:HMENU;
                                     idCmdFirst:UINT;
                                     idCmdLast:UINT;
                                     phCMExtensions:LPHANDLE):BOOL; external UserDLLAyg name 'SHLoadContextMenuExtensions'; // index 100
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function SHGetAppKeyAssoc( ptszApp: LPCTSTR ): Byte; external UserDLLAyg name 'SHGetAppKeyAssoc';
function SHSetAppKeyWndAssoc( bVk: BYTE ; hwnd : HWND ): WINBOOL; external UserDLLAyg name 'SHSetAppKeyWndAssoc';
function SHSetNavBarText(hwndRequester : HWND; pszText : LPCTSTR): WINBOOL; external UserDLLAyg name 'SHSetNavBarText';
function SHLoadImageResource(hinst: HINST; uIdGif: UINT ): HBITMAP; external UserDLLAyg index 64;
function SHLoadImageFile(pszFileName: LPCTSTR ) : HBITMAP; external UserDLLAyg index 75;
procedure SHNavigateBack; external UserDLLAyg index 183;
function SHSipInfo(uiAction: UINT; uiParam: UINT; pvParam: PVOID; fWinIni: UINT  ): WINBOOL; external UserDLLAyg name 'SHSipInfo';
function SHSipPreference(hwnd: HWND ; st : SIPSTATE ) : WINBOOL; external UserDLLAyg name 'SHSipPreference';
function SHRecognizeGesture(var shrg : SHRGINFO): DWORD; external UserDLLAyg name 'SHRecognizeGesture';
<<<<<<< HEAD
<<<<<<< HEAD
function SHCameraCapture(var shcc: TSHCAMERACAPTURE): HRESULT; external UserDLLAyg name 'SHCameraCapture';

//////////////////////////////////////////////////////////////////////////////
//
// Input Context API
//
// These are definitions and APIs for the interacting with the input context
// properties of individual windows
//

//++++++
//
//@func HRESULT | SHSetInputContext | Changes the state of an input context feature
//
//@parm HWND            | hwnd      | IN - Window whose context will be set
//@parm DWORD           | dwFeature | IN - Input context feature to change
//@parm const LPVOID    | lpValue   | IN - New value assigned to feature
//
//@rdesc Returns one of the following values:
//@flag S_OK                    | If everything went well
//@flag ERROR_INVALID_PARAMETER | if hwnd was NULL or lpValue was NULL for a feature
//                                that does not support it, such as SHIC_FEATURE_AUTOCORRECT,
//                                SHIC_FEATURE_AUTOCOMPLETE and SHIC_FEATURE_HAVETRAILER.
//@flag ERROR_NOT_SUPPORTED     | If the feature specified was invalid
//@flag ERROR_INVALID_DATA      | If the specified value is not a legal option
//
//@xref <l Input_Context_Features> <f SHGetInputContext>
//
function SHSetInputContext(_hwnd:HWND; dwFeature:DWORD; lpValue:LPVOID):HRESULT; external UserDLLAyg name 'SHSetInputContext'; // index 231

//++++++
//
//@func HRESULT | SHGetInputContext | Retrieves current state of an input context feature
//
//@parm HWND    | hwnd      | IN - Window whose context will be retrieved
//@parm DWORD   | dwFeature | IN - Input context feature to retrieve
//@parm LPVOID  | lpValue   | OUT - Buffer to hold current value of feature
//@parm LPDWORD | pdwSize   | IN/OUT - size of the buffer passed in to retrieve the value
//
//@rdesc Returns one of the following values:
//@flag S_OK                        | If everything went well
//@flag ERROR_INVALID_PARAMETER     | If hwnd or lpdwSize passed were NULL
//@flag ERROR_NOT_SUPPORTED         | If the feature specified was invalid
//@flag ERROR_INSUFFICIENT_BUFFER   | If buffer passed is too small
//
//@comm Retrieves the current state/value of the specified
//      input context feature. If the value is not explicitly set, it
//      looks at the features set by the context class. If no class was
//      set explicitly, or the class didn't set that value, it returns
//      the default value for that feature, which would be the
//      currently active one.
//      If lpValue is NULL and lpdwSize is not NULL, it returns the
//      size of the buffer needed in lpdwSize.
//
//@xref <l Input_Context_Features> <f SHSetInputContext>
//
function SHGetInputContext(_hwnd:HWND; dwFeature:DWORD; lpValue:LPVOID; lpdwSize:LPDWORD):HRESULT; external UserDLLAyg name 'SHGetInputContext'; // index 232
//
// end Input Context API
//
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//
// SHNAPI - Shell Notification API
//
// These are definitions and APIs for the Shell Notifications system
//
// {

// notification priority

type
     _SHNP = (SHNP_INFORM := $01B1,      // bubble shown for duration, then goes away
              SHNP_ICONIC                // no bubble, icon shown for duration then goes away
             );
     SHNP = _SHNP;

const
// notification update mask
      SHNUM_PRIORITY     = $0001;
      SHNUM_DURATION     = $0002;
      SHNUM_ICON         = $0004;
      SHNUM_HTML         = $0008;
      SHNUM_TITLE        = $0010;
      SHNUM_SOFTKEYS     = $0020;
      SHNUM_TODAYKEY     = $0040;
      SHNUM_TODAYEXEC    = $0080;
      SHNUM_SOFTKEYCMDS  = $0100;
      SHNUM_FLAGS        = $0200;


// notification data

// Softkey Flags for use with SFTKEYNOTIFY structure

      NOTIF_SOFTKEY_FLAGS_DISMISS       = $0000;  // Remove the notification when the softkey is pressed
      NOTIF_SOFTKEY_FLAGS_HIDE          = $0001;  // Hide the notification when the softkey is pressed (but do not dismiss)
      NOTIF_SOFTKEY_FLAGS_STAYOPEN      = $0002;  // Do not dismiss or hide the notification when the softkey is pressed
      NOTIF_SOFTKEY_FLAGS_SUBMIT_FORM   = $0004;  // Submit the HTML form in the associated notification instead of sending WM_COMMAND to the sink
      NOTIF_SOFTKEY_FLAGS_DISABLED      = $0008;  // This softkey is disabled

// Structure used to associate a command id from a notification's softkey bar
// with a particular behaviour.

type
     SOFTKEYCMD = record
       wpCmd:WPARAM;                   // command to send with WM_COMMAND when pressed.
       grfFlags:DWORD;                 // define special behaviour for this softkey as
                                       // per  NOTIF_SOFTKEY_FLAGS
     end;
     LPSOFTKEYCMD = ^SOFTKEYCMD;


// structure to define a single softkey for use in SHNOTIFICATIONDATA
type
     _SOFTKEYNOTIFY = record
       pszTitle:LPCTSTR;        // Title to use on the softkey
       skc:SOFTKEYCMD;          // Behaviour flags
     end;
     SOFTKEYNOTIFY = _SOFTKEYNOTIFY;
     LPSOFTKEYNOTIFY = ^_SOFTKEYNOTIFY;

// structure to define a menu for use in SHNOTIFICATIONDATA
type
     _SOFTKEYMENU = record
       hMenu:HMENU;
       prgskc:LPSOFTKEYCMD;    // optional array of SOFTKEYCMD values
       cskc:UINT;          // number of members of pskCmd
     end;
     SOFTKEYMENU = _SOFTKEYMENU;
     LPSOFTKEYMENU = ^_SOFTKEYMENU;


const
// number of soft keys on the notification soft key bar.
      NOTIF_NUM_SOFTKEYS = 2;


type
     _SHNOTIFICATIONDATA =record
       cbStruct:DWORD;     // for verification and versioning
       dwID:DWORD;         // identifier for this particular notification
       npPriority:SHNP;   // priority
       csDuration:DWORD;   // duration of the notification (usage depends on prio)
       hicon:HICON;        // the icon for the notification
       grfFlags:DWORD;     // flags - see SHNF_ flags below
       clsid:CLSID;        // unique identifier for the notification class
       hwndSink:HWND;     // window to receive command choices, dismiss, etc.
       pszHTML:LPCTSTR;      // HTML content for the bubble
       pszTitle:LPCTSTR;     // Optional title for bubble
       lParam:LPARAM;       // User-defined parameter
       case longint of    // Defines the softkey bar for the notification
         0: (skm:SOFTKEYMENU;        // Either pass an HMENU in skn (and set SHNF_HASMENU)
             pszTodaySK:LPCTSTR;  // Text to put on SK2 on the Today screen. If NULL, will default to "Notification"
             pszTodayExec:LPCTSTR    // What to execute when SK2 is pressed. If NULL, the toast will be displayed.
            );
         1: (rgskn:array[0..NOTIF_NUM_SOFTKEYS-1] of SOFTKEYNOTIFY;  // or two softkeys in rgskn.
             pszTodaySK1:LPCTSTR;  // Text to put on SK2 on the Today screen. If NULL, will default to "Notification"
             pszTodayExec1:LPCTSTR    // What to execute when SK2 is pressed. If NULL, the toast will be displayed.
            );
     end;
     SHNOTIFICATIONDATA = _SHNOTIFICATIONDATA;
     LPSHNOTIFICATIONDATA = ^_SHNOTIFICATIONDATA;


// Flags
const
// For SHNP_INFORM priority and above, don't display the notification bubble
// when it's initially added; the icon will display for the duration then it
// will go straight into the tray.  The user can view the icon / see the
// bubble by opening the tray.
      SHNF_STRAIGHTTOTRAY  = $00000001;

// Critical information - highlights the border and title of the bubble.
      SHNF_CRITICAL        = $00000002;

// Force the message (bubble) to display even if settings says not to.
      SHNF_FORCEMESSAGE    = $00000008;

// Force the display to turn on for notification.
      SHNF_DISPLAYON       = $00000010;

// Force the notification to be silent and not vibrate, regardless of Settings
      SHNF_SILENT          = $00000020;

// Softkey bar is created from an HMENU passed in skm structure
      SHNF_HASMENU         = $00000040;

// Draw the current time with the title
      SHNF_TITLETIME       = $00000080;

// A notification with "stack" support
      SHNF_SPINNERS        = $00000100;

// RE-play physical alerts on an update
      SHNF_ALERTONUPDATE   = $00000200;

//Capture the VK_TTALK button and forward it to the notification's sink window
      SHNF_WANTVKTTALK     = $00000400;

// notification message and codes for window-based notification
// the notification's dwID is in hdr.idFrom

type
     _NMSHN = record
       hdr:NMHDR;
       lParam:LPARAM;
       dwReturn:DWORD;
       case longint of
         0: (pszLink:LPCTSTR);
         1: (fTimeout:BOOL);
         2: (pt:POINT);
     end;
     NMSHN = _NMSHN;
     LPNMSHN = ^_NMSHN;

//#define SHN_FIRST               (0U-1000U)       // Shell reserved
const    
      SHNN_FIRST              = UINT(0)-UINT(1000);        // Shell Notifications
      SHNN_LAST               = UINT(0)-UINT(1020);        // Shell Notifications
//#define SHN_LAST                (0U-11000U)

const
      SHNN_LINKSEL            = SHNN_FIRST - 0;
// nmshn.pszLink contains the link text of the choice that was selected

      SHNN_DISMISS            = SHNN_FIRST - 1;
// nmshn.fTimeout is TRUE if duration expired, FALSE if user tapped away

      SHNN_SHOW               = SHNN_FIRST - 2;
// nmshn.pt contains the point to which the bubble points

      SHNN_NAVPREV            = SHNN_FIRST - 3;
// Toast stack left spinner clicked / DPAD LEFT

      SHNN_NAVNEXT            = SHNN_FIRST - 4;
// Toast stack right spinner clicked / DPAD RIGHT

      SHNN_ACTIVATE           = SHNN_FIRST - 5;
// Toast DPAD Action

      SHNN_ICONCLICKED        = SHNN_FIRST - 6;
// nmshn.pt contains the point where the user clicked

      SHNN_HOTKEY             = SHNN_FIRST - 7;
// A hotkey has been pressed - modifiers are in the loword of the nmshn.lParam,
// the virtual key code is in the hiword.
// If the sink window returns 0 in response to this notification, then
// the notification toast will be hidden and VK_TTALK key default behavior
// will be performed.

//===========================================================================
//
// Interface: IShellNotificationCallback
//
//  The IShellNotificationCallback interface is used by the Shell to advise
// the notification owner of actions taken on the notification.
//
// [Member functions]
//
// IShellNotificationCallback::OnShow
//
//  Reserved.  Return E_NOTIMPL.
//
// IShellNotificationCallback::OnCommandSelected
//
//  This member function is called when the user selects a link of the form
// <A HREF="cmd:#">link</A>.
//
//  Parameters:
//   dwID       -- the identifier of the notification
//   wCmdID     -- this is the # in the link
//
// IShellNotificationCallback::OnLinkSelected
//
//  This member function is called when the user selects one of the action
// choice links in the notification bubble window.
//
//  Parameters:
//   dwID       -- the identifier of the notification
//   pszLink    -- the link content that was selected
//   lParam     -- the lParam of the notification
//
// IShellNotificationCallback::OnDismiss
//
//  This member function is called when the user taps away from the bubble
// window or if a SHNP_INFORM priority notification's duration expires.
//
//  Parameters:
//   dwID       -- the identifier of the notification
//   fTimeout   -- the notification timed out (SHNP_INFORM only)
//   lParam     -- the lParam of the notification
//   
//===========================================================================

const
      IID_IShellNotificationCallback:TIID = '{000214C0-0000-0000-C000-000000000046}';

type
     IShellNotificationCallback = interface(IUnknown)
      ['{000214C0-0000-0000-C000-000000000046}']
       function OnShow(dwID:DWORD; pt:POINT; lParam:LPARAM):HRESULT; stdcall;
       function OnCommandSelected(dwID:DWORD; wCmdID:word):HRESULT; stdcall;
       function OnLinkSelected(dwID:DWORD; pszLink:LPCTSTR; lParam:LPARAM):HRESULT; stdcall;
       function OnDismiss(dwID:DWORD; fTimeout:BOOL; lParam:LPARAM):HRESULT; stdcall;
     end;

//++++++
//
// SHNotificationAdd
//
//   Add a notification.

function SHNotificationAdd(pndAdd:LPSHNOTIFICATIONDATA):LRESULT; external UserDLLAyg name 'SHNotificationAdd'; // index 155

//
// End SHNotificationAdd
//
//------


//++++++
//
// SHNotificationUpdate
//
//   Update aspects of a pending notification.

function SHNotificationUpdate(grnumUpdateMask:DWORD; pndNew:LPSHNOTIFICATIONDATA):LRESULT; external UserDLLAyg name 'SHNotificationUpdate'; // index 156

//
// End SHNotificationUpdate
//
//------

//++++++
//
// SHNotificationRemove
//
//   Remove a notification.  This is usually in reponse to some
//   action taken on the data outside of the notification system - for example
//   if a message is read or deleted.

function SHNotificationRemove(pclsid:LPCLSID; dwID:DWORD):LRESULT; external UserDLLAyg name 'SHNotificationRemove'; // index 157

//
// End SHNotificationRemove
//
//------


//++++++
//
// SHNotificationGetData
//
//   Get the data for a notification.  Used by a handler to get information
//   stored in the notification by the poster.

function SHNotificationGetData(pclsid:LPCLSID; dwID:DWORD; pndBuffer:LPSHNOTIFICATIONDATA):LRESULT; external UserDLLAyg name 'SHNotificationGetData'; // index 173

//
// End SHNotificationGetData
//
//------


// }
//
// end SHNAPI
//
//////////////////////////////////////////////////////////////////////////////


// This function is not implemented.
// It is provided as a stub in the operating system (OS) for application compatibility.
procedure SHSetBack(eOp:longint; _hwnd:HWND); external UserDLLAyg name 'SHSetBack'; // index 184    
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

implementation

end.
