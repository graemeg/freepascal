{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2002 by Nils Sjoholm
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

    Added MessageBox for error report.
    31 Jul 2000.

    Added functions and procedures with array of const.
    For use with fpc 1.0.
    07 Dec 2002.

    Added the defines use_amiga_smartlink and
    use_auto_openlib.
    12 Jan 2003.

    Changed startcode for unit.
    11 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se
}

{$mode objfpc}
{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
    {$smartlink on}
{$endif use_amiga_smartlink}

UNIT REQTOOLS;


INTERFACE

USES Exec, intuition, amigados, diskfont, graphics, utility;



{***********************
*                      *
*     Preferences      *
*                      *
***********************}

CONST   RTPREF_FILEREQ          = 0;
        RTPREF_FONTREQ          = 1;
        RTPREF_PALETTEREQ       = 2;
        RTPREF_SCREENMODEREQ    = 3;
        RTPREF_VOLUMEREQ        = 4;
        RTPREF_OTHERREQ         = 5;
        RTPREF_NR_OF_REQ        = 6;

Type
    pReqDefaults = ^tReqDefaults;
    tReqDefaults = Record
        Size       : ULONG;
        ReqPos     : ULONG;
        LeftOffset : WORD;
        TopOffset  : WORD;
        MinEntries : WORD;
        MaxEntries : WORD;
    END;

    pReqToolsPrefs = ^tReqToolsPrefs;
    tReqToolsPrefs = Record
    { Size of preferences (_without_ this field and the semaphore) }
        PrefsSize      : ULONG;
        PrefsSemaphore : tSignalSemaphore;
    { Start of real preferences }
        Flags          : ULONG;
        ReqDefaults   : ARRAY [0..RTPREF_NR_OF_REQ-1] OF tReqDefaults;
    END;

CONST
     RTPREFS_SIZE = 100;

{ Flags }

     RTPRB_DIRSFIRST       = 0;
     RTPRF_DIRSFIRST       = 1;
     RTPRB_DIRSMIXED       = 1;
     RTPRF_DIRSMIXED       = 2;
     RTPRB_IMMSORT         = 2;
     RTPRF_IMMSORT         = 4;
     RTPRB_NOSCRTOFRONT    = 3;
     RTPRF_NOSCRTOFRONT    = 8;
     RTPRB_NOLED           = 4;
     RTPRF_NOLED           = 16;
     RTPRB_DEFAULTFONT     = 5;
     RTPRF_DEFAULTFONT     = 32;
     RTPRB_DOWHEEL         = 6;
     RTPRF_DOWHEEL         = 64;
     RTPRB_FKEYS           = 7;
     RTPRF_FKEYS           = 128;
     RTPRB_FANCYWHEEL      = 8;
     RTPRF_FANCYWHEEL      = 256;
     RTPRB_MMBPARENT       = 9;
     RTPRF_MMBPARENT       = 512;

{***********************
*                      *
*     Library Base     *
*                      *
***********************}

Const

    REQTOOLSNAME  : PChar = 'reqtools.library';
    REQTOOLSVERSION = 38;

Type

    pReqToolsBase = ^tReqToolsBase;
    tReqToolsBase = Record
        LibNode           : pLibrary;
        RTFlags           : Byte;
        Pad               : Array[0..2] of Byte;
        SegList           : BPTR;

        { The following library bases may be read and used by your program }

        IntuitionBase     : pLibrary;
        GfxBase           : pLibrary;
        DOSBase           : pLibrary;

        { Next two library bases are only (and always) valid on Kickstart 2.0!
          (1.3 version of reqtools also initializes these when run on 2.0) }

        GadToolsBase      : pLibrary;
        UtilityBase       : pLibrary;

        { PRIVATE FIELDS, THESE WILL CHANGE FROM RELEASE TO RELEASE! }

        { The RealOpenCnt is for the buffered AvailFonts feature.  Since
          Kickstart 3.0 offers low memory handlers a release of ReqTools for 3.0
          will not use this field and start using the normal OpenCnt again. }

        RealOpenCnt       : WORD;
        AvailFontsLock    : WORD;
        AvailFontsHeader : pAvailFontsHeader;
        FontsAssignType   : ULONG;
        FontsAssignLock   : BPTR;
        FontsAssignList   : pAssignList;
        ReqToolsPrefs    : tReqToolsPrefs;
        prefspad          : WORD;
    end;


Const

{ types of requesters, for rtAllocRequestA() }

    RT_FILEREQ       = 0;
    RT_REQINFO       = 1;
    RT_FONTREQ       = 2;
    { (V38) }
    RT_SCREENMODEREQ = 3;

{***********************
*                      *
*    File requester    *
*                      *
***********************}

type

{ structure _MUST_ be allocated with rtAllocRequest() }

    prtFileRequester = ^trtFileRequester;
    trtFileRequester  = Record
         ReqPos      : ULONG;
         LeftOffset  : WORD;
         TopOffset   : WORD;
         Flags       : ULONG;

         { OBSOLETE IN V38! DON'T USE! } Hook: pHook;

         Dir         : STRPTR;     { READ ONLY! Change with rtChangeReqAttrA()! }
         MatchPat    : STRPTR;     { READ ONLY! Change with rtChangeReqAttrA()! }
         DefaultFont : pTextFont;
         WaitPointer : ULONG;
         { (V38) }
         LockWindow  : ULONG;
         ShareIDCMP  : ULONG;
         IntuiMsgFunc: pHook;
         reserved1   : WORD;
         reserved2   : WORD;
         reserved3   : WORD;
         ReqHeight   : WORD;     { READ ONLY! Use RTFI_Height tag! }
         { Private data follows! HANDS OFF }
    end;


{ returned by rtFileRequestA() if multiselect is enabled,
  free list with rtFreeFileList() }

    prtFileList = ^trtFileList;
    trtFileList = Record
         Next    : prtFileList;
         StrLen  : ULONG;        { -1 for directories }
         Name    : STRPTR;
    end;

{ structure passed to RTFI_FilterFunc callback hook by
   volume requester (see RTFI_VolumeRequest tag) }

    prtVolumeEntry = ^trtVolumeEntry;
    trtVolumeEntry = Record
        Type_ : ULONG;           { DLT_DEVICE or DLT_DIRECTORY }
        Name  : STRPTR;
    END;

{***********************
*                      *
*    Font requester    *
*                      *
***********************}

{ structure _MUST_ be allocated with rtAllocRequest() }

    prtFontRequester = ^trtFontRequester;
    trtFontRequester = Record
         ReqPos        : ULONG;
         LeftOffset    : WORD;
         TopOffset     : WORD;
         Flags         : ULONG;
         { OBSOLETE IN V38! DON'T USE! } Hook: pHook;
         Attr          : tTextAttr; { READ ONLY! }
         DefaultFont   : pTextFont;
         WaitPointer   : ULONG;
         { (V38) }
         LockWindow    : ULONG;
         ShareIDCMP    : ULONG;
         IntuiMsgFunc  : pHook;
         reserved1     : WORD;
         reserved2     : WORD;
         reserved3     : WORD;
         ReqHeight     : WORD; { READ ONLY!  Use RTFO_Height tag! }
         { Private data follows! HANDS OFF }
    end;


{*************************
*                        *
*  ScreenMode requester  *
*                        *
*************************}

{ structure _MUST_ be allocated with rtAllocRequest() }

    prtScreenModeRequester = ^trtScreenModeRequester;
    trtScreenModeRequester = Record
         ReqPos      : ULONG;
         LeftOffset  : WORD;
         TopOffset   : WORD;
         Flags       : ULONG;
         private1    : ULONG;
         DisplayID   : ULONG;  { READ ONLY! }
         DisplayWidth: WORD;    { READ ONLY! }
         DisplayHeight: WORD;   { READ ONLY! }
         DefaultFont : pTextFont;
         WaitPointer : ULONG;
         LockWindow  : ULONG;
         ShareIDCMP  : ULONG;
         IntuiMsgFunc: pHook;
         reserved1   : WORD;
         reserved2   : WORD;
         reserved3   : WORD;
         ReqHeight   : WORD;    { READ ONLY!  Use RTSC_Height tag! }
         DisplayDepth: WORD;    { READ ONLY! }
         OverscanType: WORD;    { READ ONLY! }
         AutoScroll  : ULONG;  { READ ONLY! }
         { Private data follows! HANDS OFF }
    end;


{***********************
*                      *
*    Requester Info    *
*                      *
***********************}

{ for rtEZRequestA(), rtGetLongA(), rtGetStringA() and rtPaletteRequestA(),
   _MUST_ be allocated with rtAllocRequest() }

    prtReqInfo = ^trtReqInfo;
    trtReqInfo = Record
         ReqPos      : ULONG;
         LeftOffset  : WORD;
         TopOffset   : WORD;
         Width       : ULONG;        { not for rtEZRequestA() }
         ReqTitle    : STRPTR;         { currently only for rtEZRequestA() }
         Flags       : ULONG;        { currently only for rtEZRequestA() }
         DefaultFont : pTextFont;    { currently only for rtPaletteRequestA() }
         WaitPointer : ULONG;
         { (V38) }
         LockWindow  : ULONG;
         ShareIDCMP  : ULONG;
         IntuiMsgFunc: pHook;
         { structure may be extended in future }
     end;


{***********************
*                      *
*     Handler Info     *
*                      *
***********************}

{ for rtReqHandlerA(), will be allocated for you when you use
   the RT_ReqHandler tag, never try to allocate this yourself! }

    prtHandlerInfo = ^trtHandlerInfo;
    trtHandlerInfo = Record
         private1  : ULONG;
         WaitMask  : ULONG;
         DoNotWait : ULONG;
         { Private data follows, HANDS OFF }
    end;


Const

{ possible return codes from rtReqHandlerA() }

    CALL_HANDLER = $80000000;

{*************************************
*                                    *
*                TAGS                *
*                                    *
*************************************}

    RT_TagBase = TAG_USER;

{ *** tags understood by most requester functions *** }

RT_Window       = $80000001; { Optional pointer to window }
RT_IDCMPFlags   = $80000002; { idcmp flags requester should abort on (useful for
IDCMP_DISKINSERTED) }
RT_ReqPos       = $80000003; { position of requester window (see below) - default
REQPOS_POINTER }
RT_LeftOffset   = $80000004; { signal mask to wait for abort signal }
RT_TopOffset    = $80000005; { topedge offset of requester relative to position
specified by RT_ReqPos }
RT_PubScrName   = $80000006; { name of public screen to put requester on (Kickstart
2.0 only!) }
RT_Screen       = $80000007; { address of screen to put requester on }
RT_ReqHandler   = $80000008; { tagdata must hold the address of (!) an APTR variable }
RT_DefaultFont  = $80000009; { font to use when screen font is rejected, _MUST_ be
fixed-width font! pTextFont , not pTextAttr ) - default GfxBase^.DefaultFont }

RT_WaitPointer  = $8000000A; { boolean to set the standard wait pointer in window -
default FALSE }
RT_Underscore   = $8000000B; { (V38) char preceding keyboard shortcut characters (will
be underlined) }
RT_ShareIDCMP   = $8000000C; { (V38) share IDCMP port with window - default FALSE }
RT_LockWindow   = $8000000D; { (V38) lock window and set standard wait pointer -
default FALSE }
RT_ScreenToFront= $8000000E; { (V38) boolean to make requester's screen pop to front -
default TRUE }
RT_TextAttr     = $8000000F; { (V38) Requester should use this font - default: screen
font }
RT_IntuiMsgFunc = $80000010; { (V38) call this hook for every IDCMP message not for
requester }
RT_Locale       = $80000011; { (V38) Locale ReqTools should use for text }

{ *** tags specific to rtEZRequestA *** }
RTEZ_ReqTitle   = $80000014; { title of requester window - english default "Request"
or "Information" }
                             { ($80000015) reserved }
RTEZ_Flags      = $80000016; { various flags (see below) }
RTEZ_DefaultResponse
                = $80000017; { default response (activated by pressing RETURN) -
default TRUE }


{ *** tags specific to rtGetLongA *** }
RTGL_Min        = $8000001E; { minimum allowed value - default MININT }
RTGL_Max        = $8000001F; { maximum allowed value - default MAXINT }
RTGL_Width      = $80000020; { suggested width of requester window (in pixels) }
RTGL_ShowDefault= $80000021; { boolean to show the default value - default TRUE }
RTGL_GadFmt     = $80000022; { (V38) string with possible responses - english default
" _Ok |_Cancel" }
RTGL_GadFmtArgs = $80000023; { (V38) optional arguments for RTGL_GadFmt }
RTGL_Invisible  = $80000024; { (V38) invisible typing - default FALSE }
RTGL_Backfill   = $80000025; { (V38) window backfill - default TRUE }
RTGL_TextFmt    = $80000026; { (V38) optional text above gadget }
RTGL_TextFmtArgs= $80000027; { (V38) optional arguments for RTGS_TextFmt }
RTGL_Flags = RTEZ_Flags;     { (V38) various flags (see below) }


{ *** tags specific to rtGetStringA *** }
RTGS_Width      = RTGL_Width;       { suggested width of requester window (in pixels)
}
RTGS_AllowEmpty = $80000050;        { allow empty string to be accepted - default
FALSE }
RTGS_GadFmt     = RTGL_GadFmt;      { (V38) string with possible responses - english
default " _Ok |_Cancel" }
RTGS_GadFmtArgs = RTGL_GadFmtArgs;  { (V38) optional arguments for RTGS_GadFmt }
RTGS_Invisible  = RTGL_Invisible;   { (V38) invisible typing - default FALSE }
RTGS_Backfill   = RTGL_Backfill;    { (V38) window backfill - default TRUE }
RTGS_TextFmt    = RTGL_TextFmt;     { (V38) optional text above gadget }
RTGS_TextFmtArgs= RTGL_TextFmtArgs; { (V38) optional arguments for RTGS_TextFmt }
RTGS_Flags      = RTEZ_Flags;       { (V38) various flags (see below) }


{ *** tags specific to rtFileRequestA *** }
RTFI_Flags      = $80000028; { various flags (see below) }
RTFI_Height     = $80000029; { suggested height of file requester }
RTFI_OkText     = $8000002A; { replacement text for 'Ok' gadget (max 6 chars) }
RTFI_VolumeRequest=$8000002B;{ (V38) bring up volume requester, tag data holds flags
(see below) }
RTFI_FilterFunc = $8000002C; { (V38) call this hook for every file in the directory }
RTFI_AllowEmpty = $8000002D; { (V38) allow empty file to be accepted - default FALSE }


{ *** tags specific to rtFontRequestA *** }
RTFO_Flags      = RTFI_Flags;  { various flags (see below) }
RTFO_Height     = RTFI_Height; { suggested height of font requester }
RTFO_OkText     = RTFI_OkText; { replacement text for 'Ok' gadget (max 6 chars) }
RTFO_SampleHeight=$8000003C;   { suggested height of font sample display - default 24
}
RTFO_MinHeight  = $8000003D;   { minimum height of font displayed }
RTFO_MaxHeight  = $8000003E;   { maximum height of font displayed }
{ [($8000003F) to ($80000042) used below] }
RTFO_FilterFunc = RTFI_FilterFunc;{ (V38) call this hook for every font }


{ *** (V38) tags for rtScreenModeRequestA *** }
RTSC_Flags         = RTFI_Flags;{ various flags (see below) }
RTSC_Height        = RTFI_Height;{ suggested height of screenmode requester }
RTSC_OkText        = RTFI_OkText;{ replacement text for 'Ok' gadget (max 6 chars) }
RTSC_PropertyFlags = $8000005A;{ property flags (see also RTSC_PropertyMask) }
RTSC_PropertyMask  = $8000005B;{ property mask - default all bits in
RTSC_PropertyFlags considered }
RTSC_MinWidth      = $8000005C;{ minimum display width allowed }
RTSC_MaxWidth      = $8000005D;{ maximum display width allowed }
RTSC_MinHeight     = $8000005E;{ minimum display height allowed }
RTSC_MaxHeight     = $8000005F;{ maximum display height allowed }
RTSC_MinDepth      = $80000060;{ minimum display depth allowed }
RTSC_MaxDepth      = $80000061;{ maximum display depth allowed }
RTSC_FilterFunc    = RTFI_FilterFunc;{ call this hook for every display mode id }


{ *** tags for rtChangeReqAttrA *** }
RTFI_Dir = $80000032;{ file requester - set directory }
RTFI_MatchPat = $80000033;{ file requester - set wildcard pattern }
RTFI_AddEntry = $80000034;{ file requester - add a file or directory to the buffer }
RTFI_RemoveEntry = $80000035;{ file requester - remove a file or directory from the
buffer }
RTFO_FontName = $8000003F;{ font requester - set font name of selected font }
RTFO_FontHeight = $80000040;{ font requester - set font size }
RTFO_FontStyle = $80000041;{ font requester - set font style }
RTFO_FontFlags = $80000042;{ font requester - set font flags }
RTSC_ModeFromScreen = $80000050;{ (V38) screenmode requester - get display attributes
from screen }
RTSC_DisplayID = $80000051;{ (V38) screenmode requester - set display mode id (32-bit
extended) }
RTSC_DisplayWidth = $80000052;{ (V38) screenmode requester - set display width }
RTSC_DisplayHeight = $80000053;{ (V38) screenmode requester - set display height }
RTSC_DisplayDepth = $80000054;{ (V38) screenmode requester - set display depth }
RTSC_OverscanType = $80000055;{ (V38) screenmode requester - set overscan type, 0 for
regular size }
RTSC_AutoScroll = $80000056;{ (V38) screenmode requester - set autoscroll }


{ *** tags for rtPaletteRequestA *** }
{ initially selected color - default 1 }
    RTPA_Color = $80000046;

{ *** tags for rtReqHandlerA *** }
{ end requester by software control, set tagdata to REQ_CANCEL, REQ_OK or
  in case of rtEZRequest to the return value }
    RTRH_EndRequest = $800003C;

{ *** tags for rtAllocRequestA *** }
{ no tags defined yet }

{************
* RT_ReqPos *
************}
    REQPOS_POINTER    = 0;
    REQPOS_CENTERWIN  = 1;
    REQPOS_CENTERSCR  = 2;
    REQPOS_TOPLEFTWIN = 3;
    REQPOS_TOPLEFTSCR = 4;

{******************
* RTRH_EndRequest *
******************}
    REQ_CANCEL = 0;
    REQ_OK     = 1;

{***************************************
* flags for RTFI_Flags and RTFO_Flags  *
* or filereq->Flags and fontreq->Flags *
***************************************}
    FREQB_NOBUFFER = 2;
    FREQF_NOBUFFER = 4;

{*****************************************
* flags for RTFI_Flags or filereq->Flags *
*****************************************}
    FREQB_MULTISELECT = 0;
    FREQF_MULTISELECT = 1;
    FREQB_SAVE        = 1;
    FREQF_SAVE        = 2;
    FREQB_NOFILES     = 3;
    FREQF_NOFILES     = 8;
    FREQB_PATGAD      = 4;
    FREQF_PATGAD      = 16;
    FREQB_SELECTDIRS  = 12;
    FREQF_SELECTDIRS  = 4096;

{*****************************************
* flags for RTFO_Flags or fontreq->Flags *
*****************************************}
    FREQB_FIXEDWIDTH    = 5;
    FREQF_FIXEDWIDTH    = 32;
    FREQB_COLORFONTS    = 6;
    FREQF_COLORFONTS    = 64;
    FREQB_CHANGEPALETTE = 7;
    FREQF_CHANGEPALETTE = 128;
    FREQB_LEAVEPALETTE  = 8;
    FREQF_LEAVEPALETTE  = 256;
    FREQB_SCALE         = 9;
    FREQF_SCALE         = 512;
    FREQB_STYLE         = 10;
    FREQF_STYLE         = 1024;

{*****************************************************
* (V38) flags for RTSC_Flags or screenmodereq->Flags *
*****************************************************}
    SCREQB_SIZEGADS      = 13;
    SCREQF_SIZEGADS      = 8192;
    SCREQB_DEPTHGAD      = 14;
    SCREQF_DEPTHGAD      = 16384;
    SCREQB_NONSTDMODES   = 15;
    SCREQF_NONSTDMODES   = 32768;
    SCREQB_GUIMODES      = 16;
    SCREQF_GUIMODES      = 65536;
    SCREQB_AUTOSCROLLGAD = 18;
    SCREQF_AUTOSCROLLGAD = 262144;
    SCREQB_OVERSCANGAD   = 19;
    SCREQF_OVERSCANGAD   = 524288;

{*****************************************
* flags for RTEZ_Flags or reqinfo->Flags *
*****************************************}
    EZREQB_NORETURNKEY = 0;
    EZREQF_NORETURNKEY = 1;
    EZREQB_LAMIGAQUAL  = 1;
    EZREQF_LAMIGAQUAL  = 2;
    EZREQB_CENTERTEXT  = 2;
    EZREQF_CENTERTEXT  = 4;

{***********************************************
* (V38) flags for RTGL_Flags or reqinfo->Flags *
***********************************************}
    GLREQB_CENTERTEXT    = EZREQB_CENTERTEXT;
    GLREQF_CENTERTEXT    = EZREQF_CENTERTEXT;
    GLREQB_HIGHLIGHTTEXT = 3;
    GLREQF_HIGHLIGHTTEXT = 8;

{***********************************************
* (V38) flags for RTGS_Flags or reqinfo->Flags *
***********************************************}
    GSREQB_CENTERTEXT    = EZREQB_CENTERTEXT;
    GSREQF_CENTERTEXT    = EZREQF_CENTERTEXT;
    GSREQB_HIGHLIGHTTEXT = GLREQB_HIGHLIGHTTEXT;
    GSREQF_HIGHLIGHTTEXT = GLREQF_HIGHLIGHTTEXT;

{*****************************************
* (V38) flags for RTFI_VolumeRequest tag *
*****************************************}
    VREQB_NOASSIGNS   = 0;
    VREQF_NOASSIGNS   = 1;
    VREQB_NODISKS     = 1;
    VREQF_NODISKS     = 2;
    VREQB_ALLDISKS    = 2;
    VREQF_ALLDISKS    = 4;

{*
   Following things are obsolete in ReqTools V38.
   DON'T USE THESE IN NEW CODE!
*}
    REQHOOK_WILDFILE  = 0;
    REQHOOK_WILDFONT  = 1;
    FREQB_DOWILDFUNC  = 11;
    FREQF_DOWILDFUNC  = 2048;

VAR ReqToolsBase : pLibrary;


FUNCTION rtAllocRequestA(typ : ULONG; taglist : pTagItem) : POINTER;
FUNCTION rtChangeReqAttrA(req : POINTER; taglist : pTagItem) : LONGINT;
PROCEDURE rtCloseWindowSafely(win : pWindow);
FUNCTION rtEZRequestA(bodyfmt : PChar; gadfmt : PChar; reqinfo : prtReqInfo; argarray : POINTER; taglist : pTagItem) : ULONG;
FUNCTION rtFileRequestA(filereq : prtFileRequester; filebuf : PChar; title : PChar; taglist : pTagItem) : POINTER;
FUNCTION rtFontRequestA(fontreq : prtFontRequester; title : PChar; taglist : pTagItem) : ULONG;
PROCEDURE rtFreeFileList(filelist : prtFileList);
PROCEDURE rtFreeReqBuffer(req : POINTER);
PROCEDURE rtFreeRequest(req : POINTER);
FUNCTION rtGetLongA(VAR longptr : ULONG; title : PChar; reqinfo : prtReqInfo; taglist : pTagItem) : ULONG;
FUNCTION rtGetStringA(buffer : pCHAR; maxchars : ULONG; title : PChar; reqinfo : prtReqInfo; taglist : pTagItem) : ULONG;
FUNCTION rtGetVScreenSize(screen : pScreen; VAR widthptr : ULONG; VAR heightptr : ULONG) : ULONG;
FUNCTION rtLockWindow(win : pWindow) : POINTER;
FUNCTION rtPaletteRequestA(title : PChar; reqinfo : prtReqInfo; taglist : pTagItem) : LONGINT;
FUNCTION rtReqHandlerA(handlerinfo : prtHandlerInfo; sigs : ULONG; taglist : pTagItem) : ULONG;
FUNCTION rtScreenModeRequestA(screenmodereq : prtScreenModeRequester; title : PChar; taglist : pTagItem) : ULONG;
PROCEDURE rtScreenToFrontSafely(screen : pScreen);
PROCEDURE rtSetReqPosition(reqpos : ULONG; newwindow : pNewWindow; screen : pScreen; window : pWindow);
PROCEDURE rtSetWaitPointer(window : pWindow);
PROCEDURE rtSpread(VAR posarray : ULONG; VAR sizearray : ULONG; length : ULONG; min : ULONG; max : ULONG; num : ULONG);
PROCEDURE rtUnlockWindow(win : pWindow; winlock : POINTER);

{
     This is functions and procedures with array of const.
     For use with fpc 1.0 and above.
}

FUNCTION rtAllocRequest(typ : ULONG; const argv : Array Of Const) : POINTER;
FUNCTION rtChangeReqAttr(req : POINTER; const argv : Array Of Const) : LONGINT;
FUNCTION rtEZRequest(bodyfmt : PChar; gadfmt : PChar; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
FUNCTION rtFileRequest(filereq : prtFileRequester; filebuf : PChar; title : PChar; const argv : Array Of Const) : POINTER;
FUNCTION rtFontRequest(fontreq : prtFontRequester; title : PChar; const argv : Array Of Const) : ULONG;
FUNCTION rtGetLong(VAR longptr : ULONG; title : PChar; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
FUNCTION rtGetString(buffer : pCHAR; maxchars : ULONG; title : PChar; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
FUNCTION rtPaletteRequest(title : PChar; reqinfo : prtReqInfo; const argv : Array Of Const) : LONGINT;
FUNCTION rtReqHandler(handlerinfo : prtHandlerInfo; sigs : ULONG; const argv : Array Of Const) : ULONG;
FUNCTION rtScreenModeRequest(screenmodereq : prtScreenModeRequester; title : PChar; const argv : Array Of Const) : ULONG;

{overlay functions}

FUNCTION rtEZRequestA(bodyfmt : PChar; gadfmt : String; reqinfo : prtReqInfo; argarray : POINTER; taglist : pTagItem) : ULONG;
FUNCTION rtEZRequestA(bodyfmt : String; gadfmt : PChar; reqinfo : prtReqInfo; argarray : POINTER; taglist : pTagItem) : ULONG;
FUNCTION rtEZRequestA(bodyfmt : String; gadfmt : String; reqinfo : prtReqInfo; argarray : POINTER; taglist : pTagItem) : ULONG;
FUNCTION rtFileRequestA(filereq : prtFileRequester; filebuf : PChar; title : String; taglist : pTagItem) : POINTER;
FUNCTION rtFontRequestA(fontreq : prtFontRequester; title : String; taglist : pTagItem) : ULONG;
FUNCTION rtGetLongA(VAR longptr : ULONG; title : String; reqinfo : prtReqInfo; taglist : pTagItem) : ULONG;
FUNCTION rtGetStringA(buffer : pCHAR; maxchars : ULONG; title : String; reqinfo : prtReqInfo; taglist : pTagItem) : ULONG;
FUNCTION rtPaletteRequestA(title : String; reqinfo : prtReqInfo; taglist : pTagItem) : LONGINT;
FUNCTION rtScreenModeRequestA(screenmodereq : prtScreenModeRequester; title : String; taglist : pTagItem) : ULONG;

{
     This is functions and procedures with array of const.
     For use with fpc 1.0 and above. + overlay
}

FUNCTION rtEZRequest(bodyfmt : PChar; gadfmt : String; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
FUNCTION rtEZRequest(bodyfmt : String; gadfmt : PChar; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
FUNCTION rtEZRequest(bodyfmt : String; gadfmt : String; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
FUNCTION rtFileRequest(filereq : prtFileRequester; filebuf : PChar; title : String; const argv : Array Of Const) : POINTER;
FUNCTION rtFontRequest(fontreq : prtFontRequester; title : String; const argv : Array Of Const) : ULONG;
FUNCTION rtGetLong(VAR longptr : ULONG; title : String; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
FUNCTION rtGetString(buffer : pCHAR; maxchars : ULONG; title : String; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
FUNCTION rtPaletteRequest(title : String; reqinfo : prtReqInfo; const argv : Array Of Const) : LONGINT;
FUNCTION rtScreenModeRequest(screenmodereq : prtScreenModeRequester; title : String; const argv : Array Of Const) : ULONG;


{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitREQTOOLSLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    REQTOOLSIsCompiledHow : longint;


IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
tagsarray,pastoc;

FUNCTION rtAllocRequestA(typ : ULONG; taglist : pTagItem) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  typ,D0
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtChangeReqAttrA(req : POINTER; taglist : pTagItem) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L req,A1
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE rtCloseWindowSafely(win : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L ReqToolsBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION rtEZRequestA(bodyfmt : PChar; gadfmt : PChar; reqinfo : prtReqInfo; argarray
: POINTER; taglist : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L bodyfmt,A1
    MOVEA.L gadfmt,A2
    MOVEA.L reqinfo,A3
    MOVEA.L argarray,A4
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtFileRequestA(filereq : prtFileRequester; filebuf : PChar; title : PChar;
taglist : pTagItem) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L filereq,A1
    MOVEA.L filebuf,A2
    MOVEA.L title,A3
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtFontRequestA(fontreq : prtFontRequester; title : PChar; taglist : pTagItem)
: ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L fontreq,A1
    MOVEA.L title,A3
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE rtFreeFileList(filelist : prtFileList);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L filelist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE rtFreeReqBuffer(req : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L req,A1
    MOVEA.L ReqToolsBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE rtFreeRequest(req : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L req,A1
    MOVEA.L ReqToolsBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION rtGetLongA(VAR longptr : ULONG; title : PChar; reqinfo : prtReqInfo; taglist
: pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L longptr,A1
    MOVEA.L title,A2
    MOVEA.L reqinfo,A3
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;


FUNCTION rtGetStringA(buffer : pCHAR; maxchars : ULONG; title : PChar; reqinfo :
prtReqInfo; taglist : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L buffer,A1
    MOVE.L  maxchars,D0
    MOVEA.L title,A2
    MOVEA.L reqinfo,A3
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtGetVScreenSize(screen : pScreen; VAR widthptr : ULONG; VAR heightptr :
ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L widthptr,A1
    MOVEA.L heightptr,A2
    MOVEA.L ReqToolsBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtLockWindow(win : pWindow) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L ReqToolsBase,A6
    JSR -156(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtPaletteRequestA(title : PChar; reqinfo : prtReqInfo; taglist : pTagItem) :
LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L title,A2
    MOVEA.L reqinfo,A3
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtReqHandlerA(handlerinfo : prtHandlerInfo; sigs : ULONG; taglist : pTagItem)
: ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L handlerinfo,A1
    MOVE.L  sigs,D0
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION rtScreenModeRequestA(screenmodereq : prtScreenModeRequester; title : PChar;
taglist : pTagItem) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screenmodereq,A1
    MOVEA.L title,A3
    MOVEA.L taglist,A0
    MOVEA.L ReqToolsBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE rtScreenToFrontSafely(screen : pScreen);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L screen,A0
    MOVEA.L ReqToolsBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE rtSetReqPosition(reqpos : ULONG; newwindow : pNewWindow; screen : pScreen;
window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  reqpos,D0
    MOVEA.L newwindow,A0
    MOVEA.L screen,A1
    MOVEA.L window,A2
    MOVEA.L ReqToolsBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE rtSetWaitPointer(window : pWindow);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L window,A0
    MOVEA.L ReqToolsBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE rtSpread(VAR posarray : ULONG; VAR sizearray : ULONG; length : ULONG; min :
ULONG; max : ULONG; num : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L posarray,A0
    MOVEA.L sizearray,A1
    MOVE.L  length,D0
    MOVE.L  min,D1
    MOVE.L  max,D2
    MOVE.L  num,D3
    MOVEA.L ReqToolsBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE rtUnlockWindow(win : pWindow; winlock : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L win,A0
    MOVEA.L winlock,A1
    MOVEA.L ReqToolsBase,A6
    JSR -162(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION rtEZRequestA(bodyfmt : PChar; gadfmt : String; reqinfo : prtReqInfo; argarray
: POINTER; taglist : pTagItem) : ULONG;
BEGIN
    rtEZRequestA := rtEZRequestA(bodyfmt,pas2c(gadfmt),reqinfo,argarray,taglist);
END;

FUNCTION rtEZRequestA(bodyfmt : String; gadfmt : PChar; reqinfo : prtReqInfo; argarray
: POINTER; taglist : pTagItem) : ULONG;
BEGIN
    rtEZRequestA := rtEZRequestA(pas2c(bodyfmt),gadfmt,reqinfo,argarray,taglist);
END;

FUNCTION rtEZRequestA(bodyfmt : String; gadfmt : String; reqinfo : prtReqInfo;
argarray : POINTER; taglist : pTagItem) : ULONG;
BEGIN
    rtEZRequestA :=
rtEZRequestA(pas2c(bodyfmt),pas2c(gadfmt),reqinfo,argarray,taglist);
END;

FUNCTION rtFileRequestA(filereq : prtFileRequester; filebuf : PChar; title : String;
taglist : pTagItem) : POINTER;
BEGIN
    rtFileRequestA := rtFileRequestA(filereq,filebuf,pas2c(title),taglist);
END;

FUNCTION rtFontRequestA(fontreq : prtFontRequester; title : String; taglist :
pTagItem) : ULONG;
BEGIN
    rtFontRequestA := rtFontRequestA(fontreq,pas2c(title),taglist);
END;

FUNCTION rtGetLongA(VAR longptr : ULONG; title : String; reqinfo : prtReqInfo; taglist
: pTagItem) : ULONG;
BEGIN
    rtGetLongA := rtGetLongA(longptr,pas2c(title),reqinfo,taglist);
END;

FUNCTION rtGetStringA(buffer : pCHAR; maxchars : ULONG; title : String; reqinfo :
prtReqInfo; taglist : pTagItem) : ULONG;
BEGIN
    rtGetStringA := rtGetStringA(buffer,maxchars,pas2c(title),reqinfo,taglist);
END;

FUNCTION rtPaletteRequestA(title : String; reqinfo : prtReqInfo; taglist : pTagItem) :
LONGINT;
BEGIN
    rtPaletteRequestA := rtPaletteRequestA(pas2c(title),reqinfo,taglist);
END;

FUNCTION rtScreenModeRequestA(screenmodereq : prtScreenModeRequester; title : String;
taglist : pTagItem) : ULONG;
BEGIN
    rtScreenModeRequestA := rtScreenModeRequestA(screenmodereq,pas2c(title),taglist);
END;


FUNCTION rtAllocRequest(typ : ULONG; const argv : Array Of Const) : POINTER;
begin
    rtAllocRequest := rtAllocRequestA(typ,readintags(argv));
end;

FUNCTION rtChangeReqAttr(req : POINTER; const argv : Array Of Const) : LONGINT;
begin
    rtChangeReqAttr := rtChangeReqAttrA(req,readintags(argv));
end;

FUNCTION rtEZRequest(bodyfmt : PChar; gadfmt : PChar; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
begin
    rtEZRequest := rtEZRequestA(bodyfmt,gadfmt,reqinfo,argarray,readintags(argv));
end;

FUNCTION rtFileRequest(filereq : prtFileRequester; filebuf : PChar; title : PChar; const argv : Array Of Const) : POINTER;
begin
    rtFileRequest := rtFileRequestA(filereq,filebuf,title,readintags(argv));
end;

FUNCTION rtFontRequest(fontreq : prtFontRequester; title : PChar; const argv : Array Of Const) : ULONG;
begin
    rtFontRequest := rtFontRequestA(fontreq,title,readintags(argv));
end;

FUNCTION rtGetLong(VAR longptr : ULONG; title : PChar; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
begin
    rtGetLong := rtGetLongA(longptr,title,reqinfo,readintags(argv));
end;

FUNCTION rtGetString(buffer : pCHAR; maxchars : ULONG; title : PChar; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
begin
    rtGetString := rtGetStringA(buffer,maxchars,title,reqinfo,readintags(argv));
end;

FUNCTION rtPaletteRequest(title : PChar; reqinfo : prtReqInfo; const argv : Array Of Const) : LONGINT;
begin
    rtPaletteRequest := rtPaletteRequestA(title,reqinfo,readintags(argv));
end;

FUNCTION rtReqHandler(handlerinfo : prtHandlerInfo; sigs : ULONG; const argv : Array Of Const) : ULONG;
begin
    rtReqHandler := rtReqHandlerA(handlerinfo,sigs,readintags(argv));
end;

FUNCTION rtScreenModeRequest(screenmodereq : prtScreenModeRequester; title : PChar; const argv : Array Of Const) : ULONG;
begin
    rtScreenModeRequest := rtScreenModeRequestA(screenmodereq,title,readintags(argv));
end;

FUNCTION rtEZRequest(bodyfmt : PChar; gadfmt : String; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
begin
    rtEZRequest := rtEZRequestA(bodyfmt,gadfmt,reqinfo,argarray,readintags(argv));
end;

FUNCTION rtEZRequest(bodyfmt : String; gadfmt : PChar; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
begin
    rtEZRequest := rtEZRequestA(bodyfmt,gadfmt,reqinfo,argarray,readintags(argv));
end;

FUNCTION rtEZRequest(bodyfmt : String; gadfmt : String; reqinfo : prtReqInfo; argarray : POINTER; const argv : Array Of Const) : ULONG;
begin
    rtEZRequest := rtEZRequestA(bodyfmt,gadfmt,reqinfo,argarray,readintags(argv));
end;

FUNCTION rtFileRequest(filereq : prtFileRequester; filebuf : PChar; title : String; const argv : Array Of Const) : POINTER;
begin
     rtFileRequest :=  rtFileRequestA(filereq,filebuf,title,readintags(argv));
end;

FUNCTION rtFontRequest(fontreq : prtFontRequester; title : String; const argv : Array Of Const) : ULONG;
begin
     rtFontRequest := rtFontRequestA(fontreq,title,readintags(argv));
end;

FUNCTION rtGetLong(VAR longptr : ULONG; title : String; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
begin
     rtGetLong := rtGetLongA(longptr,title,reqinfo,readintags(argv));
end;

FUNCTION rtGetString(buffer : pCHAR; maxchars : ULONG; title : String; reqinfo : prtReqInfo; const argv : Array Of Const) : ULONG;
begin
     rtGetString := rtGetStringA(buffer,maxchars,title,reqinfo,readintags(argv));
end;

FUNCTION rtPaletteRequest(title : String; reqinfo : prtReqInfo; const argv : Array Of Const) : LONGINT;
begin
     rtPaletteRequest := rtPaletteRequestA(title,reqinfo,readintags(argv));
end;

FUNCTION rtScreenModeRequest(screenmodereq : prtScreenModeRequester; title : String; const argv : Array Of Const) : ULONG;
begin
     rtScreenModeRequest := rtScreenModeRequestA(screenmodereq,title,readintags(argv));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of reqtools.library}
  {$Info don't forget to use InitREQTOOLSLibrary in the beginning of your program}

var
    reqtools_exit : Pointer;

procedure ClosereqtoolsLibrary;
begin
    ExitProc := reqtools_exit;
    if ReqToolsBase <> nil then begin
        CloseLibrary(ReqToolsBase);
        ReqToolsBase := nil;
    end;
end;

procedure InitREQTOOLSLibrary;
begin
    ReqToolsBase := nil;
    ReqToolsBase := OpenLibrary(REQTOOLSNAME,LIBVERSION);
    if ReqToolsBase <> nil then begin
        reqtools_exit := ExitProc;
        ExitProc := @ClosereqtoolsLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open reqtools.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    REQTOOLSIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of reqtools.library}

var
    reqtools_exit : Pointer;

procedure ClosereqtoolsLibrary;
begin
    ExitProc := reqtools_exit;
    if ReqToolsBase <> nil then begin
        CloseLibrary(ReqToolsBase);
        ReqToolsBase := nil;
    end;
end;

begin
    ReqToolsBase := nil;
    ReqToolsBase := OpenLibrary(REQTOOLSNAME,LIBVERSION);
    if ReqToolsBase <> nil then begin
        reqtools_exit := ExitProc;
        ExitProc := @ClosereqtoolsLibrary;
        REQTOOLSIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open reqtools.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    REQTOOLSIsCompiledHow := 3;
   {$Warning No autoopening of reqtools.library compiled}
   {$Warning Make sure you open reqtools.library yourself}
{$endif dont_use_openlib}


END. (* UNIT REQTOOLS *)



