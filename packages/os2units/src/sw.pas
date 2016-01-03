{
    Copyright (c) 1990-1991 International Business Machines Corporation
    Copyright (c) 2003 by Yuri Prokushev (prokushev@freemail.ru)

    This is the multimedia unit file that has the typedefs, defines and
    function prototypes for Multimedia Applets.

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU Library General Public License (LGPL) as
    published by the Free Software Foundation; either version 2 of the
    License, or (at your option) any later version. This program is
    distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.

    See the GNU Library General Public License for more details. You should
    have received a copy of the GNU Library General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 **********************************************************************}

{
@abstract(OS/2 Multimedia Applets)
@author(Yuri Prokushev (prokushev@freemail.ru))
@created(19 Jan 2003)
@lastmod(23 Jan 2003)
This is the multimedia unit file that has the typedefs, defines and
function prototypes for Multimedia Applets.
Warning: This code is alfa. Future versions of this unit will propably
not be compatible.
}
Unit SW;

Interface

Uses
  Os2Def,
  PmWin;

Const
  MAX_SMBDTEXT=35;

  MB_ICONCUSTOM=$0001;

  SC_DEFAULTSIZE=$c000;          // WM_COMMAND from SysMenu

  WM_INITSECONDARYWINDOW=$0519;  // MP1: NULL, MP2: CreateParams

  QS_FRAME=$1;             // Flag set to query frame
  QS_DIALOG=$2;            // Flag set to query dialog

//      #pragma pack(4)

Type
  TSMBD=record
    achText: Array[0..MAX_SMBDTEXT + 1] of Char; // Text of the button. eg. "~Cancel"
    idButton: Cardinal;        // Button ID returned when user chooses
    flStyle: Longint;          // Button style or'ed with internal
  end;
  PSMBD=^TSMBD;

  TSMBINFO=record
    hIcon: Cardinal;           // Icon handle
    cButtons: Cardinal;        // Number of buttons
    flStyle: Cardinal;         // Icon style flags (MB_ICONQUESTION, etc...)
    hwndNotify: HWND;          // Reserved
    smbd: PSMBD;               // Array of button definitions
  end;
  PSMBINFO=^TSMBINFO;

//      #pragma pack()

// Analogous to WinDlgBox
function WinSecondaryWindow(hwndParent: hwnd; hwndOwner: hwnd;
  pfnDlgProc: proc; hmod: Cardinal; idDlg: Cardinal; pCreateParams: Pointer): Cardinal; cdecl;

// Analogous to WinLoadDlg
function WinLoadSecondaryWindow(hwndParent: hwnd; hwndOwner: hwnd;
  pfnDlgProc: proc; hmod: Cardinal; idDlg: Cardinal; pCreateParams: Cardinal): hwnd; cdecl;

// Analogous to WinProcessDlg
function WinProcessSecondaryWindow(hwndSW: hwnd): Cardinal; cdecl;

// Analogous to WinCreateDlg
function WinCreateSecondaryWindow(hwndParent: hwnd; hwndOwner: hwnd;
  pfnDlgProc: proc; idDlg: Cardinal; pCreateParams: Pointer): hwnd; cdecl;

function WinDefaultSize(Wnd: hwnd): Longbool; cdecl;

function WinInsertDefaultSize(Wnd: hwnd; pszDefaultSize: pChar): Longbool; cdecl;

function WinQuerySecondaryhwnd(Wnd: hwnd; ulFlag: Cardinal): hwnd; cdecl;

//************************************************************************/
//* WinSecondaryMessageBox                                               */
//*                                                                      */
//* Parameters: HWND   hwndParent   - handle of the parent window.       */
//*             HWND   hwndOwner    - handle of the owner window.        */
//*             PSZ    pszText      - message text.                      */
//*             PSZ    pszCaption   - title of the message box.          */
//*             ULONG  idWindow     - Message box id                     */
//*             PSMBINFO psmbinfo   - pointer to button/icon info        */
//************************************************************************/
function WinSecondaryMessageBox(hwndParent: hwnd; hwndOwner: hwnd;
  pszText: pChar; pszCaption: pChar; idWindow: Cardinal; smb: psmbinfo): Cardinal; cdecl;

//************************************************************************/
//* WinDismissSecondaryWindow                                            */
//*                                                                      */
//* This function should be called from within the dlg proc. The hwnd    */
//* passed in MUST be the handle to the actual dialog.                   */
//************************************************************************/

function WinDismissSecondaryWindow(hwndDlg: hwnd; ulResult: Cardinal): Longbool; cdecl;

//************************************************************************/
//* The parameter hwnd can be either the secondary window or the actual  */
//* dialog.                                                              */
//************************************************************************/

function WinDestroySecondaryWindow(Wnd: hwnd): Longbool; cdecl;

function WinDefSecondaryWindowProc(Wnd: hwnd; msg: Cardinal;
                                 mp1: mParam; mp2: mParam): mResult; cdecl;

//************************************************************************/
//*                         Graphic Buttons                              */
//************************************************************************/

//************************************************************************/
//* Notes on Using GraphicButtons                                        */
//*                                                                      */
//* GraphicButton CONTROL DATA                                           */
//*                                                                      */
//*  The format of the control data for GraphicButtons is                */
//*         "button Text, number of bitmaps, bitmap resource id's ..."   */
//*                                                                      */
//*                                                                      */
//*  Following are two example window templates of GraphicButtons:       */
//*                                                                      */
//*      1)  CONTROL  "", IDD_MP_REV, 120, 10, TS_PB_WIDTH, TS_PB_HEIGHT,*/
//*                 WC_GRAPHICBUTTON,                                    */
//*                 GBS_TWOSTATE | GBS_HILITEBITMAP |                    */
//*                 WS_VISIBLE | WS_TABSTOP                              */
//*                 CTLDATA GB_RESOURCE,"~REV", 2, ID_MP_REV1, ID_MP_REV0*/
//*                                                                      */
//*  The above graphicbutton has id IDD_MP_REV and is of type            */
//*  GBS_TWOSTATE and GBS_HILITEBITMAP.  The GBS_HILITEBITMAP allows     */
//*  a different bitmap to be displayed when the button is in the        */
//*  hilite state. The graphicbutton will be displayed with text         */
//*  "REV" and has "R" as the mnemonic.  It has 2 bitmaps associated     */
//*  with it.  Their resource id are ID_MP_REV1 and ID_MP_REV0.          */
//*                                                                      */
//*                                                                      */
//*      2)   CONTROL  "", IDD_MP_PLAY, 175, 10, TS_PB_WIDTH,            */
//*                        TS_PB_HEIGHT, WC_GRAPHICBUTTON,               */
//*                        GBS_AUTOTWOSTATE | GBS_AUTOANIMATION |        */
//*                        WS_VISIBLE | WS_TABSTOP                       */
//*                        CTLDATA GB_RESOURCE, "~PLAY", 9,              */
//*                                ID_MP_STOP0, ID_MP_PLAY1, ID_MP_PLAY2,*/
//*                                ID_MP_PLAY3, ID_MP_PLAY4, ID_MP_PLAY5,*/
//*                                ID_MP_PLAY6, ID_MP_PLAY7, ID_MP_REV1  */
//*                                                                      */
//*  The above graphicbutton has id IDD_MP_PLAY and is of type           */
//*  GBS_AUTOTWOSTATE and GBS_AUTOANIMATE.  When clicked upon,           */
//*  the button will automatically toggle the state and animation.       */
//*  The graphicbutton will be displayed with text "PLAY"                */
//*  and mnemonic "P".  It has 9 bitmaps associated with it.             */
//*                                                                      */
//*                                                                      */
//*                                                                      */
//*  GraphicButton Painting                                              */
//*                                                                      */
//*  Due to the PM design, whenever a graphicbutton is clicked,          */
//*  it is sent a BN_PAINT (to paint a non-hilite state) and then        */
//*  BN_CLICKED.  Thus, for GBS_AUTO* style graphicbuttons, a paint      */
//*  request is generated before the button has a chance to change       */
//*  its state (BN_CLICKED).  To avoid this premature painting,          */
//*  code was inserted to delay the painting of graphicbuttons           */
//*  GB_PAINT_RESOLUTION milliseconds whenever the button is switching   */
//*  FROM the hilite paint state.                                        */
//*                                                                      */
//************************************************************************/

Function WinRegisterGraphicButton: Longbool; cdecl;

Const
  WC_GRAPHICBUTTON=PChar($ffff0040);

//************************************************************************/
//*             GraphicButton Animation/TwoState constants               */
//************************************************************************/

//************************************************************************/
//*                     Graphic Button Style bits                        */
//************************************************************************/

  GBS_TWOSTATE                  = $1000;       // indicates TwoState button
  GBS_AUTOTWOSTATE              = $2000;       // will auto toggle state up/down
  GBS_ANIMATION                 = $4000;       // indicates Animatable button
  GBS_AUTOANIMATION             = $8000;       // will auto toggle anim. on/off
  GBS_DISABLEBITMAP             = $0010;       // allows a diff. bitmap when disabled
  GBS_HILITEBITMAP              = $0020;       // allows a diff. bitmap when hilited
  GBS_3D_TEXTRECESSED           = $0040;       // display text in 3-D recessed
  GBS_3D_TEXTRAISED             = $0080;       // display text in 3-D raised
  GBS_MINIBUTTON                = $0001;       // mini button style

//************************************************************************/
//*                     Graphic Button User Constants                    */
//************************************************************************/

//************************************************************************/
//* Codes to reference allowed GraphicButton states (or paint states)    */
//************************************************************************/

  GB_ERROR                      = -1;          // GraphicButton Error
  GB_UP                         = 1;           // GraphicButton up (and paint) state
  GB_DOWN                       = 2;           // GraphicButton down (and paint) state
  GB_DISABLE                    = 3;           // GraphicButton disabled state
  GB_HILITE                     = 4;           // GraphicButton paint state only
  GB_OUTOFHILITE                = 5;           // Changing out of hilite paint state

//************************************************************************/
//* Codes for various GraphicButton message function parameters          */
//************************************************************************/

  GB_TOGGLE                     = 10;          // GraphicButton toggle
  GB_CURRENTSTATE               = 11;          // GraphicButton's current state
  GB_ANIMATIONBEGIN             = 12;          // when refering to index of anim start
  GB_ANIMATIONEND               = 13;          // when refering to index of anim end
  GB_MAXINDEX                   = 14;          // GraphicButton max. index

//************************************************************************/
//* Codes to set/query text position relative to the bitmap              */
//************************************************************************/

  GB_TEXTBELOW                  = 1;           // place text below bitmap
  GB_TEXTABOVE                  = 2;           // place text above bitmap

//************************************************************************/
//* Codes used to set the animation frame with message GBM_SETBITMAPINDEX*/
//************************************************************************/

  GB_INDEX_FORWARD              = -1;          // advance one frame foward
  GB_INDEX_BACKWARD             = -2;          // advance one frame backwards
  GB_INDEX_FIRST                = -3;          // set to first frame of sequence
  GB_INDEX_LAST                 = -4;          // set to last frame of sequence

//  #pragma pack(1)

type
  TgbtnCdata = packed record
    usReserved: Word;
    pszText: PChar;
    hmod: Cardinal;
    cBitmaps: Word;
    aidBitmap: Array[0..1] of Word;
  end;
  pgbtnCdata = ^TgbtnCdata;

//#pragma pack()

const
  GB_RESOURCE                   = 1;
  GB_STRUCTURE                  = 0;


//************************************************************************/
//*          Notification Messages received by GraphicButton Parent      */
//***********************************************************************/
//************************************************************************/
//* GBN_BUTTONDOWN, GBN_BUTTONUP, and GBN_BUTTONHILITE                   */
//*                                                                      */
//* The notification messages are passed as part of the WM_CONTROL       */
//* message.                                                             */
//*                                                                      */
//* msg = WM_CONTROL                                                     */
//* mp1 = MPFROM2SHORT(gpb_id, button_state)                             */
//*         gpd_id       = identity of the displayed graphic pushbutton  */
//*         button_state = GBN_BUTTONUP, GBN_BUTTONDOWN, or              */
//*                        GBN_BUTTONHILITE                              */
//*                                                                      */
//************************************************************************/

  GBN_BUTTONUP                  = $0524;
  GBN_BUTTONDOWN                = $0525;
  GBN_BUTTONHILITE              = $0526;
  GBN_SETFOCUS                  = $0527;   // mp2 TRUE for gaining focus

//************************************************************************/
//*          Messages to GraphicButton Windows                           */
//************************************************************************/

//************************************************************************/
//*                             GBM_SETGRAPHICDATA                       */
//************************************************************************/
//* mp1 = MPFROMP((PGBTNCDATA)&gbtncdata);    Graphic button control data*/
//* mp2 = NULL;                               not used                   */
//*                                                                      */
//* WARNING: This message resets all button parameters.                  */
//*                                                                      */
//************************************************************************/

  GBM_SETGRAPHICDATA            = $052A;

//************************************************************************/
//*                             GBM_ANIMATE                              */
//************************************************************************/
//* mp1 = MPFROMSHORT(fStart)      TRUE to start animation, FALSE to stop*/
//* mp2 = MPFROMSHORT(fContinue)   TRUE continue anim. at currently      */
//*                                displayed bitmap, FALSE restart at    */
//*                                the beginning.                        */
//*                                                                      */
//* Returns TRUE on Success                                              */
//*         FALSE on Failure                                             */
//*                                                                      */
//************************************************************************/

  GBM_ANIMATE                   = $052B;

//************************************************************************/
//*                             GBM_SETANIMATIONRATE                     */
//************************************************************************/
//* mp1 = MPFROMSHORT(ULmIL);      Animation rate in milliseconds        */
//* mp2 = NULL                     not used                              */
//*                                                                      */
//*                                                                      */
//* Returns TRUE on Success                                              */
//*         FALSE on Failure                                             */
//*                                                                      */
//************************************************************************/

  GBM_SETANIMATIONRATE          = $052C;

//************************************************************************/
//*                             GBM_QUERYANIMATIONACTIVE                 */
//************************************************************************/
//* mp1 = NULL;                    not used                              */
//* mp2 = NULL;                    not used                              */
//*                                                                      */
//*                                                                      */
//* Returns TRUE if animation is active, else GB_ERROR                   */
//*                                                                      */
//*                                                                      */
//************************************************************************/

  GBM_QUERYANIMATIONACTIVE      = $052D;

//************************************************************************/
//*                             GBM_QUERYANIMATIONRATE                   */
//************************************************************************/
//* mp1 = NULL;                    not used                              */
//* mp2 = NULL;                    not used                              */
//*                                                                      */
//*                                                                      */
//* Returns ULONG sepcifying animation rate in milliseconds              */
//*                                                                      */
//*                                                                      */
//************************************************************************/

  GBM_QUERYANIMATIONRATE        = $052E;

//************************************************************************/
//*                             GBM_SETBITMAPINDEX                       */
//************************************************************************/
//* mp1 = MPFROMSHORT(usGB_State)       Bitmap index to change           */
//*             GB_UP,                                                   */
//*             GB_DOWN,                                                 */
//*             GB_DISABLE,                                              */
//*             GB_HILITE,                                               */
//*             GB_ANIMATIONBEGIN,                                       */
//*             GB_ANIMATIONEND,                                         */
//*          or GB_CURRENTSTATE    chng current state (up or down) bitmap*/
//* mp2 = MPFROMSHORT(sFrameCode)  Set according to code or frame desire */
//*             GB_INDEX_FORWARD,  chng to next bitmap in circular array */
//*             GB_INDEX_BACKWARD, "   "  prev   "    "     "       "    */
//*             GB_INDEX_FIRST,    "   "  first  "    "     "       "    */
//*             GB_INDEX_LAST,     "   "  last   "    "     "       "    */
//*          or desired_bitmap     otherwise desired bmp index specified */
//*                                                                      */
//* Returns TRUE on Success, otherwise FALSE                             */
//*                                                                      */
//************************************************************************/

  GBM_SETBITMAPINDEX            = $052F;

//************************************************************************/
//*                             GBM_QUERYBITMAPINDEX                     */
//************************************************************************/
//*  mp1 = MPFROMSHORT(usGB_State)       Query bitmap index              */
//*            GB_UP,                                                    */
//*            GB_DOWN,                                                  */
//*            GB_DISABLE,                                               */
//*            GB_HILITE,                                                */
//*            GB_ANIMATIONBEGIN,                                        */
//*            GB_ANIMATIONEND,                                          */
//*         or GB_CURRENTSTATE  query current state (up or down) bitmap  */
//*  mp2 = NULL;                    not used                             */
//*                                                                      */
//*  Returns USHORT specifying the index                                 */
//*                                                                      */
//*                                                                      */
//************************************************************************/

  GBM_QUERYBITMAPINDEX          = $0530;

//************************************************************************/
//*                             GBM_SETSTATE                             */
//************************************************************************/
//* mp1 = MPFROMSHORT(usStateCode)   Set twostate button to specified    */
//*                                  state                               */
//*             GB_UP,                                                   */
//*             GB_DOWN,                                                 */
//*          or GB_TOGGLE          toggle (up/down) to (down/up)         */
//* mp2 = MPFROMBOOL(fRepaint)     TRUE  - state changed and displayed   */
//*                                FALSE - state changed, not displayed  */
//*                                                                      */
//* Returns TRUE on Success                                              */
//*         FALSE on Failure                                             */
//*                                                                      */
//*                                                                      */
//************************************************************************/

  GBM_SETSTATE                  = $0531;

//************************************************************************/
//*                             GBM_QUERYSTATE                           */
//************************************************************************/
//*                                                                      */
//* mp1 = NULL                        not used                           */
//* mp2 = NULL                        not used                           */
//*                                                                      */
//* Returns the state (GB_UP or GB_DOWN) else GB_ERROR.                  */
//*                                                                      */
//*                                                                      */
//************************************************************************/

  GBM_QUERYSTATE                = $0532;

//************************************************************************/
//*                             GBM_SETTEXTPOSITION                      */
//************************************************************************/
//* mp1 = MPFROMSHORT(usTextPos)   How to position text relative to      */
//*                                  bitmap                              */
//*               GB_TEXTBELOW,                                          */
//*            or GB_TEXTABOVE                                           */
//*   mp2 = NULL                     not used                            */
//*                                                                      */
//*   Returns TRUE on Success, otherwise FALSE                           */
//*                                                                      */
//************************************************************************/

  GBM_SETTEXTPOSITION           = $0533;

//************************************************************************/
//*                             GBM_QUERYTEXTPOSITION                    */
//************************************************************************/
//*                                                                      */
//*  mp1 = NULL                          not used                        */
//*  mp2 = NULL                          not used                        */
//*                                                                      */
//*  Returns GB_TEXTBELOW, GB_TEXTABOVE, GB_TEXTRIGHT, GB_TEXTLEFT       */
//*  on success, otherwise FALSE                                         */
//*                                                                      */
//************************************************************************/

  GBM_QUERYTEXTPOSITION         = $0534;

//************************************************************************/
//*                             GraphicButton END                        */
//************************************************************************/

  MM_TABHELP                    = $054C;

Implementation

function WinSecondaryWindow(hwndParent: hwnd; hwndOwner: hwnd;
  pfnDlgProc: proc; hmod: Cardinal; idDlg: Cardinal; pCreateParams: Pointer): Cardinal; cdecl;
    external 'SW' index 1;

function WinLoadSecondaryWindow(hwndParent: hwnd; hwndOwner: hwnd;
  pfnDlgProc: proc; hmod: Cardinal; idDlg: Cardinal; pCreateParams: Cardinal): hwnd; cdecl;
    external 'SW' index 2;

function WinProcessSecondaryWindow(hwndSW: hwnd): Cardinal; cdecl;
    external 'SW' index 3;

function WinCreateSecondaryWindow(hwndParent: hwnd; hwndOwner: hwnd;
  pfnDlgProc: proc; idDlg: Cardinal; pCreateParams: Pointer): hwnd; cdecl;
    external 'SW' index 4;

function WinDefaultSize(Wnd: hwnd): Longbool; cdecl;
    external 'SW' index 11;

function WinInsertDefaultSize(Wnd: hwnd; pszDefaultSize: pChar): Longbool; cdecl;
    external 'SW' index 12;

function WinQuerySecondaryhwnd(Wnd: hwnd; ulFlag: Cardinal): hwnd; cdecl;
    external 'SW' index 52;

function WinSecondaryMessageBox(hwndParent: hwnd; hwndOwner: hwnd;
  pszText: pChar; pszCaption: pChar; idWindow: Cardinal; smb: psmbinfo): Cardinal; cdecl;
    external 'SW' index 5;

function WinDismissSecondaryWindow(hwndDlg: hwnd; ulResult: Cardinal): Longbool; cdecl;
    external 'SW' index 6;

function WinDestroySecondaryWindow(Wnd: hwnd): Longbool; cdecl;
    external 'SW' index 7;

function WinDefSecondaryWindowProc(Wnd: hwnd; msg: Cardinal;
                                 mp1: mParam; mp2: mParam): mResult; cdecl;
    external 'SW' index 8;

Function WinRegisterGraphicButton: Longbool; cdecl;
    external 'SW' index 14;

End.
