{
  Interface to the ncurses library. Original ncurses library copyright:

****************************************************************************
 * Copyright (c) 1998-2012,2013 Free Software Foundation, Inc.              *
 *                                                                          *
 * Permission is hereby granted, free of charge, to any person obtaining a  *
 * copy of this software and associated documentation files (the            *
 * "Software"), to deal in the Software without restriction, including      *
 * without limitation the rights to use, copy, modify, merge, publish,      *
 * distribute, distribute with modifications, sublicense, and/or sell       *
 * copies of the Software, and to permit persons to whom the Software is    *
 * furnished to do so, subject to the following conditions:                 *
 *                                                                          *
 * The above copyright notice and this permission notice shall be included  *
 * in all copies or substantial portions of the Software.                   *
 *                                                                          *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS  *
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF               *
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.   *
 * IN NO EVENT SHALL THE ABOVE COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,   *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR    *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR    *
 * THE USE OR OTHER DEALINGS IN THE SOFTWARE.                               *
 *                                                                          *
 * Except as contained in this notice, the name(s) of the above copyright   *
 * holders shall not be used in advertising or otherwise to promote the     *
 * sale, use or other dealings in this Software without prior written       *
 * authorization.                                                           *
 ****************************************************************************}

{$MODE OBJFPC}


unit ncurses;
interface

uses
  unixtype;

{$PACKRECORDS C}
{$LINKLIB ncursesw}
{$LINKLIB c} // should be uses initc ?

{$DEFINE USE_FPC_BYTEBOOL}

const
   libncurses = 'ncursesw';

type
   PFILE = Pointer;
{$IFDEF USE_FPC_BYTEBOOL}
   Bool = ByteBool;
{$ELSE USE_FPC_BYTEBOOL}
   Bool = Byte;
{$ENDIF USE_FPC_BYTEBOOL}

const
{$IFDEF USE_FPC_BYTEBOOL}
   NC_FPC_TRUE  = true;
   NC_FPC_FALSE = false;
{$ELSE USE_FPC_BYTEBOOL}
   NC_FPC_TRUE  = 1;
   NC_FPC_FALSE = 0;
{$ENDIF USE_FPC_BYTEBOOL}

const
   NCURSES_VERSION_MAJOR = 5;
   NCURSES_VERSION_MINOR = 6;
   NCURSES_VERSION_PATCH = 20061217;
   NCURSES_VERSION = '5.6';

{$MACRO ON}
{$IFNDEF NCURSES_MOUSE_VERSION}
{$DEFINE NCURSES_MOUSE_VERSION:=2} // Can be changed for ancient versions of ncurses
{$ENDIF}
{$MACRO OFF}

type
   pchtype = ^chtype;
   chtype  = culong;
   pmmask_t = ^mmask_t;
   mmask_t  = culong;

{ colors  }
var
{$IFNDEF darwin}
   COLORS : cint cvar; external;
   COLOR_PAIRS : cint cvar; external;
{$ELSE darwin}
   COLORS : cint external libncurses name 'COLORS';
   COLOR_PAIRS : cint external libncurses name 'COLOR_PAIRS';
{$ENDIF darwin}

const
   COLOR_BLACK = 0;
   COLOR_RED = 1;
   COLOR_GREEN = 2;
   COLOR_YELLOW = 3;
   COLOR_BLUE = 4;
   COLOR_MAGENTA = 5;
   COLOR_CYAN = 6;
   COLOR_WHITE = 7;

type
   pNC_FPC_COLOR = ^NC_FPC_COLOR;
   NC_FPC_COLOR = Smallint;

{ line graphics  }
type
   tacs_map = array [char] of chtype;
   pacs_map = ^tacs_map;

var
{$IFNDEF darwin}
   acs_map : tacs_map cvar; external;
{$ELSE darwin}
   acs_map : tacs_map external libncurses name 'acs_map';
{$ENDIF darwin}

//function NCURSES_ACS(c : Longint) : Longint;
(* VT100 symbols begin here  *)
function ACS_ULCORNER : chtype; inline; { upper left corner }
function ACS_LLCORNER : chtype; inline; { lower left corner }
function ACS_URCORNER : chtype; inline; { upper right corner }
function ACS_LRCORNER : chtype; inline; { lower right corner }
function ACS_LTEE     : chtype; inline; { tee pointing right }
function ACS_RTEE     : chtype; inline; { tee pointing left }
function ACS_BTEE     : chtype; inline; { tee pointing up }
function ACS_TTEE     : chtype; inline; { tee pointing down }
function ACS_HLINE    : chtype; inline; { horizontal line }
function ACS_VLINE    : chtype; inline; { vertical line }
function ACS_PLUS     : chtype; inline; { large plus or crossover }
function ACS_S1       : chtype; inline; { scan line 1 }
function ACS_S9       : chtype; inline; { scan line 9 }
function ACS_DIAMOND  : chtype; inline; { diamond }
function ACS_CKBOARD  : chtype; inline; { checker board (stipple) }
function ACS_DEGREE   : chtype; inline; { degree symbol }
function ACS_PLMINUS  : chtype; inline; { plus/minus }
function ACS_BULLET   : chtype; inline; { bullet }
(* Teletype 5410v1 symbols begin here *)
function ACS_LARROW   : chtype; inline; { arrow pointing left }
function ACS_RARROW   : chtype; inline; { arrow pointing right }
function ACS_DARROW   : chtype; inline; { arrow pointing down }
function ACS_UARROW   : chtype; inline; { arrow pointing up }
function ACS_BOARD    : chtype; inline; { board of squares }
function ACS_LANTERN  : chtype; inline; { lantern symbol }
function ACS_BLOCK    : chtype; inline; { solid square block }
(*
 * These aren't documented, but a lot of System Vs have them anyway
 * (you can spot pprryyzz{{||}} in a lot of AT&T terminfo strings).
 * The ACS_names may not match AT&T's, our source didn't know them.
 *)
function ACS_S3       : chtype; inline; { scan line 3 }
function ACS_S7       : chtype; inline; { scan line 7 }
function ACS_LEQUAL   : chtype; inline; { less/equal }
function ACS_GEQUAL   : chtype; inline; { greater/equal }
function ACS_PI       : chtype; inline; { Pi }
function ACS_NEQUAL   : chtype; inline; { not equal }
function ACS_STERLING : chtype; inline; { UK pound sign }

(*
 * Line drawing ACS names are of the form ACS_trbl, where t is the top, r
 * is the right, b is the bottom, and l is the left.  t, r, b, and l might
 * be B (blank), S (single), D (double), or T (thick).  The subset defined
 * here only uses B and S.
 *)

{$IFDEF FPC_OBJFPC}
property ACS_BSSB : chtype read ACS_ULCORNER;
property ACS_SSBB : chtype read ACS_LLCORNER;
property ACS_BBSS : chtype read ACS_URCORNER;
property ACS_SBBS : chtype read ACS_LRCORNER;
property ACS_SBSS : chtype read ACS_RTEE;
property ACS_SSSB : chtype read ACS_LTEE;
property ACS_SSBS : chtype read ACS_BTEE;
property ACS_BSSS : chtype read ACS_TTEE;
property ACS_BSBS : chtype read ACS_HLINE;
property ACS_SBSB : chtype read ACS_VLINE;
property ACS_SSSS : chtype read ACS_PLUS;
{$ENDIF FPC_OBJFPC}

const
   ERR = -(1);
   OK = 0;
   _SUBWIN    = $01; { is this a sub-window? }
   _ENDLINE   = $02;  { is the window flush right? }
   _FULLWIN   = $04;  { is the window full-screen? }
   _SCROLLWIN = $08;  { bottom edge is at screen bottom? }
   _ISPAD     = $10;  { is this window a pad? }
   _HASMOVED  = $20;  { has cursor moved since last refresh? }
   _WRAPPED   = $40;  { cursor was just wrappped }
   _NOCHANGE  = -(1);
{
  this value is used in the firstchar and lastchar fields to mark
  unchanged lines
}
   _NEWINDEX  = -(1);

   CCHARW_MAX = 5;


type
   pattr_t = ^attr_t;
   attr_t = chtype;     { ...must be at least as wide as chtype  }
   pcchar_t  = ^cchar_t;

   cchar_t = record
     attr : attr_t;
     chars : array[0..CCHARW_MAX - 1] of wchar_t;
{$IFDEF NCURSES_EXT_COLORS}
     ext_color : cint;  { color pair, must be more than 16-bits }
{$ENDIF NCURSES_EXT_COLORS}
   end;

   ldat = record
     text : pcchar_t;             { text of the line }
     firstchar : Smallint;        { first changed character in the line }
     lastchar : Smallint;         { last changed character in the line }
     oldindex : Smallint;         { index of the line at last update }
   end;

  PWINDOW = ^TWINDOW;

   _win_st = record
     _cury, _curx : Smallint;     { current cursor position  }
{ window location and size  }
     _maxy, _maxx : Smallint;     { maximums of x and y, NOT window size  }
     _begy, _begx : Smallint;     { screen coords of upper-left-hand corner  }
     _flags : Smallint;           { window state flags  }
{ attribute tracking  }
     _attrs : attr_t;             { current attribute for non-space character }
     _bkgd : chtype;              { current background char/attribute pair  }
{ option values set by user  }
     _notimeout : Bool;           { no time out on function-key entry?  }
     _clear : Bool;               { consider all data in the window invalid?  }
     _leaveok : Bool;             { OK to not reset cursor on exit?  }
     _scroll : Bool;              { OK to scroll this window?  }
     _idlok : Bool;               { OK to use insert/delete line?  }
     _idcok : Bool;               { OK to use insert/delete char?  }
     _immed : Bool;               { window in immed mode? (not yet used)  }
     _sync : Bool;                { window in sync mode?  }
     _use_keypad : Bool;          { process function keys into KEY_ symbols?  }
     _delay : cint;               { 0 = nodelay, <0 = blocking, >0 = delay  }
     _line : ^ldat;               { the actual line data  }
{ global screen state  }
     _regtop : Smallint;          { top line of scrolling region  }
     _regbottom : Smallint;       { bottom line of scrolling region  }
{ these are used only if this is a sub-window  }
     _parx : cint;                { x coordinate of this window in parent  }
     _pary : cint;                { y coordinate of this window in parent  }
     _parent : PWINDOW;           { pointer to parent if a sub-window  }
{ these are used only if this is a pad  }
     _pad : record
        _pad_y : Smallint;
        _pad_x : Smallint;
        _pad_top : Smallint;
        _pad_left : Smallint;
        _pad_bottom : Smallint;
        _pad_right : Smallint;
     end;
        _yoffset : Smallint;     { real begy is _begy + _yoffset  }
        _bkgrnd : cchar_t;       { current background char/attribute pair  }
{$IFDEF NCURSES_EXT_COLORS}
     _color : cint;              { current color-pair for non-space character }
{$ENDIF NCURSES_EXT_COLORS}
     end;

   TWINDOW = _win_st;
   TSCREEN = TWINDOW;
   PSCREEN = ^TSCREEN;

      var
{$IFNDEF darwin}
       stdscr  : PWINDOW cvar; external;
       curscr  : PWINDOW cvar; external;
       newscr  : PWINDOW cvar; external;
       LINES   : Longint cvar; external;
       COLS    : Longint cvar; external;
       TABSIZE : Longint cvar; external;
       ESCDELAY: Longint cvar; external;  { ESC expire time in milliseconds  }
{$ELSE darwin}
       stdscr  : PWINDOW external libncurses name 'stdscr';
       curscr  : PWINDOW external libncurses name 'curscr';
       newscr  : PWINDOW external libncurses name 'newscr';
       LINES   : Longint external libncurses name 'LINES';
       COLS    : Longint external libncurses name 'COLS';
       TABSIZE : Longint external libncurses name 'TABSIZE';
       ESCDELAY: Longint external libncurses name 'ESCDELAY';
{$ENDIF darwin}

(*
 * These functions are extensions - not in XSI Curses.
 *)

function is_term_resized(_para1:Longint; _para2:Longint):Bool;cdecl;external libncurses;
function keybound(_para1:Longint; _para2:Longint):PChar;cdecl;external  libncurses;
function curses_version:PChar;cdecl;external  libncurses;
function assume_default_colors(_para1:Longint; _para2:Longint):Longint; cdecl;external libncurses;
function define_key(_para1:PChar; _para2:Longint):Longint; cdecl;external libncurses;
function key_defined(_para1:PChar):Longint; cdecl;external libncurses;
function keyok(_para1:Longint; _para2:Bool):Longint; cdecl;external libncurses;
function resize_term(_para1:Longint; _para2:Longint):Longint; cdecl;external libncurses;
function resizeterm(_para1:Longint; _para2:Longint):Longint; cdecl;external libncurses;
function use_default_colors:Longint;cdecl;external libncurses;
function use_extended_names(_para1:Bool):Longint; cdecl;external libncurses;
function use_legacy_coding(_para1:Longint):Longint; cdecl;external libncurses;
function wresize(_para1:PWINDOW; _para2:Longint; _para3:Longint):Longint; cdecl;external libncurses;
procedure nofilter;cdecl;external  libncurses;

(*
 * Function prototypes.  This is the complete XSI Curses list of required
 * functions.
 *)
function baudrate:Longint; cdecl;external libncurses;
function beep:Longint; cdecl;external libncurses;
function can_change_color:Bool;cdecl;external libncurses;
function cbreak:Longint; cdecl;external libncurses;
//function color_content(_para1:Smallint; _para2:PSmallint; _para3:PSmallint; _para4:PSmallInt):Longint; cdecl;external libncurses;
function color_content(color: NC_FPC_COLOR; r, g, b: PSmallint):Longint; cdecl;external libncurses;
function copywin(_para1:PWINDOW; _para2:PWINDOW; _para3:Longint; _para4:Longint; _para5:Longint;
           _para6:Longint; _para7:Longint; _para8:Longint; _para9:Longint):Longint; cdecl;external libncurses;
function curs_set(_para1:Longint):Longint; cdecl;external libncurses;
function def_prog_mode:Longint; cdecl;external libncurses;
function def_shell_mode:Longint; cdecl;external libncurses;
function delay_output(_para1:Longint):Longint; cdecl;external libncurses;
procedure delscreen(_para1:PSCREEN);cdecl;external libncurses;
function delwin(_para1:PWINDOW):Longint; cdecl;external libncurses;
function derwin(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:Longint; _para5:Longint):PWINDOW;cdecl;external libncurses;
function doupdate:Longint; cdecl;external libncurses;
function dupwin(_para1:PWINDOW):PWINDOW;cdecl;external libncurses;
function echo:Longint; cdecl;external libncurses;
function endwin:Longint; cdecl;external libncurses;
function erasechar:PChar;cdecl;external libncurses;
procedure filter;cdecl;external libncurses;
function flash:Longint; cdecl;external libncurses;
function flushinp:Longint; cdecl;external libncurses;
function getwin(_para1:PFILE):PWINDOW;cdecl;external libncurses;
function halfdelay(_para1:Longint):Longint; cdecl;external libncurses;
function has_colors:Bool;cdecl;external libncurses;
function has_ic:Bool;cdecl;external libncurses;
function has_il:Bool;cdecl;external libncurses;
procedure idcok(_para1:PWINDOW; _para2:Bool);cdecl;external libncurses;
function idlok(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
procedure immedok(_para1:PWINDOW; _para2:Bool);cdecl;external libncurses;
function initscr:PWINDOW;cdecl;external libncurses;
//function init_color(_para1:Smallint; _para2:Smallint; _para3:Smallint; _para4:Smallint):Longint; cdecl;external libncurses;
function init_color(color: NC_FPC_COLOR; r, g, b: Smallint):Longint; cdecl;external libncurses;
//function init_pair(_para1:Smallint; _para2:Smallint; _para3:Smallint):Longint; cdecl;external libncurses;
function init_pair(pair: Smallint; f, b: NC_FPC_COLOR):Longint; cdecl;external libncurses; overload;
function intrflush(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function isendwin:Bool;cdecl;external libncurses;
function is_linetouched(_para1:PWINDOW; _para2:Longint):Bool;cdecl;external libncurses;
function is_wintouched(_para1:PWINDOW):Bool;cdecl;external libncurses;
function keyname(_para1:Longint):PChar;cdecl;external libncurses;
function keypad(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function killchar:PChar;cdecl;external libncurses;
function leaveok(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function longname:PChar;cdecl;external libncurses;
function meta(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function mvcur(_para1:Longint; _para2:Longint; _para3:Longint; _para4:Longint):Longint; cdecl;external libncurses;
function mvderwin(_para1:PWINDOW; _para2:Longint; _para3:Longint):Longint; cdecl;external libncurses;
function mvwin(_para1:PWINDOW; _para2:Longint; _para3:Longint):Longint; cdecl;external libncurses;
function napms(_para1:Longint):Longint; cdecl;external libncurses;
function newpad(_para1:Longint; _para2:Longint):PWINDOW;cdecl;external libncurses;
function newterm(_para1:PChar; _para2:PFILE; _para3:PFILE):PSCREEN;cdecl;external libncurses;
function newwin(_para1:Longint; _para2:Longint; _para3:Longint; _para4:Longint):PWINDOW;cdecl;external libncurses;
function nl:Longint; cdecl;external libncurses;
function nocbreak:Longint; cdecl;external libncurses;
function nodelay(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function noecho:Longint; cdecl;external libncurses;
function nonl:Longint; cdecl;external libncurses;
procedure noqiflush;cdecl;external libncurses;
function noraw:Longint; cdecl;external libncurses;
function notimeout(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function overlay(_para1:PWINDOW; _para2:PWINDOW):Longint; cdecl;external libncurses;
function overwrite(_para1:PWINDOW; _para2:PWINDOW):Longint; cdecl;external libncurses;
//function pair_content(_para1:Smallint; _para2:PSmallint; _para3:PSmallInt):Longint; cdecl;external libncurses;
function pair_content(pair: Smallint; f, b: pNC_FPC_COLOR):Longint; cdecl;external libncurses;
function pechochar(_para1:PWINDOW; _para2:chtype):Longint; cdecl;external libncurses;
function pnoutrefresh(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:Longint; _para5:Longint;
           _para6:Longint; _para7:Longint):Longint; cdecl;external libncurses;
function prefresh(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:Longint; _para5:Longint;
           _para6:Longint; _para7:Longint):Longint; cdecl;external libncurses;
function putwin(_para1:PWINDOW; _para2:PFILE):Longint; cdecl;external libncurses;
procedure qiflush;cdecl;external libncurses;
function raw:Longint; cdecl;external libncurses;
function resetty:Longint; cdecl;external libncurses;
function reset_prog_mode:Longint; cdecl;external libncurses;
function reset_shell_mode:Longint; cdecl;external libncurses;

type TWinInit = function (win: PWINDOW; ncols: Longint): Longint; cdecl;

function ripoffline(line: Longint; init: TWinInit):Longint; cdecl;external libncurses;

function savetty:Longint; cdecl;external libncurses;
function scr_dump(_para1:PChar):Longint; cdecl;external libncurses;
function scr_init(_para1:PChar):Longint; cdecl;external libncurses;
function scrollok(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function scr_restore(_para1:PChar):Longint; cdecl;external libncurses;
function scr_set(_para1:PChar):Longint; cdecl;external libncurses;
function set_term(_para1:PSCREEN):PSCREEN;cdecl;external libncurses;
function slk_attroff(_para1:chtype):Longint; cdecl;external libncurses;

function slk_attron(_para1:chtype):Longint; cdecl;external libncurses;

function slk_attrset(_para1:chtype):Longint; cdecl;external libncurses;
function slk_attr:attr_t;cdecl;external libncurses;
function slk_attr_set(_para1:attr_t; _para2:Smallint; _para3:Pointer):Longint; cdecl;external libncurses;
function slk_clear:Longint; cdecl;external libncurses;
function slk_color(_para1:Smallint):Longint; cdecl;external libncurses;
function slk_init(_para1:Longint):Longint; cdecl;external libncurses;
function slk_label(_para1:Longint):PChar;cdecl;external libncurses;
function slk_noutrefresh:Longint; cdecl;external libncurses;
function slk_refresh:Longint; cdecl;external libncurses;
function slk_restore:Longint; cdecl;external libncurses;
function slk_set(_para1:Longint; _para2:PChar; _para3:Longint):Longint; cdecl;external libncurses;
function slk_touch:Longint; cdecl;external libncurses;
function start_color:Longint; cdecl;external libncurses;
function subpad(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:Longint; _para5:Longint):PWINDOW;cdecl;external libncurses;
function subwin(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:Longint; _para5:Longint):PWINDOW;cdecl;external libncurses;
function syncok(_para1:PWINDOW; _para2:Bool):Longint; cdecl;external libncurses;
function termattrs:chtype;cdecl;external libncurses;
function termname:PChar;cdecl;external libncurses;
function typeahead(_para1:Longint):Longint; cdecl;external libncurses;
function ungetch(_para1:Longint):Longint; cdecl;external libncurses;
procedure use_env(_para1:Bool);cdecl;external libncurses;
function vidattr(_para1:chtype):Longint; cdecl;external libncurses;
function waddch(_para1:PWINDOW; _para2:chtype):Longint; cdecl;external libncurses;
function waddchnstr(_para1:PWINDOW; _para2:Pchtype; _para3:Longint):Longint; cdecl;external libncurses;
function waddnstr(_para1:PWINDOW; _para2:PChar; _para3:Longint):Longint; cdecl;external libncurses;
function wattr_on(_para1:PWINDOW; _para2:attr_t; _para3:Pointer):Longint; cdecl;external libncurses;
function wattr_off(_para1:PWINDOW; _para2:attr_t; _para3:Pointer):Longint; cdecl;external libncurses;
function wbkgd(_para1:PWINDOW; _para2:chtype):Longint; cdecl;external libncurses;
procedure wbkgdset(_para1:PWINDOW; _para2:chtype);cdecl;external libncurses;
function wborder(_para1:PWINDOW; _para2:chtype; _para3:chtype; _para4:chtype; _para5:chtype;
           _para6:chtype; _para7:chtype; _para8:chtype; _para9:chtype):Longint; cdecl;external libncurses;
function wchgat(_para1:PWINDOW; _para2:Longint; _para3:attr_t; _para4:Smallint; _para5:Pointer):Longint; cdecl;external libncurses;
function wclear(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wclrtobot(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wclrtoeol(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wcolor_set(_para1:PWINDOW; _para2:Smallint; _para3:Pointer):Longint; cdecl;external libncurses;
procedure wcursyncup(_para1:PWINDOW);cdecl;external libncurses;
function wdelch(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wechochar(_para1:PWINDOW; _para2:chtype):Longint; cdecl;external libncurses;
function werase(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wgetch(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wgetnstr(_para1:PWINDOW; _para2:PChar; _para3:Longint):Longint; cdecl;external libncurses;
function whline(_para1:PWINDOW; _para2:chtype; _para3:Longint):Longint; cdecl;external libncurses;
function winch(_para1:PWINDOW):chtype;cdecl;external libncurses;
function winchnstr(_para1:PWINDOW; _para2:Pchtype; _para3:Longint):Longint; cdecl;external libncurses;
function winnstr(_para1:PWINDOW; _para2:PChar; _para3:Longint):Longint; cdecl;external libncurses;
function winsch(_para1:PWINDOW; _para2:chtype):Longint; cdecl;external libncurses;
function winsdelln(_para1:PWINDOW; _para2:Longint):Longint; cdecl;external libncurses;
function winsnstr(_para1:PWINDOW; _para2:PChar; _para3:Longint):Longint; cdecl;external libncurses;
{ realised as inline function }
//function wmove(_para1:PWINDOW; _para2:Longint; _para3:Longint):Longint; cdecl;external libncurses;
function wnoutrefresh(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wredrawln(_para1:PWINDOW; _para2:Longint; _para3:Longint):Longint; cdecl;external libncurses;
function wrefresh(_para1:PWINDOW):Longint; cdecl;external libncurses;
function wscrl(_para1:PWINDOW; _para2:Longint):Longint; cdecl;external libncurses;
function wsetscrreg(_para1:PWINDOW; _para2:Longint; _para3:Longint):Longint; cdecl;external libncurses;
procedure wsyncdown(_para1:PWINDOW);cdecl;external libncurses;
procedure wsyncup(_para1:PWINDOW);cdecl;external libncurses;
procedure wtimeout(_para1:PWINDOW; _para2:Longint);cdecl;external libncurses;
function wtouchln(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:Longint):Longint; cdecl;external libncurses;
function wvline(_para1:PWINDOW; _para2:chtype; _para3:Longint):Longint; cdecl;external libncurses;

(*
 * These are also declared in <ncursesw/term.h>:
 *)

function tigetflag(_para1:PChar):Longint; cdecl;external libncurses;
function tigetnum(_para1:PChar):Longint; cdecl;external libncurses;
function tigetstr(_para1:PChar):PChar;cdecl;external libncurses;
function putp(_para1:PChar):Longint; cdecl;external libncurses;

var
{$IFNDEF darwin}
   ttytype : array of PChar cvar; external;  { needed for backward compatibility  }
{$ELSE darwin}
   ttytype : array of PChar external libncurses  name 'ttytype';
{$ENDIF darwin}

(*
 * Function prototypes for wide-character operations.
 *)
{function vid_puts(_para1:attr_t; _para2:Smallint; _para3:pointer; _para4:function (_para1:cint):cint):longint; cdecl;external libncurses;
function vidputs(_para1:chtype; _para2:function (_para1:Longint):Longint):Longint; cdecl;external libncurses;}
type TPutc = function(arg:Longint):Longint; cdecl;
function vid_puts(attrs:attr_t; pair:Smallint; opts:Pointer; _putc:TPutc):Longint; cdecl;external libncurses;
function vidputs(attrs:chtype; _putc:TPutc):Longint; cdecl;external libncurses;

function erasewchar(_para1:Pwchar_t):Longint; cdecl;external libncurses;
function getcchar(_para1:Pcchar_t; _para2:Pwchar_t; _para3:Pattr_t; _para4:PSmallInt; _para5:Pointer):Longint; cdecl;external libncurses;
function key_name(_para1:wchar_t):PChar;cdecl;external libncurses;
function killwchar(_para1:Pwchar_t):Longint; cdecl;external libncurses;
function pecho_wchar(_para1:PWINDOW; _para2:Pcchar_t):Longint; cdecl;external libncurses;
function setcchar(_para1:Pcchar_t; _para2:Pwchar_t; _para3:attr_t; _para4:Smallint; _para5:Pointer):Longint; cdecl;external libncurses;
function slk_wset(_para1:Longint; _para2:Pwchar_t; _para3:Longint):Longint; cdecl;external libncurses;
function term_attrs:attr_t;cdecl;external  libncurses;
function unget_wch(_para1:wchar_t):longint; cdecl;external libncurses;
function vid_attr(_para1:attr_t; _para2:Smallint; _para3:pointer):longint; cdecl;external libncurses;
function wadd_wch(_para1:PWINDOW; _para2:Pcchar_t):longint; cdecl;external libncurses;
function wadd_wchnstr(_para1:PWINDOW; _para2:Pcchar_t; _para3:Longint):longint; cdecl;external libncurses;
function waddnwstr(_para1:PWINDOW; _para2:Pwchar_t; _para3:Longint):longint; cdecl;external libncurses;
function wbkgrnd(_para1:PWINDOW; _para2:Pcchar_t):longint; cdecl;external libncurses;
procedure wbkgrndset(_para1:PWINDOW; _para2:Pcchar_t);cdecl;external  libncurses;
function wborder_set(_para1:PWINDOW; _para2:Pcchar_t; _para3:Pcchar_t; _para4:Pcchar_t; _para5:Pcchar_t;
           _para6:Pcchar_t; _para7:Pcchar_t; _para8:Pcchar_t; _para9:Pcchar_t):longint; cdecl;external libncurses;
function wecho_wchar(_para1:PWINDOW; _para2:Pcchar_t):longint; cdecl;external libncurses;
function wget_wch(_para1:PWINDOW; _para2:PLongint):longint; cdecl;external libncurses;
function wgetbkgrnd(_para1:PWINDOW; _para2:Pcchar_t):longint; cdecl;external libncurses;
function wgetn_wstr(_para1:PWINDOW; _para2:PLongint; _para3:Longint):longint; cdecl;external libncurses;
function whline_set(_para1:PWINDOW; _para2:Pcchar_t; _para3:Longint):longint; cdecl;external libncurses;
function win_wch(_para1:PWINDOW; _para2:Pcchar_t):longint; cdecl;external libncurses;
function win_wchnstr(_para1:PWINDOW; _para2:Pcchar_t; _para3:Longint):longint; cdecl;external libncurses;
function winnwstr(_para1:PWINDOW; _para2:Pwchar_t; _para3:Longint):longint; cdecl;external libncurses;
function wins_nwstr(_para1:PWINDOW; _para2:Pwchar_t; _para3:Longint):longint; cdecl;external libncurses;
function wins_wch(_para1:PWINDOW; _para2:Pcchar_t):longint; cdecl;external libncurses;
function winwstr(_para1:PWINDOW; _para2:Pwchar_t):longint; cdecl;external libncurses;
function wunctrl(_para1:Pcchar_t):Pwchar_t;cdecl;external libncurses;
function wvline_set(_para1:PWINDOW; _para2:Pcchar_t; _para3:Longint):longint; cdecl;external libncurses;

const
   A_NORMAL = 0;
   A_ATTRIBUTES = (not 0) shl 8;
   A_CHARTEXT=(1 shl (0 + 8)) - 1;
   A_COLOR=((1 shl 8) - 1) shl 8;
   A_STANDOUT = 1 shl (8 + 8);
   A_UNDERLINE = 1 shl (9 + 8);
   A_REVERSE = 1 shl (10 + 8);
   A_BLINK = 1 shl (11 + 8);
   A_DIM = 1 shl (12 + 8);
   A_BOLD = 1 shl (13 + 8);
   A_ALTCHARSET = 1 shl (14 + 8);
   A_INVIS = 1 shl (15 + 8);
   A_PROTECT = 1 shl (16 + 8);
   A_HORIZONTAL = 1 shl (17 + 8);
   A_LEFT = 1 shl (18 + 8);
   A_LOW = 1 shl (19 + 8);
   A_RIGHT = 1 shl (20 + 8);
   A_TOP = 1 shl (21 + 8);
   A_VERTICAL = 1 shl (22 + 8);

const
   WA_ATTRIBUTES = A_ATTRIBUTES;
   WA_NORMAL = A_NORMAL;
   WA_STANDOUT = A_STANDOUT;
   WA_UNDERLINE = A_UNDERLINE;
   WA_REVERSE = A_REVERSE;
   WA_BLINK = A_BLINK;
   WA_DIM = A_DIM;
   WA_BOLD = A_BOLD;
   WA_ALTCHARSET = A_ALTCHARSET;
   WA_INVIS = A_INVIS;
   WA_PROTECT = A_PROTECT;
   WA_HORIZONTAL = A_HORIZONTAL;
   WA_LEFT = A_LEFT;
   WA_LOW = A_LOW;
   WA_RIGHT = A_RIGHT;
   WA_TOP = A_TOP;
   WA_VERTICAL = A_VERTICAL;

function COLOR_PAIR(n: cint): cint; inline;
function PAIR_NUMBER(attr: attr_t): longint; inline;
function color_set(color_pair_number: Smallint; opts: Pointer): longint; inline;

(*  pseudo functions  *)

function wgetstr(win: PWINDOW; s: PChar): Longint;
function getnstr(s: PChar;n: Longint): Longint; inline;
function setterm(term: PChar): Longint; inline;
function fixterm: Longint; inline;
function resetterm: Longint; inline;
function saveterm: Longint; inline;
function crmode: Longint; inline;
function nocrmode: Longint; inline;

// formal parameter versions not endiansafe?
procedure getyx   (win: PWINDOW; var y,x: Smallint); inline; overload;
procedure getbegyx(win: PWINDOW; var y,x: Smallint); inline; overload;
procedure getmaxyx(win: PWINDOW; var y,x: Smallint); inline; overload;
procedure getparyx(win: PWINDOW; var y,x: Smallint); inline; overload;
procedure getsyx  (var y,x: Smallint); inline; overload;

procedure getyx   (win: PWINDOW; var y,x: Longint); inline; overload;
procedure getbegyx(win: PWINDOW; var y,x: Longint); inline; overload;
procedure getmaxyx(win: PWINDOW; var y,x: Longint); inline; overload;
procedure getparyx(win: PWINDOW; var y,x: Longint); inline; overload;
procedure getsyx  (var y,x: Longint); inline; overload;

procedure setsyx  (y,x: Smallint); inline;
function getattrs(win: PWINDOW): attr_t; inline;
function getcurx(win: PWINDOW): Smallint; inline;
function getcury(win: PWINDOW): Smallint; inline;
function getbegx(win: PWINDOW): Smallint; inline;
function getbegy(win: PWINDOW): Smallint; inline;
function getmaxx(win: PWINDOW): Smallint; inline;
function getmaxy(win: PWINDOW): Smallint; inline;
function getparx(win: PWINDOW): Smallint; inline;
function getpary(win: PWINDOW): Smallint; inline;
function wstandout(win: PWINDOW): Longint; inline;
function wstandend(win: PWINDOW): Longint; inline;
function wattr_set(win:PWINDOW; attrs:attr_t; pair:Smallint; opts:Pointer):Longint; inline;
//function wattr_get(win: PWINDOW): longint;// original fpc ncurses?
function wattr_get(win:PWINDOW; attrs:Pattr_t; pair:PSmallint; opts:Pointer):longint; // inline; breaks compilation on some architectures currently
function wattron(win:PWINDOW; attrs: attr_t): Longint; inline;
function wattroff(win:PWINDOW; attrs: attr_t): Longint; inline;
function wattrset(win: PWINDOW; attrs: attr_t): longint; inline;
function scroll(win: PWINDOW): longint; inline;
function touchwin(win: PWINDOW): longint; inline;
function touchline(win: PWINDOW;s,c: longint): longint; inline;
function untouchwin(win: PWINDOW): longint; inline;
function box(win:PWINDOW; v,h :chtype):Longint; inline;
function border(ls,rs,ts,bs,tl,tr,bl,br: chtype): longint; inline;
function hline(ch:chtype; n:longint): longint; inline;
function vline(ch:chtype; n:longint): longint; inline;
function winstr(win: PWINDOW;s: PChar): longint; inline;
function winchstr(win: PWINDOW; chstr: pchtype): longint; inline;
function winsstr(win: PWINDOW;s: PChar): longint; inline;
function redrawwin(win: PWINDOW): longint; inline;
function waddstr(win: PWINDOW;st: PChar): longint; inline;
function waddchstr(win: PWINDOW; chstr: pchtype): longint; inline;
{
   pseudo functions for standard screen
}
function addch(ch: chtype): longint; inline;
function addchnstr(chstr: pchtype; n: longint): longint; inline;
function addchstr(chstr: pchtype): longint; inline;
function addnstr(str: PChar;n: longint): longint; inline;
function addstr(str: PChar): longint; inline;
function attroff(attrs: attr_t): longint; inline;
function attron(attrs: attr_t): longint; inline;
function attrset(attrs: attr_t): longint; inline;
function bkgd(ch: chtype): longint; inline;
procedure bkgdset(ch: chtype); inline;
function clear: longint; inline;
function clrtobot: longint; inline;
function clrtoeol: longint; inline;
function delch: longint; inline;
function deleteln: longint; inline;
function echochar(ch: chtype): longint; inline;
function erase: longint; inline;
function getch: longint; inline;
function getstr(str: PChar): longint; inline;
function inch: chtype; inline;
function inchnstr(chstr: pchtype;n: longint): longint; inline;
function inchstr(chstr: pchtype): longint; inline;
function innstr(str: PChar;n: longint): longint; inline;
function insch(ch: chtype): longint; inline;
function insdelln(n: longint): longint; inline;
function insertln: longint; inline;
function insnstr(str: PChar;n: longint): longint; inline;
function insstr(str: PChar): longint; inline;
function instr(str: PChar): longint; inline;
function move(y,x: Smallint): longint; inline;
function refresh: longint; inline;
function scrl(n: longint): longint; inline;
function setscrreg(t,b: longint): longint; inline;
function standend: longint; inline;
function standout: longint; inline;
procedure timeout(delay: longint); inline;
function wdeleteln(win: PWINDOW): longint; inline;
function winsertln(win: PWINDOW): longint; inline;

{
   mv functions
 }

function mvwaddch(win: PWINDOW;y,x: Smallint; ch: chtype): longint; inline;
function mvwaddchnstr(win: PWINDOW;y,x: Smallint; chstr: pchtype; n: longint): longint; inline;
function mvwaddchstr(win: PWINDOW;y,x: Smallint; chstr: pchtype): longint; inline;
function mvwaddnstr(win: PWINDOW;y,x: Smallint; str: PChar; n: longint): longint; inline;
function mvwaddstr(win: PWINDOW;y,x: Smallint; str: PChar): longint; inline;
function mvwdelch(win: PWINDOW; y,x: Smallint): longint;
function mvwchgat(win: PWINDOW;y, x: Smallint; n: Longint;attr: attr_t;
                            color: Smallint; opts: Pointer): longint; inline;
function mvwgetch(win: PWINDOW; y,x: Smallint): longint; inline;
function mvwgetnstr(win: PWINDOW; y,x: Smallint; str: PChar; n: longint): longint; inline;
function mvwgetstr(win: PWINDOW; y,x: Smallint; str: PChar): longint; inline;
function mvwhline(win: PWINDOW; y,x: Smallint; ch: chtype; n: longint): longint; inline;
function mvwinch(win: PWINDOW;y,x: Smallint): chtype; inline;
function mvwinchnstr(win: PWINDOW;y,x: Smallint;chstr: pchtype; n: longint): longint; inline;
function mvwinchstr(win: PWINDOW;y,x: Smallint;chstr: pchtype): longint; inline;
function mvwinnstr(win: PWINDOW;y,x: Smallint;str: PChar;n: longint): longint; inline;
function mvwinsch(win: PWINDOW;y,x: Smallint;ch: chtype): longint; inline;
function mvwinsnstr(win: PWINDOW;y,x: Smallint;str: PChar;n: longint): longint; inline;
function mvwinsstr(win: PWINDOW;y,x: Smallint;str: PChar): longint; inline;
function mvwinstr(win: PWINDOW;y,x: Smallint;str: PChar): longint; inline;
function mvwvline(win: PWINDOW;y,x: Smallint;ch: chtype;n: longint): longint; inline;
function mvaddch(y,x: Smallint; ch: chtype): longint; inline;
function mvaddchnstr(y,x: Smallint; chstr: pchtype;n: longint): longint; inline;
function mvaddchstr(y,x: Smallint; chstr: pchtype): longint; inline;
function mvaddnstr(y,x: Smallint; str: PChar;n: longint): longint; inline;
function mvaddstr(y,x: Smallint; str: PChar): longint; inline;
function mvdelch(y,x: Smallint): longint; inline;
function mvgetch(y,x: Smallint): longint; inline;
function mvgetnstr(y,x: Smallint; str: PChar;n: longint): longint; inline;
function mvgetstr(y,x: Smallint; str: PChar): longint; inline;
function mvhline(y,x: Smallint;ch: chtype;n: longint): longint; inline;
function mvinch(y,x: Smallint): chtype; inline;
function mvinchnstr(y,x: Smallint; chstr: pchtype;n: longint): longint; inline;
function mvinchstr(y,x: Smallint; chstr: pchtype): longint; inline;
function mvinnstr(y,x: Smallint; str: PChar;n: longint): longint; inline;
function mvinsch(y,x: Smallint;ch: chtype): longint; inline;
function mvinsnstr(y,x: Smallint; str: PChar;n: longint): longint; inline;
function mvinsstr(y,x: Smallint; str: PChar): longint; inline;
function mvinstr(y,x: Smallint; str: PChar): longint; inline;
function mvvline(y,x: Smallint; ch:chtype; n:longint): longint; inline;
function attr_get(attrs:Pattr_t; pair:PSmallint; opts:Pointer): longint; inline;
function attr_off(attrs:attr_t; opts:Pointer): longint; inline;
function attr_on(attrs:attr_t; opts:Pointer): longint; inline;
function attr_set(attrs:attr_t; pair:Smallint; opts:Pointer): longint; inline;
function chgat(n: Longint;attr: attr_t;color: Smallint; opts: Pointer): longint; inline;
{not present in fpc ncurses}
function mvchgat(y, x: Smallint; n: Longint;attr: attr_t;color: Smallint; opts: Pointer):Longint; inline;
function getbkgd(win: PWINDOW): chtype; inline;
function slk_attr_off(attrs: attr_t; opts: Pointer) : longint; inline;
function slk_attr_on(attrs: attr_t; opts: Pointer): longint; inline;





(*
 * Pseudo-character tokens outside ASCII range.  The curses wgetch() function
 * will return any given one of these only if the corresponding k- capability
 * is defined in your terminal's terminfo entry.
 *
 * Some keys (KEY_A1, etc) are arranged like this:
 *  a1     up    a3
 *  left   b2    right
 *  c1     down  c3
 *
 * A few key codes do not depend upon the terminfo entry.
 *)

const
   KEY_CODE_YES = 256;  { A wchar_t contains a key code    &0400 }
   KEY_MIN = 257;       { Minimum curses key    &0401 }
   KEY_BREAK = 257;     { Break key (unreliable)   &0401 }
   KEY_SRESET = 344;    { Soft (partial) reset (unreliable)    &0530 }
   KEY_RESET = 345;     { Reset or hard reset (unreliable)    &0531 }
   KEY_DOWN = 258;      { down-arrow key    &0402 }
   KEY_UP = 259;        { up-arrow key    &0403 }
   KEY_LEFT = 260;      { left-arrow key    &0404 }
   KEY_RIGHT = 261;     { right-arrow key    &0405 }
   KEY_HOME = 262;      { home key    &0406 }
   KEY_BACKSPACE = 263; { backspace key    &0407 }
   KEY_F0 = 264;        { Function keys.  Space for 64    &0410 }

{ Manually Added KEY_F1 .. KEY_F12 }

   KEY_F1 = KEY_F0 + 1;
   KEY_F2 = KEY_F0 + 2;
   KEY_F3 = KEY_F0 + 3;
   KEY_F4 = KEY_F0 + 4;
   KEY_F5 = KEY_F0 + 5;
   KEY_F6 = KEY_F0 + 6;
   KEY_F7 = KEY_F0 + 7;
   KEY_F8 = KEY_F0 + 8;
   KEY_F9 = KEY_F0 + 9;
   KEY_F10 = KEY_F0 + 10;
   KEY_F11 = KEY_F0 + 11;
   KEY_F12 = KEY_F0 + 12;

function KEY_F(n : Byte) : chtype; inline;

const
   KEY_DL = 328;        { delete-line key    &0510 }
   KEY_IL = 329;        { insert-line key    &0511 }
   KEY_DC = 330;        { delete-character key    &0512 }
   KEY_IC = 331;        { insert-character key    &0513 }
   KEY_EIC = 332;       { sent by rmir or smir in insert mode    &0514 }
   KEY_CLEAR = 333;     { clear-screen or erase key    &0515 }
   KEY_EOS = 334;       { clear-to-end-of-screen key    &0516 }
   KEY_EOL = 335;       { clear-to-end-of-line key    &0517 }
   KEY_SF = 336;        { scroll-forward key    &0520 }
   KEY_SR = 337;        { scroll-backward key    &0521 }
   KEY_NPAGE = 338;     { next-page key    &0522 }
   KEY_PPAGE = 339;     { previous-page key    &0523 }
   KEY_STAB = 340;      { set-tab key    &0524 }
   KEY_CTAB = 341;      { clear-tab key    &0525 }
   KEY_CATAB = 342;     { clear-all-tabs key    &0526 }
   KEY_ENTER = 343;     { enter/send key    &0527 }
   KEY_PRINT = 346;     { print key    &0532 }
   KEY_LL = 347;        { lower-left key (home down)    &0533 }
   KEY_A1 = 348;        { upper left of keypad    &0534 }
   KEY_A3 = 349;        { upper right of keypad    &0535 }
   KEY_B2 = 350;        { center of keypad    &0536 }
   KEY_C1 = 351;        { lower left of keypad    &0537 }
   KEY_C3 = 352;        { lower right of keypad    &0540 }
   KEY_BTAB = 353;      { back-tab key    &0541 }
   KEY_BEG = 354;       { begin key    &0542 }
   KEY_CANCEL = 355;    { cancel key    &0543 }
   KEY_CLOSE = 356;     { close key    &0544 }
   KEY_COMMAND = 357;   { command key   &0545  }
   KEY_COPY = 358;      { copy key    &0546 }
   KEY_CREATE = 359;    { create key    &0547 }
   KEY_END = 360;       { end key    &0550 }
   KEY_EXIT = 361;      { exit key    &0551 }
   KEY_FIND = 362;      { find key    &0552 }
   KEY_HELP = 363;      { help key    &0553 }
   KEY_MARK = 364;      { mark key    &0554 }
   KEY_MESSAGE = 365;   { message key    &0555 }
   KEY_MOVE = 366;      { move key    &0556 }
   KEY_NEXT = 367;      { next key    &0557 }
   KEY_OPEN = 368;      { open key    &0560 }
   KEY_OPTIONS = 369;   { options key    &0561 }
   KEY_PREVIOUS = 370;  { previous key    &0562 }
   KEY_REDO = 371;      { redo key    &0563 }
   KEY_REFERENCE = 372; { reference key    &0564 }
   KEY_REFRESH = 373;   { refresh key    &0565 }
   KEY_REPLACE = 374;   { replace key    &0566 }
   KEY_RESTART = 375;   { restart key    &0567 }
   KEY_RESUME = 376;    { resume key    &0570 }
   KEY_SAVE = 377;      { save key    &0571 }
   KEY_SBEG = 378;      { shifted begin key    &0572 }
   KEY_SCANCEL = 379;   { shifted cancel key    &0573 }
   KEY_SCOMMAND = 380;  { shifted command key    &0574 }
   KEY_SCOPY = 381;     { shifted copy key    &0575 }
   KEY_SCREATE = 382;   { shifted create key    &0576 }
   KEY_SDC = 383;       { shifted delete-character key    &0577 }
   KEY_SDL = 384;       { shifted delete-line key    &0600 }
   KEY_SELECT = 385;    { select key    &0601 }
   KEY_SEND = 386;      { shifted end key    &0602 }
   KEY_SEOL = 387;      { shifted clear-to-end-of-line key    &0603 }
   KEY_SEXIT = 388;     { shifted exit key    &0604 }
   KEY_SFIND = 389;     { shifted find key    &0605 }
   KEY_SHELP = 390;     { shifted help key    &0606 }
   KEY_SHOME = 391;     { shifted home key    &0607 }
   KEY_SIC = 392;       { shifted insert-character key    &0610 }
   KEY_SLEFT = 393;     { shifted left-arrow key    &0611 }
   KEY_SMESSAGE = 394;  { shifted message key    &0612 }
   KEY_SMOVE = 395;     { shifted move key    &0613 }
   KEY_SNEXT = 396;     { shifted next key    &0614 }
   KEY_SOPTIONS = 397;  { shifted options key    &0615 }
   KEY_SPREVIOUS = 398; { shifted previous key    &0616 }
   KEY_SPRINT = 399;    { shifted print key   &0617 }
   KEY_SREDO = 400;     { shifted redo key    &0620 }
   KEY_SREPLACE = 401;  { shifted replace key    &0621 }
   KEY_SRIGHT = 402;    { shifted right-arrow key    &0622 }
   KEY_SRSUME = 403;    { shifted resume key    &0623 }
   KEY_SSAVE = 404;     { shifted save key    &0624 }
   KEY_SSUSPEND = 405;  { shifted suspend key    &0625 }
   KEY_SUNDO = 406;     { shifted undo key    &0626 }
   KEY_SUSPEND = 407;   { suspend key    &0627 }
   KEY_UNDO = 408;      { undo key     &0630}
   KEY_MOUSE = 409;     { Mouse event has occurred    &0631 }
   KEY_RESIZE = 410;    { Terminal resize event    &0632 }
   KEY_EVENT = 411;     { We were interrupted by an event    &0633 }
   KEY_MAX = 511;       { Maximum key value is 0633    &0777 }

type
   //tnc_wacs= array [char] of cchar_t;
   tnc_wacs= array of cchar_t;
   pnc_wacs = ^tacs_map;

var
{$IFNDEF darwin}
   //_nc_wacs : pcchar_t cvar; external;
   _nc_wacs : tnc_wacs cvar; external;
{$ELSE darwin}
   //_nc_wacs : pcchar_t external libncurses name '_nc_wacs';
   _nc_wacs : tnc_wacs external libncurses name '_nc_wacs';
{$ENDIF darwin}

function NCURSES_WACS(c: chtype): cchar_t; inline;

function WACS_BSSB: cchar_t; inline;
function WACS_SSBB: cchar_t; inline;
function WACS_BBSS: cchar_t; inline;
function WACS_SBBS: cchar_t; inline;
function WACS_SBSS: cchar_t; inline;
function WACS_SSSB: cchar_t; inline;
function WACS_SSBS: cchar_t; inline;
function WACS_BSSS: cchar_t; inline;
function WACS_BSBS: cchar_t; inline;
function WACS_SBSB: cchar_t; inline;
function WACS_SSSS: cchar_t; inline;
function WACS_S1: cchar_t; inline;
function WACS_S9: cchar_t; inline;
function WACS_DIAMOND: cchar_t; inline;
function WACS_CKBOARD: cchar_t; inline;
function WACS_DEGREE: cchar_t; inline;
function WACS_PLMINUS: cchar_t; inline;
function WACS_BULLET: cchar_t; inline;
function WACS_LARROW: cchar_t; inline;
function WACS_RARROW: cchar_t; inline;
function WACS_DARROW: cchar_t; inline;
function WACS_UARROW: cchar_t; inline;
function WACS_BOARD: cchar_t; inline;
function WACS_LANTERN: cchar_t; inline;
function WACS_BLOCK: cchar_t; inline;
function WACS_S3: cchar_t; inline;
function WACS_S7: cchar_t; inline;
function WACS_LEQUAL: cchar_t; inline;
function WACS_GEQUAL: cchar_t; inline;
function WACS_PI: cchar_t; inline;
function WACS_NEQUAL: cchar_t; inline;
function WACS_STERLING: cchar_t; inline;

{$IFDEF FPC_OBJFPC}
property WACS_ULCORNER : cchar_t read WACS_BSSB;
property WACS_LLCORNER : cchar_t read WACS_SSBB;
property WACS_URCORNER : cchar_t read WACS_BBSS;
property WACS_LRCORNER : cchar_t read WACS_SBBS;
property WACS_RTEE : cchar_t read WACS_SBSS;
property WACS_LTEE : cchar_t read WACS_SSSB;
property WACS_BTEE : cchar_t read WACS_SSBS;
property WACS_TTEE : cchar_t read WACS_BSSS;
property WACS_HLINE : cchar_t read WACS_BSBS;
property WACS_VLINE : cchar_t read WACS_SBSB;
property WACS_PLUS : cchar_t read WACS_SSSS;
{$ENDIF FPC_OBJFPC}

(* mouse interface *)

(*  event masks  *)

const
   BUTTON1_RELEASED       = 1;
   BUTTON1_PRESSED        = 2;
   BUTTON1_CLICKED        = 4;
   BUTTON1_DOUBLE_CLICKED = 8;
   BUTTON1_TRIPLE_CLICKED = 16;

   BUTTON2_RELEASED       = 1 shl 6;
   BUTTON2_PRESSED        = 2 shl 6;
   BUTTON2_CLICKED        = 4 shl 6;
   BUTTON2_DOUBLE_CLICKED = 8 shl 6;
   BUTTON2_TRIPLE_CLICKED = 16 shl 6;

   BUTTON3_RELEASED       = 1 shl (2 * 6);
   BUTTON3_PRESSED        = 2 shl (2 * 6);
   BUTTON3_CLICKED        = 4 shl (2 * 6);
   BUTTON3_DOUBLE_CLICKED = 8 shl (2 * 6);
   BUTTON3_TRIPLE_CLICKED = 16 shl (2 * 6);

   BUTTON4_RELEASED       = 1 shl (3 * 6);
   BUTTON4_PRESSED        = 2 shl (3 * 6);
   BUTTON4_CLICKED        = 4 shl (3 * 6);
   BUTTON4_DOUBLE_CLICKED = 8 shl (3 * 6);
   BUTTON4_TRIPLE_CLICKED = 16 shl (3 * 6);

{$IF NCURSES_MOUSE_VERSION > 1}
   BUTTON5_RELEASED       = BUTTON1_RELEASED shl (4 * 6);
   BUTTON5_PRESSED        = BUTTON1_PRESSED shl (4 * 6);
   BUTTON5_CLICKED        = BUTTON1_CLICKED shl (4 * 6);
   BUTTON5_DOUBLE_CLICKED = BUTTON1_DOUBLE_CLICKED shl (4 * 6);
   BUTTON5_TRIPLE_CLICKED = BUTTON1_TRIPLE_CLICKED shl (4 * 6);

   BUTTON_CTRL            = 1 shl (5 * 6);
   BUTTON_SHIFT           = 2 shl (5 * 6);
   BUTTON_ALT             = 4 shl (5 * 6);
   REPORT_MOUSE_POSITION  = 8 shl (5 * 6);
{$ELSE}
   BUTTON1_RESERVED_EVENT = 32;
   BUTTON2_RESERVED_EVENT = 32 shl 6;
   BUTTON3_RESERVED_EVENT = 32 shl (2 * 6);
   BUTTON4_RESERVED_EVENT = 32 shl (3 * 6);

   BUTTON_CTRL            = 1 shl (4 * 6);
   BUTTON_SHIFT           = 2 shl (4 * 6);
   BUTTON_ALT             = 4 shl (4 * 6);
   REPORT_MOUSE_POSITION  = 8 shl (4 * 6);
{$ENDIF}

   ALL_MOUSE_EVENTS       = REPORT_MOUSE_POSITION - 1;

(* macros to extract single event-bits from masks *)

function BUTTON_RELEASE(e,x: longint): longint; inline;
function BUTTON_PRESS(e,x: longint): longint; inline;
function BUTTON_CLICK(e,x: longint): longint; inline;
function BUTTON_DOUBLE_CLICK(e,x: longint): longint; inline;
function BUTTON_TRIPLE_CLICK(e,x: longint): longint; inline;
function BUTTON_RESERVED_EVENT(e,x: longint): longint; inline;
function mouse_trafo(pY,pX: PLongint; to_screen: Bool): Bool; inline;

type
   PMEVENT = ^MEVENT;
   MEVENT = record
   id : Smallint;        { ID to distinguish multiple devices }
   x, y, z : Longint;    { event coordinates (character-cell) }
   bstate : mmask_t;     { button state bits }
end;

function getmouse(_para1:PMEVENT):longint; cdecl;external libncurses;
function ungetmouse(_para1:PMEVENT):longint; cdecl;external libncurses;
function mousemask(_para1:mmask_t; _para2:Pmmask_t):mmask_t;cdecl;external;
function wenclose(_para1:PWINDOW; _para2:Longint; _para3:Longint):Bool;cdecl;external;
function mouseinterval(_para1:Longint):longint; cdecl;external libncurses;
function wmouse_trafo(_para1:PWINDOW; _para2:PLongint; _para3:PLongint; _para4:Bool):Bool;cdecl;external;

{
 wide-character (enhanced) functionality
}

function add_wch(wch: pcchar_t): longint; inline;
function add_wchnstr(wchstr: pcchar_t; n: longint): longint; inline;
function add_wchstr(wchstr: pcchar_t): longint; inline;
function addnwstr(wchstr: pwchar_t; n : longint) : longint; inline;
function addwstr(wchstr: pwchar_t): longint; inline;
function bkgrnd(wch: pcchar_t): longint; inline;
procedure bkgrndset(wch: pcchar_t); inline;
function border_set(ls,rs,ts,bs,tl,tr,bl,br: pcchar_t): longint; inline;
function box_set(win: PWINDOW; verch,horch: pcchar_t): longint; inline;
function echo_wchar(wch: Pcchar_t): longint; inline;
function get_wch(wch: Plongint): longint; inline;
function get_wstr(wstr: PLongint): longint; inline;
function getbkgrnd(wch: pcchar_t): longint; inline;
function getn_wstr(wstr: PLongint; n: longint): longint; inline;
function hline_set(wch: pcchar_t; n: longint): longint; inline;
function in_wch(wch: pcchar_t) : longint; inline;
function in_wchnstr(wchstr: pcchar_t; n: longint): longint; inline;
function in_wchstr(wchstr: pcchar_t): longint; inline;
function innwstr(wstr: pwchar_t; n: longint): longint; inline;
function ins_nwstr(wstr: pwchar_t; n: longint): longint; inline;
function ins_wch(wch: pcchar_t): longint; inline;
function ins_wstr(wstr: pwchar_t): longint; inline;
function inwstr(wstr: pwchar_t): longint; inline;
function vline_set(wch: pcchar_t; n: longint): longint; inline;
function wadd_wchstr(win: PWINDOW; wchstr: pcchar_t): longint; inline;
function waddwstr(win: PWINDOW; wstr :pwchar_t): longint; inline;
function wget_wstr(win: PWINDOW; wstr: PLongint): longint; inline;
function win_wchstr(win: PWINDOW; wchstr: pcchar_t): longint; inline;
function wins_wstr(win: PWINDOW; wstr: pwchar_t): longint; inline;
function mvadd_wch(y,x: Smallint;  wch: pcchar_t): longint; inline;
function mvadd_wchnstr(y,x: Smallint; wchstr: pcchar_t; n: longint): longint; inline;
function mvadd_wchstr(y,x: Smallint; wchstr: pcchar_t): longint; inline;
function mvaddnwstr(y,x: Smallint; wstr: pwchar_t; n: longint): longint; inline;
function mvaddwstr(y,x: Smallint; wstr: pwchar_t): longint; inline;
function mvget_wch(y,x: Smallint; wch: Plongint): longint; inline;
function mvget_wstr(y,x: Smallint; wstr: Plongint): longint; inline;
function mvgetn_wstr(y,x: Smallint; wstr: Plongint; n: longint): longint; inline;
function mvhline_set(y,x: Smallint; wch: pcchar_t; n: longint): longint; inline;
function mvin_wch(y,x: Smallint; wch: pcchar_t) : longint; inline;
function mvin_wchnstr(y,x: Smallint; wchstr: pcchar_t; n: longint): longint; inline;
function mvin_wchstr(y,x: Smallint; wchstr: pcchar_t): longint; inline;
function mvinnwstr(y,x: Smallint; wstr: pwchar_t; n : longint): longint; inline;
function mvins_nwstr(y,x: Smallint; wstr: pwchar_t; n : longint): longint; inline;
function mvins_wch(y,x: Smallint; wch: pcchar_t): longint; inline;
function mvins_wstr(y,x: Smallint; wstr: pwchar_t): longint; inline;
function mvinwstr(y,x: Smallint; wstr: pwchar_t): longint; inline;
function mvvline_set(y,x: Smallint; wch: pcchar_t; n : longint): longint; inline;
function mvwadd_wch(win: PWINDOW; y,x: Smallint; wch: pcchar_t): longint; inline;
function mvwadd_wchnstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t; n: longint): longint; inline;
function mvwadd_wchstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t): longint; inline;
function mvwaddnwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t; n: longint): longint; inline;
function mvwaddwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t): longint; inline;
function mvwget_wch(win: PWINDOW; y,x: Smallint; wch: Plongint): longint; inline;
function mvwget_wstr(win: PWINDOW; y,x: Smallint; wstr: Plongint): longint; inline;
function mvwgetn_wstr(win: PWINDOW; y,x: Smallint; wstr: Plongint; n: longint): longint; inline;
function mvwhline_set(win: PWINDOW; y,x: Smallint; wch: pcchar_t; n: longint): longint; inline;
function mvwin_wch(win: PWINDOW; y,x: Smallint; wch: pcchar_t): longint; inline;
function mvwin_wchnstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t; n: longint): longint; inline;
function mvwin_wchstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t): longint; inline;
function mvwinnwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t; n : longint): longint; inline;
function mvwins_nwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t; n: longint): longint; inline;
function mvwins_wch(win: PWINDOW; y,x: Smallint; wch: pcchar_t): longint; inline;
function mvwins_wstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t) : longint; inline;
function mvwinwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t) : longint; inline;
function mvwvline_set(win: PWINDOW; y,x: Smallint; wch: pcchar_t; n: longint) : longint; inline;

function wmove(win: PWINDOW; y,x: Smallint): Longint; inline;

(* C varargs  procedures*)

function tparm(_para1:PChar):PChar;cdecl; varargs; external libncurses; overload;
function mvprintw(_para1:Longint; _para2:Longint; _para3:PChar):Longint; cdecl; varargs; external libncurses; overload;
function mvscanw(_para1:Longint; _para2:Longint; _para3:PChar):Longint; cdecl; varargs;external libncurses; overload;
function mvwprintw(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:PChar):Longint; cdecl; varargs; external libncurses; overload;
function mvwscanw(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:PChar):Longint; cdecl; varargs; external libncurses; overload;
function scanw(_para1:PChar):Longint; cdecl; varargs; external libncurses; overload;
function wprintw(_para1:PWINDOW; _para2:PChar):Longint; cdecl; varargs; external libncurses; overload;
function wscanw(_para1:PWINDOW; _para2:PChar):Longint; cdecl; varargs; external libncurses; overload;
function printw(_para1:PChar):Longint; cdecl; varargs; external libncurses; overload;
{function vwprintw(_para1:PWINDOW; _para2:PChar; _para3:va_list):Longint; cdecl;external libncurses;
function vw_printw(_para1:PWINDOW; _para2:PChar; _para3:va_list):Longint; cdecl;external libncurses;
function vwscanw(_para1:PWINDOW; _para2:PChar; _para3:va_list):Longint; cdecl;external libncurses;
function vw_scanw(_para1:PWINDOW; _para2:PChar; _para3:va_list):Longint; cdecl;external libncurses;}

{$IF DEFINED(FPC_OBJFPC) OR DEFINED(FPC_DELPHI)}
{function tparm(_para1:PChar; args:array of const):PChar;cdecl;external libncurses; overload;
function mvprintw(_para1:Longint; _para2:Longint; _para3:PChar; args:array of const):Longint; cdecl;external libncurses; overload;
function mvscanw(_para1:Longint; _para2:Longint; _para3:PChar; args:array of const):Longint; cdecl;external libncurses; overload;
function mvwprintw(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:PChar; args:array of const):Longint; cdecl;external libncurses; overload;
function mvwscanw(_para1:PWINDOW; _para2:Longint; _para3:Longint; _para4:PChar; args:array of const):Longint; cdecl;external libncurses; overload;
function scanw(_para1:PChar; args:array of const):Longint; cdecl;external libncurses; overload;
function wprintw(_para1:PWINDOW; _para2:PChar; args:array of const):Longint; cdecl;external libncurses; overload;
function wscanw(_para1:PWINDOW; _para2:PChar; args:array of const):Longint; cdecl;external libncurses; overload;
function printw(_para1:PChar; args:array of const):Longint; cdecl;external libncurses; overload;}

{$ENDIF}

implementation

function ACS_ULCORNER : chtype;
begin
  ACS_ULCORNER:=acs_map['l'];
end;

function ACS_LLCORNER : chtype;
begin
  ACS_LLCORNER:=acs_map['m'];
end;

function ACS_URCORNER : chtype;
begin
  ACS_URCORNER:=acs_map['k'];
end;

function ACS_LRCORNER : chtype;
begin
  ACS_LRCORNER:=acs_map['j'];
end;

function ACS_LTEE : chtype;
begin
  ACS_LTEE:=acs_map['t'];
end;

function ACS_RTEE : chtype;
begin
  ACS_RTEE:=acs_map['u'];
end;

function ACS_BTEE : chtype;
begin
  ACS_BTEE:=acs_map['v'];
end;

function ACS_TTEE : chtype;
begin
  ACS_TTEE:=acs_map['w'];
end;

function ACS_HLINE : chtype;
begin
  ACS_HLINE:=acs_map['q'];
end;

function ACS_VLINE : chtype;
begin
  ACS_VLINE:=acs_map['x'];
end;

function ACS_PLUS : chtype;
begin
  ACS_PLUS:=acs_map['n'];
end;

function ACS_S1      : chtype;
begin
  ACS_S1    :=acs_map['o'];
end;

function ACS_S9      : chtype;
begin
  ACS_S9    :=acs_map['s'];
end;

function ACS_DIAMOND : chtype;
begin
  ACS_DIAMOND:=acs_map['`'];
end;

function ACS_CKBOARD : chtype;
begin
  ACS_CKBOARD:=acs_map['a'];
end;

function ACS_DEGREE : chtype;
begin
  ACS_DEGREE:=acs_map['f'];
end;

function ACS_PLMINUS : chtype;
begin
  ACS_PLMINUS:=acs_map['g'];
end;

function ACS_BULLET : chtype;
begin
  ACS_BULLET:=acs_map['~'];
end;

function ACS_LARROW : chtype;
begin
  ACS_LARROW:=acs_map[','];
end;

function ACS_RARROW : chtype;
begin
  ACS_RARROW:=acs_map['+'];
end;

function ACS_DARROW : chtype;
begin
  ACS_DARROW:=acs_map['.'];
end;

function ACS_UARROW : chtype;
begin
  ACS_UARROW:=acs_map['-'];
end;

function ACS_BOARD : chtype;
begin
  ACS_BOARD:=acs_map['h'];
end;

function ACS_LANTERN : chtype;
begin
  ACS_LANTERN:=acs_map['i'];
end;

function ACS_BLOCK : chtype;
begin
  ACS_BLOCK:=acs_map['0'];
end;

function ACS_S3      : chtype;
begin
  ACS_S3    :=acs_map['p'];
end;

function ACS_S7      : chtype;
begin
  ACS_S7    :=acs_map['r'];
end;

function ACS_LEQUAL : chtype;
begin
  ACS_LEQUAL:=acs_map['y'];
end;

function ACS_GEQUAL : chtype;
begin
  ACS_GEQUAL:=acs_map['z'];
end;

function ACS_PI      : chtype;
begin
  ACS_PI    :=acs_map['{'];
end;

function ACS_NEQUAL : chtype;
begin
  ACS_NEQUAL:=acs_map['|'];
end;

function ACS_STERLING : chtype;
begin
  ACS_STERLING:=acs_map['}'];
end;

function COLOR_PAIR(n : longint): longint;
begin
  COLOR_PAIR:=n shl 8;
end;

function PAIR_NUMBER(attr: attr_t) : longint;
begin
  PAIR_NUMBER := (attr AND A_COLOR) shr 8
//#define PAIR_NUMBER(a)  (NCURSES_CAST(int,(((a) & A_COLOR) >> NCURSES_ATTR_SHIFT)))
end;

function color_set(color_pair_number: Smallint; opts: Pointer): longint; inline;
begin
  color_set:=wcolor_set(stdscr,color_pair_number,opts);
end;

function wgetstr(win: PWINDOW; s: PChar): Longint;
begin
  wgetstr := wgetnstr(win, s, -1);
end;

function getnstr(s: PChar;n: Longint): Longint;
begin
  getnstr := wgetnstr(stdscr,s,n);
end;

{
  setupterm are declared in <ncursesw/term.h>:
  int setupterm(char *term, int fildes, int *errret);
}
function setupterm(term:PChar; fildes:Longint; errret:Plongint):Longint; cdecl;external libncurses;

function setterm(term: PChar): Longint;
begin
  setterm:=setupterm(term,1,plongint(0));
end;

function fixterm: Longint;
begin
  fixterm:=reset_prog_mode;
end;

function resetterm: Longint;
begin
  resetterm:=reset_shell_mode;
end;

function saveterm: Longint;
begin
  saveterm:=def_prog_mode;
end;

function crmode: Longint;
begin
  crmode:=cbreak;
end;

function nocrmode: Longint;
begin
  nocrmode:=nocbreak;
end;

procedure getyx(win: PWINDOW; var y,x: Smallint);
begin
   if win<>nil then
   begin
      y := win^._cury; x := win^._curx
   end
   else
   begin
      y := ERR; x := ERR
   end
end;

procedure getbegyx(win: PWINDOW; var y,x: Smallint);
begin
   if win<>nil then
   begin
      y := win^._begy; x := win^._begx
   end
   else
   begin
      y := ERR; x := ERR
   end
end;

procedure getmaxyx(win: PWINDOW; var y,x: Smallint);
begin
   if win<>nil then
   begin
     y := win^._maxy+1; x := win^._maxx+1
   end
   else
   begin
     y := ERR; x := ERR
   end
end;

procedure getparyx(win: PWINDOW; var y,x: Smallint);
begin
  if win<>nil then
   begin
     y := win^._pary; x :=win^._parx
   end
   else
   begin
     y := ERR; x := ERR
   end
end;

procedure getsyx(var y,x: Smallint);
begin
{$IFNDEF USE_FPC_BYTEBOOL}
   if newscr^._leaveok = NC_FPC_TRUE then
{$ELSE USE_FPC_BYTEBOOL}
   if newscr^._leaveok then
{$ENDIF USE_FPC_BYTEBOOL}
   begin
     y := -1; x := -1
   end
   else
   begin
     y := newscr^._cury; x := newscr^._curx
   end
end;

procedure getyx(win: PWINDOW; var y,x: Longint);
begin
   if win<>nil then
   begin
      y := win^._cury; x := win^._curx
   end
   else
   begin
      y := ERR; x := ERR
   end
end;

procedure getbegyx(win: PWINDOW; var y,x: Longint);
begin
   if win<>nil then
   begin
      y := win^._begy; x := win^._begx
   end
   else
   begin
      y := ERR; x := ERR
   end
end;

procedure getmaxyx(win: PWINDOW; var y,x: Longint);
begin
   if win<>nil then
   begin
     y := win^._maxy+1; x := win^._maxx+1
   end
   else
   begin
     y := ERR; x := ERR
   end
end;

procedure getparyx(win: PWINDOW; var y,x: Longint);
begin
  if win<>nil then
   begin
     y := win^._pary; x :=win^._parx
   end
   else
   begin
     y := ERR; x := ERR
   end
end;

procedure getsyx(var y,x: Longint);
begin
{$IFNDEF USE_FPC_BYTEBOOL}
   if newscr^._leaveok = NC_FPC_TRUE then
{$ELSE USE_FPC_BYTEBOOL}
   if newscr^._leaveok then
{$ENDIF USE_FPC_BYTEBOOL}
   begin
     y := -1; x := -1
   end
   else
   begin
     y := newscr^._cury; x := newscr^._curx
   end
end;

procedure setsyx(y,x: Smallint);
begin
(*
C macros:
#define setsyx(y,x) do { if((y)==-1 && (x)==-1) newscr->_leaveok=TRUE; \
       else {newscr->_leaveok=FALSE;wmove(newscr,(y),(x));} \
        } while(0)
*)
   if (x>=0) and (y >=0) AND (x<=newscr^._maxx) AND (y<=newscr^._maxy) then
   begin
      newscr^._curx := x;
      newscr^._cury := y;
      newscr^._flags := newscr^._flags AND not _WRAPPED;
      newscr^._flags := newscr^._flags OR _HASMOVED;
      newscr^._leaveok := NC_FPC_FALSE;
   end
   else if (x OR y = -1) then
      newscr^._leaveok := NC_FPC_TRUE

  //stdscr^._cury := y; //fpc ncurses original
  //stdscr^._curx := x;
end;

function getattrs(win: PWINDOW): attr_t;
begin
  if win<>nil then
    getattrs:=win^._attrs
  else
    getattrs:=A_NORMAL;
end;

function getcurx(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getcurx:=win^._curx
  else
    getcurx:=ERR;
end;

function getcury(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getcury:=win^._cury
  else
    getcury:=ERR;
end;

function getbegx(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getbegx:=win^._begx
  else
    getbegx:=ERR;
end;

function getbegy(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getbegy:=win^._begy
  else
    getbegy:=ERR;
end;

function getmaxx(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getmaxx:=(win^._maxx) + 1
  else
    getmaxx:=ERR;
end;

function getmaxy(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getmaxy:=(win^._maxy) + 1
  else
    getmaxy:=ERR;
end;

function getparx(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getparx:=win^._parx
  else
    getparx:=ERR;
end;

function getpary(win: PWINDOW): Smallint;
begin
  if win<>nil then
    getpary:=win^._pary
  else
    getpary:=ERR;
end;

function wstandout(win: PWINDOW): Longint;
begin
  wstandout:=wattrset(win,A_STANDOUT);
end;

function wstandend(win: PWINDOW): Longint;
begin
  wstandend:=wattrset(win,A_NORMAL);
end;

function wattr_set(win: PWINDOW; attrs: attr_t; pair: Smallint; opts: Pointer): Longint;
begin
{ The parameter opts is reserved for future use, applications must supply a null pointer. }
{
 C macros
#define wattr_set(win,a,p,opts)   ((win)->_attrs = (((a) & ~A_COLOR) | COLOR_PAIR(p)), OK)
}
  win^._attrs := (attrs AND not A_COLOR) OR COLOR_PAIR(pair);
  wattr_set := OK
end;

function wattr_get(win:PWINDOW; attrs:Pattr_t; pair:PSmallint; opts:Pointer):longint;
begin
{
 C macros
#define wattr_get(win,a,p,opts)   ((void)((a) != 0 && (*(a) = (win)->_attrs)), \
           (void)((p) != 0 && (*(p) = PAIR_NUMBER((win)->_attrs))), \
           OK)
}
    attrs^ := win^._attrs;
    pair^ := PAIR_NUMBER(win^._attrs);
    wattr_get := OK
end;

function wattron(win:PWINDOW; attrs: attr_t): Longint;
begin
  wattron:=wattr_on(win, attrs, nil);
end;

function wattroff(win:PWINDOW; attrs: attr_t): longint;
begin
  wattroff:=wattr_off(win, attrs, nil);
end;

function wattrset(win: PWINDOW; attrs: attr_t): longint;
begin
  win^._attrs := attrs;
  wattrset := OK
end;

function scroll(win: PWINDOW): longint;
begin
  scroll:=wscrl(win,1);
end;

function touchwin(win: PWINDOW): longint;
begin
  touchwin:=wtouchln(win,0,getmaxy(win),1);
end;

function touchline(win: PWINDOW;s,c: longint): longint;
begin
  touchline:=wtouchln(win,s,c,1);
end;

function untouchwin(win: PWINDOW): longint;
begin
  untouchwin:=wtouchln(win,0,getmaxy(win),0);
end;

function box(win:PWINDOW; v,h :chtype):Longint;
begin
  box:=wborder(win,v,v,h,h,0,0,0,0);
end;

function border(ls,rs,ts,bs,tl,tr,bl,br: chtype): longint;
begin
  border:=wborder(stdscr,ls,rs,ts,bs,tl,tr,bl,br);
end;

function hline(ch:chtype; n:longint): longint;
begin
  hline:=whline(stdscr,ch,n);
end;

function vline(ch:chtype; n:longint): longint;
begin
  vline:=wvline(stdscr,ch,n);
end;


function winstr(win: PWINDOW;s: PChar): longint;
begin
  winstr:=winnstr(win,s,-(1));
end;

function winchstr(win: PWINDOW; chstr: pchtype): longint;
begin
  winchstr:=winchnstr(win,chstr,-(1));
end;

function winsstr(win: PWINDOW;s: PChar): longint;
begin
  winsstr:=winsnstr(win,s,-(1));
end;

function redrawwin(win: PWINDOW): longint;
begin
  redrawwin:=wredrawln(win,0,(win^._maxy)+1);
end;

function waddstr(win: PWINDOW;st: PChar): longint;
begin
  waddstr:=waddnstr(win,st,-(1));
end;

function waddchstr(win: PWINDOW; chstr: pchtype): longint;
begin
  waddchstr:=waddchnstr(win,chstr,-(1));
end;

{
   pseudo functions for standard screen
}
function addch(ch: chtype): longint;
begin
  addch:=waddch(stdscr,ch);
end;

function addchnstr(chstr: pchtype; n: longint): longint;
begin
  addchnstr:=waddchnstr(stdscr,chstr,n);
end;

function addchstr(chstr: pchtype): longint;
begin
  addchstr:=waddchstr(stdscr,chstr);
end;

function addnstr(str: PChar;n: longint): longint;
begin
  addnstr:=waddnstr(stdscr,str,n);
end;

function addstr(str: PChar): longint;
begin
  addstr:=waddnstr(stdscr,str,-(1));
end;

function attroff(attrs: attr_t): longint;
begin
  attroff:=wattroff(stdscr,attrs);
end;

function attron(attrs: attr_t): longint;
begin
  attron:=wattron(stdscr,attrs);
end;

function attrset(attrs: attr_t): longint;
begin
  //attrset:=wattrset(stdscr,attrs);
  stdscr^._attrs := attrs;
  attrset := OK
end;

function bkgd(ch: chtype): longint;
begin
  bkgd:=wbkgd(stdscr,ch);
end;

procedure bkgdset(ch: chtype);
begin
  wbkgdset(stdscr,ch);
end;

function clear: longint;
begin
  clear:=wclear(stdscr);
end;

function clrtobot: longint;
begin
  clrtobot:=wclrtobot(stdscr);
end;

function clrtoeol: longint;
begin
  clrtoeol:=wclrtoeol(stdscr);
end;

function delch: longint;
begin
  delch:=wdelch(stdscr);
end;

function deleteln: longint;
begin
  deleteln:=winsdelln(stdscr,-(1));
end;

function echochar(ch: chtype): longint;
begin
  echochar:=wechochar(stdscr,ch);
end;

function erase: longint;
begin
  erase:=werase(stdscr);
end;

function getch: longint;
begin
  getch:=wgetch(stdscr);
end;

function getstr(str: PChar): longint;
begin
  getstr:=wgetstr(stdscr,str);
end;

function inch: chtype;
begin
  inch:=winch(stdscr);
end;

function inchnstr(chstr: pchtype;n: longint): longint;
begin
  inchnstr:=winchnstr(stdscr,chstr ,n);
end;

function inchstr(chstr: pchtype): longint;
begin
  inchstr:=winchstr(stdscr,chstr);
end;

function innstr(str: PChar;n: longint): longint;
begin
  innstr:=winnstr(stdscr,str,n);
end;

function insch(ch: chtype): longint;
begin
  insch:=winsch(stdscr,ch);
end;

function insdelln(n: longint): longint;
begin
  insdelln:=winsdelln(stdscr,n);
end;

function insertln: longint;
begin
  insertln:=winsdelln(stdscr,1);
end;

function insnstr(str: PChar;n: longint): longint;
begin
  insnstr:=winsnstr(stdscr,str,n);
end;

function insstr(str: PChar): longint;
begin
  insstr:=winsstr(stdscr,str);
end;

function instr(str: PChar): longint;
begin
  instr:=winstr(stdscr,str);
end;

function move(y,x: Smallint): longint;
begin
  if (x OR y >=0)AND(x<=stdscr^._maxx)AND(y<=stdscr^._maxy) then
  begin
    stdscr^._curx := x;
    stdscr^._cury := y;
    stdscr^._flags := stdscr^._flags AND not _WRAPPED;
    stdscr^._flags := stdscr^._flags OR _HASMOVED;
    move := OK
  end
  else
    move := ERR
end;

function refresh: longint;
begin
   refresh:=wrefresh(stdscr);
end;

function scrl(n: longint): longint;
begin
   scrl:=wscrl(stdscr,n);
end;

function setscrreg(t,b: longint): longint;
begin
   setscrreg:=wsetscrreg(stdscr,t,b);
end;

function standend: longint;
begin
   standend:=wstandend(stdscr);
end;

function standout: longint;
begin
   standout:=wstandout(stdscr);
end;

procedure timeout(delay: longint);
begin
  wtimeout(stdscr,delay);
end;

function wdeleteln(win: PWINDOW): longint;
begin
   wdeleteln:=winsdelln(win,-(1));
end;

function winsertln(win: PWINDOW): longint;
begin
   winsertln:=winsdelln(win,1);
end;

{
   mv functions
 }
function mvwaddch(win: PWINDOW;y,x: Smallint; ch: chtype): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwaddch := ERR
  else
    mvwaddch := waddch(win,ch)
end;

function mvwaddchnstr(win: PWINDOW;y,x: Smallint;chstr: pchtype;n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwaddchnstr := ERR
  else
    mvwaddchnstr := waddchnstr(win,chstr,n);
end;

function mvwaddchstr(win: PWINDOW;y,x: Smallint;chstr: pchtype): longint;
begin
  if wmove(win,y,x) = ERR then
     mvwaddchstr := ERR
  else
     mvwaddchstr := waddchnstr(win,chstr,-(1))
end;

function mvwaddnstr(win: PWINDOW;y,x: Smallint;str: PChar;n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
     mvwaddnstr := ERR
  else
     mvwaddnstr := waddnstr(win,str,n);
end;

function mvwaddstr(win: PWINDOW;y,x: Smallint;str: PChar): longint;
begin
  if wmove(win,y,x) = ERR then
     mvwaddstr := ERR
  else
     mvwaddstr := waddnstr(win,str,-(1));
end;

function mvwdelch(win: PWINDOW;y,x: Smallint): longint;
begin
  if wmove(win,y,x) = ERR then
     mvwdelch := ERR
  else
     mvwdelch :=wdelch(win);
end;

function mvwchgat(win: PWINDOW;y, x: Smallint; n: Longint;attr: attr_t;
                            color: Smallint; opts: Pointer): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwchgat := ERR
  else
    mvwchgat := wchgat(win,n,attr,color,opts);
end;

function mvwgetch(win: PWINDOW;y,x: Smallint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwgetch := ERR
  else
    mvwgetch := wgetch(win);
end;

function mvwgetnstr(win: PWINDOW;y,x: Smallint;str: PChar;n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwgetnstr := ERR
  else
    mvwgetnstr := wgetnstr(win,str,n);
end;

function mvwgetstr(win: PWINDOW;y,x: Smallint;str: PChar): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwgetstr := ERR
  else
    mvwgetstr := wgetstr(win,str);
end;

function mvwhline(win: PWINDOW;y,x: Smallint;ch: chtype;n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwhline := ERR
  else
    mvwhline := whline(win,ch,n);
end;

function mvwinch(win: PWINDOW;y,x: Smallint): chtype;
begin
  if wmove(win,y,x) = ERR then
    mvwinch := chtype(ERR)
  else
    mvwinch := winch(win);
end;

function mvwinchnstr(win: PWINDOW;y,x: Smallint;chstr: pchtype; n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinchnstr := ERR
  else
    mvwinchnstr := winchnstr(win,chstr,n);
end;

function mvwinchstr(win: PWINDOW;y,x: Smallint;chstr: pchtype): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinchstr := ERR
  else
    mvwinchstr := winchnstr(win,chstr,-(1));
end;

function mvwinnstr(win: PWINDOW;y,x: Smallint;str: PChar;n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinnstr := ERR
  else
    mvwinnstr := winnstr(win,str,n);
end;

function mvwinsch(win: PWINDOW;y,x: Smallint;ch: chtype): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinsch := ERR
  else
    mvwinsch := winsch(win,ch);
end;

function mvwinsnstr(win: PWINDOW;y,x: Smallint;str: PChar;n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinsnstr := ERR
  else
    mvwinsnstr := winsnstr(win,str,n);
end;

function mvwinsstr(win: PWINDOW;y,x: Smallint;str: PChar): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinsstr := ERR
  else
    mvwinsstr := winsnstr(win,str,-(1));
end;

function mvwinstr(win: PWINDOW;y,x: Smallint;str: PChar): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinstr := ERR
  else
    mvwinstr := winnstr(win,str,-(1));
end;

function mvwvline(win: PWINDOW;y,x: Smallint;ch: chtype;n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwvline := ERR
  else
    mvwvline := wvline(win,ch,n);
end;

function mvaddch(y,x: Smallint; ch: chtype): longint;
begin
  if move(y,x) = ERR then
    mvaddch := ERR
  else
    mvaddch := waddch(stdscr,ch)
end;

function mvaddchnstr(y,x: Smallint; chstr: pchtype;n: longint): longint;
begin
  if move(y,x) = ERR then
    mvaddchnstr := ERR
  else
    mvaddchnstr := waddchnstr(stdscr,chstr,n);
end;

function mvaddchstr(y,x: Smallint; chstr: pchtype): longint;
begin
  if move(y,x) = ERR then
    mvaddchstr := ERR
  else
    mvaddchstr := waddchnstr(stdscr,chstr,-(1));
end;

function mvaddnstr(y,x: Smallint; str: PChar;n: longint): longint;
begin
  if move(y,x) = ERR then
    mvaddnstr := ERR
  else
    mvaddnstr := waddnstr(stdscr,str,n);
end;

function mvaddstr(y,x: Smallint; str: PChar): longint;
begin
  if move(y,x) = ERR then
    mvaddstr := ERR
  else
    mvaddstr := waddnstr(stdscr,str,-(1));
end;

function mvdelch(y,x: Smallint): longint;
begin
  if move(y,x) = ERR then
    mvdelch := ERR
  else
    mvdelch := wdelch(stdscr);
end;

function mvgetch(y,x: Smallint): longint;
begin
  if move(y,x) = ERR then
    mvgetch := ERR
  else
    mvgetch := wgetch(stdscr);
end;

function mvgetnstr(y,x: Smallint; str: PChar;n: longint): longint;
begin
  if move(y,x) = ERR then
    mvgetnstr := ERR
  else
    mvgetnstr := wgetnstr(stdscr,str,n);
end;

function mvgetstr(y,x: Smallint; str: PChar): longint;
begin
  if move(y,x) = ERR then
    mvgetstr := ERR
  else
    mvgetstr := wgetnstr(stdscr,str,-(1));
end;

function mvhline(y,x: Smallint;ch: chtype;n: longint): longint;
begin
  if move(y,x) = ERR then
    mvhline := ERR
  else
    mvhline := whline(stdscr,ch,n);
end;

function mvinch(y,x: Smallint): chtype;
begin
  if move(y,x) = ERR then
    mvinch := chtype(ERR)
  else
    mvinch := winch(stdscr);
end;

function mvinchnstr(y,x: Smallint; chstr: pchtype;n: longint): longint;
begin
  if move(y,x) = ERR then
    mvinchnstr := ERR
  else
    mvinchnstr := winchnstr(stdscr,chstr,n);
end;

function mvinchstr(y,x: Smallint; chstr: pchtype): longint;
begin
  if move(y,x) = ERR then
    mvinchstr := ERR
  else
    mvinchstr := winchnstr(stdscr,chstr,-(1));
end;

function mvinnstr(y,x: Smallint; str: PChar;n: longint): longint;
begin
  if move(y,x) = ERR then
    mvinnstr := ERR
  else
    mvinnstr := winnstr(stdscr,str,n);
end;

function mvinsch(y,x: Smallint;ch: chtype): longint;
begin
  if move(y,x) = ERR then
    mvinsch := ERR
  else
    mvinsch := winsch(stdscr,ch);
end;

function mvinsnstr(y,x: Smallint; str: PChar;n: longint): longint;
begin
  if move(y,x) = ERR then
    mvinsnstr := ERR
  else
    mvinsnstr := winsnstr(stdscr,str,n);
end;

function mvinsstr(y,x: Smallint; str: PChar): longint;
begin
  if move(y,x) = ERR then
    mvinsstr := ERR
  else
    mvinsstr := winsstr(stdscr,str);
end;

function mvinstr(y,x: Smallint; str: PChar): longint;
begin
  if move(y,x) = ERR then
    mvinstr := ERR
  else
    mvinstr := winstr(stdscr,str);
end;

function mvvline(y,x: Smallint; ch:chtype; n:longint): longint;
begin
  if move(y,x) = ERR then
    mvvline := ERR
  else
    mvvline := wvline(stdscr,ch,n);
end;

function attr_get(attrs:Pattr_t; pair:PSmallint; opts:Pointer): longint;
begin
  attr_get := wattr_get(stdscr,attrs,pair,opts);
end;

function attr_off(attrs:attr_t; opts:Pointer): longint;
begin
  attr_off := wattr_off(stdscr,attrs,opts);
end;

function attr_on(attrs:attr_t; opts:Pointer): longint;
begin
  attr_on := wattr_on(stdscr,attrs,opts);
end;

function attr_set(attrs:attr_t; pair:Smallint; opts:Pointer): longint;
begin
  attr_set := wattr_set(stdscr,attrs,pair,opts);
end;

function chgat(n: Longint;attr: attr_t;color: Smallint; opts: Pointer): longint;
begin
  chgat := wchgat(stdscr,n,attr,color,opts);
end;

function mvchgat(y, x: Smallint; n: Longint;attr: attr_t;color: Smallint; opts: Pointer):Longint;
begin
  if move(y,x) = ERR then
    mvchgat := ERR
  else
    mvchgat := wchgat(stdscr,n,attr,color,opts);
end;

function getbkgd(win: PWINDOW): chtype;
begin
  getbkgd := win^._bkgd;
end;

function slk_attr_off(attrs: attr_t; opts: Pointer) : longint;
begin
  slk_attr_off := slk_attroff(attrs);
end;

function slk_attr_on(attrs: attr_t; opts: Pointer): longint;
begin
  slk_attr_on := slk_attron(attrs);
end;




function KEY_F(n : Byte) : chtype;
begin
  KEY_F := KEY_F0 + n;
end;



function NCURSES_WACS(c: chtype): cchar_t;
begin
  NCURSES_WACS := _nc_wacs[c];
end;

function WACS_BSSB: cchar_t;
begin
  WACS_BSSB := _nc_wacs[chtype('l')];
end;

function WACS_SSBB: cchar_t;
begin
  WACS_SSBB:= _nc_wacs[chtype('m')];
end;

function WACS_BBSS: cchar_t;
begin
  WACS_BBSS:= _nc_wacs[chtype('k')];
end;

function WACS_SBBS: cchar_t;
begin
  WACS_SBBS:= _nc_wacs[chtype('j')];
end;

function WACS_SBSS: cchar_t;
begin
  WACS_SBSS:= _nc_wacs[chtype('u')];
end;

function WACS_SSSB: cchar_t;
begin
  WACS_SSSB:= _nc_wacs[chtype('t')];
end;

function WACS_SSBS: cchar_t;
begin
  WACS_SSBS:= _nc_wacs[chtype('v')];
end;

function WACS_BSSS: cchar_t;
begin
  WACS_BSSS:= _nc_wacs[chtype('w')];
end;

function WACS_BSBS: cchar_t;
begin
  WACS_BSBS:= _nc_wacs[chtype('q')];
end;

function WACS_SBSB: cchar_t;
begin
  WACS_SBSB:= _nc_wacs[chtype('x')];
end;

function WACS_SSSS: cchar_t;
begin
  WACS_SSSS:= _nc_wacs[chtype('n')];
end;

function WACS_S1: cchar_t;
begin
  WACS_S1:= _nc_wacs[chtype('o')];
end;

function WACS_S9: cchar_t;
begin
  WACS_S9:= _nc_wacs[chtype('s')];
end;

function WACS_DIAMOND: cchar_t;
begin
  WACS_DIAMOND:= _nc_wacs[chtype('`')];
end;

function WACS_CKBOARD: cchar_t;
begin
  WACS_CKBOARD:= _nc_wacs[chtype('a')];
end;

function WACS_DEGREE: cchar_t;
begin
  WACS_DEGREE:= _nc_wacs[chtype('f')];
end;

function WACS_PLMINUS: cchar_t;
begin
  WACS_PLMINUS:= _nc_wacs[chtype('g')];
end;

function WACS_BULLET: cchar_t;
begin
  WACS_BULLET:= _nc_wacs[chtype('~')];
end;

function WACS_LARROW: cchar_t;
begin
  WACS_LARROW:= _nc_wacs[chtype(',')];
end;

function WACS_RARROW: cchar_t;
begin
  WACS_RARROW:= _nc_wacs[chtype('+')];
end;

function WACS_DARROW: cchar_t;
begin
  WACS_DARROW:= _nc_wacs[chtype('.')];
end;

function WACS_UARROW: cchar_t;
begin
  WACS_UARROW:= _nc_wacs[chtype('-')];
end;

function WACS_BOARD: cchar_t;
begin
  WACS_BOARD:= _nc_wacs[chtype('h')];
end;

function WACS_LANTERN: cchar_t;
begin
  WACS_LANTERN:= _nc_wacs[chtype('i')];
end;

function WACS_BLOCK: cchar_t;
begin
  WACS_BLOCK:= _nc_wacs[chtype('0')];
end;

function WACS_S3: cchar_t;
begin
  WACS_S3:= _nc_wacs[chtype('p')];
end;

function WACS_S7: cchar_t;
begin
  WACS_S7:= _nc_wacs[chtype('r')];
end;

function WACS_LEQUAL: cchar_t;
begin
  WACS_LEQUAL:= _nc_wacs[chtype('y')];
end;

function WACS_GEQUAL: cchar_t;
begin
  WACS_GEQUAL:= _nc_wacs[chtype('z')];
end;

function WACS_PI: cchar_t;
begin
  WACS_PI:= _nc_wacs[chtype('{')];
end;

function WACS_NEQUAL: cchar_t;
begin
  WACS_NEQUAL:= _nc_wacs[chtype('|')];
end;

function WACS_STERLING: cchar_t;
begin
  WACS_STERLING:= _nc_wacs[chtype('}')];
end;




function add_wch(wch: pcchar_t): longint;
begin
   add_wch:=wadd_wch(stdscr,wch);
end;

function add_wchnstr(wchstr: pcchar_t; n: longint): longint;
begin
   add_wchnstr:=wadd_wchnstr(stdscr,wchstr,n);
end;

function add_wchstr(wchstr: pcchar_t): longint;
begin
   add_wchstr:=wadd_wchstr(stdscr,wchstr);
end;

function addnwstr(wchstr: pwchar_t; n : longint) : longint;
begin
   addnwstr:=waddnwstr(stdscr,wchstr,n);
end;

function addwstr(wchstr: pwchar_t): longint;
begin
   addwstr:=waddnwstr(stdscr,wchstr,-(1));
end;

function bkgrnd(wch: pcchar_t): longint;
begin
   bkgrnd:=wbkgrnd(stdscr,wch);
end;

procedure bkgrndset(wch: pcchar_t);
begin
   wbkgrndset(stdscr,wch);
end;

function border_set(ls,rs,ts,bs,tl,tr,bl,br: pcchar_t) : longint;
begin
   border_set:=wborder_set(stdscr,ls,rs,ts,bs,tl,tr,bl,br);
end;

function box_set(win: PWINDOW; verch,horch: pcchar_t) : longint;
begin
   //box_set:=wborder_set(win,verch,verch,horch,horch,0,0,0,0);
   box_set:=wborder_set(win,verch,verch,horch,horch,nil,nil,nil,nil);
end;

function echo_wchar(wch: Pcchar_t): longint;
begin
   echo_wchar:=wecho_wchar(stdscr,wch);
end;

function get_wch(wch: Plongint): longint;
begin
   get_wch:=wget_wch(stdscr,wch);
end;

function get_wstr(wstr: PLongint): longint;
begin
   get_wstr:=wget_wstr(stdscr,wstr);
end;




function getbkgrnd(wch: pcchar_t): longint;
begin
   getbkgrnd := wgetbkgrnd(stdscr,wch);
end;

function getn_wstr(wstr: PLongint; n: longint): longint;
begin
   getn_wstr := wgetn_wstr(stdscr,wstr,n);
end;

function hline_set(wch: pcchar_t; n: longint): longint;
begin
   hline_set := whline_set(stdscr,wch,n);
end;

function in_wch(wch: pcchar_t) : longint;
begin
   in_wch := win_wch(stdscr,wch);
end;

function in_wchnstr(wchstr: pcchar_t; n: longint): longint;
begin
   in_wchnstr := win_wchnstr(stdscr,wchstr,n);
end;

function in_wchstr(wchstr: pcchar_t) : longint;
begin
   in_wchstr := win_wchstr(stdscr,wchstr);
end;

function innwstr(wstr: pwchar_t; n : longint) : longint;
begin
   innwstr:=winnwstr(stdscr,wstr,n);
end;

function ins_nwstr(wstr: pwchar_t; n: longint): longint;
begin
   ins_nwstr:=wins_nwstr(stdscr,wstr,n);
end;

function ins_wch(wch: pcchar_t): longint;
begin
   ins_wch:=wins_wch(stdscr,wch);
end;

function ins_wstr(wstr: pwchar_t): longint;
begin
   ins_wstr:=wins_wstr(stdscr,wstr);
end;

function inwstr(wstr: pwchar_t): longint;
begin
   inwstr:=winwstr(stdscr,wstr);
end;

function vline_set(wch: pcchar_t; n: longint): longint;
begin
   vline_set:=wvline_set(stdscr,wch,n);
end;

function wadd_wchstr(win: PWINDOW; wchstr: pcchar_t): longint;
begin
   wadd_wchstr:=wadd_wchnstr(win,wchstr,-(1));
end;

function waddwstr(win: PWINDOW; wstr: pwchar_t): longint;
begin
   waddwstr:=waddnwstr(win,wstr,-(1));
end;

function wget_wstr(win: PWINDOW; wstr: PLongint): longint;
begin
   wget_wstr:=wgetn_wstr(win,wstr,-(1));
end;

function win_wchstr(win: PWINDOW; wchstr: pcchar_t): longint;
begin
   win_wchstr:=win_wchnstr(win,wchstr,-(1));
end;

function wins_wstr(win: PWINDOW; wstr: pwchar_t) : longint;
begin
   wins_wstr:=wins_nwstr(win,wstr,-(1));
end;

function mvadd_wch(y,x: Smallint;  wch: pcchar_t): longint;
begin
  if move(y,x) = ERR then
    mvadd_wch := ERR
  else
    mvadd_wch := wadd_wch(stdscr,wch);
end;

function mvadd_wchnstr(y,x: Smallint; wchstr: pcchar_t; n: longint): longint;
begin
  if move(y,x) = ERR then
    mvadd_wchnstr := ERR
  else
    mvadd_wchnstr := wadd_wchnstr(stdscr,wchstr,n);
end;

function mvadd_wchstr(y,x: Smallint; wchstr: pcchar_t): longint;
begin
  if move(y,x) = ERR then
    mvadd_wchstr := ERR
  else
    mvadd_wchstr := wadd_wchnstr(stdscr,wchstr,-(1));
end;

function mvaddnwstr(y,x: Smallint; wstr: pwchar_t; n: longint): longint;
begin
  if move(y,x) = ERR then
    mvaddnwstr := ERR
  else
    mvaddnwstr := waddnwstr(stdscr,wstr,n);
end;

function mvaddwstr(y,x: Smallint; wstr: pwchar_t) : longint;
begin
  if move(y,x) = ERR then
    mvaddwstr := ERR
  else
    mvaddwstr := waddnwstr(stdscr,wstr,-(1));
end;

function mvget_wch(y,x: Smallint; wch: Plongint) : longint;
begin
  if move(y,x) = ERR then
    mvget_wch := ERR
  else
    mvget_wch := wget_wch(stdscr,wch);
end;


function mvget_wstr(y,x: Smallint; wstr: Plongint): longint;
begin
  if move(y,x) = ERR then
    mvget_wstr := ERR
  else
    mvget_wstr := wgetn_wstr(stdscr,wstr,-(1));
end;

function mvgetn_wstr(y,x: Smallint; wstr: Plongint; n: longint): longint;
begin
  if move(y,x) = ERR then
    mvgetn_wstr := ERR
  else
    mvgetn_wstr := wgetn_wstr(stdscr,wstr,n);
end;


function mvhline_set(y,x: Smallint; wch: pcchar_t; n: longint): longint;
begin
  if move(y,x) = ERR then
    mvhline_set := ERR
  else
    mvhline_set := whline_set(stdscr,wch,n);
end;

function mvin_wch(y,x: Smallint; wch: pcchar_t) : longint;
begin
  if move(y,x) = ERR then
    mvin_wch := ERR
  else
    mvin_wch := win_wch(stdscr,wch);
end;

function mvin_wchnstr(y,x: Smallint; wchstr: pcchar_t; n: longint): longint;
begin
  if move(y,x) = ERR then
    mvin_wchnstr := ERR
  else
    mvin_wchnstr := win_wchnstr(stdscr,wchstr,n);
end;

function mvin_wchstr(y,x: Smallint; wchstr: pcchar_t): longint;
begin
  if move(y,x) = ERR then
    mvin_wchstr := ERR
  else
    mvin_wchstr := win_wchnstr(stdscr,wchstr,-(1));
end;

function mvinnwstr(y,x: Smallint; wstr: pwchar_t; n : longint): longint;
begin
  if move(y,x) = ERR then
    mvinnwstr := ERR
  else
    mvinnwstr := winnwstr(stdscr,wstr,n);
end;

function mvins_nwstr(y,x: Smallint; wstr: pwchar_t; n : longint): longint;
begin
  if move(y,x) = ERR then
    mvins_nwstr := ERR
  else
    mvins_nwstr := wins_nwstr(stdscr,wstr,n);
end;

function mvins_wch(y,x: Smallint; wch: pcchar_t): longint;
begin
  if move(y,x) = ERR then
    mvins_wch := ERR
  else
    mvins_wch := wins_wch(stdscr,wch);
end;

function mvins_wstr(y,x: Smallint; wstr: pwchar_t): longint;
begin
  if move(y,x) = ERR then
    mvins_wstr := ERR
  else
    mvins_wstr := wins_nwstr(stdscr,wstr,-(1));
end;

function mvinwstr(y,x: Smallint; wstr: pwchar_t): longint;
begin
  if move(y,x) = ERR then
    mvinwstr := ERR
  else
    mvinwstr := winwstr(stdscr,wstr);
end;

function mvvline_set(y,x: Smallint; wch: pcchar_t; n : longint) : longint;
begin
  if move(y,x) = ERR then
    mvvline_set := ERR
  else
    mvvline_set := wvline_set(stdscr,wch,n);
end;

function mvwadd_wch(win: PWINDOW; y,x: Smallint; wch: pcchar_t): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwadd_wch := ERR
  else
    mvwadd_wch := wadd_wch(win,wch);
end;

function mvwadd_wchnstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t; n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwadd_wchnstr := ERR
  else
    mvwadd_wchnstr := wadd_wchnstr(win,wchstr,n);
end;

function mvwadd_wchstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwadd_wchstr := ERR
  else
    mvwadd_wchstr := wadd_wchnstr(win,wchstr,-(1));
end;

function mvwaddnwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t; n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwaddnwstr := ERR
  else
    mvwaddnwstr := waddnwstr(win,wstr,n);
end;

function mvwaddwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwaddwstr := ERR
  else
    mvwaddwstr := waddnwstr(win,wstr,-(1));
end;

function mvwget_wch(win: PWINDOW; y,x: Smallint; wch: Plongint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwget_wch := ERR
  else
    mvwget_wch := wget_wch(win,wch);
end;

function mvwget_wstr(win: PWINDOW; y,x: Smallint; wstr: Plongint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwget_wstr := ERR
  else
    mvwget_wstr := wgetn_wstr(win,wstr,-(1));
end;

function mvwgetn_wstr(win: PWINDOW; y,x: Smallint; wstr: Plongint; n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwgetn_wstr := ERR
  else
    mvwgetn_wstr := wgetn_wstr(win,wstr,n);
end;

function mvwhline_set(win: PWINDOW; y,x: Smallint; wch: pcchar_t; n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwhline_set := ERR
  else
    mvwhline_set := whline_set(win,wch,n);
end;

function mvwin_wch(win: PWINDOW; y,x: Smallint; wch: pcchar_t): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwin_wch := ERR
  else
    mvwin_wch := win_wch(win,wch);
end;

function mvwin_wchnstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t; n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwin_wchnstr := ERR
  else
    mvwin_wchnstr := win_wchnstr(win,wchstr,n);
end;

function mvwin_wchstr(win: PWINDOW; y,x: Smallint; wchstr: pcchar_t): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwin_wchstr := ERR
  else
    mvwin_wchstr := win_wchstr(win,wchstr);
end;

function mvwinnwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t; n : longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinnwstr := ERR
  else
    mvwinnwstr := winnwstr(win,wstr,n);
end;

function mvwins_nwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t; n: longint): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwins_nwstr := ERR
  else
    mvwins_nwstr := wins_nwstr(win,wstr,n);
end;

function mvwins_wch(win: PWINDOW; y,x: Smallint; wch: pcchar_t): longint;
begin
  if wmove(win,y,x) = ERR then
    mvwins_wch := ERR
  else
    mvwins_wch := wins_wch(win,wch);
end;

function mvwins_wstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t) : longint;
begin
  if wmove(win,y,x) = ERR then
    mvwins_wstr := ERR
  else
    mvwins_wstr := wins_wstr(win,wstr);
end;

function mvwinwstr(win: PWINDOW; y,x: Smallint; wstr: pwchar_t) : longint;
begin
  if wmove(win,y,x) = ERR then
    mvwinwstr := ERR
  else
    mvwinwstr := winwstr(stdscr,wstr);
end;

function mvwvline_set(win: PWINDOW; y,x: Smallint; wch: pcchar_t; n: longint) : longint;
begin
  if wmove(win,y,x) = ERR then
    mvwvline_set := ERR
  else
    mvwvline_set := wvline_set(win,wch,n);
end;


function wmove(win: PWINDOW; y,x: Smallint): Longint;
begin
  //if (win!=nil)AND(x>=0)AND(x<=win^._maxx)AND(y>=0)AND(y<=win^._maxy) then
  if (x>=0) and ( y>=0)AND(x<=win^._maxx)AND(y<=win^._maxy) then
  begin
    win^._curx := x;
    win^._cury := y;
    win^._flags := win^._flags AND not _WRAPPED;
    win^._flags := win^._flags OR _HASMOVED;
    wmove := OK
  end
  else
    wmove := ERR
end;


(* macros to extract single event-bits from masks *)

function BUTTON_RELEASE(e,x: longint): longint;
{ #define BUTTON_RELEASE(e, x)    ((e) & (001 << (6 * ((x) - 1)))) }
begin
   BUTTON_RELEASE:=e AND (001 shl (6*(x-1)));
end;

function BUTTON_PRESS(e,x: longint): longint;
{ #define BUTTON_PRESS(e, x)    ((e) & (002 << (6 * ((x) - 1)))) }
begin
   BUTTON_PRESS:=e AND (002 shl (6*(x-1)));
end;

function BUTTON_CLICK(e,x: longint): longint;
//#define BUTTON_CLICK(e, x)    ((e) & (004 << (6 * ((x) - 1))))
begin
   BUTTON_CLICK:=e AND (004 shl (6*(x-1)));
end;

function BUTTON_DOUBLE_CLICK(e,x: longint): longint;
//#define BUTTON_DOUBLE_CLICK(e, x) ((e) & (010 << (6 * ((x) - 1))))
begin
   BUTTON_DOUBLE_CLICK:=e AND (8 shl (6*(x-1)));
end;

function BUTTON_TRIPLE_CLICK(e,x: longint): longint;
//#define BUTTON_TRIPLE_CLICK(e, x) ((e) & (020 << (6 * ((x) - 1))))
begin
   BUTTON_TRIPLE_CLICK:=e AND (16 shl (6*(x-1)));
end;

function BUTTON_RESERVED_EVENT(e,x: longint): longint;
//#define BUTTON_RESERVED_EVENT(e, x) ((e) & (040 << (6 * ((x) - 1))))
begin
   BUTTON_RESERVED_EVENT:=e AND (32 shl (6*(x-1)));
end;


function mouse_trafo(pY,pX: PLongint; to_screen: Bool): Bool;
begin
   mouse_trafo:=wmouse_trafo(stdscr,pY,pX,to_screen);
end;


end.
