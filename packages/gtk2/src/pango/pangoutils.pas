{ Pango
   PangoUtils - Utilities for internal functions and modules

   Copyright (C) 2000 Red Hat Software

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public
   License along with this library; if not, write to the
   Free Software Foundation, Inc., 59 Temple Place - Suite 330,
   Boston, MA 02111-1307, USA.
  }
unit pangoutils;

{$IFDEF FPC}
  {$MODE objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

uses
  glib2, pango;

{ Pointers to basic pascal types, inserted by h2pas conversion program.}
Type
  PLongint  = ^Longint;
  PSmallInt = ^SmallInt;
  PByte     = ^Byte;
  PWord     = ^Word;
  PDWord    = ^DWord;
  PDouble   = ^Double;
{$IFDEF KYLIX}
  PFile     = ^file;
{$ENDIF}

{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}

function pango_split_file_list(str:Pchar):PPchar; cdecl; external pangolib;
function pango_trim_string(str:Pchar):Pchar; cdecl; external pangolib;
function pango_read_line(stream:PFILE; str:PGString):gint; cdecl; external pangolib;
function pango_skip_space(pos:PPchar):gboolean; cdecl; external pangolib;
function pango_scan_word(pos:PPchar; OutStr:PGString):gboolean; cdecl; external pangolib;
function pango_scan_string(pos:PPchar; OutStr:PGString):gboolean; cdecl; external pangolib;
function pango_scan_int(pos:PPchar; OutInt:Plongint):gboolean; cdecl; external pangolib;

{$ifdef PANGO_ENABLE_BACKEND}
function pango_config_key_get(key:Pchar):Pchar; cdecl; external pangolib;
procedure pango_lookup_aliases(fontname:Pchar; families:PPPchar; n_families:Plongint); cdecl; external pangolib;
{$endif}
{ PANGO_ENABLE_BACKEND  }

{ Functions for parsing textual representations
   of PangoFontDescription fields. They return TRUE if the input string
   contains a valid value, which then has been assigned to the corresponding
   field in the PangoFontDescription. If the warn parameter is TRUE,
   a warning is printed (with g_warning) if the string does not
   contain a valid value.
  }
function pango_parse_style(str:Pchar; style:PPangoStyle; warn:gboolean):gboolean; cdecl; external pangolib;
function pango_parse_variant(str:Pchar; variant:PPangoVariant; warn:gboolean):gboolean; cdecl; external pangolib;
function pango_parse_weight(str:Pchar; weight:PPangoWeight; warn:gboolean):gboolean; cdecl; external pangolib;
function pango_parse_stretch(str:Pchar; stretch:PPangoStretch; warn:gboolean):gboolean; cdecl; external pangolib;
{$ifdef PANGO_ENABLE_BACKEND}

{ On Unix, return the name of the "pango" subdirectory of SYSCONFDIR
   (which is set at compile time). On Win32, return the Pango
   installation directory (which is set at installation time, and
   stored in the registry). The returned string should not be
   g_free'd.
  }
function pango_get_sysconf_subdirectory:Pchar; cdecl; external pangolib;

{ Ditto for LIBDIR/pango. On Win32, use the same Pango
   installation directory. This returned string should not be
   g_free'd either.
  }
function pango_get_lib_subdirectory:Pchar; cdecl; external pangolib;
{$endif}
{ PANGO_ENABLE_BACKEND  }

{ A couple of routines from fribidi that we either wrap or
   provide ourselves.
  }

function pango_log2vis_get_embedding_levels(str:Pgunichar; len:longint; pbase_dir:PPangoDirection; embedding_level_list:Pguint8):gboolean; cdecl; external pangolib;
function pango_get_mirror_char(ch:gunichar; mirrored_ch:Pgunichar):gboolean; cdecl; external pangolib;
function pango_language_get_sample_string(language:PPangoLanguage):Pchar; cdecl; external pangolib;

implementation

end.
