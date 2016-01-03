{
 *****************************************************************************
 *                                                                           *
 *  See the file COPYING.modifiedLGPL, included in this distribution,        *
 *  for details about the copyright.                                         *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                     *
 *                                                                           *
 *****************************************************************************

  Author: Mattias Gaertner

}
unit gdk2x;

{$IFDEF FPC}
  {$mode objfpc}
{$ENDIF}
{$IFDEF VER140}
  {$DEFINE KYLIX}
{$ENDIF}

interface

uses
  Classes, SysUtils, Unix, BaseUnix, glib2, gdk2, XLib, X, XRender;

{$ifdef FREEBSD}
  {$linklib pthread}
{$endif}

{$IFNDEF KYLIX}
  {$PACKRECORDS C}
{$ELSE}
  {$ALIGN 4}
  {$WEAKPACKAGEUNIT}
  {$WARNINGS OFF}
{$ENDIF}

{$DEFINE read_forward_definitions}
type
{$I include/gdk2x11includes.inc}
{$UNDEF read_forward_definitions}

{$DEFINE read_interface_rest}
{$I include/gdk2x11includes.inc}
{$UNDEF read_interface_rest}

implementation

{*****************************************************************************
 * macro functions
 *
 *****************************************************************************}

// call implementation parts of header files
{$DEFINE read_implementation}
{$I include/gdk2x11includes.inc}
{$UNDEF read_implementation}

end.

