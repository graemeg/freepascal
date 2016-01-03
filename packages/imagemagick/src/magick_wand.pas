{
  Copyright 1999-2005 ImageMagick Studio LLC, a non-profit organization
  dedicated to making software imaging solutions freely available.
  
  You may not use this file except in compliance with the License.
  obtain a copy of the License at
  
    http://www.imagemagick.org/script/license.php
  
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

  ImageMagick MagickWand API.
<<<<<<< HEAD
<<<<<<< HEAD
}
{
  Based on ImageMagick 6.2

=======
  
>>>>>>> graemeg/fixes_2_2
=======
  
>>>>>>> origin/fixes_2_2
  Converted from c by: Felipe Monteiro de Carvalho Dez/2005

	Bug-fixed by Ángel Eduardo García Hernández
	Thanks to Marc Geldon and RuBBeR
}
{Version 0.4}
unit magick_wand;

{$IFDEF FPC}
  {$mode objfpc}
	{$PACKRECORDS C}
{$ENDIF}

<<<<<<< HEAD
<<<<<<< HEAD
{$z4}

interface

uses ImageMagick, ctypes;
=======
=======
>>>>>>> origin/fixes_2_2
{$MINENUMSIZE 1}

interface

uses ImageMagick;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{ Various types }
type
  MagickWand = record
<<<<<<< HEAD
<<<<<<< HEAD
    id: culong;
=======
    id: Cardinal;
>>>>>>> graemeg/fixes_2_2
=======
    id: Cardinal;
>>>>>>> origin/fixes_2_2
    name: array[1..MaxTextExtent] of Char;
    exception: ExceptionInfo;
    image_info: PImageInfo;
    quantize_info: PQuantizeInfo;
    images: PImage;
    active, pend, debug: MagickBooleanType;
<<<<<<< HEAD
<<<<<<< HEAD
    signature: culong;
=======
    signature: Cardinal;
>>>>>>> graemeg/fixes_2_2
=======
    signature: Cardinal;
>>>>>>> origin/fixes_2_2
  end;

  PMagickWand = ^MagickWand;

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  size_t = Integer;
  
  Psize_t = ^size_t;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$include pixel_wand.inc}
{$include drawing_wand.inc}
{$include magick_attribute.inc}
{$include magick_image.inc}
{$include pixel_iterator.inc}

function IsMagickWand(const wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;
function MagickClearException(wand: PMagickWand): MagickBooleanType; cdecl; external WandExport;

function CloneMagickWand(const wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function DestroyMagickWand(wand: PMagickWand): PMagickWand; cdecl; external WandExport;
function NewMagickWand: PMagickWand; cdecl; external WandExport;

procedure ClearMagickWand(wand: PMagickWand); cdecl; external WandExport;
procedure MagickWandGenesis; cdecl; external WandExport;
procedure MagickWandTerminus; cdecl; external WandExport;
function MagickRelinquishMemory(resource: Pointer): Pointer; cdecl; external WandExport;
procedure MagickResetIterator(wand: PMagickWand); cdecl; external WandExport;
procedure MagickSetFirstIterator(wand: PMagickWand); cdecl; external WandExport;
procedure MagickSetLastIterator(wand: PMagickWand); cdecl; external WandExport;

implementation

end.
