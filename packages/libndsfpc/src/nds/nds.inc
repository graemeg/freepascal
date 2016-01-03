(* 
<<<<<<< HEAD
<<<<<<< HEAD
=======
  $Id: nds.inc 25 2007-12-10 21:06:46Z p4p3r0 $
>>>>>>> graemeg/fixes_2_2
=======
  $Id: nds.inc 25 2007-12-10 21:06:46Z p4p3r0 $
>>>>>>> origin/fixes_2_2
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
  
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------
  Use this file as inclusion if you don't want to use the nds7/nds9 ppu library:
    
      program main;
      
      {$apptype arm9} // or arm7
      {$define arm9}  // or arm7
      {$mode objfpc}


      uses
        ctypes;

      {$include nds.inc}

      begin
        // do stuff
      end.
  ------------------------------------------------------------------------------

<<<<<<< HEAD
<<<<<<< HEAD
*)

=======
=======
>>>>>>> origin/fixes_2_2
  $Log$

*)
*
*
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$ifndef NDS_INC}
{$define NDS_INC}

{$J+}
{$INLINE ON}
{$MACRO ON}
{$PACKRECORDS C}

{$if not defined(ARM7) and not defined (ARM9)}
  {$error Either ARM7 or ARM9 must be defined}
{$endif}


{$define NDS_INCLUSION}
{$define NDS_INTERFACE}
{$define NDS_IMPLEMENTATION}

{$include helper.inc}

<<<<<<< HEAD
<<<<<<< HEAD
{$include jtypes.inc} // testing...
{$include ndstypes.inc} // testing...
{$include bios.inc}
{$include card.inc}
{$include debug.inc}
=======
{$include jtypes.inc}
{$include bios.inc}
{$include card.inc}
>>>>>>> graemeg/fixes_2_2
=======
{$include jtypes.inc}
{$include bios.inc}
{$include card.inc}
>>>>>>> origin/fixes_2_2
{$include dma.inc}
{$include interrupts.inc}
{$include ipc.inc}
{$include memory.inc}
{$include system.inc}
{$include timers.inc}
<<<<<<< HEAD
<<<<<<< HEAD
{$include fifocommon.inc}
{$include touch.inc}
{$include input.inc}

{$include dynamicArray.inc} // testing...
{$include linkedlist.inc} // testing...
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifdef ARM9}
  {$include arm9/background.inc}
  {$include arm9/boxtest.inc}
  {$include arm9/cache.inc}
  {$include arm9/console.inc}
<<<<<<< HEAD
<<<<<<< HEAD
  {$include arm9/decompress.inc}
  {$include arm9/exceptions.inc}
  {$include arm9/image.inc}
  {$include arm9/input.inc}
  {$include arm9/keyboard.inc}
  {$include arm9/math.inc}
  {$include arm9/pcx.inc}
  {$include arm9/rumble.inc}
  {$include arm9/sassert.inc}
  {$include arm9/sound.inc}
  {$include arm9/sprite.inc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$include arm9/window.inc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  {$include arm9/trig_lut.inc}
  {$include arm9/video.inc}
  {$include arm9/videoGL.inc}
=======
=======
>>>>>>> origin/fixes_2_2
  {$include arm9/exceptions.inc}
  {$include arm9/image.inc}
  {$include arm9/input.inc}
  {$include arm9/math.inc}
  {$include arm9/pcx.inc}
  { $include nds/arm9/postest.inc}  // da rimuovere
  {$include arm9/rumble.inc}
  {$include arm9/sound.inc}
  {$include arm9/trig_lut.inc}
  {$include arm9/video.inc}
  {$include arm9/videoGL.inc}
  {$include arm9/sprite.inc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif ARM9}

{$ifdef ARM7}
  {$include arm7/audio.inc}
  {$include arm7/clock.inc}
<<<<<<< HEAD
<<<<<<< HEAD
  {$include arm7/input.inc}  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$include arm7/serial.inc}
  {$include arm7/touch.inc}
{$endif ARM7}

{$ifdef ARM7}
  {$linklib libnds7.a}
{$endif ARM7}
{$ifdef ARM9}
  {$linklib libnds9.a}
{$endif ARM9}

{$linklib libc.a}
{$linklib libgcc.a}
{$linklib libsysbase.a}

{$endif NDS_INC}

