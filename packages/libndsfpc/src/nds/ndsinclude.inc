
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$include libversion.inc} 
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
>>>>>>> origin/fixes_2.4
{ $include helper.inc}
{$include ndstypes.inc}
{$include bios.inc}
{$include card.inc}
{$include debug.inc}
{$include disc_io.inc}
=======
=======
>>>>>>> origin/fixes_2_2
{$include helper.inc}
{$include jtypes.inc}
{$include bios.inc}
{$include card.inc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$include dma.inc}
{$include interrupts.inc}
{$include ipc.inc}
{$include memory.inc}
<<<<<<< HEAD
<<<<<<< HEAD
{$include touch.inc}
{$include fifocommon.inc}
{$include system.inc}
{$include timers.inc}
{$include fifomessages.inc} 
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$include input.inc} 
=======
{$include input.inc} // testing...
>>>>>>> graemeg/cpstrnew
=======
{$include input.inc} // testing...
>>>>>>> graemeg/cpstrnew
=======
{$include input.inc} // testing...
>>>>>>> graemeg/cpstrnew
=======
{$include input.inc} // testing...
>>>>>>> origin/cpstrnew
 
{$ifdef ARM9}
  {$include arm9/dynamicArray.inc}
  {$include arm9/linkedlist.inc}
  {$include arm9/sassert.inc}
  {$include arm9/background.inc}
  {$include arm9/cache.inc}
  {$include arm9/console.inc}
  {$include arm9/keyboard.inc}
=======
=======
>>>>>>> origin/fixes_2_2
{$include system.inc}
{$include timers.inc}
{$include reload.inc}

{$ifdef ARM9}
  {$include arm9/background.inc}
  {$include arm9/boxtest.inc}
  {$include arm9/cache.inc}
  {$include arm9/console.inc}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  {$include arm9/exceptions.inc}
  {$include arm9/image.inc}
  {$include arm9/input.inc}
  {$include arm9/math.inc}
<<<<<<< HEAD
<<<<<<< HEAD
  {$include arm9/pcx.inc}
  {$include arm9/rumble.inc}
  {$include arm9/sound.inc}
  {$include arm9/trig_lut.inc}
  {$include arm9/video.inc}
  {$include arm9/videoGL.inc}
  {$include arm9/boxtest.inc}
  {$include arm9/postest.inc}
  {$include arm9/sprite.inc}
  {$include arm9/decompress.inc}
  {$include arm9/ndsmotion.inc}
  {$include arm9/guitarGrip.inc}
  {$include arm9/paddle.inc}
  {$include arm9/piano.inc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$include arm9/dldi.inc}
  {$include arm9/window.inc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
=======
>>>>>>> origin/fixes_2_2
  {$include arm9/ndsmotion.inc}
  {$include arm9/pcx.inc}
  {$include arm9/rumble.inc}
  {$include arm9/sound.inc}
  {$include arm9/sprite.inc}
  {$include arm9/trig_lut.inc}
  {$include arm9/video.inc}
  {$include arm9/videoGL.inc}
  {$include arm9/postest.inc}
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
  {$include arm7/i2c.inc}
  {$include arm7/serial.inc}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$include arm7/sdmmc.inc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
=======
  {$include arm7/serial.inc}
>>>>>>> graemeg/fixes_2_2
=======
  {$include arm7/serial.inc}
>>>>>>> origin/fixes_2_2
  {$include arm7/touch.inc}
{$endif ARM7}
