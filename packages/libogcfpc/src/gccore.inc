<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$include ogc/libversion.inc}
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
{$include ogc/disc_io.inc}

{$include debug.inc}
{$include ogcsys.inc}


{$ifdef HW_RVL}
  {$include ogc/machine/asm.inc}
  {$include ogc/machine/processor.inc}
  {$include ogc/machine/spinlock.inc}
{$endif HW_RVL}


{$include ogc/lwp_queue.inc}
{$include ogc/dsp.inc}
{$include ogc/aram.inc}
{$include ogc/arqueue.inc}
{$include ogc/arqmgr.inc}
{$include ogc/audio.inc}
{$include ogc/cache.inc}
{$include ogc/card.inc}
{$include ogc/cast.inc}   // TODO: inlined ASM to convert...
{$include ogc/color.inc}
{$include ogc/context.inc}
{$include ogc/gx_struct.inc}

{$include ogc/lwp.inc}
 {$include ogc/lwp_config.inc}
 {$include ogc/lwp_heap.inc}

 {$include ogc/lwp_tqdata.inc}
 {$include ogc/lwp_objmgr.inc}
 {$include ogc/lwp_priority.inc}
 {$include ogc/lwp_watchdog.inc}
 {$include ogc/lwp_threads.inc}  // tradurre lwp_threads.inl


 {$include ogc/lwp_messages.inc}
 {$include ogc/lwp_mutex.inc}
 {$include ogc/lwp_sema.inc}
 {$include ogc/lwp_stack.inc}
 {$include ogc/lwp_states.inc}
 {$include ogc/lwp_threadq.inc}
 {$include ogc/lwp_wkspace.inc}


{$include ogc/gu.inc}
{$include ogc/gx.inc}
{$include ogc/consol.inc}
{$include ogc/dvd.inc}
{$include ogc/exi.inc}
{$include ogc/si.inc}
{$include ogc/irq.inc}
{$include ogc/mutex.inc}
{$include ogc/message.inc}
{$include ogc/semaphore.inc}
{$include ogc/pad.inc}
{$include ogc/tpl.inc}
{$include ogc/system.inc}
{$include ogc/video.inc}
{$include ogc/usbgecko.inc}
{$include ogc/video_types.inc}
{$include ogc/texconv.inc}

{$include sdcard/card_buf.inc}
{$include sdcard/card_cmn.inc}
{$include sdcard/card_io.inc}
{$include sdcard/wiisd_io.inc}


{$ifdef HW_RVL}
  {$include ogc/ipc.inc}
  {$include ogc/es.inc}
  {$include ogc/stm.inc}
  {$include ogc/ios.inc}
  {$include ogc/usb.inc}
  {$include ogc/isfs.inc}
  {$include ogc/conf.inc}

  {$include ogc/usbstorage.inc}
  {$include ogc/wiilaunch.inc}

  {$include bte/bd_addr.inc}
  {$include bte/bte.inc}

  {$include wiiuse/wiiuse.inc}
  {$include wiiuse/wpad.inc}
  
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  {$include wiikeyboard/wsksymdef.inc}
=======
  { $include wiikeyboard/wsksymdef.inc}
>>>>>>> graemeg/cpstrnew
=======
  { $include wiikeyboard/wsksymdef.inc}
>>>>>>> graemeg/cpstrnew
=======
  { $include wiikeyboard/wsksymdef.inc}
>>>>>>> graemeg/cpstrnew
=======
  { $include wiikeyboard/wsksymdef.inc}
>>>>>>> origin/cpstrnew
  {$include wiikeyboard/keyboard.inc}
  {$include wiikeyboard/usbkeyboard.inc}

{$endif HW_RVL}

