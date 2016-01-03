program main9;

{$L data/module.bin.o}

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

{$define HW_MIXER}   // It works fine on hardware, but on no$gba sound is distorted 
{ $define SW_MIXER}    // It works fine both on hardware and no$gba
{ $define NO_MIXER}   // No mixer, no sound :)

uses
  ctypes, nds9, mikmod9;
{ $include nds.inc}

{ $include mikmod.inc}

var
  module_bin_end: array [0..0] of cuint8; cvar; external;
  module_bin: array [0..0] of cuint8; cvar; external;
  module_bin_size: cuint32; cvar; external;

// called by the drivers in mikmod library
procedure MikMod9_SendCommand(command: cuint); cdecl; export;
begin
	while (REG_IPC_FIFO_CR^ and IPC_FIFO_SEND_FULL) <> 0 do;
	REG_IPC_FIFO_TX^ := command;
end;

procedure TimerInterrupt();
begin
	// player tick
  MikMod_Update();
	// the bpm can change in the middle of the song
	TIMER0_DATA^ := TIMER_FREQ_256((md_bpm * 50) div 125);
end;

var
  song: PModule;

begin
	REG_IPC_FIFO_CR^ := IPC_FIFO_ENABLE or IPC_FIFO_SEND_CLEAR;

	consoleDemoInit();
	irqInit();
	
	irqEnable(IRQ_VBLANK);

{$ifdef HW_MIXER}
	MikMod_RegisterDriver(@drv_nds_hw);
{$endif HW_MIXER}

{$ifdef SW_MIXER}
	MikMod_RegisterDriver(@drv_nds_sw);
{$endif HW_MIXER}

{$ifdef NO_MIXER}
	MikMod_RegisterDriver(@drv_nos);
{$endif NO_MIXER}

  // if we don't know what kind of module we're going to load we can register
  // all loaders, but that will result in a larger binary
  //MikMod_RegisterAllLoaders();
  MikMod_RegisterLoader(@load_it);
  
  printf('Initializing library' + #10);
  if (MikMod_Init('') <> 0) then
  begin
    printf('Could not initialize sound, reason: ' + #10 + '%s'  + #10, MikMod_strerror(MikMod_errno));
    exit;
  end;
  
  printf(#10 + 'Loading module' + #10);
  // Player_LoadMemory() loads a module directly from memory
  // it could be possible to use Player_Load() to load from FAT,
  // but I've never tried this
  song := Player_LoadMemory(@module_bin, module_bin_size, 64, 0);
  if assigned(song) then
  begin
    printf('Title:    %s' + #10, song^.songname);
    printf('Channels: %u' + #10, song^.numchn);
    printf('bpm: %u' + #10, md_bpm);
    
    printf(#10 + 'Starting module'  + #10);
    
    Player_Start(song);
    
    irqSet(IRQ_TIMER0, @TimerInterrupt);
    // call update with correct timing
    TIMER0_CR^ := TIMER0_CR^ and not TIMER_ENABLE;
    TIMER0_DATA^ := TIMER_FREQ_256(md_bpm * 50 div 125);
    TIMER0_CR^ := TIMER_ENABLE or TIMER_DIV_256 or TIMER_IRQ_REQ;
    irqEnable(IRQ_TIMER0);
    
    // save cursor position
    printf(#27 + '[s');
    
    while (Player_Active() <> 0) do
    begin
      // when using the software driver we could call update
      // here instead
      //MikMod_Update();
      
      // I need to fix this part, because I get crap values :P
      //printf('Time: %i : %02i : %02i' + #27 + '[u', (song^.sngtime div 60000),(song^.sngtime div 1000) mod 60,(song^.sngtime div 10) mod 100);
      //printf(#10 + 'Time: %u' + #27 + '[u', [(song^.sngtime div 1000) mod 60]);
      
      swiWaitForVBlank();
    end;
  
    printf(#10 + 'Stopping module' + #10);
    Player_Stop();
    Player_Free(song);
  end else 
    printf('Could not load module, reason: ' + #10 + '%s' + #10, MikMod_strerror(MikMod_errno));
  
  printf(#10 + 'Exit library' + #10);
  MikMod_Exit();
end.
