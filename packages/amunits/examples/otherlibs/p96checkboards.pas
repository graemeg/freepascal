Program P96CheckBoards;

{***********************************************************************
* This is example shows how to use p96GetRTGDataTagList and p96GetBoardDataTagList
*
* tabt (Sat Sep 12 23:06:28 1998)
***********************************************************************}

{
    Translated to fpc pascal.
    15 Mars 2001.

    Updated for fpc 1.0.7
    08 Jan 2003.

    nils.sjoholm@mailbox.swipnet.se
}


uses exec, amigados, graphics, picasso96api,utility,amigalib;

var
   NumBoards : Longint;
   i, clock  : Longint;
   tmp       : Longint;
   RGBFormats,
   MemorySize,
   FreeMemory,
   LargestFreeMemory,
   MemoryClock,
   MoniSwitch  : Longint;
   BoardName   : Pchar;
   boardtmp    : array[0..200] of char;

FUNCTION GetMonitorValue(value : longint): STRING;
BEGIN
    IF value = 0 THEN GetMonitorValue := 'not set'
    ELSE GetMonitorValue := 'set';
END;

begin
   BoardName := @boardtmp;

   tmp := p96GetRTGDataTags([P96RD_NumberOfBoards, @NumBoards, TAG_END]);

   writeln('Looking through all boards installed for Picasso96');

   for i := 0 to NumBoards-1 do begin
       p96GetBoardDataTags(i,[P96BD_BoardName, @BoardName,
                              P96BD_RGBFormats, @RGBFormats,
                              P96BD_TotalMemory, @MemorySize,
                              P96BD_FreeMemory, @FreeMemory,
                              P96BD_LargestFreeMemory, @LargestFreeMemory,
                              P96BD_MemoryClock, @MemoryClock,
                              P96BD_MonitorSwitch, @MoniSwitch,
                              TAG_END]);

      writeln('--------------------------------------------------');
      printf('Board %ld:      %s'#10,[ i, BoardName]);
      printf('Total size of memory:     %8ld'#10,[ MemorySize]);
      printf('Size of free memory:      %8ld'#10,[ FreeMemory]);
      printf('Largest free chunk:       %8ld'#10,[ LargestFreeMemory]);
      printf('Monitor switch:   %s'#10,[ GetMonitorValue(MoniSwitch)]);
      writeln('This board supports:');
      writeln(#9,'following rgb formats:');

      if (RGBFormats and RGBFF_NONE) <> 0 then writeln(#9,#9,'PLANAR');
      if (RGBFormats and RGBFF_CLUT) <> 0 then writeln(#9,#9,'CHUNKY');
      if (RGBFormats and RGBFF_R5G5B5) <> 0 then writeln(#9,#9,'tR5G5B5');
      if (RGBFormats and RGBFF_R5G5B5PC) <> 0 then writeln(#9,#9,'R5G5B5PC');
      if (RGBFormats and RGBFF_B5G5R5PC) <> 0 then writeln(#9,#9,'B5G5R5PC');
      if (RGBFormats and RGBFF_R5G6B5) <> 0 then writeln(#9,#9,'R5G6B5');
      if (RGBFormats and RGBFF_R5G6B5PC) <> 0 then writeln(#9,#9,'R5G6B5PC');
      if (RGBFormats and RGBFF_B5G6R5PC) <> 0 then writeln(#9,#9,'B5G6R5PC');
      if (RGBFormats and RGBFF_R8G8B8) <> 0 then writeln(#9,#9,'R8G8B8');
      if (RGBFormats and RGBFF_B8G8R8) <> 0 then writeln(#9,#9,'B8G8R8');
      if (RGBFormats and RGBFF_A8R8G8B8) <> 0 then writeln(#9,#9,'A8R8G8B8');
      if (RGBFormats and RGBFF_A8B8G8R8) <> 0 then writeln(#9,#9,'A8B8G8R8');
      if (RGBFormats and RGBFF_R8G8B8A8) <> 0 then writeln(#9,#9,'R8G8B8A8');
      if (RGBFormats and RGBFF_B8G8R8A8) <> 0 then writeln(#9,#9,'B8G8R8A8');
      if (RGBFormats and RGBFF_Y4U2V2) <> 0 then writeln(#9,#9,'Y4U2V2');
      if (RGBFormats and RGBFF_Y4U1V1) <> 0 then writeln(#9,#9,'Y4U1V1');

      clock := (MemoryClock+50000) div 100000;
      write(#9);
      printf('memory clock set to %ld.%1ld MHz,'#10,[(clock div 10),(clock mod 10)]);

   end;
end.
