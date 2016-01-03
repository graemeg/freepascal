program main7;

{$apptype arm7}
{$define ARM7}

{$mode objfpc}

uses
  ctypes, nds7, dswifi7;
  
{ $include dswifi7.inc}


procedure startSound(sampleRate: cint; const data: pointer; bytes: cuint32; channel, vol, pan, format: cuint8);
var
  snd_format: integer;
begin
  if format = 1 then 
    snd_format := SOUND_8BIT 
  else 
    snd_format := SOUND_16BIT;
	SCHANNEL_TIMER(channel)^  := SOUND_FREQ(sampleRate);
	SCHANNEL_SOURCE(channel)^ := cuint32(data^);
	SCHANNEL_LENGTH(channel)^ := bytes shr 2;
	SCHANNEL_CR(channel)^     := SCHANNEL_ENABLE or SOUND_ONE_SHOT or SOUND_VOL(vol) or SOUND_PAN(pan) or (snd_format);
end;

function getFreeSoundChannel(): csint;
var	
  i: integer;
begin
	for i := 0 to 15 do
		if ((SCHANNEL_CR(i)^ and SCHANNEL_ENABLE)) = 0  then 
      result := i;
	result := -1;
end;

var
  vcount: integer;
  first, tempPos: touchPosition;
	lastbut: integer = -1;

procedure VcountHandler();
var
  but: integer;
  x, y, xpx, ypx, z1, z2: cuint16;
begin
  but := REG_KEYXY^;
  if (( (but xor lastbut) and (1 shl 6))) = 0 then
  begin 
    tempPos := touchReadXY();
    x := tempPos.x;
    y := tempPos.y;
    xpx := tempPos.px;
    ypx := tempPos.py;
    z1 := tempPos.z1;
    z2 := tempPos.z2;
  end else 
  begin
    lastbut := but;
    but := but or (1 shl 6);
  end;

  if ( vcount = 80 ) then
  begin
    first := tempPos;
  end else 
  begin
		if (abs(xpx - first.px) > 10) or (abs(ypx - first.py) > 10) or  ((but and (1 shl 6)) <> 0) then 
    begin
      but := but or (1 shl 6);
      lastbut := but;
    end else 
    begin
      IPC.mailBusy := 1;
      IPC.touchX := x;
      IPC.touchY := y;
      IPC.touchXpx := xpx;
      IPC.touchYpx := ypx;
      IPC.touchZ1 := z1;
      IPC.touchZ2 := z2;
      IPC.mailBusy := 0;
    end;
  end;
  IPC.buttons		:= but;
  vcount := vcount xor (80 xor 130);
  SetYtrigger(vcount);
end;

procedure VblankHandler();
var
	i: integer;
	snd: PTransferSound;
	chan: csint;
begin
	//sound code  :)
	snd := IPC.soundData;
	IPC.soundData := nil;

	if (snd <> nil) then
	begin
		for i := 0 to snd^.count - 1 do
		begin
			chan := getFreeSoundChannel();
			if (chan >= 0) then
			begin
				startSound(snd^.data[i].rate, snd^.data[i].data, snd^.data[i].len, chan, snd^.data[i].vol, snd^.data[i].pan, snd^.data[i].format);
			end;
		end;
	end;
	Wifi_Update();
end;

procedure arm7_synctoarm9();
begin
  // send fifo message
  REG_IPC_FIFO_TX^ := $87654321;
end;

procedure arm7_fifo();
var
  msg: cuint32;
begin
  // check incoming fifo messages
  msg := REG_IPC_FIFO_RX^;
  if (msg = $87654321) then 
    Wifi_Sync();
end;

var
  fifo_temp: cuint32;    

begin
	REG_IPC_FIFO_CR^ := IPC_FIFO_ENABLE or IPC_FIFO_SEND_CLEAR;

	// Reset the clock if needed
	rtcReset();

	//enable sound
	powerON(POWER_SOUND);
	SOUND_CR^ := SOUND_ENABLE or SOUND_VOL($7F);
	IPC.soundData := nil;
	IPC.mailBusy := 0;

	irqInit();
	irqSet(IRQ_VBLANK, @VblankHandler);
	SetYtrigger(80);
	vcount := 80;

	irqSet(IRQ_VCOUNT, @VcountHandler);
	irqEnable(IRQ_VBLANK or IRQ_VCOUNT);
	
  irqSet(IRQ_WIFI, @Wifi_Interrupt);
  irqEnable(IRQ_WIFI);

	// trade some mail, to get a pointer from arm9
  while	fifo_temp <> $12345678 do
  begin
    while (REG_IPC_FIFO_CR^ and IPC_FIFO_RECV_EMPTY) <> 0 do
      swiWaitForVBlank();
    fifo_temp := REG_IPC_FIFO_RX^;
  end;
  
  while (REG_IPC_FIFO_CR^ and IPC_FIFO_RECV_EMPTY) <> 0 do 
    swiWaitForVBlank();

	fifo_temp := REG_IPC_FIFO_RX^;
	
	Wifi_Init(fifo_temp);

	irqSet(IRQ_FIFO_NOT_EMPTY, @arm7_fifo);
	irqEnable(IRQ_FIFO_NOT_EMPTY);
	REG_IPC_FIFO_CR^ := IPC_FIFO_ENABLE or IPC_FIFO_RECV_IRQ;

	Wifi_SetSyncHandler(@arm7_synctoarm9);


	// Keep the ARM7 idle
	while true do
    swiWaitForVBlank();
end.


