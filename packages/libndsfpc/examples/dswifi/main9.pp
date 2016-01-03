(*---------------------------------------------------------------------------------
	$Id: template.c,v 1.4 2005/09/17 23:15:13 wntrmute Exp $

	Basic Hello World

	$Log: template.c,v $
	Revision 1.4  2005/09/17 23:15:13  wntrmute
	corrected iprintAt in templates
	
	Revision 1.3  2005/09/05 00:32:20  wntrmute
	removed references to IPC struct
	replaced with API functions
	
	Revision 1.2  2005/08/31 01:24:21  wntrmute
	updated for new stdio support

	Revision 1.1  2005/08/03 06:29:56  wntrmute
	added templates


---------------------------------------------------------------------------------*)
program main9;

{$apptype arm9}
{$define ARM9}

{$mode objfpc}


uses
  ctypes, nds9, dswifi9;

{ $include dswifi9.inc}
{ $include socket.inc}
{ $include in.inc}
{ $include netdb.inc}

//---------------------------------------------------------------------------------
// Dswifi helper functions

// wifi timer function, to update internals of sgIP
procedure Timer_50ms();
begin
  Wifi_Timer(50);
end;

// notification function to send fifo message to arm7
procedure arm9_synctoarm7(); 
begin
// send fifo message
  REG_IPC_FIFO_TX^ := $87654321;
end;

// interrupt handler to receive fifo messages from arm7
procedure arm9_fifo();
var
  value: cuint32;
begin
 // check incoming fifo messages
  value := REG_IPC_FIFO_RX^;
  if value = $87654321 then
    Wifi_Sync();
end;

procedure vblank_irq_handler();
begin
  Wifi_Update();
end;

var
  Wifi_pass: cuint32;
  i: integer;
  myhost: PHostent;
  my_socket: cint;
  sain: sockaddr_in;
  recvd_len: cint;
  incoming_buffer: array [0..255] of char;
  
const 
  request_text = 
        'GET /dswifi/example1.php HTTP/1.1'#13#10 +
        'Host: www.akkit.org'#13#10 +
        'User-Agent: Nintendo DS'#13#10 +#13#10;
  
  
begin
	videoSetMode(0);	//not using the main screen
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE);	//sub bg 0 will be used to print text
	vramSetBankC(VRAM_C_SUB_BG);

	SUB_BG0_CR^ := BG_MAP_BASE(31);
	BG_PALETTE_SUB[255] := RGB15(31,31,31);	//by default font will be rendered with color 255

	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pcuint16(SCREEN_BASE_BLOCK_SUB(31)), pcuint16(CHAR_BASE_BLOCK_SUB(0)), 16);
	iprintf(#10+#10+#9+'Hello World!' + #10);

	// send fifo message to initialize the arm7 wifi
	REG_IPC_FIFO_CR^ := IPC_FIFO_ENABLE or IPC_FIFO_SEND_CLEAR; // enable & clear FIFO
	
	Wifi_pass := Wifi_Init(WIFIINIT_OPTION_USELED);
 	REG_IPC_FIFO_TX^ := $12345678;
 	REG_IPC_FIFO_TX^ := Wifi_pass;

	TIMER3_CR^ := 0; // disable timer3
	
	irqInit(); 
  irqSet(IRQ_VBLANK, @vblank_irq_handler);
	irqSet(IRQ_TIMER3, @Timer_50ms); // setup timer IRQ
	irqEnable(IRQ_VBLANK or IRQ_TIMER3);

 	irqSet(IRQ_FIFO_NOT_EMPTY, @arm9_fifo); // setup fifo IRQ
 	irqEnable(IRQ_FIFO_NOT_EMPTY);

 	REG_IPC_FIFO_CR^ := IPC_FIFO_ENABLE or IPC_FIFO_RECV_IRQ; // enable FIFO IRQ
 	
 	Wifi_SetSyncHandler(@arm9_synctoarm7); // tell wifi lib to use our handler to notify arm7

	// set timer3
	//TIMER3_DATA^ := -6553; // 6553.1 * 256 cycles = ~50ms;
	TIMER3_DATA^ := TIMER_FREQ_256(1000 div 50);
	TIMER0_CR^ := TIMER_ENABLE or TIMER_DIV_256 or TIMER_IRQ_REQ; // enable, irq, 1/256 clock
	
	while (Wifi_CheckInit() = 0) do // wait for arm7 to be initted successfully
	begin
		// wait for vblank
		swiWaitForVBlank();
	end;
	// wifi init complete - wifi lib can now be used!
	
	iprintf('Connecting via WFC data' + #10);
  
  // simple WFC connect:
	Wifi_AutoConnect(); // request connect
	
	while true do
	begin
		i := Wifi_AssocStatus(); // check status
		if (i = cint(ASSOCSTATUS_ASSOCIATED)) then
		begin
			iprintf('Connected successfully!' + #10);
			break;
		end;
		if (i = cint(ASSOCSTATUS_CANNOTCONNECT)) then
		begin
			iprintf('Could not connect!' +#10);
      while true do;
			break;
		end;
	end;
  // if connected, you can now use the berkley sockets interface to connect to the internet!

  //////////////////////////////////////////////////////////////////////////
  // Let's send a simple HTTP request to a server and print the results!
  // store the HTTP request for later
  // Find the IP address of the server, with gethostbyname
  myhost := gethostbyname('www.akkit.org');
  iprintf('Found IP Address!' + #10);

  // Create a TCP socket
  my_socket := socket( AF_INET, SOCK_STREAM, 0 );
  iprintf('Created Socket!' + #10);

  // Tell the socket to connect to the IP address we found, on port 80 (HTTP)
  sain.sin_family := AF_INET;
  sain.sin_port := htons(80);

  sain.sin_addr.s_addr := culong(Pointer(myhost^.h_addr_list^)^);

  iprintf(#10 + 'Accepted %d.%d.%d.%d:%d' + #10, [sain.sin_addr.s_addr          and $FF,
                                                 (sain.sin_addr.s_addr shr 8)   and $FF,
                                                 (sain.sin_addr.s_addr shr 16)  and $FF,
                                                 (sain.sin_addr.s_addr shr 24)  and $FF,
                                                  ntohs(sain.sin_port)]);

  connect(my_socket, psockaddr(@sain), sizeof(sain));
  iprintf('Connected to server!' + #10);

  // send our request
  send(my_socket, pchar(request_text), strlen(request_text), 0);
  iprintf('Sent our request!' + #10);

  // Print incoming data
  iprintf('Printing incoming data:' + #10);


  repeat 
    recvd_len := recv( my_socket, @incoming_buffer, 255, 0 ) ;
    // if recv returns 0, the socket has been closed.
    if (recvd_len > 0) then // data was received!
    begin
      incoming_buffer[recvd_len] := #0; // null-terminate
      iprintf(incoming_buffer);
    end;
  until recvd_len <= 0;

  iprintf('Other side closed connection!' + #10);
  shutdown(my_socket, 0); // good practice to shutdown the socket.
  closesocket(my_socket); // remove the socket.
  
  while true do;
end.
