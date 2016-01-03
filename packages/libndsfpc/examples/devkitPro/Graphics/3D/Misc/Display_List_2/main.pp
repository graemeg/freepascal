program main;
{$L teapot.o}

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;

//teapot display list provided by Mike260, as well as the display list gl code.

var
  teapot_bin_end: array [0..0] of u8; cvar; external;
  teapot_bin: array [0..0] of u16; cvar; external;
  teapot_bin_size: u32; cvar; external;
	rotateX: cfloat = 0.0;
	rotateY: cfloat = 0.0;
  keys: cuint16; 

begin
	powerON(POWER_ALL);

	//set mode 0, enable BG0 and set it to 3D
	videoSetMode(MODE_0_3D);

	//irqs are nice
	irqInit();
	irqSet(IRQ_VBLANK, nil);

	// initialize gl
	glInit();
	
	// enable antialiasing
	glEnable(GL_ANTIALIAS);
	
	// setup the rear plane
	glClearColor(0,0,0,31); // BG must be opaque for AA to work
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);

	//this should work the same as the normal gl call
	glViewPort(0,0,255,191);
	
	//any floating point gl call is being converted to fixed prior to being implemented
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(35, 256.0 / 192.0, 0.1, 40);
	
	gluLookAt(	0.0, 0.0, 3.5,		//camera possition 
				0.0, 0.0, 0.0,		//look at
				0.0, 1.0, 0.0);		//up
	
	glLight(0, RGB15(31,31,31) , 0,				  floattov10(-1.0),		 0);
	glLight(1, RGB15(31,0,31),   0,				  floattov10(1) - 1,			 0);
	glLight(2, RGB15(0,31,0) ,   floattov10(-1.0), 0,					 0);
	glLight(3, RGB15(0,0,31) ,   floattov10(1.0) - 1,  0,					 0);
	
	//not a real gl function and will likely change
	glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0 or POLY_FORMAT_LIGHT1 or 
			  POLY_FORMAT_LIGHT2 or POLY_FORMAT_LIGHT3 ) ;
	
	while true do
	begin
		glPushMatrix();
				
		glRotateX(rotateX);
		glRotateY(rotateY);
		
		scanKeys();
		keys := keysHeld();
		if (keys and KEY_UP) = 0 then rotateX := rotateX +3;
		if (keys and KEY_DOWN) = 0 then rotateX := rotateX -3;
		if (keys and KEY_LEFT) = 0 then rotateY := rotateY +3;
		if (keys and KEY_RIGHT) = 0 then rotateY := rotateY -3;
		
		glCallList(@teapot_bin);	

		glPopMatrix(1);
			
		glFlush(0);
	end;
end.
