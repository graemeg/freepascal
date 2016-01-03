program main;
{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!


var
  rotateX: cfloat = 0.0;
	rotateY: cfloat = 0.0;
	keys: cuint16;
	
begin	
	powerON(POWER_ALL);

	//set mode 0, enable BG0 and set it to 3D
	videoSetMode(MODE_0_3D);

	//irqs are nice
	irqInit();
	irqEnable(IRQ_VBLANK);

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
	
	gluLookAt(	0.0, 0.0, 1.0,		//camera possition 
				0.0, 0.0, 0.0,		//look at
				0.0, 1.0, 0.0);		//up
	
	while true do
	begin
		glPushMatrix();

		//move it away from the camera
		glTranslate3f32(0, 0, floattof32(-1));
				
		glRotateX(rotateX);
		glRotateY(rotateY);
		
		
		glMatrixMode(GL_MODELVIEW);



		//not a real gl function and will likely change
		glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE);
		
    scanKeys();

		keys := keysHeld();

		if ((keys and KEY_UP)) <> 0 then rotateX := rotateX + 3;
		if((keys and KEY_DOWN)) <> 0 then rotateX := rotateX - 3;
		if((keys and KEY_LEFT)) <> 0 then rotateY := rotateY + 3;
		if((keys and KEY_RIGHT)) <> 0 then rotateY := rotateY - 3;
		

		//draw the obj
		glBegin(GL_TRIANGLE);
			
			glColor3b(255,0,0);
			glVertex3v16(inttov16(-1),inttov16(-1),0);

			glColor3b(0,255,0);
			glVertex3v16(inttov16(1), inttov16(-1), 0);

			glColor3b(0,0,255);
			glVertex3v16(inttov16(0), inttov16(1), 0);
			
		glEnd();
		
		glPopMatrix(1);
			
		glFlush(0);

		swiWaitForVBlank();
	end;

end.
