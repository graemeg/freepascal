(****************************************
 * 		NDS NeHe Lesson 01    			*
 * 		Author: Dovoto					*
 ****************************************)

program main;

{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!

function DrawGLScene(): boolean;
begin
	//we are going to use floating point for the tutorial...keep in mind the DS has no
	//floating point hardware.  For real life use the built in fixed point types.
	//this is where the magic happens
	glLoadIdentity();
	DrawGLScene := TRUE;
end;

begin	
  // Turn on everything
	powerON(POWER_ALL);
	
	// Setup the Main screen for 3D 
	videoSetMode(MODE_0_3D);
	
	// IRQ basic setup (not strickly required but nice
	irqInit();
	irqSet(IRQ_VBLANK, nil);
	
	// initialize the geometry engine
	glInit();

	// enable antialiasing
  glEnable(GL_ANTIALIAS);

	// setup the rear plane
	glClearColor(0, 0, 0, 31); // BG must be opaque for AA to work
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);

	// Set our viewport to be the same size as the screen
	glViewPort(0, 0, 255, 191);
	
	// setup the view
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(35, 256.0 / 192.0, 0.1, 100);

	//ds specific, several attributes can be set here	
	glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE);
	
	
	while true do
	begin
		// Set the current matrix to be the model matrix
		glMatrixMode(GL_MODELVIEW);
		
		glColor3f(1, 1, 1);									// Set the color..not in nehe source...ds gl default will be black
		
		//Push our original Matrix onto the stack (save state)
		glPushMatrix();	

		DrawGLScene();
		
		// Pop our Matrix from the stack (restore state)
		glPopMatrix(1);

		//a handy little built in function to wait for a screen refresh
		swiWaitForVBlank();

		// flush to screen	
		glFlush(0);	
	end;
	
end.
