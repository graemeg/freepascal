(****************************************
 * 		NDS NeHe Lesson 04    			*
 * 		Author: Dovoto					*
 ****************************************)

program main;

{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!

var
  rtri: cfloat = 0.0;				// Angle For The Triangle ( NEW )
  rquad: cfloat = 0.0;				// Angle For The Quad ( NEW )


function DrawGLScene(): boolean;
begin
	glIdentity();									// Reset The Current Modelview Matrix
	glTranslatef(-1.5,0.0,-6.0);						// Move Left 1.5 Units And Into The Screen 6.0
	glRotatef(rtri,0.0,1.0,0.0);						// Rotate The Triangle On The Y axis ( NEW )
	glColor3f(1, 1, 1);									// set the vertex color
	glBegin(GL_TRIANGLES);								// Drawing Using Triangles
		glColor3f(1.0,0.0,0.0);						// Set The Color To Red
		glVertex3f( 0.0, 1.0, 0.0);					// Top
		glColor3f(0.0,1.0,0.0);						// Set The Color To Green
		glVertex3f(-1.0,-1.0, 0.0);					// Bottom Left
		glColor3f(0.0,0.0,1.0);						// Set The Color To Blue
		glVertex3f( 1.0,-1.0, 0.0);					// Bottom Right
	glEnd();											// Finished Drawing The Triangle
	glIdentity();									// Reset The Current Modelview Matrix
	glTranslatef(1.5,0.0,-6.0);						// Move Right 3 Units
	glRotatef(rquad,1.0,0.0,0.0);					// Rotate The Quad On The X axis ( NEW )
	glColor3f(0.5,0.5,1.0);							// Set The Color To Blue One Time Only
	glBegin(GL_QUADS);									// Draw A Quad
		glVertex3f(-1.0, 1.0, 0.0);					// Top Left
		glVertex3f( 1.0, 1.0, 0.0);					// Top Right
		glVertex3f( 1.0,-1.0, 0.0);					// Bottom Right
		glVertex3f(-1.0,-1.0, 0.0);					// Bottom Left
	glEnd();											// Done Drawing The Quad
	rtri := rtri + 0.9;											// Increase The Rotation Variable For The Triangle ( NEW )
	rquad := rquad - 0.75;										// Decrease The Rotation Variable For The Quad ( NEW )
  DrawGLScene := True;										// Keep Going
end;



begin
//  rtri := 0;											// Increase The Rotation Variable For The Triangle ( NEW )
//	rquad := 0;										// Decrease The Rotation Variable For The Quad ( NEW )

	// Turn on everything
	powerON(POWER_ALL);

	// Setup the Main screen for 3D 
	videoSetMode(MODE_0_3D);
	
	// IRQ basic setup
	irqInit();
	irqSet(IRQ_VBLANK, nil);

	// initialize the geometry engine
	glInit();
	
	// enable antialiasing
	glEnable(GL_ANTIALIAS);

	// setup the rear plane
	glClearColor(0,0,0,31); // BG must be opaque for AA to work
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);

	// Set our viewport to be the same size as the screen
	glViewPort(0,0,255,191);

	

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(35, 256.0 / 192.0, 0.1, 100);


	//ds specific, several attributes can be set here	
	glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE);
	
  // Set the current matrix to be the model matrix
	glMatrixMode(GL_MODELVIEW);	
	
  while true do
	begin
		DrawGLScene();
		//a handy little built in function to wait for a screen refresh
		swiWaitForVBlank();
		// flush to screen	
		glFlush(0);
	end;
end.
