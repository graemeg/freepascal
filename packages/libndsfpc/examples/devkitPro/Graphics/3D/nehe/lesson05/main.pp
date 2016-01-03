(****************************************
 * 		NDS NeHe Lesson 05    			*
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
	glLoadIdentity();									// Reset The Current Modelview Matrix
	glTranslatef(-1.5,0.0,-6.0);						// Move Left 1.5 Units And Into The Screen 6.0
	glRotatef(rtri,0.0,1.0,0.0);						// Rotate The Triangle On The Y axis ( NEW )
	glBegin(GL_TRIANGLES);								// Start Drawing A Triangle
		glColor3f(1.0,0.0,0.0);						// Red
		glVertex3f( 0.0, 1.0, 0.0);					// Top Of Triangle (Front)
		glColor3f(0.0,1.0,0.0);						// Green
		glVertex3f(-1.0,-1.0, 1.0);					// Left Of Triangle (Front)
		glColor3f(0.0,0.0,1.0);						// Blue
		glVertex3f( 1.0,-1.0, 1.0);					// Right Of Triangle (Front)
		glColor3f(1.0,0.0,0.0);						// Red
		glVertex3f( 0.0, 1.0, 0.0);					// Top Of Triangle (Right)
		glColor3f(0.0,0.0,1.0);						// Blue
		glVertex3f( 1.0,-1.0, 1.0);					// Left Of Triangle (Right)
		glColor3f(0.0,1.0,0.0);						// Green
		glVertex3f( 1.0,-1.0, -1.0);					// Right Of Triangle (Right)
		glColor3f(1.0,0.0,0.0);						// Red
		glVertex3f( 0.0, 1.0, 0.0);					// Top Of Triangle (Back)
		glColor3f(0.0,1.0,0.0);						// Green
		glVertex3f( 1.0,-1.0, -1.0);					// Left Of Triangle (Back)
		glColor3f(0.0,0.0,1.0);						// Blue
		glVertex3f(-1.0,-1.0, -1.0);					// Right Of Triangle (Back)
		glColor3f(1.0,0.0,0.0);						// Red
		glVertex3f( 0.0, 1.0, 0.0);					// Top Of Triangle (Left)
		glColor3f(0.0,0.0,1.0);						// Blue
		glVertex3f(-1.0,-1.0,-1.0);					// Left Of Triangle (Left)
		glColor3f(0.0,1.0,0.0);						// Green
		glVertex3f(-1.0,-1.0, 1.0);					// Right Of Triangle (Left)
	glEnd();											// Done Drawing The Pyramid

	glLoadIdentity();									// Reset The Current Modelview Matrix
	glTranslatef(1.5,0.0,-7.0);						// Move Right 1.5 Units And Into The Screen 7.0
	glRotatef(rquad,1.0,1.0,1.0);					// Rotate The Quad On The X axis ( NEW )
	glBegin(GL_QUADS);									// Draw A Quad
		glColor3f(0.0,1.0,0.0);						// Set The Color To Green
		glVertex3f( 1.0, 1.0,-1.0);					// Top Right Of The Quad (Top)
		glVertex3f(-1.0, 1.0,-1.0);					// Top Left Of The Quad (Top)
		glVertex3f(-1.0, 1.0, 1.0);					// Bottom Left Of The Quad (Top)
		glVertex3f( 1.0, 1.0, 1.0);					// Bottom Right Of The Quad (Top)
		glColor3f(1.0,0.5,0.0); 					// Set The Color To Orange
		glVertex3f( 1.0,-1.0, 1.0);					// Top Right Of The Quad (Bottom)
		glVertex3f(-1.0,-1.0, 1.0);					// Top Left Of The Quad (Bottom)
		glVertex3f(-1.0,-1.0,-1.0);					// Bottom Left Of The Quad (Bottom)
		glVertex3f( 1.0,-1.0,-1.0);					// Bottom Right Of The Quad (Bottom)
		glColor3f(1.0,0.0,0.0);						// Set The Color To Red
		glVertex3f( 1.0, 1.0, 1.0);					// Top Right Of The Quad (Front)
		glVertex3f(-1.0, 1.0, 1.0);					// Top Left Of The Quad (Front)
		glVertex3f(-1.0,-1.0, 1.0);					// Bottom Left Of The Quad (Front)
		glVertex3f( 1.0,-1.0, 1.0);					// Bottom Right Of The Quad (Front)
		glColor3f(1.0,1.0,0.0);						// Set The Color To Yellow
		glVertex3f( 1.0,-1.0,-1.0);					// Top Right Of The Quad (Back)
		glVertex3f(-1.0,-1.0,-1.0);					// Top Left Of The Quad (Back)
		glVertex3f(-1.0, 1.0,-1.0);					// Bottom Left Of The Quad (Back)
		glVertex3f( 1.0, 1.0,-1.0);					// Bottom Right Of The Quad (Back)
		glColor3f(0.0,0.0,1.0);						// Set The Color To Blue
		glVertex3f(-1.0, 1.0, 1.0);					// Top Right Of The Quad (Left)
		glVertex3f(-1.0, 1.0,-1.0);					// Top Left Of The Quad (Left)
		glVertex3f(-1.0,-1.0,-1.0);					// Bottom Left Of The Quad (Left)
		glVertex3f(-1.0,-1.0, 1.0);					// Bottom Right Of The Quad (Left)
		glColor3f(1.0,0.0,1.0);						// Set The Color To Violet
		glVertex3f( 1.0, 1.0,-1.0);					// Top Right Of The Quad (Right)
		glVertex3f( 1.0, 1.0, 1.0);					// Top Left Of The Quad (Right)
		glVertex3f( 1.0,-1.0, 1.0);					// Bottom Left Of The Quad (Right)
		glVertex3f( 1.0,-1.0,-1.0);					// Bottom Right Of The Quad (Right)
	glEnd();											// Done Drawing The Quad

	rtri:=rtri+0.2;											// Increase The Rotation Variable For The Triangle ( NEW )
	rquad:=rquad-0.15;										// Decrease The Rotation Variable For The Quad ( NEW )
	DrawGLScene := TRUE;										// Keep Going
end;


begin	
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
	
	// Specify the Clear Color and Depth 
	glClearColor(0,0,0,31);
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);
	
		// Set our viewport to be the same size as the screen
	glViewPort(0,0,255,191);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();	
	gluPerspective(35, 256.0 / 192.0, 0.1, 100);
	
	//ds specific, several attributes can be set here	
	glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE);
		
  while true do
	begin
		// Set the current matrix to be the model matrix
		glMatrixMode(GL_MODELVIEW);
		
		//Push our original Matrix onto the stack (save state)
		glPushMatrix();	

		DrawGLScene();
		
		// Pop our Matrix from the stack (restore state)
		glPopMatrix(1);

		// flush to screen	
		glFlush(0);
    
    swiWaitForVBlank();	
	end;
end.
