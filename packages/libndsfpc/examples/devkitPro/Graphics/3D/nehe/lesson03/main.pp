(****************************************
 * 		NDS NeHe Lesson 03    			*
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
	
	glLoadIdentity();									// Reset The Current Modelview Matrix
	glTranslatef(-1.5,0.0,-6.0);						// Move Left 1.5 Units And Into The Screen 6.0
	glBegin(GL_TRIANGLES);								// Drawing Using Triangles
		glColor3f(1.0,0.0,0.0);						// Set The Color To Red
		glVertex3f( 0.0, 1.0, 0.0);					// Top
		glColor3f(0.0,1.0,0.0);						// Set The Color To Green
		glVertex3f(-1.0,-1.0, 0.0);					// Bottom Left
		glColor3f(0.0,0.0,1.0);						// Set The Color To Blue
		glVertex3f( 1.0,-1.0, 0.0);					// Bottom Right
	glEnd();											// Finished Drawing The Triangle
	glTranslatef(3.0,0.0,0.0);						// Move Right 3 Units
	glColor3f(0.5,0.5,1.0);							// Set The Color To Blue One Time Only
	glBegin(GL_QUADS);									// Draw A Quad
		glVertex3f(-1.0, 1.0, 0.0);					// Top Left
		glVertex3f( 1.0, 1.0, 0.0);					// Top Right
		glVertex3f( 1.0,-1.0, 0.0);					// Bottom Right
		glVertex3f(-1.0,-1.0, 0.0);					// Bottom Left
	glEnd();											// Done Drawing The Quad
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

  glInit();
  
	// Set our viewport to be the same size as the screen
	glViewPort(0,0,255,191);
	
	// Specify the Clear Color and Depth 
	glClearColor(0,0,0,31);
	glClearDepth($7FFF);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
	gluPerspective(35, 256.0 / 192.0, 0.1, 100);

	//ds specific, several attributes can be set here	
	glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE);
		
	// Set the current matrix to be the model matrix
	glMatrixMode(GL_MODELVIEW);
	
  glColor3f(1, 1, 1);									// Set the color..not in nehe source...ds gl default will be black
	
	while true do 
	begin

		DrawGLScene();
		
		// flush to screen	
		glFlush(0);

		//a handy little built in function to wait for a screen refresh
		swiWaitForVBlank();
		
	end;
	
end.
