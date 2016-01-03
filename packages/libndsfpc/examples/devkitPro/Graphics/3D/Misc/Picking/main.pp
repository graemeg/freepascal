(*---------------------------------------------------------------------------------
Demonstrates how to use 3D picking on the DS

Author: Gabe Ghearing
Created: Feb 2007

This file is released into the public domain

Basic idea behind picking;
    Draw the scene a second time with a projection matrix that only renders what is
directly below the cursor. The GPU keeps track of how many polygons are drawn, so
if a polygon is drawn in this limited view the polygon is directly below the cursor.
Several polygons may be drawn under the cursor, so a position test is used for each
object(a collection of polygons) to tell which object is closest to the camera.
The object that is closest to the camera and under the cursor is the one that the
user is clicking on.

There are several optimizations that are not done in this example, such as:
   - Simplify models during the picking pass, the model needs to occupy the
       same area, but can usually use fewer polygons.
   - Save the projection matrix with glGetFixed() instead of recreating it
       every pass.

*--------------------------------------------------------------------------------*)

program main;
{$L cone.o}
{$L cylinder.o}
{$L sphere.o}

{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!


type
  TClickable = ( clNothing, clCone, clCylinder, clSphere);

var
  cone_bin_end: array [0..0] of u8; cvar; external;
  cone_bin: array [0..0] of u16; cvar; external;
  cone_bin_size: u32; cvar; external;
  cylinder_bin_end: array [0..0] of u8; cvar; external;
  cylinder_bin: array [0..0] of u16; cvar; external;
  cylinder_bin_size: u32; cvar; external;
  sphere_bin_end: array [0..0] of u8; cvar; external;
  sphere_bin: array [0..0] of u16; cvar; external;
  sphere_bin_size: u32; cvar; external;

  clicked: TClickable; // what is being clicked
  closeW: integer; // closest distace to camera
  polyCount: integer; // keeps track of the number of polygons drawn

// run before starting to draw an object while picking
procedure startCheck(); 
begin
	while PosTestBusy() do; // wait for the position test to finish
	while GFX_BUSY do; // wait for all the polygons from the last object to be drawn
	PosTest_Asynch(0,0,0); // start a position test at the current translated position
	polyCount := GFX_POLYGON_RAM_USAGE^; // save the polygon count
end;

// run afer drawing an object while picking
procedure endCheck(obj: TClickable);
begin
	while GFX_BUSY do; // wait for all the polygons to get drawn
	while PosTestBusy() do; // wait for the position test to finish
	if (GFX_POLYGON_RAM_USAGE^ > polyCount) then // if a polygon was drawn
	begin
		if PosTestWresult() <= closeW then
		begin
			// this is currently the closest object under the cursor!
			closeW := PosTestWresult();
			clicked := obj;
		end;
	end;
end;

var
	rotateX: cuint32 = 0;
	rotateY: cuint32 = 0;
  touchXY: touchPosition;
  viewport: array [0..3] of integer = (0,0,255,191); // used later for gluPickMatrix()
	keys: u16;
	
begin
	// power up everything; this a bit wasteful but this isn't a power management example
	powerON(POWER_ALL);

	//set mode 0, enable BG0 and set it to 3D
	videoSetMode(MODE_0_3D);

	//irqs are nice
	irqInit();
	irqSet(IRQ_VBLANK, nil);
	
	lcdMainOnBottom(); // we are going to be touching the 3D display

	// initialize gl
	glInit();
	
	// enable edge outlining, this will be used to show which object is selected
	glEnable(GL_OUTLINE);
	
	//set the first outline color to white
	glSetOutlineColor(0,RGB15(31,31,31));
		
	// setup the rear plane
	glClearColor(0,0,0,0); // set BG to black and clear
	glClearPolyID(0); // the BG and polygons will have the same ID unless a polygon is highlighted
	glClearDepth($7FFF);
	
	// setup the camera
	gluLookAt(	0.0, 0.0, 1.0,		//camera possition 
				0.0, 0.0, 0.0,		//look at
				0.0, 1.0, 0.0);		//up
	
	glLight(0, RGB15(31,31,31) , 0, floattov10(-1.0), 0); // setup the light
	
	while true do
	begin
		// handle key input
		scanKeys();
		keys := keysHeld();
		if ((keys and KEY_UP)) = 0 then rotateX := rotateX +3;
		if((keys and KEY_DOWN)) = 0 then rotateX := rotateX -3;
		if((keys and KEY_LEFT)) = 0 then rotateY := rotateY +3;
		if((keys and KEY_RIGHT)) = 0 then rotateY := rotateY -3;
		
		// get touchscreen position
		touchXY := touchReadXY();
		
		glViewPort(0,0,255,191); // set the viewport to fullscreen
		
		// setup the projection matrix for regular drawing
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(30, 256.0 / 192.0, 0.1, 20); 
		
		glMatrixMode(GL_MODELVIEW); // use the modelview matrix while drawing
		
		glPushMatrix(); // save the state of the current matrix(the modelview matrix)
			glTranslate3f32(0,0,floattof32(-6));
			glRotateXi(rotateX); // add X rotation to the modelview matrix
			glRotateYi(rotateY); // add Y rotation to the modelview matrix
			
			glPushMatrix(); // save the state of the modelview matrix while making the first pass
				// draw the scene for displaying
				
				glTranslate3f32(floattof32(2.9),floattof32(0),floattof32(0)); // translate the modelview matrix to the drawing location
				if (clicked = clCone) then
					glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0 or POLY_ID(1)) // set a poly ID for outlining
				else 
					glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0 or POLY_ID(0)); // set a poly ID for no outlining (same as BG)
				
				glCallList((@cone_bin)); // draw a green cone from a predefined packed command list
				
				
				glTranslate3f32(floattof32(-3),floattof32(1.8),floattof32(2)); // translate the modelview matrix to the drawing location
				if (clicked = clCylinder) then
					glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0 or POLY_ID(1)) // set a poly ID for outlining
				else 
					glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0 or POLY_ID(0)); // set a poly ID for no outlining (same as BG)
				
        glCallList((@cylinder_bin)); // draw a blue cylinder from a predefined packed command list
				
				
				glTranslate3f32(floattof32(0.5),floattof32(-2.6),floattof32(-4)); // translate the modelview matrix to the drawing location
				if(clicked = clSphere) then
					glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0 or POLY_ID(1)) // set a poly ID for outlining
				else 
					glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0 or POLY_ID(0)); // set a poly ID for no outlining (same as BG)
				
				glCallList((@sphere_bin)); // draw a red sphere from a predefined packed command list

			
			glPopMatrix(1); // restores the modelview matrix to where it was just rotated
			
			// draw the scene again for picking
				
				clicked := clNothing; //reset what was clicked on
				closeW := $7FFFFFFF; //reset the distance
				
				//set the viewport to just off-screen, this hides all rendering that will be done during picking
				glViewPort(0,192,0,192);
				
				// setup the projection matrix for picking
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				gluPickMatrix((touchXY.px),(191-touchXY.py),4,4,viewport); // render only what is below the cursor
				gluPerspective(30, 256.0 / 192.0, 0.1, 20); // this must be the same as the original perspective matrix
				
				glMatrixMode(GL_MODELVIEW); // switch back to modifying the modelview matrix for drawing
				
				glTranslate3f32(floattof32(2.9),floattof32(0),floattof32(0)); // translate the modelview matrix to the drawing location
				startCheck();
				glCallList((@cone_bin)); // draw a cone from a predefined packed command list
				endCheck(clCone);
				
				glTranslate3f32(floattof32(-3),floattof32(1.8),floattof32(2)); // translate the modelview matrix to the drawing location
				startCheck();
				glCallList((@cylinder_bin)); // draw a cylinder from a predefined packed command list
				endCheck(clCylinder);
				
				glTranslate3f32(floattof32(0.5),floattof32(-2.6),floattof32(-4)); // translate the modelview matrix to the drawing location
				startCheck();
				glCallList((@sphere_bin)); // draw a sphere from a predefined packed command list
				endCheck(clSphere);
			
		glPopMatrix(1); // restores the modelview matrix to its original state
		
		glFlush(0); // wait for everything to be drawn before starting on the next frame
	end;

end.
