(*---------------------------------------------------------------------------------

	$id $
	
	Box test to demonstrate 3D bounding box es.  also shows the effect of culling and
	clipping on vertex usage
	$log $
*--------------------------------------------------------------------------------*)

program main;

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;
  
  
var
	rotX: cfloat = 0; 
	rotY: cfloat = 0;
	translate: cfloat = -5.0;

	//some profiling code
	time: cuint16;

	//keep track of vertex ram usage
	polygon_count, vertex_count: integer;

	//object 
	rx: integer = 50;
	ry: integer = 15;
	oldx: integer = 0;
	oldy: integer = 0;

	hit: integer;

//some code for profiling
function startTimer(timer: cint): cuint16;
begin
	TIMER_CR(timer)^ := (0);
	TIMER_DATA(0)^ := (0);
	TIMER_CR(timer)^ := (TIMER_DIV_1);
	startTimer := TIMER_DATA(0)^;
end;

function getTimer(timer: integer): cuint32;
begin
  getTimer := TIMER_DATA(timer)^;
end;
{ $define getTimer(timer) := (TIMER_DATA(timer))^}

//draws a box...same signature as boxTest
procedure DrawBox(x, y, z, height, width, depth: cfloat);
begin
	glBegin(GL_QUADS);
	//z  face
	glColor3f(1,0,0);
	glVertex3f(x , y , z );
	glVertex3f(x + width, y , z );
	glVertex3f(x + width, y + height, z );
	glVertex3f(x , y + height, z );

	//z + depth face
	glColor3f(1,0,1);
	glVertex3f(x , y , z + depth);
	glVertex3f(x , y + height, z + depth);
	glVertex3f(x + width, y + height, z + depth);
	glVertex3f(x + width, y , z + depth);


	//x  face
	glColor3f(1,1,0);
	glVertex3f(x , y , z );
	glVertex3f(x , y + height, z );
	glVertex3f(x , y + height, z + depth);
	glVertex3f(x , y , z + depth);

	//x + width face
	glColor3f(1,1,1);
	glVertex3f(x + width, y , z );
	glVertex3f(x + width, y , z + depth);
	glVertex3f(x + width, y + height, z + depth);
	glVertex3f(x + width, y + height, z );

	//y  face
	glColor3f(0,1,0);
	glVertex3f(x , y , z );
	glVertex3f(x , y , z + depth);
	glVertex3f(x + width, y , z + depth);
	glVertex3f(x + width, y , z );

	//y  + height face
	glColor3f(0,1,1);
	glVertex3f(x , y + height, z );
	glVertex3f(x + width, y + height, z );
	glVertex3f(x + width, y + height, z + depth);
	glVertex3f(x , y + height, z + depth);

	glEnd();

end;

var
  i: integer;
//draw the clock
begin

	// Turn on everything
	powerON(POWER_ALL);

	//put 3D on top
	lcdMainOnTop();

	//setup the sub screen for basic printing
	consoleDemoInit();

	// Setup the Main screen for 3D 
	videoSetMode(MODE_0_3D);

	// IRQ basic setup
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
	
	// Set our view port to be the same size as the screen
	glViewPort(0,0,255,191);

	//main loop
	while true do
	begin

		scanKeys();

		//process input
		if (keysHeld() and KEY_LEFT) <> 0 then rotY := rotY + 1;
		if (keysHeld() and KEY_RIGHT) <> 0 then rotY := rotY - 1;
		if (keysHeld() and KEY_UP) <> 0 then rotX := rotX + 1;
		if (keysHeld() and KEY_DOWN) <> 0 then rotX := rotX - 1;
		if (keysHeld() and KEY_L) <> 0 then translate := translate + 0.1;
		if (keysHeld() and KEY_R) <> 0 then translate := translate - 0.1;

		//reset x and y when user touches screen
		if (keysDown() and KEY_TOUCH) <> 0 then
		begin
			oldx := touchReadXY().px;
			oldy := touchReadXY().py;
		end;

		//if user drags then grab the delta
		if (keysHeld() and KEY_TOUCH) <> 0 then 
		begin
			rx := rx + touchReadXY().px - oldx; 
			ry := ry + touchReadXY().py - oldy;
			oldx := touchReadXY().px;
			oldy := touchReadXY().py;
		end;

		//change ortho vs perspective
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		if (keysHeld() and KEY_B) <> 0 then
			glOrtho(-4,4,-3,3,0.1,10)	
		else
			gluPerspective(35, 256.0 / 192.0, 0.1, 10);								

		//change cull mode
		if (keysHeld() and KEY_A) <> 0 then
			glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE )
		else
			glPolyFmt(POLY_ALPHA(31) or POLY_CULL_FRONT );

		// Set the current matrix to be the model matrix
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		//handle camera
		glRotateY(rotY);
		glRotateX(rotX);
		glTranslatef(0,0,translate);

		//move the cube
		glRotateX(ry);
		glRotateY(rx);

		DrawBox(-1,-1,-1,2,2,2);

		printf(#27 + '[2J' + 'Box test cycle count');

		time := startTimer(0);
		hit := BoxTestf(-1,-1,-1,2,2,2);
		printf(#10 + 'Single test (float): %i', [2*(getTimer(0) - time)]);

		time := startTimer(0);
		BoxTest(inttov16(-1),inttov16(-1),inttov16(-1),inttov16(2),inttov16(2),inttov16(2));
		printf(#10 + 'Single test (fixed): %i', [2*(getTimer(0) - time)]);

		time := startTimer(0);
		
		for	i := 0 to 63 do
			BoxTest(inttov16(-1),inttov16(-1),inttov16(-1),inttov16(2),inttov16(2),inttov16(2));

		printf(#10 + '64 tests avg. (fixed): %i', [(getTimer(0) - time) / 32]);
		if hit > 0 then
		  printf(#10 + 'Box Test result: hit')
		else
		  printf(#10 + 'Box Test result: miss');

		while (GFX_STATUS^ and (1 shl 27)) <> 0 do; // wait until the geometry engine is not busy		

		glGetInt(GL_GET_VERTEX_RAM_COUNT, vertex_count);
		glGetInt(GL_GET_POLYGON_RAM_COUNT, polygon_count);

		
    if (keysHeld() and KEY_A) <> 0 then
      printf(#10#10 + 'Ram usage: Culling none')
    else
      printf(#10#10 + 'Ram usage: Culling back faces');

		printf(#10 + 'Vertex ram: %i', vertex_count);
		printf(#10 + 'Polygon ram: %i', polygon_count);
		printf(#10#10 + 'Press A to change culling');
		printf(#10#10 + 'Press B to change Ortho vs Persp');
		printf(#10 + 'Press Left and Right Up and Down to rotate');
		printf(#10 + 'Press L and R to zoom');
		printf(#10 + 'Touch screen to rotate cube');

		//a handy little built in function to wait for a screen refresh
		swiWaitForVBlank();

		// flush to screen	
		glFlush(0);

	end;

end.
