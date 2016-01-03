program main;

{$apptype arm9}
{$define ARM9}

{$mode objfpc}

uses
  ctypes, nds9;


var
//display list
{
	triangle: array [0..12] of cuint32 = (
	12,
	FIFO_COMMAND_PACK(FIFO_BEGIN, FIFO_COLOR, FIFO_VERTEX16, FIFO_COLOR),
	GL_TRIANGLE,
	RGB15(31,0,0),
	VERTEX_PACK(inttov16(-1),inttov16(-1)), VERTEX_PACK(0,0),
	RGB15(0,31,0),
	FIFO_COMMAND_PACK(FIFO_VERTEX16, FIFO_COLOR, FIFO_VERTEX16, FIFO_END),
	VERTEX_PACK(inttov16(1),inttov16(-1)), VERTEX_PACK(0,0),
	RGB15(0,0,31),
	VERTEX_PACK(inttov16(0),inttov16(1)), VERTEX_PACK(0,0)
);
}
  triangle: array [0..12] of cuint32;
	rotateX: cfloat = 0.0;
	rotateY: cfloat = 0.0;
	keys: u16;


procedure SetDisplayList;
begin
	triangle[0] := 12;
	triangle[1] := FIFO_COMMAND_PACK(FIFO_BEGIN, FIFO_COLOR, FIFO_VERTEX16, FIFO_COLOR);
	triangle[2] := GL_TRIANGLE;
	triangle[3] := RGB15(31,0,0);
	triangle[4] := VERTEX_PACK(inttov16(-1),inttov16(-1)); 
	triangle[5] := VERTEX_PACK(0,0);
	triangle[6] := RGB15(0,31,0);
	triangle[7] := FIFO_COMMAND_PACK(FIFO_VERTEX16, FIFO_COLOR, FIFO_VERTEX16, FIFO_END);
	triangle[8] := VERTEX_PACK(inttov16(1),inttov16(-1)); 
	triangle[9] := VERTEX_PACK(0,0);
	triangle[10] := RGB15(0,0,31);
	triangle[11] := VERTEX_PACK(inttov16(0),inttov16(1)); 
	triangle[12] := VERTEX_PACK(0,0);
end;

begin
	powerON(POWER_ALL);
	
	//set mode 0, enable BG0 and set it to 3D
	videoSetMode(MODE_0_3D);
	
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
	SetDisplayList;	

		
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
		
		glMatrixMode(GL_TEXTURE);
		glIdentity();
		
		glMatrixMode(GL_MODELVIEW);
		
		
		//not a real gl function and will likely change
		glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE);
		
		scanKeys();
		
		keys := keysHeld();
		
		if ((keys and KEY_UP)) <> 0 then rotateX := rotateX + 3;
		if ((keys and KEY_DOWN)) <> 0 then rotateX := rotateX - 3;
		if ((keys and KEY_LEFT)) <> 0 then rotateY := rotateY + 3;
		if ((keys and KEY_RIGHT)) <> 0 then rotateY := rotateY - 3;
		
		glCallList(@triangle);	
		
		glPopMatrix(1);
		
		glFlush(0);
		
	end;
	
end.
