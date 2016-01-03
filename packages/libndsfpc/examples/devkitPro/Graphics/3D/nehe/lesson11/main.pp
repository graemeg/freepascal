(****************************************
 * 		NDS NeHe Lesson 11    			*
 * 		Author: Dovoto					*
 ****************************************)
program main;
{$L drunkenlogo.pcx.o}
{$apptype arm9} 
{$define ARM9}  

{$mode objfpc} 

uses
  ctypes, nds9; 

var
  drunkenlogo_pcx_end: array [0..0] of u8; cvar; external;
  drunkenlogo_pcx: array [0..0] of u8; cvar; external;
  drunkenlogo_pcx_size: u32; cvar; external;

  points: array [0..63, 0..31, 0..2] of v16;    // The Array For The Points On The Grid Of Our "Wave"
  wiggle_count: integer = 0;		// Counter Used To Control How Fast Flag Waves

  xrot: GLfloat;				// X Rotation ( NEW )
  yrot: GLfloat;				// Y Rotation ( NEW )
  zrot: GLfloat;				// Z Rotation ( NEW )
  hold: v16;				// Temporarily Holds A Floating Point Value

  texture: array [0..0] of integer;			// Storage For 3 Textures (only going to use 1 on the DS for this demo)

function DrawGLScene(): boolean;											// Here's Where We Do All The Drawing
var
	x, y: integer;
	float_x, float_y, float_xb, float_yb: t16;
begin

	glColor3b(255,255,255);    // set the vertex color
	
	glLoadIdentity();									// Reset The View

	glTranslatef(0.0,0.0,-12.0);
	  
	glRotatef(xrot,1.0,0.0,0.0);
	glRotatef(yrot,0.0,1.0,0.0);  
	glRotatef(zrot,0.0,0.0,1.0);

	glBindTexture(GL_TEXTURE_2D, texture[0]);

	glBegin(GL_QUADS);
	for x := 0 to 30 do
	begin
		for  y := 0 to 30 do
		begin
			float_x := inttot16(x) shl 2;
			float_y := inttot16(y) shl 2;
			float_xb := inttot16(x+1) shl 2;
			float_yb := inttot16(y+1) shl 2;

			glTexCoord2t16( float_x, float_y);
			glVertex3v16( points[x][y][0], points[x][y][1], points[x][y][2] );

			glTexCoord2t16( float_x, float_yb );
			glVertex3v16( points[x][y+1][0], points[x][y+1][1], points[x][y+1][2] );

			glTexCoord2t16( float_xb, float_yb );
			glVertex3v16( points[x+1][y+1][0], points[x+1][y+1][1], points[x+1][y+1][2] );

			glTexCoord2t16( float_xb, float_y );
			glVertex3v16( points[x+1][y][0], points[x+1][y][1], points[x+1][y][2] );
		end;
	end;
	glEnd();

	if ( wiggle_count = 2 ) then
	begin
		for y := 0 to 31 do
		begin
			hold := points[0][y][2];
			for x := 0 to 31 do
			begin
				points[x][y][2] := points[x+1][y][2];
			end;
			points[31][y][2]:=hold;
		end;
		wiggle_count := 0;
	end;

	inc(wiggle_count);

	xrot:=xrot+0.3;
	yrot:=yrot+0.2;
	zrot:=zrot+0.4;

	drawGLScene := TRUE;											// Everything Went OK

end;


 
function LoadGLTextures(): boolean;									// Load PCX files And Convert To Textures
var
	pcx: sImage;                //////////////(NEW) and different from nehe.
begin
	//load our texture
	loadPCX(pu8(drunkenlogo_pcx), @pcx);
	
	image8to16(@pcx);

	glGenTextures(1, @texture[0]);
	glBindTexture(0, texture[0]);
	glTexImage2D(0, 0, GL_RGB, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcx.image.data8);

	//imageDestroy(@pcx);
	result := TRUE;
end;

procedure InitGL();
var
  x, y:integer;
begin
		// Turn on everything
	powerON(POWER_ALL);
	
	// Setup the Main screen for 3D 
	videoSetMode(MODE_0_3D);
	vramSetBankA(VRAM_A_TEXTURE);                        //NEW  must set up some memory for textures

	// IRQ basic setup
	irqInit();
	irqSet(IRQ_VBLANK, nil);

	
	// initialize the geometry engine
	glInit();
	
	// enable textures
	glEnable(GL_TEXTURE_2D);
	
	// Set our viewport to be the same size as the screen
	glViewPort(0,0,255,191);
	
	// enable antialiasing
	glEnable(GL_ANTIALIAS);
	
	// setup the rear plane
	glClearColor(0,0,0,31); // BG must be opaque for AA to work
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);
	
	LoadGLTextures();
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(35, 256.0 / 192.0, 0.1, 100);
	
	//need to set up some material properties since DS does not have them set by default
	glMaterialf(GL_AMBIENT, RGB15(31,31,31));
	glMaterialf(GL_DIFFUSE, RGB15(31,31,31));
	glMaterialf(GL_SPECULAR, BIT(15) or RGB15(16,16,16));
	glMaterialf(GL_EMISSION, RGB15(31,31,31));
	
	//ds uses a table for shinyness..this generates a half-ass one
	glMaterialShinyness();
	
	//ds specific, several attributes can be set here	
	glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE );
	

	for x:=0 to 31 do
	begin
		for y:=0 to 31 do
		begin
			points[x][y][0] := (inttov16(x) div 4);
			points[x][y][1] := (inttov16(y) div 4);
			points[x][y][2] := SIN_bin[ (x * 16) and LUT_MASK];
		end;
	end;
end;


begin	
	InitGL();
	
	glMatrixMode(GL_MODELVIEW);

	while true do
	begin	
		DrawGLScene();
		
		// flush to screen	
		glFlush(0);
		
		// wait for the screen to refresh
		swiWaitForVBlank();
	end;
end.
