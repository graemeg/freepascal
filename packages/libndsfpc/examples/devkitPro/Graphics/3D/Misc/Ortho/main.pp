program main;
{$L drunkenlogo.pcx.o}
{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!


var
  drunkenlogo_pcx_end: array [0..0] of cuint8; cvar; external;
  drunkenlogo_pcx: array [0..0] of cuint8; cvar; external;
  drunkenlogo_pcx_size: cuint32; cvar; external;

var
  xrot: cfloat;				// X Rotation ( NEW )
  yrot: cfloat;				// Y Rotation ( NEW )
  zrot: cfloat;				// Z Rotation ( NEW )
  
  texture: array [0..0] of integer;			// Storage For One Texture ( NEW )



function LoadGLTextures(): boolean;									// Load PCX files And Convert To Textures
var
	pcx: sImage;                //////////////(NEW) and different from nehe.
begin
	//load our texture
	loadPCX(pcuint8(drunkenlogo_pcx), @pcx);
	
	image8to16(@pcx);

	glGenTextures(1, @texture[0]);
	glBindTexture(0, texture[0]);
	glTexImage2D(0, 0, GL_RGB, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcx.image.data8);

	imageDestroy(@pcx);

	LoadGLTextures := true;
end;


function DrawGLScene(): boolean;											// Here's Where We Do All The Drawing
begin
	glTranslatef(0.0,0.0,-5.0);

	glRotatef(xrot,1.0,0.0,0.0);
	glRotatef(yrot,0.0,1.0,0.0);
	glRotatef(zrot,0.0,0.0,1.0);

	glBindTexture(GL_TEXTURE_2D, texture[0]);

	glBegin(GL_QUADS);
		// Front Face
		glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0,  1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0,  1.0);
		// Back Face
		glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0, -1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0, -1.0);
		// Top Face
		glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f(-1.0,  1.0,  1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,  1.0,  1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
		// Bottom Face
		glTexCoord2f(1.0, 1.0); glVertex3f(-1.0, -1.0, -1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f( 1.0, -1.0, -1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
		// Right face
		glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0, -1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0,  1.0);
		glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0);
		// Left Face
		glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0, -1.0);
		glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0);
		glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0,  1.0);
		glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0);
	glEnd();

	xrot := xrot + 0.3;
	yrot := yrot + 0.2;
	zrot := zrot + 0.4;
	DrawGLScene := true;			
end;



begin
	// Turn on everything
	powerON(POWER_ALL);
	
	// Setup the Main screen for 3D 
	videoSetMode(MODE_0_3D);
	vramSetBankA(VRAM_A_TEXTURE); // reserve some memory for textures

	// IRQ basic setup
	irqInit();
	irqSet(IRQ_VBLANK, nil);

	// initialize gl
	glInit();
	
	//enable textures
	//glEnable(GL_TEXTURE_2D);
	
	// enable antialiasing
	glEnable(GL_ANTIALIAS);
	
	// setup the rear plane
	glClearColor(0,0,0,31); // BG must be opaque for AA to work
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);
	
	glLight(0, RGB15(31,31,31) , 0,				  floattov10(-1.0),		 0);
	glLight(1, RGB15(31,31,31) , 0,				  0,	floattov10(-1.0));
	glLight(2, RGB15(31,31,31) , 0,				  0,	floattov10(1.0));
	
	glMatrixMode(GL_TEXTURE);
	glIdentity();
	
	glMatrixMode(GL_MODELVIEW);
	
	//need to set up some material properties since DS does not have them set by default
	glMaterialf(GL_AMBIENT, RGB15(16,16,16));
	glMaterialf(GL_DIFFUSE, RGB15(16,16,16));
	glMaterialf(GL_SPECULAR, BIT(15) or RGB15(8,8,8));
	glMaterialf(GL_EMISSION, RGB15(16,16,16));
	
	//ds uses a table for shinyness..this generates a half-ass one
	glMaterialShinyness();

	// Set our viewport to be the same size as the screen
	glViewPort(0,0,255,191);
	
	LoadGLTextures();
	
	// set the vertex color to white
	glColor3f(1,1,1);

	while true do
	begin
		scanKeys();
		
		//reset the projection matrix
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		
		// set the projection matrix as either ortho or perspective
		if (keysHeld() and KEY_R) = 0 then
			gluPerspective(35, 256.0 / 192.0, 0.1, 100)
		else
			glOrtho(-3, 3,-2, 2, 0.1, 100);

		// Set the current matrix to be the model matrix
		glMatrixMode(GL_MODELVIEW);
		
		//ds specific, several attributes can be set here	
		if (keysHeld() and KEY_L) <> 0 then
			glPolyFmt(POLY_ALPHA(0) or POLY_CULL_NONE  or POLY_FORMAT_LIGHT0 or POLY_FORMAT_LIGHT1 or POLY_FORMAT_LIGHT2)
		else
			glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE or POLY_FORMAT_LIGHT0 or POLY_FORMAT_LIGHT1 or POLY_FORMAT_LIGHT2);
		
	
		//Push our original Matrix onto the stack (save state)
		glPushMatrix();	

		DrawGLScene();
		
		// Pop our Matrix from the stack (restore state)
		glPopMatrix(1);

		// flush to screen	
		glFlush(0);
	end;
end.
