program main;
{$L texture.o}

{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!


//texture_bin.h is created automagicaly from the texture.bin placed in arm9/resources
//texture.bin is a raw 128x128 16 bit image.  I will release a tool for texture conversion 
//later
var
  texture_bin_end: array [0..0] of u8; cvar; external;
  texture_bin: array [0..0] of u16; cvar; external;
  texture_bin_size: u32; cvar; external;
	textureID: integer;

	rotateX: cfloat = 0.0;
	rotateY: cfloat = 0.0;
	keys: u16;


begin

	powerON(POWER_ALL);

	//set mode 0, enable BG0 and set it to 3D
	videoSetMode(MODE_0_3D);

	//irqs are nice
	irqInit();
	irqEnable(IRQ_VBLANK);

	// initialize gl
	glInit();
	
	//enable textures
	glEnable(GL_TEXTURE_2D);
	
	// enable antialiasing
	glEnable(GL_ANTIALIAS);
	
	// setup the rear plane
	glClearColor(0,0,0,31); // BG must be opaque for AA to work
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);

	//this should work the same as the normal gl call
	glViewPort(0,0,255,191);
	
	vramSetBankA(VRAM_A_TEXTURE);

	glGenTextures(1, @textureID);
	glBindTexture(0, textureID);
	glTexImage2D(0, 0, GL_RGB, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture_bin));
	
	
	//any floating point gl call is being converted to fixed prior to being implemented
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(35, 256.0 / 192.0, 0.1, 40);
	
	gluLookAt(	0.0, 0.0, 1.0,		//camera possition 
				0.0, 0.0, 0.0,		//look at
				0.0, 1.0, 0.0);		//up	
	
	while true do
	begin
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		//move it away from the camera
		glTranslate3f32(0, 0, floattof32(-1));
				
		glRotateX(rotateX);
		glRotateY(rotateY);
		
		

		glMaterialf(GL_AMBIENT, RGB15(16,16,16));
		glMaterialf(GL_DIFFUSE, RGB15(16,16,16));
		glMaterialf(GL_SPECULAR, BIT(15) or RGB15(8,8,8));
		glMaterialf(GL_EMISSION, RGB15(16,16,16));

		//ds uses a table for shinyness..this generates a half-ass one
		glMaterialShinyness();

		//not a real gl function and will likely change
		glPolyFmt(POLY_ALPHA(31) or POLY_CULL_BACK);

		scanKeys();
		
		keys := keysHeld();
		
		if((keys and KEY_UP)) <> 0 then rotateX := rotateX +3;
		if((keys and KEY_DOWN)) <> 0 then rotateX := rotateX -3;
		if((keys and KEY_LEFT)) <> 0 then rotateY := rotateY +3;
		if((keys and KEY_RIGHT)) <> 0 then rotateY := rotateY -3;
		
		glBindTexture(0, textureID);

		//draw the obj
		glBegin(GL_QUAD);
			glNormal(NORMAL_PACK(0,inttov10(-1),0));

			GFX_TEX_COORD^ := (TEXTURE_PACK(0, inttot16(128)));
			glVertex3v16(floattov16(-0.5),	floattov16(-0.5), 0 );
	
			GFX_TEX_COORD^ := (TEXTURE_PACK(inttot16(128),inttot16(128)));
			glVertex3v16(floattov16(0.5),	floattov16(-0.5), 0 );
	
			GFX_TEX_COORD^ := (TEXTURE_PACK(inttot16(128), 0));
			glVertex3v16(floattov16(0.5),	floattov16(0.5), 0 );

			GFX_TEX_COORD^ := (TEXTURE_PACK(0,0));
			glVertex3v16(floattov16(-0.5),	floattov16(0.5), 0 );
		
		glEnd();
		
		glPopMatrix(1);
			
		glFlush(0);

		swiWaitForVBlank();
	end;
end.
