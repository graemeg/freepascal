program main;
{$L texture.o}
{$L texture1_RGB16_pal.o}
{$L texture1_RGB16_tex.o}
{$L texture2_RGB16_pal.o}
{$L texture2_RGB16_tex.o}
{$L texture3_RGB16_pal.o}
{$L texture3_RGB16_tex.o}
{$L texture4_RGB16_pal.o}
{$L texture4_RGB16_tex.o}
{$L texture5_RGB16_pal.o}
{$L texture5_RGB16_tex.o}
{$L texture6_RGB4_pal.o}
{$L texture6_RGB4_tex.o}
{$L texture7_RGB4_pal.o}
{$L texture7_RGB4_tex.o}
{$L texture8_RGB32_A3_pal.o}
{$L texture8_RGB32_A3_tex.o}
{$L texture9_RGB32_A3_pal.o}
{$L texture9_RGB32_A3_tex.o}




{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!

var
{$include data.inc}

var
//verticies for the cube
  CubeVectors: array [0..23] of v16;
//polys
  CubeFaces: array [0..23] of u8;
  //texture coordinates
  uv: array [0..3] of u32;
  normals: array [0..5] of u32;


procedure Initialize();
begin
//verticies for the cube
  CubeVectors[0] := floattov16(-0.5); CubeVectors[1] := floattov16(-0.5); CubeVectors[2] := floattov16(0.5); 
	CubeVectors[3] := floattov16(0.5); CubeVectors[4] := floattov16(-0.5); CubeVectors[5] := floattov16(0.5);
	CubeVectors[6] := floattov16(0.5); CubeVectors[7] := floattov16(-0.5); CubeVectors[8] := floattov16(-0.5);
	CubeVectors[9] := floattov16(-0.5); CubeVectors[10] := floattov16(-0.5); CubeVectors[11] := floattov16(-0.5);
	CubeVectors[12] := floattov16(-0.5); CubeVectors[13] := floattov16(0.5); CubeVectors[14] := floattov16(0.5);
	CubeVectors[15] := floattov16(0.5); CubeVectors[16] := floattov16(0.5); CubeVectors[17] := floattov16(0.5);
	CubeVectors[18] := floattov16(0.5); CubeVectors[19] := floattov16(0.5); CubeVectors[20] := floattov16(-0.5);
	CubeVectors[21] := floattov16(-0.5); CubeVectors[22] := floattov16(0.5); CubeVectors[23] := floattov16(-0.5);

//polys
  CubeFaces[0] := 3; CubeFaces[1] := 2; CubeFaces[2] := 1; CubeFaces[3] := 0;
	CubeFaces[4] := 0; CubeFaces[5] := 1; CubeFaces[6] := 5; CubeFaces[7] := 4;
	CubeFaces[8] := 1; CubeFaces[9] := 2; CubeFaces[10] := 6; CubeFaces[11] := 5;
  CubeFaces[12] := 2; CubeFaces[13] := 3; CubeFaces[14] := 7; CubeFaces[15] := 6;
	CubeFaces[16] := 3; CubeFaces[17] := 0; CubeFaces[18] := 4; CubeFaces[19] := 7;
	CubeFaces[20] := 5; CubeFaces[21] := 6; CubeFaces[22] := 7; CubeFaces[23] := 4;

  //texture coordinates
  uv[0] := TEXTURE_PACK(inttot16(128), 0);
  uv[1] := TEXTURE_PACK(inttot16(128),inttot16(128));
	uv[2] := TEXTURE_PACK(0, inttot16(128));
	uv[3] := TEXTURE_PACK(0,0);

  normals[0] := NORMAL_PACK(0,floattov10(-0.97),0);
	normals[1] := NORMAL_PACK(0,0,floattov10(0.97));
	normals[2] := NORMAL_PACK(floattov10(0.97),0,0);
	normals[3] := NORMAL_PACK(0,0,floattov10(-0.97));
	normals[4] := NORMAL_PACK(floattov10(-0.97),0,0);
	normals[5] := NORMAL_PACK(0,floattov10(0.97),0);
end;

//draw a cube face at the specified color
procedure drawQuad(poly: integer);
var
  f1, f2, f3, f4: u32;
begin	
	f1 := CubeFaces[poly * 4] ;
	f2 := CubeFaces[poly * 4 + 1] ;
	f3 := CubeFaces[poly * 4 + 2] ;
	f4 := CubeFaces[poly * 4 + 3] ;


	glNormal(normals[poly]);

	GFX_TEX_COORD^ := (uv[0]);
	glVertex3v16(CubeVectors[f1*3], CubeVectors[f1*3 + 1], CubeVectors[f1*3 +  2] );
	
	GFX_TEX_COORD^ := (uv[1]);
	glVertex3v16(CubeVectors[f2*3], CubeVectors[f2*3 + 1], CubeVectors[f2*3 + 2] );
	
	GFX_TEX_COORD^ := (uv[2]);
	glVertex3v16(CubeVectors[f3*3], CubeVectors[f3*3 + 1], CubeVectors[f3*3 + 2] );

	GFX_TEX_COORD^ := (uv[3]);
	glVertex3v16(CubeVectors[f4*3], CubeVectors[f4*3 + 1], CubeVectors[f4*3 + 2] );
end;

type
  TTextures = record
    format, pal_addr: cint; 
    size: u32;
  end;

var
  textureIDS: array [0..9] of cint;
	textures: array [0..9] of TTextures;
	i: integer;
	rotateX: cfloat = 0.0;
	rotateY: cfloat = 0.0;
  keyspressed: u16;
  keys: u16;
  nTexture: integer;

begin	
  Initialize();
	powerON(POWER_ALL);
	lcdMainOnTop();
	
	//set mode 0, enable BG0 and set it to 3D
	videoSetMode(MODE_0_3D);
	
	videoSetModeSub(MODE_0_2D or DISPLAY_BG0_ACTIVE); //sub bg 0 will be used to print text
  	vramSetBankC(VRAM_C_SUB_BG); 
	// black backdrop
	BG_PALETTE_SUB[0] := RGB15(0,0,0);
	SUB_BG0_CR^ := BG_MAP_BASE(31);
	BG_PALETTE_SUB[255] := RGB15(31,31,31);//by default font rendered with color 255


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
	
	//ds uses a table for shinyness..this generates a half-ass one
	glMaterialShinyness();
	
	// setup other material properties
	glMaterialf(GL_AMBIENT, RGB15(16,16,16));
	glMaterialf(GL_DIFFUSE, RGB15(20,20,20));
	glMaterialf(GL_SPECULAR, BIT(15) or RGB15(8,8,8));
	glMaterialf(GL_EMISSION, RGB15(5,5,5));
	
	// setup the lighting
	glLight(0, RGB15(31,31,31) , 0, floattov10(-0.5), floattov10(-0.85));
	
	vramSetBankA(VRAM_A_TEXTURE);

	glGenTextures(10, textureIDS);
	
	// inital full 16 bit colour texture
	glBindTexture(0, textureIDS[0]);
	glTexImage2D(0, 0, GL_RGB, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture_bin));
	textures[0].format := GL_RGB;
	textures[0].pal_addr := 0;	
	textures[0].size := texture_bin_size; 	// size field just recorded for on-screen info
	

	// Load some 16 colour textures
	glBindTexture(0, textureIDS[1]);
	glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture1_RGB16_tex_bin));
	textures[1].format := GL_RGB16;
	textures[1].pal_addr := gluTexLoadPal( pu16(texture1_RGB16_pal_bin), 16, GL_RGB16 );
	textures[1].size := texture1_RGB16_tex_bin_size+texture1_RGB16_pal_bin_size;

	glBindTexture(0, textureIDS[2]);
	glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture2_RGB16_tex_bin));
	textures[2].format := GL_RGB16;
	textures[2].pal_addr := gluTexLoadPal( pu16(texture2_RGB16_pal_bin), 16, GL_RGB16 );
	textures[2].size := texture2_RGB16_tex_bin_size+texture2_RGB16_pal_bin_size;
	
	glBindTexture(0, textureIDS[3]);
	glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture3_RGB16_tex_bin));
	textures[3].format := GL_RGB16;
	textures[3].pal_addr := gluTexLoadPal( pu16(texture3_RGB16_pal_bin), 16, GL_RGB16 );
	textures[3].size := texture3_RGB16_tex_bin_size+texture3_RGB16_pal_bin_size;
	
	glBindTexture(0, textureIDS[4]);
	glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture4_RGB16_tex_bin));
	textures[4].format := GL_RGB16;
	textures[4].pal_addr := gluTexLoadPal( pu16(texture4_RGB16_pal_bin), 16, GL_RGB16 );
	textures[4].size := texture4_RGB16_tex_bin_size+texture4_RGB16_pal_bin_size;
	
	glBindTexture(0, textureIDS[5]);
	glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture5_RGB16_tex_bin));
	textures[5].format := GL_RGB16;
	textures[5].pal_addr := gluTexLoadPal( pu16(texture5_RGB16_pal_bin), 16, GL_RGB16 );
	textures[5].size := texture5_RGB16_tex_bin_size+texture5_RGB16_pal_bin_size;


	// Load some 4 colour textures
	glBindTexture(0, textureIDS[6]);
	glTexImage2D(0, 0, GL_RGB4, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture6_RGB4_tex_bin));
	textures[6].format := GL_RGB4;
	textures[6].pal_addr := gluTexLoadPal( pu16(texture6_RGB4_pal_bin), 4, GL_RGB4 );
	textures[6].size := texture6_RGB4_tex_bin_size+texture6_RGB4_pal_bin_size;

	glBindTexture(0, textureIDS[7]);
	glTexImage2D(0, 0, GL_RGB4, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture7_RGB4_tex_bin));
	textures[7].format := GL_RGB4;
	textures[7].pal_addr := gluTexLoadPal( pu16(texture7_RGB4_pal_bin), 4, GL_RGB4 );
	textures[7].size := texture7_RGB4_tex_bin_size+texture7_RGB4_pal_bin_size;


	// Load some 32 colour textures, 8 levels of alpha
	glBindTexture(0, textureIDS[8]);
	glTexImage2D(0, 0, GL_RGB32_A3, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture8_RGB32_A3_tex_bin));
	textures[8].format := GL_RGB32_A3;
	textures[8].pal_addr := gluTexLoadPal( pu16(texture8_RGB32_A3_pal_bin), 32, GL_RGB32_A3 );
	textures[8].size := texture8_RGB32_A3_tex_bin_size+texture8_RGB32_A3_pal_bin_size;

	glBindTexture(0, textureIDS[9]);
	glTexImage2D(0, 0, GL_RGB32_A3, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pu8(@texture9_RGB32_A3_tex_bin));
	textures[9].format := GL_RGB32_A3;
	textures[9].pal_addr := gluTexLoadPal( pu16(texture9_RGB32_A3_pal_bin), 32, GL_RGB32_A3 );
	textures[9].size := texture9_RGB32_A3_tex_bin_size+texture9_RGB32_A3_pal_bin_size;
	
	
	//consoleInit() is a lot more flexible but this gets you up and running quick
	consoleInitDefault(pu16(SCREEN_BASE_BLOCK_SUB(31)), pu16(CHAR_BASE_BLOCK_SUB(0)), 16);
	iprintf(#27 + '[4;8H' + 'Paletted Cube');
	iprintf(#27 + '[6;2H' + 'Right/Left shoulder to switch');
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(35, 256.0 / 192.0, 0.1, 40);
	
	gluLookAt(	0.0, 0.0, 2.0,		//camera possition 
				0.0, 0.0, 0.0,		//look at
				0.0, 1.0, 0.0);		//up
	
	//not a real gl function and will likely change
	glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE or POLY_FORMAT_LIGHT0 or POLY_ID(1) ) ;
	
	glColor3f(1,1,1);
	
	glMatrixMode(GL_MODELVIEW);
	
	nTexture := 0;
	while true do
	begin
		glPushMatrix();
				
		glRotateX(rotateX);
		glRotateY(rotateY);
		
		scanKeys();
		keys := keysHeld();
		if((keys and KEY_UP)) <> 0 then rotateX := rotateX +3;
		if((keys and KEY_DOWN)) <> 0 then rotateX := rotateX -3;
		if((keys and KEY_LEFT)) <> 0 then rotateY := rotateY +3;
		if((keys and KEY_RIGHT)) <> 0 then rotateY := rotateY -3;
		
		keysPressed := keysDown();
		if (keysPressed and KEY_R) = 0 then
		begin
		  inc(nTexture);
			if( nTexture = 10 )	then
				nTexture := 0;
		end;
    if (keysPressed and KEY_L) = 0 then
		begin
		  dec(nTexture);
			if( nTexture = -1 )	then
				nTexture := 9;
		end;
		
		glBindTexture(nTexture, textureIDS[nTexture]);
		if( textures[nTexture].format <> GL_RGB ) then
			glColorTable(textures[nTexture].format, textures[nTexture].pal_addr);

		//draw the obj
		glBegin(GL_QUAD);
		for i := 0 to 5 do
			drawQuad(i);
		glEnd();
		
		glPopMatrix(1);
			
		glFlush(0);
	end;

end.
