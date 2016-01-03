program PalettedCube;
{$L build/texture.bin.o}
{$L build/texture1_RGB16_pal.bin.o}
{$L build/texture1_RGB16_tex.bin.o}
{$L build/texture2_RGB16_pal.bin.o}
{$L build/texture2_RGB16_tex.bin.o}
{$L build/texture3_RGB16_pal.bin.o}
{$L build/texture3_RGB16_tex.bin.o}
{$L build/texture4_RGB16_pal.bin.o}
{$L build/texture4_RGB16_tex.bin.o}
{$L build/texture5_RGB16_pal.bin.o}
{$L build/texture5_RGB16_tex.bin.o}
{$L build/texture6_RGB4_pal.bin.o}
{$L build/texture6_RGB4_tex.bin.o}
{$L build/texture7_RGB4_pal.bin.o}
{$L build/texture7_RGB4_tex.bin.o}
{$L build/texture8_RGB32_A3_pal.bin.o}
{$L build/texture8_RGB32_A3_tex.bin.o}
{$L build/texture9_RGB32_A3_pal.bin.o}
{$L build/texture9_RGB32_A3_tex.bin.o}
{$L build/texture10_COMP_tex.bin.o}
{$L build/texture10_COMP_texExt.bin.o}
{$L build/texture10_COMP_pal.bin.o}


{$mode objfpc}

{$mode objfpc}

uses
  ctypes, nds9;

// most of the following textures were generated from online samples available at
// http://www.marlinstudios.com/samples/sampvtf.htm and http://www.3dtotal.com/textures_v15/

{$include inc/texture.bin.inc}
{$include inc/texture1_RGB16_pal.bin.inc}
{$include inc/texture1_RGB16_tex.bin.inc}
{$include inc/texture2_RGB16_pal.bin.inc}
{$include inc/texture2_RGB16_tex.bin.inc}
{$include inc/texture3_RGB16_pal.bin.inc}
{$include inc/texture3_RGB16_tex.bin.inc}
{$include inc/texture4_RGB16_pal.bin.inc}
{$include inc/texture4_RGB16_tex.bin.inc}
{$include inc/texture5_RGB16_pal.bin.inc}
{$include inc/texture5_RGB16_tex.bin.inc}
{$include inc/texture6_RGB4_pal.bin.inc}
{$include inc/texture6_RGB4_tex.bin.inc}
{$include inc/texture7_RGB4_pal.bin.inc}
{$include inc/texture7_RGB4_tex.bin.inc}
{$include inc/texture8_RGB32_A3_pal.bin.inc}
{$include inc/texture8_RGB32_A3_tex.bin.inc}
{$include inc/texture9_RGB32_A3_pal.bin.inc}
{$include inc/texture9_RGB32_A3_tex.bin.inc}
{$include inc/texture10_COMP_tex.bin.inc}
{$include inc/texture10_COMP_texExt.bin.inc}
{$include inc/texture10_COMP_pal.bin.inc}

var
  //verticies for the cube
  CubeVectors: array [0..23] of v16;

  //polys
  CubeFaces: array [0..23] of cuint8;

  //texture coordinates
  uv: array [0..3] of cuint32;
  normals: array [0..5] of cuint32;

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
  uv[1] := TEXTURE_PACK(inttot16(128), inttot16(128));
  uv[2] := TEXTURE_PACK(0, inttot16(128));
  uv[3] := TEXTURE_PACK(0, 0);

  normals[0] := NORMAL_PACK(0, floattov10(-0.97), 0);
  normals[1] := NORMAL_PACK(0, 0, floattov10(0.97));
  normals[2] := NORMAL_PACK(floattov10(0.97), 0, 0);
  normals[3] := NORMAL_PACK(0, 0, floattov10(-0.97));
  normals[4] := NORMAL_PACK(floattov10(-0.97), 0, 0);
  normals[5] := NORMAL_PACK(0, floattov10(0.97), 0);
end;

//draw a cube face at the specified color
procedure drawQuad(poly: integer);
var
  f1, f2, f3, f4: cuint32;
begin
  f1 := CubeFaces[poly * 4] ;
  f2 := CubeFaces[poly * 4 + 1] ;
  f3 := CubeFaces[poly * 4 + 2] ;
  f4 := CubeFaces[poly * 4 + 3] ;

  glNormal(normals[poly]);


  GFX_TEX_COORD^ := (uv[0]);
  glVertex3v16(CubeVectors[f1*3], CubeVectors[f1*3 + 1], CubeVectors[f1*3 + 2]);

  GFX_TEX_COORD^ := (uv[1]);
  glVertex3v16(CubeVectors[f2*3], CubeVectors[f2*3 + 1], CubeVectors[f2*3 + 2]);

  GFX_TEX_COORD^ := (uv[2]);
  glVertex3v16(CubeVectors[f3*3], CubeVectors[f3*3 + 1], CubeVectors[f3*3 + 2]);

  GFX_TEX_COORD^ := (uv[3]);
  glVertex3v16(CubeVectors[f4*3], CubeVectors[f4*3 + 1], CubeVectors[f4*3 + 2]);
end;

var
  textureIDS: array [0..10] of cint;
  i, j: integer;
  rotateX: cfloat = 0.0;
  rotateY: cfloat = 0.0;
  keyspressed: cuint16;
  keys: cuint16;
  nTexture: integer;
  compTexture: pcuint8;
begin
  Initialize();
  lcdMainOnTop();

  //set mode 0, enable BG0 and set it to 3D
  videoSetMode(MODE_0_3D);
  //Because of letting the user manipulate which video banks the program will use,
  // I chose to manually set the console data into Bank I, as the demo default uses Bank C.
  videoSetModeSub(MODE_0_2D);
  vramSetBankI(VRAM_I_SUB_BG_0x06208000);
  consoleInit(nil, 0, BgType_Text4bpp, BgSize_T_256x256, 23, 2, false, true);

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
  glViewport(0,0,255,191);

  //ds uses a table for shinyness..this generates a half-ass one
  glMaterialShinyness();

  // setup other material properties
  glMaterialf(GL_AMBIENT, RGB15(16,16,16));
  glMaterialf(GL_DIFFUSE, RGB15(20,20,20));
  glMaterialf(GL_SPECULAR, BIT(15) or RGB15(8,8,8));
  glMaterialf(GL_EMISSION, RGB15(5,5,5));

  // setup the lighting
  glLight(0, RGB15(31,31,31) , 0, floattov10(-0.5), floattov10(-0.85));


  //You may comment/uncomment what you like, as the integration of nglVideo into libnds works
  // by examining the state of the banks, and deciding where to put textures/texpalettes based on that.
  //There are some exceptions to get certain stuff working though...
  // At least one main bank (A-D) must be allocated to textures to load/use them obviously, as well as 
  //  sub banks (E-G) for texture palettes
  // Compressed textures require bank B allocated, as well as bank A or C (or both) to be loadable/usable
  // 4 color palettes (not 4-bit) require either bank E, or bank F/G as slot0/1
  
  
  //vramSetBankA(VRAM_A_TEXTURE);
  vramSetBankB(VRAM_B_TEXTURE);
  vramSetBankC(VRAM_C_TEXTURE);
  //vramSetBankD(VRAM_D_TEXTURE);
  //vramSetBankE(VRAM_E_TEX_PALETTE);
  vramSetBankF(VRAM_F_TEX_PALETTE_SLOT0);
  vramSetBankG(VRAM_G_TEX_PALETTE_SLOT5); 

  glGenTextures(11, textureIDS);

  // inital full 16 bit colour texture
  glBindTexture(0, textureIDS[0]);
  glTexImage2D(0, 0, GL_RGB, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture_bin));

  // Load a 16 colour texture
  glBindTexture(0, textureIDS[1]);
  glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture1_RGB16_tex_bin));

  glColorTableEXT(0, 0, 16, 0, 0, pcuint16(@texture1_RGB16_pal_bin));

  // Just to show that this works, let's go and delete that very first texture that was loaded
  glDeleteTextures(1, @textureIDS[0]);  

  // Load some more 16 color textures
  glBindTexture(0, textureIDS[2]);
  glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture2_RGB16_tex_bin));
  glColorTableEXT( 0, 0, 16, 0, 0, pcuint16(@texture2_RGB16_pal_bin));

  glBindTexture(0, textureIDS[3]);
  glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture3_RGB16_tex_bin));
  glColorTableEXT( 0, 0, 16, 0, 0, pcuint16(@texture3_RGB16_pal_bin));

  // Now, re-generate the first texture, who's VRAM position won't be the same as before in the end
  glGenTextures(1, @textureIDS[0]);

  glBindTexture(0, textureIDS[4]);
  glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture4_RGB16_tex_bin));
  glColorTableEXT( 0, 0, 16, 0, 0, pcuint16(@texture4_RGB16_pal_bin));

  glBindTexture(0, textureIDS[5]);
  glTexImage2D(0, 0, GL_RGB16, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture5_RGB16_tex_bin));
  glColorTableEXT(0, 0, 16, 0, 0, pcuint16(@texture5_RGB16_pal_bin));

  // Load some 4 colour textures
  glBindTexture(0, textureIDS[6]);
  glTexImage2D(0, 0, GL_RGB4, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture6_RGB4_tex_bin));
  glColorTableEXT(0, 0, 4, 0, 0, pcuint16(@texture6_RGB4_pal_bin));

  glBindTexture(0, textureIDS[7]);
  glTexImage2D(0, 0, GL_RGB4, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture7_RGB4_tex_bin));
  glColorTableEXT(0, 0, 4, 0, 0, pcuint16(@texture7_RGB4_pal_bin));

  // Load some 32 colour textures, 8 levels of alpha
  glBindTexture(0, textureIDS[8]);
  glTexImage2D(0, 0, GL_RGB32_A3, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture8_RGB32_A3_tex_bin));
  glColorTableEXT(0, 0, 32, 0, 0, pcuint16(@texture8_RGB32_A3_pal_bin));

  glBindTexture(0, textureIDS[9]);
  glTexImage2D(0, 0, GL_RGB32_A3, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture9_RGB32_A3_tex_bin));
  glColorTableEXT(0, 0, 32, 0, 0, pcuint16(@texture9_RGB32_A3_pal_bin));

  // Load a 4x4 texel compressed texture
  // The tiles and header need to be combined together in that order
  // If this data is already pre-combined together, then you can just send it into the nglTexImage2D function
  compTexture := pcuint8(malloc(texture10_COMP_tex_bin_size + texture10_COMP_texExt_bin_size));
  swiCopy(@texture10_COMP_tex_bin, compTexture, (texture10_COMP_tex_bin_size shr 2) or COPY_MODE_WORD);
  swiCopy(@texture10_COMP_texExt_bin, compTexture + texture10_COMP_tex_bin_size, (texture10_COMP_texExt_bin_size shr 2) or COPY_MODE_WORD);

  glBindTexture(0, textureIDS[10]);
  glTexImage2D(0, 0, GL_COMPRESSED, TEXTURE_SIZE_128, TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@compTexture));
  glColorTableEXT(0, 0, texture10_COMP_pal_bin_size shr 1, 0, 0, pcuint16(@texture10_COMP_pal_bin));

  // Now, let's reload the full 16 bit color texture was had it's name deleted and regenerated
  glBindTexture(0, textureIDS[0]);
  glTexImage2D(0, 0, GL_RGB, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcuint8(@texture_bin));

  iprintf(#$1b'[4;8HPaletted Cube');
  iprintf(#$1b'[6;2HRight/Left shoulder to switch');

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(70, 256.0 / 192.0, 0.1, 40);

  gluLookAt( 0.0, 0.0, 2.0,    //camera possition
             0.0, 0.0, 0.0,    //look at
             0.0, 1.0, 0.0);   //up

  //not a real gl function and will likely change
  glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE or POLY_FORMAT_LIGHT0 or POLY_ID(1));

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
      if( nTexture = 11 ) then
        nTexture := 0;
    end;
    if (keysPressed and KEY_L) = 0 then
    begin
      dec(nTexture);
      if( nTexture = -1 ) then
        nTexture := 10;
    end;

    glBindTexture(nTexture, textureIDS[nTexture]);

    //draw the obj
    glBegin(GL_QUAD);
    for i := 0 to 5 do
      drawQuad(i);
    glEnd();

    glPopMatrix(1);

    glFlush(0);
    swiWaitForVBlank();
  end;

end.