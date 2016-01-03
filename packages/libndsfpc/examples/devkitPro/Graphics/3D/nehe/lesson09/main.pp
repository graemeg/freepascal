(****************************************
 * 		NDS NeHe Lesson 09    			*
 * 		Author: dovoto
 *		DS does not appear to support 
		the features needed for this demo
 ****************************************)

program main;
{$L Star.pcx.o}
{$apptype arm9} //...or arm7
{$define ARM9}   //...or arm7, according to apptype

{$mode objfpc}   // required for some libc funcs implementation

uses
  ctypes, nds9; // required by nds headers!


type
  TStars = record       // Create A Structure For Star
    r, g, b: integer;   // Stars Color
    dist: cfloat;       // Stars Distance From Center
    angle: cfloat;      // Stars Current Angle
  end;


var
  Star_pcx_end: array [0..0] of u8; cvar; external;
  Star_pcx: array [0..0] of u8; cvar; external;
  Star_pcx_size: u32; cvar; external;
  
  twinkle: boolean;		// Twinkling Stars
  tp: boolean;				// 'T' Key Pressed?

const	
  num = 50;		// Number Of Stars To Draw


var
  star: array [0..num-1] of TStars;		// Need To Keep Track Of 'num' Stars
  zoom: double = -15.0;	// Distance Away From Stars
  tilt: double = 90.0;		// Tilt The View
  spin: double;			// Spin Stars

  loop: integer;			// General Loop Variable
  texture: array [0..0] of integer;			// Storage For One textures



// Load PCX files And Convert To Textures
function LoadGLTextures(): boolean;									// Load PCX files And Convert To Textures
var
	pcx: sImage;                //////////////(NEW) and different from nehe.
begin
	//load our texture
	loadPCX(pu8(Star_pcx), @pcx);
	
	image8to16(@pcx);

	glGenTextures(1, @texture[0]);
	glBindTexture(0, texture[0]);
	glTexImage2D(0, 0, GL_RGBA, TEXTURE_SIZE_128 , TEXTURE_SIZE_128, 0, TEXGEN_TEXCOORD, pcx.image.data8);

	imageDestroy(@pcx);

	result := TRUE;
end;

function DrawGLScene(): boolean;											// Here's Where We Do All The Drawing
var
  loop: integer;
begin
	glBindTexture(GL_TEXTURE_2D, texture[0]);			// Select Our Texture

	for loop := 0 to num - 1 do						// Loop Through All The Stars
	begin
		glLoadIdentity();								// Reset The View Before We Draw Each Star
		glTranslatef(0.0, 0.0, zoom);					// Zoom Into The Screen (Using The Value In 'zoom')
		glRotatef(tilt, 1.0, 0.0, 0.0);					// Tilt The View (Using The Value In 'tilt')
		glRotatef(star[loop].angle, 0.0, 1.0, 0.0);		// Rotate To The Current Stars Angle
		glTranslatef(star[loop].dist, 0.0, 0.0);		// Move Forward On The X Plane
		glRotatef(-star[loop].angle, 0.0, 1.0, 0.0);	// Cancel The Current Stars Angle
		glRotatef(-tilt, 1.0, 0.0, 0.0);				// Cancel The Screen Tilt
		
		if (twinkle) then
		begin
			glColor3b(star[(num-loop)-1].r,star[(num-loop)-1].g,star[(num-loop)-1].b);  ///different
			glBegin(GL_QUADS);
				glTexCoord2f(0.0, 0.0); glVertex3f(-1.0,-1.0, 0.0);
				glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,-1.0, 0.0);
				glTexCoord2f(1.0, 1.0); glVertex3f( 1.0, 1.0, 0.0);
				glTexCoord2f(0.0, 1.0); glVertex3f(-1.0, 1.0, 0.0);
			glEnd();
		end;

		glRotatef(spin, 0.0, 0.0, 1.0);
		glColor3b(star[loop].r,star[loop].g,star[loop].b);                            //different
		glBegin(GL_QUADS);
			glTexCoord2f(0.0, 0.0); glVertex3f(-1.0,-1.0, 0.0);
			glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,-1.0, 0.0);
			glTexCoord2f(1.0, 1.0); glVertex3f( 1.0, 1.0, 0.0);
			glTexCoord2f(0.0, 1.0); glVertex3f(-1.0, 1.0, 0.0);
		glEnd();

		spin := spin + 0.01;
		star[loop].angle := star[loop].angle + (loop / num);
		star[loop].dist := star[loop].dist - 0.01;
		if (star[loop].dist < 0.0) then
		begin
			star[loop].dist := star[loop].dist + 5.0;
			star[loop].r :=rand(256);
			star[loop].g :=rand(256);
			star[loop].b :=rand(256);
		end;
	end;
	result := TRUE;															// Keep Going

end;


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
	
	// enable antialiasing
	glEnable(GL_ANTIALIAS);
	
	// setup the rear plane
	glClearColor(0,0,0,31); // BG must be opaque for AA to work
	glClearPolyID(63); // BG must have a unique polygon ID for AA to work
	glClearDepth($7FFF);
	
	// enable textures
	glEnable(GL_TEXTURE_2D);
	
	// enable alpha blending
	glEnable(GL_BLEND);

	// Set our viewport to be the same size as the screen
	glViewport(0,0,255,191);
	
	LoadGLTextures();
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70, 256.0 / 192.0, 0.1, 100);
	glColor3f(1,1,1);
	
	//set up a directional ligth arguments are light number (0-3), light color, 
	//and an x,y,z vector that points in the direction of the light
	glLight(0, RGB15(31,31,31) , 0, 0,floattov10(-1.0));
	
	//need to set up some material properties since DS does not have them set by default
	glMaterialf(GL_AMBIENT, RGB15(16,16,16));
	glMaterialf(GL_DIFFUSE, RGB15(16,16,16));
	glMaterialf(GL_SPECULAR, BIT(15) or RGB15(8,8,8));
	glMaterialf(GL_EMISSION, RGB15(16,16,16));
	
	//ds uses a table for shinyness..this generates a half-ass one
	glMaterialShinyness();
	
	glPolyFmt(POLY_ALPHA(15) or POLY_CULL_BACK or POLY_FORMAT_LIGHT0);

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
