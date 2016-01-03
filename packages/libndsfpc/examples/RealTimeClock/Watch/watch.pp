program watch;

{$mode objfpc}

uses
  ctypes, nds9;
  
var
  hours, seconds, minutes: integer; 
  months: array [0..11] of pchar = ('January', 'February', 'March', 'April', 
                                    'May', 'June', 'July', 'August', 
                                    'September', 'October', 'November', 'December');

  unixTime: time_t;
  timeStruct: ptm;

//function errno2: pcint; cdecl; external name 'errno';

{ $define errno := __errno()^}


//draw a watch hands
procedure DrawQuad(x, y, width, height: cfloat); 
begin
  glBegin(GL_QUADS); 
  glVertex3f(x - width / 2, y, 0); 
  glVertex3f(x + width / 2, y, 0); 
  glVertex3f(x  + width / 2, y  + height, 0); 
  glVertex3f(x - width / 2, y  + height, 0); 
  glEnd(); 
end; 


begin
  //put 3D on top 
  lcdMainOnTop(); 
  
  // Setup the Main screen for 3D 
  videoSetMode(MODE_0_3D); 
  
  // Reset the screen and setup the view 
  glInit();
  
  // Set our viewport to be the same size as the screen 
  glViewport(0,0,255,191); 
  
  // Specify the Clear Color and Depth 
  glClearColor(0,0,0,31); 
  glClearDepth($7FFF); 
  
  consoleDemoInit();
  
  
  
  
  //ds specific, several attributes can be set here    
  glPolyFmt(POLY_ALPHA(31) or POLY_CULL_NONE); 
  
  // Set the current matrix to be the model matrix 
  glMatrixMode(GL_MODELVIEW); 
  glLoadIdentity(); 
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(70, 256.0 / 192.0, 0.1, 100);
  gluLookAt(  0.0, 0.0, 3.0,    //camera possition 
              0.0, 0.0, 0.0,    //look at
              0.0, 1.0, 0.0);   //up
	
  while true do 
  begin
    unixTime := time(nil);
    timeStruct := gmtime(ptime_t(@unixTime));
    
    hours := timeStruct^.tm_hour;
    minutes := timeStruct^.tm_min;
    seconds := timeStruct^.tm_sec;
    
    //Push our original Matrix onto the stack (save state) 
    glPushMatrix();    
    
    //draw second hand 
    glColor3f(0,0,1); 
    glRotateZ(-seconds * 360 / 60); 
    glTranslatef(0, 1.9, 0); 
    DrawQuad(0, 0, 0.2, 0.2); 
    
    // Pop our Matrix from the stack (restore state) 
    glPopMatrix(1); 
    
    //Push our original Matrix onto the stack (save state) 
    glPushMatrix();    
    
    //draw minute hand 
    glColor3f(0, 1, 0); 
    glRotateZ(-minutes * 360 / 60); 
    DrawQuad(0, 0, 0.2, 2); 
    
    // Pop our Matrix from the stack (restore state) 
    glPopMatrix(1); 
    
    //Push our original Matrix onto the stack (save state) 
    glPushMatrix();    
    
    //draw hourhand 
    glColor3f(1, 0, 0); 
    glRotateZ(-hours * 360 / 12); 
    DrawQuad(0, 0, 0.3, 1.8); 
    
    // Pop our Matrix from the stack (restore state) 
    glPopMatrix(1);
    
    printf(#$1B'[2J%02i:%02i:%02i', hours, minutes, seconds);
    printf(#10'%s %i %i', months[timeStruct^.tm_mon], timeStruct^.tm_mday, timeStruct^.tm_year +1900);
    
    // flush to screen    
    glFlush(0);
    
    swiWaitForVBlank();
	end;

end.
