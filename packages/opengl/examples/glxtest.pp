{
  GLX demo for FreePascal
  2005 Bart Tierens, BTierens@netscape.net

  This program is in the public domain
  
  Warning: This demo works only with FreePascal 2.1 and better, due to changes to the glx header
}
program glxTest;

{$MODE delphi}

uses glx,unix,x,xlib,xutil,gl,glu;

var
  attr: Array[0..8] of integer = (GLX_RGBA,GLX_RED_SIZE,1,GLX_GREEN_SIZE,1,GLX_BLUE_SIZE,1,GLX_DOUBLEBUFFER,none);
  visinfo: PXVisualInfo;
  cm: TColormap;
  winAttr: TXSetWindowAttributes;
  glXCont: GLXContext;
  dpy: PDisplay;
  win: TWindow;

procedure redraw();
begin
  glClear(GL_COLOR_BUFFER_BIT);

  glTranslatef(-0.5,-0.5,-2);
  glBegin(GL_QUADS);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glColor3f(0,1,0);
    glVertex3f(1,0,0);
    glColor3f(0,0,1);
    glVertex3f(1,1,0);
    glColor3f(1,1,1);
    glVertex3f(0,1,0);
  glEnd();

  glXSwapBuffers(dpy, win); //Swap the buffers
end;

procedure resize(width,height: integer);
begin
  glViewport(0,0,width,height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45,width/height,0.1,200);
  glMatrixMode(GL_MODELVIEW);
end;

procedure loop();
var
  event: TXEvent;
begin
  while true do
  begin
    XNextEvent(dpy,@event);
    case event._type of
    Expose: redraw();
    ConfigureNotify: resize(event.xconfigure.width,event.xconfigure.height);
    KeyPress: halt(1);
    end;
  end;
end;

var
  errorBase,eventBase: integer;
  window_title_property: TXTextProperty;
  title: String;
begin
  initGlx();
  dpy := XOpenDisplay(nil);
  if(dpy = nil) then
  writeLn('Error: Could not connect to X server');

  if not (glXQueryExtension(dpy,errorBase,eventBase)) then
  writeLn('Error: GLX extension not supported');

  visinfo := glXChooseVisual(dpy,DefaultScreen(dpy), Attr);
  if(visinfo = nil) then
  writeLn('Error: Could not find visual');

  //Create a new colormap
  cm := XCreateColormap(dpy,RootWindow(dpy,visinfo.screen),visinfo.visual,AllocNone);
  winAttr.colormap := cm;
  winAttr.border_pixel := 0;
  winAttr.background_pixel := 0;
  winAttr.event_mask := ExposureMask or ButtonPressMask or StructureNotifyMask or KeyPressMask;

  //Create a window
  win := XCreateWindow(dpy,RootWindow(dpy,visinfo.screen),0,0,640,480,0,visinfo.depth,InputOutput,visinfo.visual,CWBorderPixel or CWColormap or CWEventMask,@winAttr);

  title := 'FreePascal GLX demo --------- Press any key to exit';
  XStringListToTextProperty(@title,1,@window_title_property);
  XSetWMName(dpy,win,@window_title_property);

  //Create an OpenGL rendering context
  glXCont := glXCreateContext(dpy,visinfo,none,true);
  if(glXCont = nil) then
  writeLn('Error: Could not create an OpenGL rendering context');

  //Make it current
  glXMakeCurrent(dpy,win,glXCont);

  //Map the window on the display
  XMapWindow(dpy,win);
  
  loop();
end.