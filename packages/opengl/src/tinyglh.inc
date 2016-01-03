{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Karoly Balogh

     TinyGL/OpenGL wrapper include header for MorphOS/PowerPC

    Thanks to Michal 'kiero' Wozniak and Mark 'bigfoot' Olsen
    for their help.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ****************************************************************************************************** }
{ ** GL functions                                                                                     ** }
{ ****************************************************************************************************** }

{$IFDEF GL_UNIT}
procedure glAlphaFunc(func: GLenum; ref: GLclampf); inline;

procedure glBegin(mode: GLenum); inline;

procedure glCallList(list: GLuint); inline;
procedure glCallLists(n: GLsizei; atype: GLenum; const lists: Pointer); inline;
procedure glClear(mask: GLbitfield); inline;
procedure glClearColor(red, green, blue, alpha: GLclampf); inline;
procedure glClearDepth(depth: GLclampd); inline;
procedure glColor3f(red, green, blue: GLfloat); inline;
procedure glColor4f(red, green, blue, alpha: GLfloat); inline;
procedure glColorMask(red, green, blue, alpha: GLboolean); inline;
procedure glColorPointer(size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;
procedure glCopyTexImage2D(target: GLenum; level: GLint; internalFormat: GLenum; x, y: GLint; width, height: GLsizei; border: GLint); inline;
procedure glCullFace(mode: GLenum); inline;

procedure glDeleteLists(list: GLuint; range: GLsizei); inline;
procedure glDeleteTextures(n: GLsizei; const textures: PGLuint); inline;
procedure glDepthFunc(func: GLenum); inline;
procedure glDepthMask(flag: GLboolean); inline;
procedure glDisable(cap: GLenum); inline;
procedure glDisableClientState(aarray: GLenum); inline;
procedure glDrawElements(mode: GLenum; count: GLsizei; atype: GLenum; const indices: Pointer); inline;

procedure glEnable(cap: GLenum); inline;
procedure glEnableClientState(aarray: GLenum); inline;
procedure glEnd; inline;
procedure glEndList; inline;

procedure glFlush; inline;
procedure glFogf(pname: GLenum; param: GLfloat); inline;
procedure glFogfv(pname: GLenum; const params: PGLfloat); inline;
procedure glFogi(pname: GLenum; param: GLint); inline;

function  glGenLists(range: GLsizei): GLuint; inline;
procedure glGetFloatv(pname: GLenum; params: PGLfloat); inline;
function  glGetString(name: GLenum): PChar; inline;

procedure glHint(target, mode: GLenum); inline;

procedure glIndexi(c: GLint); inline;

procedure glLightf(light, pname: GLenum; param: GLfloat); inline;
procedure glLightfv(light, pname: GLenum; const params: PGLfloat); inline;
procedure glLineWidth(width: GLfloat); inline;
procedure glLoadIdentity; inline;
procedure glLightModelfv(pname: GLenum; const params: PGLfloat); inline;

procedure glMaterialf(face, pname: GLenum; param: GLfloat); inline;
procedure glMaterialfv(face, pname: GLenum; const params: PGLfloat); inline;
procedure glMateriali(face, pname: GLenum; param: GLint); inline;
procedure glMaterialiv(face, pname: GLenum; const params: PGLint); inline;
procedure glMatrixMode(mode: GLenum); inline;

procedure glNewList(list: GLuint; mode: GLenum); inline;
procedure glNormal3f(nx, ny, nz: GLfloat); inline;
procedure glNormalPointer(atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;

procedure glVertex2f(x, y: GLfloat); inline;
procedure glVertex2i(x, y: GLint); inline;
procedure glVertex3f(x, y, z: GLfloat); inline;
procedure glVertex3fv(const v: PGLfloat); inline;

procedure glPointSize(size: GLfloat); inline;
procedure glRasterPos3f(x, y, z: GLfloat); inline;
procedure glReadPixels(x, y: GLint; width, height: GLsizei; format, atype: GLenum; pixels: Pointer); inline;

procedure glShadeModel(mode: GLenum); inline;

procedure glStencilFunc(func: GLenum; ref: GLint; mask: GLuint); inline;
procedure glStencilOp(fail, zfail, zpass: GLenum); inline;

procedure glTexCoordPointer(size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;
procedure glTexParameterf(target: GLenum; pname: GLenum; param: GLfloat); inline;

procedure glFrustum(left, right, bottom, top, zNear, zFar: GLdouble); inline;
procedure glTranslatef(x, y, z: GLfloat); inline;
procedure glScalef(x, y, z: GLfloat); inline;
procedure glRotatef(angle, x, y, z: GLfloat); inline;

procedure glOrtho(left, right, bottom, top, zNear, zFar: GLdouble); inline;
procedure glPopMatrix; inline;
procedure glPushMatrix; inline;

procedure glTexImage2D(target: GLenum; level, internalformat: GLint; width, height: GLsizei; border: GLint; format, atype: GLenum; const pixels: Pointer); inline;
procedure glTexParameteri(target: GLenum; pname: GLenum; param: GLint); inline;
procedure glTexCoord2f(s, t: GLfloat); inline;
procedure glTexGeni(coord: GLenum; pname: GLenum; param: GLint); inline;
procedure glBindTexture(target: GLenum; texture: GLuint); inline;
procedure glGenTextures(n: GLsizei; textures: PGLuint); inline;

procedure glBlendFunc(sfactor, dfactor: GLenum); inline;

procedure glVertexPointer(size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;

procedure glViewport(x, y: GLint; width, height: GLsizei); inline;
{$ENDIF GL_UNIT}

{ ****************************************************************************************************** }
{ ** GLU functions                                                                                    ** }
{ ****************************************************************************************************** }

{$IFDEF GLU_UNIT}
procedure gluLookAt(eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz: GLdouble); inline;
procedure gluOrtho2D(left,right, bottom, top: GLdouble); inline;
procedure gluPerspective(fovy, aspect, zNear, zFar: GLdouble); inline;
{$ENDIF GLU_UNIT}

{ ****************************************************************************************************** }
{ ** GLUT functions                                                                                   ** }
{ ****************************************************************************************************** }

{$IFDEF GLUT_UNIT}
procedure GLUTInit(argcp: PLongInt; argv: PPChar); inline;
procedure GLUTInitDisplayMode(mode: Word); inline;
procedure GLUTInitWindowPosition(x, y: Integer); inline;
procedure GLUTInitWindowSize(width, height: LongInt); inline;

function  GLUTCreateWindow(const title: PChar): LongInt; inline;
procedure GLUTDestroyWindow(win: LongInt); inline;
procedure GLUTDisplayFunc(f: TGlutVoidCallback); inline;
procedure GLUTIdleFunc(f: TGlutVoidCallback); inline;
procedure GLUTTimerFunc(millis: Word; f: TGlut1IntCallback; value: LongInt); inline;
function  GLUTGetWindowHandle: pointer; inline; // MorphOS specific
procedure GLUTVisibilityFunc(f: TGlut1IntCallback); inline;
procedure GLUTKeyboardFunc(f: TGlut1Char2IntCallback); inline;
procedure GLUTMouseFunc(f: TGlut4IntCallback); inline;
procedure GLUTSpecialFunc(f: TGlut3IntCallback); inline;
procedure GLUTReshapeFunc(f: TGlut2IntCallback); inline;
procedure GLUTMotionFunc(f: TGlut2IntCallback); inline;
procedure GLUTPassiveMotionFunc(f: TGlut2IntCallback); inline;

procedure GLUTSwapBuffers; inline;
procedure GLUTPostRedisplay; inline;

function  GLUTGet(t: GLenum): LongInt; inline;
  
procedure GLUTMainLoop; inline;

procedure GLUTFullScreen; inline;
procedure GLUTSetColor(cell: Integer; red, green, blue: GLfloat); inline;
{$ENDIF GLUT_UNIT}
