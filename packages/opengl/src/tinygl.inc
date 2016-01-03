{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2005 by Karoly Balogh

     TinyGL/OpenGL wrapper include for MorphOS/PowerPC

    Thanks to Michal 'kiero' Wozniak and Mark 'bigfoot' Olsen
    for their help.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{ ****************************************************************************************************** }
{ ** TinyGL syscalls                                                                                  ** }
{ ****************************************************************************************************** }

{$IFDEF GL_UNIT}
procedure _GLEnable(gcl: pointer; cap: GLenum);
syscall sysvbase TinyGLBase 28;

procedure _GLDisable(gcl: pointer; cap: GLenum);
syscall sysvbase TinyGLBase 34;

procedure _GLShadeModel(gcl: pointer; mode: GLenum);
syscall sysvbase TinyGLBase 46;

procedure _GLCullFace(gcl: pointer; mode: GLenum);
syscall sysvbase TinyGLBase 52;

procedure _GLBegin(gcl: pointer; mode: GLenum);
syscall sysvbase TinyGLBase 64;

procedure _GLEnd(gcl: pointer);
syscall sysvbase TinyGLBase 70;

procedure _GLVertex3f(gcl: pointer; x, y, z: GLfloat);
syscall sysvbase TinyGLBase 76;

procedure _GlVertex3fv(gcl: pointer; const v: PGLfloat);
syscall sysvbase TinyGLBase 88;

procedure _GLColor3f(gcl: pointer; red, green, blue: GLfloat); 
syscall sysvbase TinyGLBase 100;

procedure _GLColor4f(gcl: pointer; red, green, blue, alpha: GLfloat); 
syscall sysvbase TinyGLBase 106;

procedure _GLNormal3f(gcl: pointer; nx, ny, nz: GLfloat);
syscall sysvbase TinyGLBase 148;

procedure _GLTexCoord2f(gcl: pointer; s, t: GLfloat); 
syscall sysvbase TinyGLBase 160;

procedure _GLMatrixMode(gcl: pointer; mode: GLenum);
syscall sysvbase TinyGLBase 190;

procedure _GLLoadIdentity(gcl: pointer);
syscall sysvbase TinyGLBase 202;

procedure _GLPushMatrix(gcl: pointer);
syscall sysvbase TinyGLBase 214;

procedure _GLPopMatrix(gcl: pointer);
syscall sysvbase TinyGLBase 220;

procedure _GLRotatef(gcl: pointer; angle, x, y, z: GLfloat); 
syscall sysvbase TinyGLBase 226;

procedure _GLTranslatef(gcl: pointer; x, y, z: GLfloat); 
syscall sysvbase TinyGLBase 232;

procedure _GLScalef(gcl: pointer; x, y, z: GLfloat); 
syscall sysvbase TinyGLBase 238;

procedure _GLViewPort(gcl: pointer; x, y: GLint; width, height: GLsizei);
syscall sysvbase TinyGLBase 244;

procedure _GLFrustum(gcl: pointer; left, right, bottom, top, zNear, zFar: GLdouble); 
syscall sysvbase TinyGLBase 250;
{$ENDIF GL_UNIT}

{$IF DEFINED(GL_UNIT) OR DEFINED(GLU_UNIT)}
procedure _GLOrtho(gcl: pointer; left, right, bottom, top, zNear, zFar: GLdouble);
syscall sysvbase TinyGLBase 256;
{$ENDIF}

{$IFDEF GL_UNIT}
function  _GLGenLists(gcl: pointer; range: GLsizei): GLuint;
syscall sysvbase TinyGLBase 262;

procedure _GLNewList(gcl: pointer; list: GLuint; mode: GLenum);
syscall sysvbase TinyGLBase 274;

procedure _GLEndList(gcl: pointer);
syscall sysvbase TinyGLBase 280;

procedure _GLCallList(gcl: pointer; list: GLuint);
syscall sysvbase TinyGLBase 286;

procedure _GLDeleteLists(gcl: pointer; list: GLuint; range: GLsizei);
syscall sysvbase TinyGLBase 292;

procedure _GLClear(gcl: pointer; mask: GLbitfield);
syscall sysvbase TinyGLBase 298;

procedure _GLClearColor(gcl: pointer; red, green, blue, alpha: GLclampf);
syscall sysvbase TinyGLBase 304;

procedure _GLClearDepth(gcl: pointer; depth: GLclampd);
syscall sysvbase TinyGLBase 310;

procedure _GLGenTextures(gcl: pointer; n: GLsizei; textures: PGLuint);
syscall sysvbase TinyGLBase 352;

procedure _GLDeleteTextures(gcl: pointer; n: GLsizei; const textures: PGLuint);
syscall sysvbase TinyGLBase 358;

procedure _GLBindTexture(gcl: pointer; target: GLenum; texture: GLuint);
syscall sysvbase TinyGLBase 364;

procedure _GLTexImage2D(gcl: pointer; target: GLenum; level, internalformat: GLint; width, height: GLsizei; border: GLint; format, atype: GLenum; const pixels: Pointer);
syscall sysvbase TinyGLBase 370;

procedure _GLTexParameteri(gcl: pointer; target: GLenum; pname: GLenum; param: GLint);
syscall sysvbase TinyGLBase 394;

procedure _GLTexParameterf(gcl: pointer; target: GLenum; pname: GLenum; param: GLfloat);
syscall sysvbase TinyGLBase 400;

procedure _GLTexGeni(gcl: pointer; coord: GLenum; pname: GLenum; param: GLint);
syscall sysvbase TinyGLBase 418;

procedure _GLMaterialfv(gcl: pointer; face, pname: GLenum; const params: PGLfloat);
syscall sysvbase TinyGLBase 430;

procedure _GLMaterialf(gcl: pointer; face, pname: GLenum; param: GLfloat);
syscall sysvbase TinyGLBase 436;

procedure _GLLightfv(gcl: pointer; light, pname: GLenum; const params: PGLfloat);
syscall sysvbase TinyGLBase 448;

procedure _GLLightf(gcl: pointer; light, pname: GLenum; param: GLfloat);
syscall sysvbase TinyGLBase 454;

procedure _GLLightModelfv(gcl: pointer; pname: GLenum; const params: PGLfloat);
syscall sysvbase TinyGLBase 466;

procedure _GLFlush(gcl: pointer);
syscall sysvbase TinyGLBase 478;

procedure _GLHint(gcl: pointer; target, mode: GLenum);
syscall sysvbase TinyGLBase 484;

procedure _GLGetFloatv(gcl: pointer; pname: GLenum; params: PGLfloat);
syscall sysvbase TinyGLBase 496;

procedure _GLEnableClientState(gcl: pointer; aarray: GLenum);
syscall sysvbase TinyGLBase 508;

procedure _GLDisableClientState(gcl: pointer; aarray: GLenum);
syscall sysvbase TinyGLBase 514;

procedure _GLVertexPointer(gcl: pointer; size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer);
syscall sysvbase TinyGLBase 526;

procedure _GLColorPointer(gcl: pointer; size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer);
syscall sysvbase TinyGLBase 532;

procedure _GLNormalPointer(gcl: pointer; atype: GLenum; stride: GLsizei; const pointer: Pointer);
syscall sysvbase TinyGLBase 538;

procedure _GLTexCoordPointer(gcl: pointer; size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer);
syscall sysvbase TinyGLBase 544;

procedure _GLDrawElements(gcl: pointer; mode: GLenum; count: GLsizei; atype: GLenum; const indices: Pointer);
syscall sysvbase TinyGLBase 562;

procedure _GLBlendFunc(gcl: pointer; sfactor, dfactor: GLenum);
syscall sysvbase TinyGLBase 586;

procedure _GLDepthMask(gcl: pointer; flag: GLboolean);
syscall sysvbase TinyGLBase 592;

procedure _GLAlphaFunc(gcl: pointer; func: GLenum; ref: GLclampf);
syscall sysvbase TinyGLBase 598;

procedure _GLPointSize(gcl: pointer; size: GLfloat);
syscall sysvbase TinyGLBase 604;

procedure _GLDepthFunc(gcl: pointer; func: GLenum);
syscall sysvbase TinyGLBase 628;

function _GLInit: Pointer; 
syscall sysvbase TinyGLBase 640;

procedure _GLClose(gcl: pointer);
syscall sysvbase TinyGLBase 646;
{$ENDIF GL_UNIT}

{$IFDEF GLU_UNIT}
procedure _GLUPerspective(gcl: pointer; fovy, aspect, zNear, zFar: GLdouble);
syscall sysvbase TinyGLBase 652;

procedure _GLULookAt(gcl: pointer; eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz: GLdouble);
syscall sysvbase TinyGLBase 658;
{$ENDIF GLU_UNIT}

{$IFDEF GLUT_UNIT}
procedure _GLUTInit(gcl: pointer; argcp: PLongInt; argv: PPChar);
syscall sysvbase TinyGLBase 664;

procedure _GLUTInitWindowPosition(gcl: pointer; x, y: Integer);
syscall sysvbase TinyGLBase 670;

procedure _GLUTInitWindowSize(gcl: pointer; width, height: LongInt);
syscall sysvbase TinyGLBase 676;

function  _GLUTCreateWindow(gcl: pointer; const title: PChar): LongInt;
syscall sysvbase TinyGLBase 682;

procedure _GLUTFullScreen(gcl: pointer);
syscall sysvbase TinyGLBase 688;

procedure _GLUTDestroyWindow(gcl: pointer; win: LongInt);
syscall sysvbase TinyGLBase 706;

procedure _GLUTSwapBuffers(gcl: pointer);
syscall sysvbase TinyGLBase 712;

procedure _GLUTMainLoop(gcl: pointer);
syscall sysvbase TinyGLBase 718;

procedure _GLUTPostRedisplay(gcl: pointer);
syscall sysvbase TinyGLBase 724;

procedure _GLUTDisplayFunc(gcl: pointer; f: TGlutVoidCallback);
syscall sysvbase TinyGLBase 730;

procedure _GLUTReshapeFunc(gcl: pointer; f: TGlut2IntCallback);
syscall sysvbase TinyGLBase 736;

procedure _GLUTIdleFunc(gcl: pointer; f: TGlutVoidCallback);
syscall sysvbase TinyGLBase 742; 

procedure _GLUTKeyboardFunc(gcl: pointer; f: TGlut1Char2IntCallback);
syscall sysvbase TinyGLBase 748;

procedure _GLUTMouseFunc(gcl: pointer; f: TGlut4IntCallback);
syscall sysvbase TinyGLBase 760;

procedure _GLUTMotionFunc(gcl: pointer; f: TGlut2IntCallback);
syscall sysvbase TinyGLBase 766;

function _GLUTGet(gcl: pointer; t: GLenum): LongInt;
syscall sysvbase TinyGLBase 802;

procedure _GLUTInitDisplayMode(gcl: pointer; mode: Word);
syscall sysvbase TinyGLBase 808;

procedure _GLUTSpecialFunc(gcl: pointer; f: TGlut3IntCallback);
syscall sysvbase TinyGLBase 814;

procedure _GLUTTimerFunc(gcl: pointer; millis: Word; f: TGlut1IntCallback; value: LongInt);
syscall sysvbase TinyGLBase 820;

procedure _GLUTPassiveMotionFunc(gcl: pointer; f: TGlut2IntCallback);
syscall sysvbase TinyGLBase 832;

// MorphOS specific GLUT call, returns Intuition window handle
function _GLUTGetWindowHandle(gcl: pointer): Pointer;
syscall sysvbase TinyGLBase 856;
{$ENDIF GLUT_UNIT}

{$IFDEF GL_UNIT}
function _GLGetString(gcl: pointer; name: GLenum): PChar;
syscall sysvbase TinyGLBase 862;

procedure _GLCallLists(gcl: pointer; n: GLsizei; atype: GLenum; const lists: Pointer);
syscall sysvbase TinyGLBase 892;

procedure _GLLineWidth(gcl: pointer; width: GLfloat);
syscall sysvbase TinyGLBase 910;

procedure _GLRasterPos3f(gcl: pointer; x, y, z: GLfloat);
syscall sysvbase TinyGLBase 1108;
{$ENDIF GL_UNIT}

{$IFDEF GLUT_UNIT}
procedure _GLUTVisibilityFunc(gcl: pointer; f: TGlut1IntCallback);
syscall sysvbase TinyGLBase 1114;
{$ENDIF GLUT_UNIT}

{$IFDEF GL_UNIT}
procedure _GLFogf(gcl: pointer; pname: GLenum; param: GLfloat);
syscall sysvbase TinyGLBase 1120;

procedure _GLFogfv(gcl: pointer; pname: GLenum; const params: PGLfloat); 
syscall sysvbase TinyGLBase 1126;

procedure _GLReadPixels(gcl: pointer; x, y: GLint; width, height: GLsizei; format, atype: GLenum; pixels: Pointer);
syscall sysvbase TinyGLBase 1180;

procedure _GLStencilFunc(gcl: pointer; func: GLenum; ref: GLint; mask: GLuint);
syscall sysvbase TinyGLBase 1210;

procedure _GLStencilOp(gcl: pointer; fail, zfail, zpass: GLenum);
syscall sysvbase TinyGLBase 1216;

procedure _GLColorMask(gcl: pointer; red, green, blue, alpha: GLboolean);
syscall sysvbase TinyGLBase 1282;
{$ENDIF GL_UNIT}


{ ****************************************************************************************************** }
{ ** GL functions                                                                                     ** }
{ ****************************************************************************************************** }

{$IFDEF GL_UNIT}
procedure glAlphaFunc(func: GLenum; ref: GLclampf); inline;
begin
  _GLAlphaFunc(tglContext, func, ref);
end;

procedure glBegin(mode: GLenum); inline;
begin
  _GLBegin(tglContext, mode);
end;

procedure glCallList(list: GLuint); inline;
begin
  _GLCallList(tglContext, list);
end;

procedure glCallLists(n: GLsizei; atype: GLenum; const lists: Pointer); inline;
begin
  _GLCallLists(tglContext, n, atype, lists);
end;

procedure glClear(mask: GLbitfield); inline;
begin
  _GLClear(tglContext, mask);
end;

procedure glClearColor(red, green, blue, alpha: GLclampf); inline;
begin
  _GLClearColor(tglContext, red, green, blue, alpha);
end;

procedure glClearDepth(depth: GLclampd); inline;
begin
  _GLClearDepth(tglContext, depth);
end;

procedure glColorMask(red, green, blue, alpha: GLboolean); inline;
begin
  _GLColorMask(tglContext, red, green, blue, alpha);
end;

procedure glColorPointer(size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;
begin
  _GLColorPointer(tglContext, size, atype, stride, pointer);
end;

procedure glCopyTexImage2D(target: GLenum; level: GLint; internalFormat: GLenum; x, y: GLint; width, height: GLsizei; border: GLint); inline;
begin
{$WARNING glCopyTexImage2D is dummy.}
  // DUMMY, FIX ME!!! 
  // TinyGL doesn't support this yet.
end;

procedure glCullFace(mode: GLenum); inline;
begin
  _GLCullFace(tglContext, mode);
end;

procedure glDeleteLists(list: GLuint; range: GLsizei); inline;
begin
  _GLDeleteLists(tglContext, list, range);
end;

procedure glEnd; inline;
begin
  _GLEnd(tglContext);
end;

procedure glEndList; inline;
begin
  _GLEndList(tglContext);
end;

procedure glFlush; inline;
begin
  _GLFlush(tglContext);
end;

function  glGenLists(range: GLsizei): GLuint; inline;
begin
  glGenLists:=_GLGenLists(tglContext, range);
end;

procedure glGetFloatv(pname: GLenum; params: PGLfloat); inline;
begin
  _GLGetFloatv(tglContext, pname, params);
end;

function glGetString(name: GLenum): PChar; inline;
begin
  glGetString:=_GLGetString(tglContext, name);
end;

procedure glHint(target, mode: GLenum); inline;
begin
  _GLHint(tglContext, target, mode);
end;

procedure glIndexi(c: GLint); inline;
begin
{$WARNING glIndexi is dummy.}
  // DUMMY, FIX ME!!! 
end;

procedure glLightModelfv(pname: GLenum; const params: PGLfloat); inline;
begin
  _GLLightModelfv(tglContext, pname, params);
end;

procedure glNewList(list: GLuint; mode: GLenum); inline;
begin
  _GLNewList(tglContext, list, mode);
end;

procedure glVertex2f(x, y: GLfloat); inline;
begin
  _GLVertex3f(tglContext, x, y, 0);
end;

procedure glVertex2i(x, y: GLint); inline;
begin
  _GLVertex3f(tglContext, x, y, 0);
end;

procedure glVertex3f(x, y, z: GLfloat); inline;
begin
  _GLVertex3f(tglContext, x, y ,z);
end; 

procedure glVertex3fv(const v: PGLfloat); inline;
begin
  _GLVertex3fv(tglContext, v);
end;

procedure glLightf(light, pname: GLenum; param: GLfloat); inline;
begin
  _GLLightf(tglContext, light, pname, param);
end;

procedure glLightfv(light, pname: GLenum; const params: PGLfloat); inline;
begin
  _GLLightfv(tglContext, light, pname, params);
end;

procedure glLineWidth(width: GLfloat); inline;
begin
  _GLLineWidth(tglContext, width);
end;

procedure glLoadIdentity; inline;
begin
  _GLLoadIdentity(tglContext);
end;

procedure glMaterialf(face, pname: GLenum; param: GLfloat); inline;
begin
  _GLMaterialf(tglContext, face, pname, param);
end;

procedure glMaterialfv(face, pname: GLenum; const params: PGLfloat); inline;
begin
  _GLMaterialfv(tglContext, face, pname, params);
end;

procedure glMateriali(face, pname: GLenum; param: GLint); inline;
begin
  // WTF?! 
  _GLMaterialf(tglContext, face, pname, param);
end;

procedure glMaterialiv(face, pname: GLenum; const params: PGLint); inline;
begin
  // WTF?!
  _GLMaterialfv(tglContext, face, pname, PGLfloat(params));
end;

procedure glMatrixMode(mode: GLenum); inline;
begin
  _GLMatrixMode(tglContext, mode);
end; 

procedure glNormalPointer(atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;
begin
  _GLNormalPointer(tglContext, atype, stride, pointer);
end;

procedure glPointSize(size: GLfloat); inline;
begin
  // 604! :)
  _GLPointSize(tglContext, size);
end;

procedure glRasterPos3f(x, y, z: GLfloat); inline;
begin
  _GLRasterPos3f(tglContext, x, y, z);
end;

procedure glReadPixels(x, y: GLint; width, height: GLsizei; format, atype: GLenum; pixels: Pointer); inline;
begin
  _GLReadPixels(tglContext, x, y, width, height, format, atype, pixels);
end;

procedure glShadeModel(mode: GLenum); inline;
begin
  _GLShadeModel(tglContext, mode);
end;

procedure glStencilFunc(func: GLenum; ref: GLint; mask: GLuint); inline;
begin
  _GLStencilFunc(tglContext, func, ref, mask);
end;

procedure glStencilOp(fail, zfail, zpass: GLenum); inline;
begin
  _GLStencilOp(tglContext, fail, zfail, zpass);
end;

procedure glTexCoordPointer(size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;
begin
  _GLTexCoordPointer(tglContext, size, atype, stride, pointer);
end;

procedure glTexParameterf(target: GLenum; pname: GLenum; param: GLfloat); inline;
begin
  _GLTexParameterf(tglContext, target, pname, param);
end;

procedure glFrustum(left, right, bottom, top, zNear, zFar: GLdouble); inline;
begin
  _GLFrustum(tglContext, left, right, bottom, top, zNear, zFar);
end; 

procedure glTranslatef(x, y, z: GLfloat); inline;
begin
  _GLTranslatef(tglContext, x, y, z);
end; 

procedure glScalef(x, y, z: GLfloat); inline;
begin
  _GLScalef(tglContext, x, y, z);
end; 

procedure glRotatef(angle, x, y, z: GLfloat); inline;
begin
  _GLRotatef(tglContext, angle, x, y, z);
end; 

procedure glColor3f(red, green, blue: GLfloat); inline;
begin
  _GLColor3f(tglContext, red, green, blue);
end; 

procedure glColor4f(red, green, blue, alpha: GLfloat); inline;
begin
  _GLColor4f(tglContext, red, green, blue, alpha);
end; 

procedure glNormal3f(nx, ny, nz: GLfloat); inline;
begin
  _GLNormal3f(tglContext, nx, ny, nz);
end;

procedure glDeleteTextures(n: GLsizei; const textures: PGLuint); inline;
begin
  _GLDeleteTextures(tglContext, n, textures);
end;

procedure glDisable(cap: GLenum); inline;
begin
  _GLDisable(tglContext, cap);
end;

procedure glDisableClientState(aarray: GLenum); inline;
begin
  _GLDisableClientState(tglContext, aarray);
end;

procedure glDrawElements(mode: GLenum; count: GLsizei; atype: GLenum; const indices: Pointer); inline;
begin
  _GLDrawElements(tglContext, mode, count, atype, indices);
end;

procedure glEnable(cap: GLenum); inline;
begin
  _GLEnable(tglContext, cap);
end;

procedure glEnableClientState(aarray: GLenum); inline;
begin
  _GLEnableClientState(tglContext, aarray);
end;

procedure glFogf(pname: GLenum; param: GLfloat); inline;
begin
  _GLFogf(tglContext, pname, param);
end;

procedure glFogfv(pname: GLenum; const params: PGLfloat); inline;
begin
  _GLFogfv(tglContext, pname, params);
end;

procedure glFogi(pname: GLenum; param: GLint); inline;
begin
  // WTF!? And it actually works!
  _GLFogf(tglContext, pname, param);
end;

procedure glOrtho(left, right, bottom, top, zNear, zFar: GLdouble); inline;
begin
  _GLOrtho(tglContext, left, right, bottom, top, zNear, zFar);
end;

procedure glPopMatrix; inline;
begin
  _GLPopMatrix(tglContext);
end;

procedure glPushMatrix; inline;
begin
  _GLPushMatrix(tglContext);
end;

procedure glTexImage2D(target: GLenum; level, internalformat: GLint; width, height: GLsizei; border: GLint; format, atype: GLenum; const pixels: Pointer); inline;
begin
  _GLTexImage2D(tglContext, target, level, internalformat, width, height, border, format, atype, pixels);
end;

procedure glTexParameteri(target: GLenum; pname: GLenum; param: GLint); inline;
begin
  _GLTexParameteri(tglContext, target, pname, param);
end;

procedure glTexCoord2f(s, t: GLfloat); inline;
begin
  _GLTexCoord2f(tglContext, s, t);
end;

procedure glTexGeni(coord: GLenum; pname: GLenum; param: GLint); inline;
begin
  _GLTexGeni(tglContext, coord, pname, param);
end;

procedure glBindTexture(target: GLenum; texture: GLuint); inline;
begin
  _GLBindTexture(tglContext, target, texture);
end;

procedure glGenTextures(n: GLsizei; textures: PGLuint); inline;
begin
  _GLGenTextures(tglContext, n, textures);
end;

procedure glBlendFunc(sfactor, dfactor: GLenum); inline;
begin
  _GLBlendFunc(tglContext, sfactor, dfactor);
end;

procedure glDepthFunc(func: GLenum); inline;
begin
  _GLDepthFunc(tglContext, func);
end;

procedure glDepthMask(flag: GLboolean); inline;
begin
  _GLDepthMask(tglContext, flag);
end;

procedure glVertexPointer(size: GLint; atype: GLenum; stride: GLsizei; const pointer: Pointer); inline;
begin
  _GLVertexPointer(tglContext, size, atype, stride, pointer);
end;

procedure glViewport(x, y: GLint; width, height: GLsizei); inline;
begin
  _GLViewport(tglContext, x, x, width, height);
end;
{$ENDIF GL_UNIT}


{ ****************************************************************************************************** }
{ ** GLU functions                                                                                    ** }
{ ****************************************************************************************************** }

{$IFDEF GLU_UNIT}
procedure gluLookAt(eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz: GLdouble); inline;
begin
  _GLULookAt(tglContext, eyex, eyey, eyez, centerx, centery, centerz, upx, upy, upz);
end;

procedure gluOrtho2D(left,right, bottom, top: GLdouble); inline;
begin
  _GLOrtho(tglContext, left, right, bottom, top, -1.0, 1.0)
end;

procedure gluPerspective(fovy, aspect, zNear, zFar: GLdouble); inline;
begin
  _GLUPerspective(tglContext, fovy, aspect, zNear, zFar);
end;
{$ENDIF GLU_UNIT}


{ ****************************************************************************************************** }
{ ** GLUT functions                                                                                   ** }
{ ****************************************************************************************************** }

{$IFDEF GLUT_UNIT}
procedure GLUTInit(argcp: PLongInt; argv: PPChar); inline;
begin
  _GLUTInit(tglContext,argcp,argv);
end;

procedure GLUTInitDisplayMode(mode: Word); inline;
begin
  _GLUTInitDisplayMode(tglContext,mode);
end;

procedure GLUTInitWindowPosition(x, y: Integer); inline;
begin
 _GLUTInitWindowPosition(tglContext, x, y);
end;

procedure GLUTInitWindowSize(width, height: LongInt); inline;
begin
  _GLUTInitWindowSize(tglContext, width, height);
end;

procedure GLUTSwapBuffers; inline;
begin
  _GLUTSwapBuffers(tglContext);
end;

procedure GLUTPostRedisplay; inline;
begin
  _GLUTPostRedisplay(tglContext);
end;

procedure GLUTReshapeFunc(f: TGlut2IntCallback); inline;
begin
  _GLUTReshapeFunc(tglContext, f);
end;

function GLUTCreateWindow(const title: PChar): LongInt; inline;
begin
  GLUTCreateWindow:=_GLUTCreateWindow(tglContext, title);
end;

procedure GLUTDestroyWindow(win: LongInt); inline;
begin
  _GLUTDestroyWindow(tglContext, win);
end;

procedure GLUTDisplayFunc(f: TGlutVoidCallback); inline;
begin
  _GLUTDisplayFunc(tglContext, f);
end;

procedure GLUTIdleFunc(f: TGlutVoidCallback); inline;
begin
  _GLUTIdleFunc(tglContext, f);
end;

procedure GLUTTimerFunc(millis: Word; f: TGlut1IntCallback; value: LongInt); inline;
begin
  _GLUTTimerFunc(tglContext, millis, f, value);
end;

procedure GLUTKeyboardFunc(f: TGlut1Char2IntCallback); inline;
begin
  _GLUTKeyboardFunc(tglContext, f);
end;

procedure GLUTMouseFunc(f: TGlut4IntCallback); inline;
begin
  _GLUTMouseFunc(tglContext, f);
end;

procedure GLUTSpecialFunc(f: TGlut3IntCallback); inline;
begin
  _GLUTSpecialFunc(tglContext, f);
end;

procedure GLUTMotionFunc(f: TGlut2IntCallback); inline;
begin
  _GLUTMotionFunc(tglContext, f);
end;

procedure GLUTPassiveMotionFunc(f: TGlut2IntCallback); inline;
begin
  _GLUTPassiveMotionFunc(tglContext, f);
end;

function GLUTGetWindowHandle: pointer; inline;
begin
  GLUTGetWindowHandle:=_GLUTGetWindowHandle(tglContext);
end;

procedure GLUTVisibilityFunc(f: TGlut1IntCallback); inline;
begin
  _GLUTVisibilityFunc(tglContext, f);
end;

function  GLUTGet(t: GLenum): LongInt; inline;
begin
  GLUTGet:=_GLUTGet(tglContext,t);
end;

procedure GLUTMainLoop; inline;
begin
  _GLUTMainLoop(tglContext);
end;

procedure GLUTFullScreen; inline;
begin
  _GLUTFullScreen(tglContext);
end;

procedure GLUTSetColor(cell: Integer; red, green, blue: GLfloat); inline;
begin
{$WARNING GLUTSetColor is dummy.}
  // DUMMY, FIX ME!!! 
end;
{$ENDIF GLUT_UNIT}
