{
    Copyright (c) 2020 Karoly Balogh

    Rotating 3D cube in a Workbench window
    Example program for Free Pascal's Amiga bindings
    on legacy systems (OS1.x)

    This example program is in the Public Domain under the terms of
    Unlicense: http://unlicense.org/

 **********************************************************************}
{$MEMORY 32768,4096}
program amicube;

uses
  exec, intuition, agraphics;

type
  tvertex = record
    x: longint;
    y: longint;
    z: longint;
    pad: longint;
  end;

const
  cube: array[0..7] of tvertex = (
     ( x: -1; y: -1; z: -1; pad: 0), // 0
     ( x:  1; y: -1; z: -1; pad: 0), // 1
     ( x:  1; y:  1; z: -1; pad: 0), // 2
     ( x: -1; y:  1; z: -1; pad: 0), // 3

     ( x: -1; y: -1; z:  1; pad: 0), // 4
     ( x:  1; y: -1; z:  1; pad: 0), // 5
     ( x:  1; y:  1; z:  1; pad: 0), // 6
     ( x: -1; y:  1; z:  1; pad: 0)  // 7
  );

type
  tface = record
    v1, v2, v3: longint;
    edge: longint;
  end;

const
  faces: array[0..11] of tface = (
    ( v1: 0; v2: 2; v3: 1; edge: 6),  // front
    ( v1: 2; v2: 0; v3: 3; edge: 6),

    ( v1: 0; v2: 1; v3: 4; edge: 5),  // top
    ( v1: 1; v2: 5; v3: 4; edge: 3),

    ( v1: 3; v2: 0; v3: 7; edge: 5),  // left
    ( v1: 0; v2: 4; v3: 7; edge: 3),

    ( v1: 1; v2: 2; v3: 5; edge: 5),  // right
    ( v1: 1; v2: 6; v3: 5; edge: 6),

    ( v1: 2; v2: 3; v3: 6; edge: 5),  // bottom
    ( v1: 3; v2: 7; v3: 6; edge: 3),

    ( v1: 4; v2: 5; v3: 6; edge: 3),  // back
    ( v1: 6; v2: 7; v3: 4; edge: 3)
  );

const
  sincos_table: array[0..255] of longint = (
         0,  1608,  3216,  4821,  6424,  8022,  9616, 11204,
     12785, 14359, 15924, 17479, 19024, 20557, 22078, 23586,
     25079, 26557, 28020, 29465, 30893, 32302, 33692, 35061,
     36409, 37736, 39039, 40319, 41575, 42806, 44011, 45189,
     46340, 47464, 48558, 49624, 50659, 51664, 52638, 53580,
     54490, 55367, 56211, 57021, 57797, 58537, 59243, 59913,
     60546, 61144, 61704, 62227, 62713, 63161, 63571, 63943,
     64276, 64570, 64826, 65042, 65219, 65357, 65456, 65515,
     65535, 65515, 65456, 65357, 65219, 65042, 64826, 64570,
     64276, 63943, 63571, 63161, 62713, 62227, 61704, 61144,
     60546, 59913, 59243, 58537, 57797, 57021, 56211, 55367,
     54490, 53580, 52638, 51664, 50659, 49624, 48558, 47464,
     46340, 45189, 44011, 42806, 41575, 40319, 39039, 37736,
     36409, 35061, 33692, 32302, 30893, 29465, 28020, 26557,
     25079, 23586, 22078, 20557, 19024, 17479, 15924, 14359,
     12785, 11204,  9616,  8022,  6424,  4821,  3216,  1608,
         0, -1608, -3216, -4821, -6424, -8022, -9616,-11204,
    -12785,-14359,-15924,-17479,-19024,-20557,-22078,-23586,
    -25079,-26557,-28020,-29465,-30893,-32302,-33692,-35061,
    -36409,-37736,-39039,-40319,-41575,-42806,-44011,-45189,
    -46340,-47464,-48558,-49624,-50659,-51664,-52638,-53580,
    -54490,-55367,-56211,-57021,-57797,-58537,-59243,-59913,
    -60546,-61144,-61704,-62227,-62713,-63161,-63571,-63943,
    -64276,-64570,-64826,-65042,-65219,-65357,-65456,-65515,
    -65535,-65515,-65456,-65357,-65219,-65042,-64826,-64570,
    -64276,-63943,-63571,-63161,-62713,-62227,-61704,-61144,
    -60546,-59913,-59243,-58537,-57797,-57021,-56211,-55367,
    -54490,-53580,-52638,-51664,-50659,-49624,-48558,-47464,
    -46340,-45189,-44011,-42806,-41575,-40319,-39039,-37736,
    -36409,-35061,-33692,-32302,-30893,-29465,-28020,-26557,
    -25079,-23586,-22078,-20557,-19024,-17479,-15924,-14359,
    -12785,-11204, -9616, -8022, -6424, -4821, -3216, -1608
  );

function sin(x: longint): longint; inline;
begin
  sin:=sincos_table[x and 255];
end;

function cos(x: longint): longint; inline;
begin
  cos:=sincos_table[(x + 64) and 255];
end;

function mulfp(a, b: longint): longint; inline;
begin
  mulfp:=sarint64((int64(a) * b),16);
end;

function divfp(a, b: longint): longint;
begin
  divfp:=(int64(a) shl 16) div b;
end;

procedure rotate_vertex(const v: tvertex; var vr: tvertex; xa, ya, za: longint);
var
  x,y,z: longint;
  s,c: longint;
begin
  s   :=sin(ya);
  c   :=cos(ya);
  x   :=mulfp(c,v.x) - mulfp(s,v.z);
  z   :=mulfp(s,v.x) + mulfp(c,v.z);
  if za <> 0 then
    begin
      vr.x:=mulfp(cos(za),x)   + mulfp(sin(za),v.y);
      y   :=mulfp(cos(za),v.y) - mulfp(sin(za),x);
    end
  else
    begin
      vr.x:=x;
      y:=v.y;
    end;
  vr.z:=mulfp(cos(xa),z)   - mulfp(sin(xa),y);
  vr.y:=mulfp(sin(xa),z)   + mulfp(cos(xa),y);
end;

procedure perspective_vertex(const v: tvertex; zc: longint; var xr,yr: longint);
var
  rzc: longint;
begin
  rzc:=divfp(1 shl 16,(v.z - zc));
  xr:=mulfp(mulfp(v.x,zc),rzc);
  yr:=mulfp(mulfp(v.y,zc),rzc);
end;

procedure init_cube;
var
  i: longint;
begin
  for i:=low(cube) to high(cube) do
    begin
      cube[i].x:=cube[i].x shl 16;
      cube[i].y:=cube[i].y shl 16;
      cube[i].z:=cube[i].z shl 16;
    end;
end;

const
  win_info: array[0..63] of char = '';

var
  win: PWindow;

const
  IDCMPS = IDCMP_CLOSEWINDOW or IDCMP_NEWSIZE or IDCMP_INTUITICKS;
  WFLGS = WFLG_DRAGBAR or WFLG_DEPTHGADGET or WFLG_CLOSEGADGET or WFLG_SIZEGADGET or WFLG_ACTIVATE or WFLG_NOCAREREFRESH;
  WINTITLE = 'FPC Amiga Cube';

const
  winlayout: TNewWindow = (
    LeftEdge: 20;
    TopEdge: 20;
    Width: 240;
    Height: 150;
    DetailPen: 0;
    BlockPen: 1;
    IDCMPFlags: IDCMPS;
    Flags: WFLGS;
    FirstGadget: nil;
    CheckMark: nil;
    Title: WINTITLE;
    Screen: nil;
    BitMap: nil;
    MinWidth: 0;
    MinHeight: 0;
    MaxWidth: 320;
    MaxHeight: 200;
    WType: WBENCHSCREEN_F;
  );

function open_win: PWindow;
var
  newwin: TNewWindow;
begin
  newwin:=winlayout;
  open_win:=OpenWindow(@newwin);
end;

function min(a, b: smallint): smallint;
begin
  if a < b then
    min:=a
  else
    min:=b;
end;

procedure win_redraw(mx, my: longint);
var
  sx,sy: string[16];
  i,cx,cy,vx,vy: longint;
  rcube: array[low(cube)..high(cube)] of tvertex;
  vr: tvertex;
  scale: longint;
  wx,wy,ww,wh: longint;
begin
  wx:=win^.borderleft;
  ww:=win^.width-(win^.borderleft+win^.borderright);
  wy:=win^.bordertop;
  wh:=win^.height-(win^.bordertop+win^.borderbottom);

  scale:=(min(wh,ww) div 4) shl 16;
  cx:=wx + ww div 2;
  cy:=wy + wh div 2;
  for i:=low(cube) to high(cube) do
    begin
      rotate_vertex(cube[i],vr,-my,-mx,0);
      perspective_vertex(vr,3 shl 16,vx,vy);
      rcube[i].x:=cx + sarlongint(mulfp(vx,scale),16);
      rcube[i].y:=cy + sarlongint(mulfp(vy,scale div 2),16);
      // the div 2 part above is a hack, to make the cube look
      // less distorted on a 640x256 screen...
    end;

  str(mx,sx);
  str(my,sy);
  win_info:='Spinning... X:'+sx+' Y:'+sy;

  SetAPen(win^.rport,0);
  RectFill(win^.rport,wx,wy,wx+ww,wy+wh);
  SetAPen(win^.rport,1);
  gfxMove(win^.rport,wx+5,wy+10);

  gfxText(win^.rport, win_info, strlen(win_info));

  for i:=low(faces) to high(faces) do
    begin
      with faces[i] do
        begin
          if (edge and 1) > 0 then
            begin
              gfxMove(win^.rport,rcube[v1].x,rcube[v1].y);
              draw(win^.rport,rcube[v2].x,rcube[v2].y);
            end;
          if (edge and 2) > 0 then
            begin
              gfxMove(win^.rport,rcube[v2].x,rcube[v2].y);
              draw(win^.rport,rcube[v3].x,rcube[v3].y);
            end;
          if (edge and 4) > 0 then
            begin
              gfxMove(win^.rport,rcube[v3].x,rcube[v3].y);
              draw(win^.rport,rcube[v1].x,rcube[v1].y);
            end;
        end;
    end;
end;

procedure event_loop;
var
  quit: boolean;
  IMsg: PIntuiMessage;

  //ICode: Word;
  //IQual: Word;
  IClass: LongWord;
  MouseX: LongInt;
  MouseY: LongInt;
  OldMouseX: LongInt;
  OldMouseY: LongInt;
begin
  quit:=false;
  OldMouseX:=-1;
  OldMouseY:=-1;

  repeat
    IMsg:=PIntuiMessage(WaitPort(win^.UserPort));
    IMsg:=PIntuiMessage(GetMsg(win^.UserPort));
    while IMsg <> nil do
      begin
        //ICode:=IMsg^.Code;
        //IQual:=IMsg^.Qualifier;
        IClass:=IMsg^.iClass;
        MouseX:=IMsg^.MouseX;
        MouseY:=IMsg^.MouseY;
        ReplyMsg(PMessage(IMsg));

        case IClass of
            IDCMP_NEWSIZE:
                begin
                  win_redraw(OldMouseX,OldMouseY);
                end;
            IDCMP_CLOSEWINDOW:
                begin
                  quit:=true;
                end;
            IDCMP_INTUITICKS:
                begin
                  if (MouseX <> OldMouseX) or (MouseY <> OldMouseY) then
                    begin
                      OldMouseX:=MouseX;
                      OldMouseY:=MouseY;
                      win_redraw(OldMouseX,OldMouseY);
                    end;
                end;
        end;

        IMsg:=PIntuiMessage(GetMsg(win^.UserPort));
      end;
  until quit;
end;

begin
  init_cube;

  win:=open_win;
  if win <> nil then
    begin
      event_loop;
      CloseWindow(win);
    end;
end.
