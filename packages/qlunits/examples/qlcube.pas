{
    Copyright (c) 2017-2020 Karoly Balogh

    Rotating 3D cube on a Sinclair QL
    Example program for Free Pascal's Sinclair QL support

    This example program is in the Public Domain under the terms of
    Unlicense: http://unlicense.org/

 **********************************************************************}

program qlcube;

uses
  qdos, qlfloat;

type
  tvertex = record
    x: longint;
    y: longint;
    z: longint;
  end;

const
  cube: array[0..7] of tvertex = (
     ( x: -1; y: -1; z: -1; ), // 0
     ( x:  1; y: -1; z: -1; ), // 1
     ( x:  1; y:  1; z: -1; ), // 2
     ( x: -1; y:  1; z: -1; ), // 3

     ( x: -1; y: -1; z:  1; ), // 4
     ( x:  1; y: -1; z:  1; ), // 5
     ( x:  1; y:  1; z:  1; ), // 6
     ( x: -1; y:  1; z:  1; )  // 7
  );

type
  tface = record
    v1, v2, v3: longint;
    edge: longint;
  end;

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


var
  mx, my: smallint;

function min(a, b: smallint): smallint;
begin
  if a < b then
    min:=a
  else
    min:=b;
end;

procedure draw_line(x1,y1,x2,y2: smallint);
begin
  sd_line(stdOutputHandle,-1,x1,y1,x2,y2);
end;

procedure cube_redraw;
var
  i,s,e,cx,cy,vx,vy: longint;
  vr: tvertex;
  scale: longint;
  rect:TQLRect;
  fcubex: array[low(cube)..high(cube)] of Tqlfloat;
  fcubey: array[low(cube)..high(cube)] of Tqlfloat;
begin
  rect.q_x:=0;
  rect.q_y:=0;
  rect.q_width:=140;
  rect.q_height:=100;

  scale:=(min(rect.q_width,rect.q_height) div 6) shl 16;
  cx:=rect.q_x + rect.q_width div 2;
  cy:=rect.q_y + rect.q_height div 2;
  for i:=low(cube) to high(cube) do
    begin
      rotate_vertex(cube[i],vr,-my,-mx,0);
      perspective_vertex(vr,3 shl 16,vx,vy);
      longint_to_qlfp(@fcubex[i],cx + sarlongint(mulfp(vx,scale),16));
      longint_to_qlfp(@fcubey[i],cy + sarlongint(mulfp(vy,scale),16));
    end;

  sd_clear(stdOutputHandle,-1);
  for i:=0 to 3 do 
    begin
      e:=(i+1) and 3;
      sd_line(stdOutputHandle,-1,@fcubex[i],@fcubey[i],@fcubex[e],@fcubey[e]);
      s:=i+4; e:=e+4;
      sd_line(stdOutputHandle,-1,@fcubex[s],@fcubey[s],@fcubex[e],@fcubey[e]);
      sd_line(stdOutputHandle,-1,@fcubex[i],@fcubey[i],@fcubex[s],@fcubey[s]);
    end;
end;

procedure main_loop;
begin
  repeat
    inc(mx,5);
    inc(my,7);
    cube_redraw;
  until false;
end;

begin
  init_cube;

  main_loop;
end.
