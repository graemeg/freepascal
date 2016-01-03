program tclock;
{$MODE OBJFPC}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  ncurses, sysutils, initc;

procedure setlocale(cat : integer; p : pchar); cdecl; external clib;


const
  LC_ALL = 6;
=======
  libc, ncurses, sysutils;
>>>>>>> graemeg/fixes_2_2
=======
  libc, ncurses, sysutils;
>>>>>>> origin/fixes_2_2

const
  ASPECT = 2.2;
  _2PI = 2.0 * PI;

function sign(_x: Integer): Integer;
begin
  if _x < 0 then
    sign := -1
  else
    sign := 1
end;

function A2X(angle,radius: Double): Integer; inline;
begin
  A2X := round(ASPECT * radius * sin(angle))
end;

function A2Y(angle,radius: Double): Integer; inline;
begin
  A2Y := round(radius * cos(angle))
end;

type
  PRchar = ^TRchar;
  TRchar = record
    ry,rx: Smallint;
    rch: chtype;
  end;

procedure restore( rest: PRchar );
var
  i: Longint = 0;
begin
  while rest[i].rch <> 0 do
  begin
    with rest[i] do
      mvaddch(ry, rx, rch);
    Inc(i);
  end;
  freemem(rest)
end;

(* Draw a diagonal(arbitrary) line using Bresenham's alogrithm. *)
procedure dline(from_y, from_x, end_y,  end_x: Smallint; ch: chtype; var rest: PRchar);
var
  dx, dy: Smallint;
  ax, ay: Smallint;
  sx, sy: Smallint;
  x, y, d, i: Smallint;
begin
  dx := end_x - from_x;
  dy := end_y - from_y;

  ax := abs(dx * 2);
  ay := abs(dy * 2);

  sx := sign(dx);
  sy := sign(dy);

  x := from_x;
  y := from_y;

  i := 0;
  if (ax > ay) then
  begin
    getmem(rest, sizeof(TRchar)*(abs(dx)+3));
    d := ay - (ax DIV 2);

    while true do
    begin
      move(y, x);
      with rest[i] do
      begin
        rch := inch;
        ry := y;
        rx := x;
        Inc(i)
      end;
      addch(ch);
      if (x = end_x) then
      begin
        rest[i].rch := 0;
        exit;
      end;

      if (d >= 0) then
      begin
        y += sy;
        d -= ax;
      end;
      x += sx;
      d += ay;
    end
  end
  else
  begin
    getmem(rest, sizeof(TRchar)*(abs(dy)+3));
    d := ax - (ay DIV 2);

    while true do
    begin
      move(y, x);
      with rest[i] do
      begin
        rch := inch;
        ry := y;
        rx := x;
        Inc(i)
      end;
      addch(ch);
      if (y = end_y) then
      begin
        rest[i].rch := 0;
        exit;
      end;

      if (d >= 0) then
      begin
        x += sx;
        d -= ay;
      end;
      y += sy;
      d += ax;
    end
  end
end;


var
  cx, cy: Integer;
  cr, sradius, mradius, hradius: Double;


procedure clockinit;
const
  title1 = 'Free pascal';
  title2 = 'ncurses clock';
  title3 = 'Press F10 or q to exit';
var
  i: Integer;
  vstr, tstr: AnsiString;
  angle: Double;
begin
  cx := (COLS - 1) DIV 2;
  cy := LINES DIV 2;
  if (cx / ASPECT < cy) then
  cr := cx / ASPECT
    else
  cr := cy;

  sradius := (8 * cr) / 9;
  mradius := (3 * cr) / 4;
  hradius := cr / 2;


  for i := 1 to 24 do
  begin
    angle := i * _2PI / 24.0;


    if (i MOD 2) = 0 then
    begin
      Str (i DIV 2, tstr);
      attron(A_BOLD OR COLOR_PAIR(5));
      mvaddstr(cy - A2Y(angle, sradius), cx + A2X(angle, sradius), @tstr[1]);
      attroff(A_BOLD OR COLOR_PAIR(5));
    end
    else
    begin
      attron(COLOR_PAIR(1));
      mvaddch(cy - A2Y(angle, sradius), cx + A2X(angle, sradius), chtype('.'));
      attroff(COLOR_PAIR(1));
    end
  end;

  vstr := curses_version;

  attron(A_DIM OR COLOR_PAIR(2));
  mvhline(cy , cx - round(sradius * ASPECT) + 1, ACS_HLINE,  round(sradius * ASPECT) * 2 - 1);
  mvvline(cy - round(sradius) + 1, cx , ACS_VLINE,  round(sradius) * 2 - 1);
  attroff(A_DIM OR COLOR_PAIR(1));
  attron(COLOR_PAIR(3));
  mvaddstr(cy - 5, cx - Length(title1) DIV 2, title1);
  mvaddstr(cy - 4, cx - Length(title2) DIV 2, title2);
  mvaddstr(cy - 3, cx - Length(vstr) DIV 2, PChar(vstr));
  attroff(COLOR_PAIR(3));
  attron(A_UNDERLINE);
  mvaddstr(cy + 2, cx - Length(title3) DIV 2, title3);
  attroff(A_UNDERLINE);
end;


var
  angle: Double;
  ch: chtype = 0;
  Hour, Min, Sec, Msec: Word;
  Hrest, Mrest, Srest: PRchar;
  timestr: AnsiString;
  my_bg: Smallint = COLOR_BLACK;
begin
  setlocale(LC_ALL, '');

  try
    initscr();
    noecho();
    cbreak();

    halfdelay(10);
    keypad(stdscr, TRUE);
    curs_set(0);

    if (has_colors()) then
    begin
      start_color();
      if (use_default_colors() = OK) then
        my_bg := -1;

      init_pair(1, COLOR_YELLOW, my_bg);
      init_pair(2, COLOR_RED, my_bg);
      init_pair(3, COLOR_GREEN, my_bg);
      init_pair(4, COLOR_CYAN, my_bg);
      init_pair(5, COLOR_YELLOW, COLOR_BLACK) ;
    end;

    clockinit;
    repeat
      if (ch = KEY_RESIZE) then
      begin
        flash();
        erase();
        wrefresh(curscr);
        clockinit;
      end;

      decodeTime(Time, Hour, Min, Sec, Msec);
      Hour := Hour MOD 12;

      timestr := DateTimeToStr(Now);
      mvaddstr(cy + round(sradius) - 4, cx - Length(timestr) DIV 2, PChar(timestr));

      angle := Hour * _2PI / 12;
      dline(cy, cx, cy - A2Y(angle, hradius), cx + A2X(angle, hradius), chtype('*'),Hrest);

      angle := Min * _2PI / 60;
      dline(cy, cx, cy - A2Y(angle, mradius), cx + A2X(angle, mradius), chtype('*'),Mrest);

      angle := Sec * _2PI / 60;
      dline(cy, cx, cy - A2Y(angle, sradius), cx + A2X(angle, sradius), chtype('.'),Srest);

      ch := getch();

      restore(Srest);
      restore(Mrest);
      restore(Hrest);

    until (ch = chtype('q')) OR (ch = KEY_F(10));
  finally
    curs_set(1);
    endwin();
  end;
end.
