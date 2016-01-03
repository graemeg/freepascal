{
  Example 17. Panel Hiding and Showing example
  from ncurses howto
}

program test_panel;

{$MODE OBJFPC}

uses
  ncurses, panel, sysutils;


Type
  PANEL_DATA = record
    hide: Boolean;       (* TRUE if panel is hidden *)
  end;
  PPWINDOW = ^PWINDOW;

const
  NLINES = 10;
  NCOLS  = 40;

procedure print_in_middle(win: PWINDOW; starty, startx, width: Longint; str: AnsiString; color: chtype);
var
  slength, x, y: Longint;
  temp: Double;
begin

  if win = nil then
    win := stdscr;
  getyx(win, y, x);
  if startx <> 0 then
    x := startx;
  if starty <> 0 then
    y := starty;
  if width = 0 then
    width := 80;

  slength := Length(str);
  temp := (width - slength)/ 2;
  x := startx + round(temp);
  wattron(win, color);
  mvwaddstr(win, y, x, PChar(str));
  wattroff(win, color);
  refresh();

end;

(* Show the window with a border and a label *)
procedure win_show(win: PWINDOW; lab: AnsiString; label_color: Longint);
var
  startx, starty, height, width: Smallint;
begin
        getbegyx(win, starty, startx);
        getmaxyx(win, height, width);

        box(win, 0, 0);
        mvwaddch(win, 2, 0, ACS_LTEE); 
        mvwhline(win, 2, 1, ACS_HLINE, width - 2);
        mvwaddch(win, 2, width - 1, ACS_RTEE);

        print_in_middle(win, 1, 0, width, lab, COLOR_PAIR(label_color));
end;

(* Put all the windows *)
procedure init_wins(wins: PPWINDOW; n: Longint);
var
  x, y, i: Longint;
  lab: AnsiString;
begin
  y := 2;
  x := 10;
  for i := 0 to n - 1 do
  begin
    wins[i] := newwin(NLINES, NCOLS, y, x);
    FmtStr(lab, 'Window Number %d', [i + 1]);
    win_show(wins[i], lab, i + 1);
<<<<<<< HEAD
<<<<<<< HEAD
    Inc(y,3);
    Inc(x,7);
=======
    y += 3;
    x += 7;
>>>>>>> graemeg/fixes_2_2
=======
    y += 3;
    x += 7;
>>>>>>> origin/fixes_2_2
  end
end;

var
  my_wins:   array[0..2] of PWINDOW;
  my_panels: array[0..2] of PPANEL;
  panel_datas:  array[0..2] of PANEL_DATA;
  temp:      ^PANEL_DATA;
  ch:  chtype;
begin
  try

(* Initialize curses *)
  initscr();
  start_color();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);

(* Initialize all the colors *)
  init_pair(1, COLOR_RED, COLOR_BLACK);
  init_pair(2, COLOR_GREEN, COLOR_BLACK);
  init_pair(3, COLOR_BLUE, COLOR_BLACK);
  init_pair(4, COLOR_CYAN, COLOR_BLACK);

  init_wins(my_wins, 3);

(* Attach a panel to each window *)        (* Order is bottom up *)
  my_panels[0] := new_panel(my_wins[0]);   (* Push 0, order: stdscr-0 *)
  my_panels[1] := new_panel(my_wins[1]);   (* Push 1, order: stdscr-0-1 *)
  my_panels[2] := new_panel(my_wins[2]);   (* Push 2, order: stdscr-0-1-2 *)

(* Initialize panel datas saying that nothing is hidden *)
  panel_datas[0].hide := FALSE;
  panel_datas[1].hide := FALSE;
  panel_datas[2].hide := FALSE;

  set_panel_userptr(my_panels[0], @panel_datas[0]);
  set_panel_userptr(my_panels[1], @panel_datas[1]);
  set_panel_userptr(my_panels[2], @panel_datas[2]);

(* Update the stacking order. 2nd panel will be on top *)
  update_panels();

(* Show it on the screen *)
  attron(COLOR_PAIR(4));
  mvprintw(LINES - 3, 0, 'Show or Hide a window with "a"(first window)  "b"(Second Window)  "c"(Third Window)');
  mvprintw(LINES - 2, 0, 'F1 to Exit');

  attroff(COLOR_PAIR(4));
  doupdate();


  ch := getch;
  while ch <> KEY_F(1) do
  begin
    case ch of
      chtype('a'):
      begin
        temp := panel_userptr(my_panels[0]);
        if temp^.hide = FALSE then
        begin
          hide_panel(my_panels[0]);
          temp^.hide := TRUE;
        end
        else
        begin
          show_panel(my_panels[0]);
          temp^.hide := FALSE;
        end
      end;
      chtype('b'):
      begin
        temp := panel_userptr(my_panels[1]);
        if temp^.hide = FALSE then
        begin
          hide_panel(my_panels[1]);
          temp^.hide := TRUE;
        end
        else
        begin
          show_panel(my_panels[1]);
          temp^.hide := FALSE;
        end
      end;
      chtype('c'):
      begin
        temp := panel_userptr(my_panels[2]);
        if temp^.hide = FALSE then
        begin
          hide_panel(my_panels[2]);
          temp^.hide := TRUE;
        end
        else
        begin
          show_panel(my_panels[2]);
          temp^.hide := FALSE;
        end
      end
      else
      end;
    update_panels();
    doupdate();
    ch := getch;
  end;

  finally
    endwin();
  end;
end.