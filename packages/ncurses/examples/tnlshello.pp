{
  rstconv -i tnlshello.rst -o tnlshello_ru_UTF8.pot
  msgfmt tnlshello_ru_UTF8.pot
  mv messages.mo ru
}

program nlshello;
{$mode objfpc}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  gettext, ncurses, initc;

procedure setlocale(cat : integer; p : pchar); cdecl; external clib;


const
  LC_ALL = 6;

=======
  gettext, libc, ncurses;
>>>>>>> graemeg/fixes_2_2
=======
  gettext, libc, ncurses;
>>>>>>> origin/fixes_2_2

resourcestring
  hello_world = 'Hello world!';
  press_key = 'Press any key to continue!';


var
  win : pWINDOW;
begin
  setlocale(LC_ALL, '');

  try
    initscr();
    start_color;
    noecho;
    win:= newwin ( 10, COLS - 20, 5, 10);

    init_pair(1,COLOR_WHITE,COLOR_BLUE);
    init_pair(2,COLOR_RED,COLOR_BLUE);
    wbkgd(win, COLOR_PAIR(1));
    erase;
    refresh;

    box(win, ACS_VLINE, ACS_HLINE);
    wrefresh(win);
    mvwaddstr(win,1,3, curses_version);

    TranslateResourcestrings('%s/messages.mo');
    wattron(win,A_BLINK OR A_BOLD OR COLOR_PAIR(2));
    mvwaddstr(win,3,3, PChar(hello_world));
    wattroff(win,A_BLINK OR A_BOLD OR COLOR_PAIR(2));
    mvwaddstr(win,5,3, PChar(press_key));
    wrefresh(win);
    getch();
  finally
    endwin();
  end;
end.