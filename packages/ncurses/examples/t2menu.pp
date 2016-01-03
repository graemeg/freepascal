{$MODE OBJFPC}
program Menu_Example;


uses
  ncurses, menu, panel, sysutils;


function st_middle(scrlen, itemlen: Smallint): Smallint; inline;
begin
  st_middle := (scrlen - itemlen) div 2;
end;

procedure print_in_middle(win: PWINDOW; starty, startx: Smallint;
                          width: Longint; pair: Smallint;
                          const fmt: AnsiString; args: array of const);
var
  tstr: AnsiString;
  my, mx: Smallint;
begin
  FmtStr(tstr, fmt, args);
  getmaxyx(win, my, mx);
<<<<<<< HEAD
<<<<<<< HEAD
  dec(mx,startx);
=======
  mx -= startx;
>>>>>>> graemeg/fixes_2_2
=======
  mx -= startx;
>>>>>>> origin/fixes_2_2

  if (width > length(tstr)) OR  (width < 2) then
    width := length(tstr);

  if width > mx then
    width := mx;

  wattron(win,COLOR_PAIR(pair));
  mvwaddnstr(win,starty,startx + st_middle(mx,width),PChar(tstr),width);
  wattroff(win,COLOR_PAIR(pair));
end;


type
  PMinfo = ^TMinfo;
  TMinfo = record
    n, d: PChar;
  end;


type
  TSubmenu = class
  private
    _win: PWINDOW;
    _pan: PPANEL;
    _items: ppITEM;
    _exit, _sitem: pITEM;
    _menu: pMENU;
  public
    function doevent: chtype;
    constructor create(szy,szx,nch: Smallint; choices: PMinfo;
                        pair: Smallint;const name: AnsiString);
    destructor destroy; override;
    property menu: pMENU read _menu;
    property items: ppITEM read _items;
    property sitem: pITEM read _sitem write _sitem ;
    property win: PWINDOW read _win;
    property pan: PPANEL read _pan;
  end;

function TSubmenu.doevent: chtype;

function doenter(var ch: chtype): Boolean;
begin
        if current_item(_menu) = _exit then
        begin
          doenter := false;
          ch := -1
        end
        else
        if current_item(_menu) = sitem then
        begin
          doenter := false;
          ch := 10
        end
        else
          doenter := true;
end;

var
  ch: chtype = 0;
  doiter: Boolean = true;
begin
  while doiter do
  begin
    ch := wgetch(_win);
    case ch of
      KEY_DOWN:    menu_driver(_menu, REQ_DOWN_ITEM);
      KEY_UP:      menu_driver(_menu, REQ_UP_ITEM);
      KEY_LEFT:    menu_driver(_menu, REQ_LEFT_ITEM);
      KEY_RIGHT:   menu_driver(_menu, REQ_RIGHT_ITEM);
      KEY_NPAGE:   menu_driver(_menu, REQ_SCR_DPAGE);
      KEY_PPAGE:   menu_driver(_menu, REQ_SCR_UPAGE);
      chtype(' '): menu_driver(_menu, REQ_TOGGLE_ITEM);
      10: doiter := doenter(ch);             (* Enter *)
    else
      if menu_driver(_menu, ch) <> E_OK then
      begin
        doiter := false;
        if (ch <> chtype('q')) AND  (ch <> KEY_F(10)) then
          ch := -1;                    (* Close menu *)
      end
      else
        if (ch = KEY_MOUSE) then
          doiter := doenter(ch);
    end;
  end;
  update_panels();
  doupdate();
  doevent := ch;
end;

constructor TSubmenu.create(szy,szx,nch: Smallint; choices: PMinfo;
                             pair: Smallint;const name: AnsiString);
var
  i: Longint = 0;
begin
  GetMem(_items, (nch+1)*sizeof(pITEM));
  for i := 0 to nch - 1 do
    _items[i] := new_item(choices[i].n, choices[i].d);
  _items[nch] := nil;
  _exit := _items[i];
  sitem := nil;

  _menu := new_menu(_items);

  //scale_menu(_menu, @mrows, @mcols);
  _win := newwin(szy,szx,st_middle(LINES,szy),st_middle(COLS,szx));
  //_win := newwin(mrows + 2, mcols + 2, st_middle(LINES,mrows+2),st_middle(COLS,mcols+2));
  _pan := new_panel(_win);

  keypad(_win, TRUE);
  box(_win, ACS_VLINE, ACS_HLINE);

  wbkgd(_win, COLOR_PAIR(pair));
  set_menu_back(_menu, COLOR_PAIR(pair));

  print_in_middle(_win,0,0,szx-2,pair,name,[]);

  set_menu_win(_menu, _win);
  set_menu_sub(_menu, derwin(_win, szy-2, szx-2, 1, 1));
  //set_menu_sub(_menu, derwin(_win, mrows, mcols, 1, 1));
  set_menu_mark(_menu, '-');
end;

destructor TSubmenu.destroy;
var
  i: Longint = 0;
begin
  unpost_menu(_menu);
  free_menu(_menu);
  while _items[i] <> nil do
  begin
    free_item(_items[i]); Inc(i);
  end;
  FreeMem(_items, (i+1)*sizeof(pITEM));

  del_panel(_pan);
  delwin(_win);
  update_panels();
  doupdate();
end;


type
  Tmainptr = function: chtype;

const
  EXIT_PROGRAM = KEY_MAX + 100;

function confirm_menu: chtype;
const
  choices: array[0..2] of TMinfo =
                       (
                         (n:'    Yes    ';d:nil),
                         (n:'I dont know';d:nil),
                         (n:'    No     ';d:nil)
                       );
var
  smenu: TSubmenu;
begin
  smenu := TSubmenu.create(3, 41,3,choices,5,'Do you really want to quit?');

  menu_opts_off(smenu.menu, O_SHOWDESC);
  set_menu_format(smenu.menu, 1, 3);
  post_menu(smenu.menu);

  smenu.sitem := smenu.items[0];

  confirm_menu := smenu.doevent;

  if (confirm_menu = 10) OR (confirm_menu = chtype('q')) OR (confirm_menu = KEY_F(10)) then
    confirm_menu := EXIT_PROGRAM
  else
    confirm_menu := -1;
  smenu.destroy;
end;


(* Scrolling Menus example *)

function scroll_menu: chtype;
const
  choices: array[0..9] of TMinfo =
                    (
                      (n: '1_'; d: 'Choice'),
                      (n: '2_'; d: 'Choice'),
                      (n: '3_'; d: 'Choice'),
                      (n: '4_'; d: 'Choice'),
                      (n: '5_'; d: 'Choice'),
                      (n: '6_'; d: 'Choice'),
                      (n: '7_'; d: 'Choice'),
                      (n: '8_'; d: 'Choice'),
                      (n: '9_'; d: 'Choice'),
                      (n: '..'; d: 'Close')
                    );
var
  smenu: TSubmenu;
begin
  mvaddstr(LINES - 3, COLS - 30, '"PAGEUP" "PAGEDOWN" - scroll');
  refresh();
  smenu := TSubmenu.create(8, 13,10,choices,6,'Scrolling');

  set_menu_format(smenu.menu, 6, 1);
  post_menu(smenu.menu);

  scroll_menu := smenu.doevent;
  smenu.destroy;
  mvaddstr(LINES - 3, COLS - 30, '                            ');
  refresh();
end;


(* Milt Columnar Menus Example *)

function multicol_menu: chtype;
const
  choices: array[0..24] of TMinfo =
  (
    (n:'1_';d:nil),(n:'2_';d:nil),(n:'3_';d:nil),(n:'4_';d:nil),(n:'5_';d:nil),
    (n:'6_';d:nil),(n:'7_';d:nil),(n:'8_';d:nil),(n:'9_';d:nil),(n:'10';d:nil),
    (n:'11';d:nil),(n:'12';d:nil),(n:'13';d:nil),(n:'14';d:nil),(n:'15';d:nil),
    (n:'16';d:nil),(n:'17';d:nil),(n:'18';d:nil),(n:'19';d:nil),(n:'20';d:nil),
    (n:'21';d:nil),(n:'22';d:nil),(n:'23';d:nil),(n:'24';d:nil),(n:'..';d:nil)
  );
var
  smenu: TSubmenu;
  i: Longint;
begin
  smenu := TSubmenu.create(7, 22,25,choices,5,'Multicol');

(* Set menu option not to show the description *)
  menu_opts_off(smenu.menu, O_SHOWDESC);
  set_menu_format(smenu.menu, 5, 5);
  post_menu(smenu.menu);

  multicol_menu := smenu.doevent;
  smenu.destroy;
end;


(* Multi Valued Menus example *)

function multival_menu: chtype;
const
  choices: array[0..5] of TMinfo =
                    (
                      (n: '1_'; d: 'Choice'),
                      (n: '2_'; d: 'Choice'),
                      (n: '3_'; d: 'Choice'),
                      (n: '4_'; d: 'Choice'),
                      (n: '5_'; d: 'Choice'),
                      (n: '..'; d: 'Close')
                    );
var
  smenu: TSubmenu;
begin
  mvaddstr(LINES - 3, COLS - 30, '"SPACE" - toggle choice');
  refresh();
  smenu := TSubmenu.create(8, 13,6,choices,7,'Multival');

  menu_opts_off(smenu.menu, O_ONEVALUE);
  post_menu(smenu.menu);

  multival_menu := smenu.doevent;
  smenu.destroy;
  mvaddstr(LINES - 3, COLS - 30, '                       ');
  refresh();
end;


const
  n_choices = 4;
  choices: array[0..3] of TMinfo =
                    (
                      (n: '1_'; d: 'Scrolling Menus'),
                      (n: '2_'; d: 'Multi Columnar Menus'),
                      (n: '3_'; d: 'Multi Valued Menus'),
                      (n: '..'; d: 'Exit')
                    );

var
  main_menu_win: PWINDOW;
  main_menu_panel: PPANEL;

function mgetch: chtype;
begin
  mgetch :=   wgetch(main_menu_win);
end;

var
  my_bg: Smallint = COLOR_BLACK;

  main_items: ppITEM;
  cur_item: pITEM;
  main_menu: pMENU;
  ptr: Tmainptr = @mgetch;

  ch: chtype = -1;
  i: Longint;
begin
  try
   (* Initialize curses *)
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    curs_set(0);
    clear();
    mousemask(ALL_MOUSE_EVENTS, nil);

    if has_colors() then
    begin
      start_color();
      if (use_default_colors() = OK) then
        my_bg := -1
      else
        my_bg := COLOR_BLACK;

      init_pair(1, COLOR_YELLOW, my_bg);
      init_pair(2, COLOR_RED, my_bg);
      init_pair(3, COLOR_MAGENTA, my_bg);
      init_pair(4, COLOR_CYAN, my_bg);
      init_pair(5, COLOR_WHITE, COLOR_RED);
      init_pair(6, COLOR_WHITE, COLOR_BLUE);
      init_pair(7, COLOR_WHITE, COLOR_GREEN);
    end;

    main_menu_win := newwin(8, 40, st_middle(LINES, 8) - 2, st_middle(COLS, 40) - 10);
    main_menu_panel := new_panel(main_menu_win);
    keypad(main_menu_win, TRUE);

   (* Create items *)
    GetMem(main_items, (n_choices+1)*sizeof(pITEM));
    for i := 0 to n_choices-1 do
      main_items[i] := new_item(choices[i].n, choices[i].d);
    main_items[n_choices] := nil;

   (* Set the user pointers *)
     set_item_userptr(main_items[0], @scroll_menu);
     set_item_userptr(main_items[1], @multicol_menu);
     set_item_userptr(main_items[2], @multival_menu);
     set_item_userptr(main_items[3], @confirm_menu);

   (* Crate menu *)
     main_menu := new_menu(main_items);

   (* Set main window and sub window *)
     set_menu_win(main_menu, main_menu_win);
     set_menu_sub(main_menu, derwin(main_menu_win, 4, 38, 3, 1));

   (* Set menu mark to the string "=>" *)
     set_menu_mark(main_menu, '=>');

   (* Print a border around the main window and print a title *)
     box(main_menu_win, 0, 0);
     wbkgd(main_menu_win, COLOR_PAIR(6));
     set_menu_back(main_menu, COLOR_PAIR(6));

     print_in_middle(main_menu_win, 1, 0, 40, COLOR_PAIR(6), 'Main Menu', []);
     mvwaddch(main_menu_win, 2, 0, ACS_LTEE);
     mvwhline(main_menu_win, 2, 1, ACS_HLINE, 38);
     mvwaddch(main_menu_win, 2, 39, ACS_RTEE);
     attron(COLOR_PAIR(4));
     mvaddstr(LINES - 1, COLS - 30, 'Press "F10" or "q" to exit  ');
     attroff(COLOR_PAIR(4));
     refresh();

   (* Post the menu *)
     post_menu(main_menu);
     wrefresh(main_menu_win);

     while ch <> EXIT_PROGRAM do
     begin
       case ch of
         KEY_DOWN: menu_driver(main_menu, REQ_DOWN_ITEM);
         KEY_UP: menu_driver(main_menu, REQ_UP_ITEM);
         -1: ptr := @mgetch;              (* Restore ptr *)
         10:                              (* Enter *)
         begin
           cur_item := current_item(main_menu);     (* get current item *)
           ptr := Tmainptr(item_userptr(cur_item)); (* set ptr to current item *)
         end;
       else
       (* Process mouse and others events *)
         if (menu_driver(main_menu, ch) = E_OK) AND (ch = KEY_MOUSE) then
         begin
           cur_item := current_item(main_menu);
           ptr := Tmainptr(item_userptr(cur_item));
           wrefresh(main_menu_win);
         end;
       end;
       ch := ptr(); (* Call ptr function *)

       if (ch = chtype('q')) OR  (ch = KEY_F(10)) then
         ch := confirm_menu();
     end;

  finally
    unpost_menu(main_menu);
    free_menu(main_menu);
    for i := 0 to n_choices - 1 do
      free_item(main_items[i]);
    FreeMem(main_items, (n_choices+1)*sizeof(pITEM));
    del_panel(main_menu_panel);
    delwin(main_menu_win);
    curs_set(1);
    endwin();
  end;
end.