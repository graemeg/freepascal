uses
  ncurses,menu;

const
  choices : array[0..5] of pchar = (
                        'Choice 1',
                        'Choice 2',
                        'Choice 3',
                        'Choice 4',
                        'Exit',
                        nil
                  );
procedure print_in_middle(win : PWINDOW;starty,startx,width : longint;_string : pchar;color : chtype);
  var
    length,x,y : longint;
    temp : single;
  begin
        if win=nil then
          win:=stdscr;
        getyx(win, y, x);
        if startx <> 0 then
          x := startx;
        if starty <> 0 then
                y := starty;
        if width=0 then
          width := 80;

        length := strlen(_string);
        temp := (width - length)/ 2;
        x := startx + round(temp);
        wattron(win, color);
        mvwprintw(win, y, x, '%s', [_string]);
        wattroff(win, color);
        refresh;
  end;

var
  my_items : ppitem;
  c : longint;
  my_menu : pmenu;
  my_menu_win : pwindow;
  i,n_choices : longint;

begin
        { Initialize curses }
        initscr;
        start_color;
        cbreak;
        noecho;
        keypad(stdscr, 1);
        init_pair(1, COLOR_RED, COLOR_BLACK);

        { Create items }
        n_choices := high(choices);
        getmem(my_items,n_choices*sizeof(pitem));
        for i:=0 to n_choices-1 do
          my_items[i] := new_item(choices[i], choices[i]);

        { Create menu }
        my_menu := new_menu(ppitem(my_items));

        { Create the window to be associated with the menu }
        my_menu_win := newwin(10, 40, 4, 4);
        keypad(my_menu_win, 1);

        { Set main window and sub window }
        set_menu_win(my_menu, my_menu_win);
        set_menu_sub(my_menu, derwin(my_menu_win, 6, 38, 3, 1));

        { Set menu mark to the string ' * ' }
        set_menu_mark(my_menu, ' * ');

        { Print a border around the main window and print a title }
        box(my_menu_win, 0, 0);
        print_in_middle(my_menu_win, 1, 0, 40, 'My Menu', COLOR_PAIR(1));
        mvwaddch(my_menu_win, 2, 0, ACS_LTEE);
        mvwhline(my_menu_win, 2, 1, ACS_HLINE, 38);
        mvwaddch(my_menu_win, 2, 39, ACS_RTEE);
        mvprintw(LINES - 2, 0, 'F1 to exit',[]);
        refresh();

        { Post the menu }
        post_menu(my_menu);
        wrefresh(my_menu_win);

        c:=wgetch(my_menu_win);
        while(c<> KEY_F(1)) do
          begin
            case c of
              KEY_DOWN:
                menu_driver(my_menu, REQ_DOWN_ITEM);
              KEY_UP:
                menu_driver(my_menu, REQ_UP_ITEM);
            end;
            wrefresh(my_menu_win);
            c:=wgetch(my_menu_win);
          end;

        { Unpost and free all the memory taken up }
        unpost_menu(my_menu);
        free_menu(my_menu);
        for i:=0 to n_choices-1 do
          free_item(my_items[i]);
        endwin();
end.
