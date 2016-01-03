program form_test_2;

{$MODE OBJFPC}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  ncurses, form, initc;


procedure setlocale(cat : integer; p : pchar); cdecl; external clib;


const
  LC_ALL = 6;

var
  my_bg: Smallint = COLOR_BLACK;
  field: array[0..5] of PFIELD;
  my_form: PFORM;
  i, ch: Longint;

begin

  try

=======
=======
>>>>>>> origin/fixes_2_2
  ncurses, form, libc;


var
  my_bg: Smallint = COLOR_BLACK;

  field: array[0..5] of PFIELD;
  my_form: PFORM;
  i, ch: Longint;
begin

try
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  setlocale(LC_ALL, ''); { Tested with Russian UTF-8 locale }

  (* Initialize curses *)
   initscr();
   cbreak();
   noecho();
   keypad(stdscr, TRUE);

  (* Initialize colors *)
   if has_colors() then
   begin
     start_color();
     if (use_default_colors() = OK) then
       my_bg := -1
     else
       my_bg := COLOR_BLACK;

     init_pair(1, COLOR_YELLOW, my_bg);
     init_pair(2, COLOR_MAGENTA, my_bg);
     init_pair(3, COLOR_WHITE, my_bg);
     init_pair(4, COLOR_WHITE, COLOR_BLUE);
     init_pair(5, COLOR_WHITE, COLOR_GREEN);
     init_pair(6, COLOR_YELLOW, COLOR_GREEN);
     init_pair(7, COLOR_BLACK, COLOR_CYAN);
   end;

  (* Initialize the fields *)
    for i := 0 to 3 do
    begin
      field[i] := new_field(1, 30, 2 + i * 2, 10, 0, 0);
      field_opts_off(field[i], O_AUTOSKIP);
    end;

   field[4] := new_field(7, 30, 2, 42, 0, 0);
   field[5] := nil;

  (* Set field options *)
    set_field_fore(field[0], COLOR_PAIR(2));
    set_field_back(field[0], A_UNDERLINE OR COLOR_PAIR(3));

    set_field_fore(field[1], COLOR_PAIR(1));
    set_field_back(field[1], A_UNDERLINE OR COLOR_PAIR(1));
    field_opts_off(field[1], O_ACTIVE);

    set_field_fore(field[2], COLOR_PAIR(4));
    set_field_back(field[2], A_UNDERLINE OR COLOR_PAIR(4));
    field_opts_off(field[2], O_PUBLIC);

    set_field_fore(field[3], COLOR_PAIR(5));
    set_field_back(field[3], A_UNDERLINE OR COLOR_PAIR(5));
    field_opts_off(field[3], O_STATIC);

    set_field_fore(field[4], COLOR_PAIR(7));
    set_field_back(field[4], COLOR_PAIR(7));

  (* Create the form and post it *)
    my_form := new_form(field);
    post_form(my_form);

  (* Center Justification *)
    set_field_just(field[0], JUSTIFY_CENTER);
    set_field_buffer(field[0], 0, 'This is a static Field');

    set_field_just(field[1], JUSTIFY_CENTER);
    set_field_buffer(field[1], 0, 'This is a inactive Field');

  (* Set focus to the blue field *)
    set_current_field(my_form, field[0]);

    for i := 0 to 3 do
      mvprintw(2 + i * 2, 2, 'Value %d:', i + 1);
    mvaddstr(LINES - 2, 0, 'F1 to Exit');
    refresh();

  (* Loop through to get user requests *)
    ch := getch();
    while (ch <> KEY_F(1)) AND (ch <> 27) do
    begin
      case ch of
        9:   { TAB }
        begin
          if form_driver(my_form, REQ_NEXT_WORD) <> E_OK then
          begin
            form_driver(my_form, REQ_VALIDATION);
            form_driver(my_form, REQ_NEXT_FIELD);
            form_driver(my_form, REQ_END_LINE);
          end;
        end;
        KEY_NPAGE:
    (* Go to next field *)
        begin
          form_driver(my_form, REQ_VALIDATION);
          form_driver(my_form, REQ_NEXT_FIELD);
            { Go to the end of the present buffer
              Leaves nicely at the last character }
          form_driver(my_form, REQ_END_LINE);
        end;
        KEY_PPAGE:
    (* Go to previous field *)
        begin
          form_driver(my_form, REQ_VALIDATION);
          form_driver(my_form, REQ_PREV_FIELD);
          form_driver(my_form, REQ_END_LINE);
        end;
        KEY_DOWN:
          if form_driver(my_form, REQ_DOWN_CHAR) <> E_OK then
          begin
            form_driver(my_form, REQ_VALIDATION);
            form_driver(my_form, REQ_DOWN_FIELD);
            form_driver(my_form, REQ_END_LINE);
          end;
        KEY_UP:
          if form_driver(my_form, REQ_UP_CHAR) <> E_OK then
          begin
            form_driver(my_form, REQ_VALIDATION);
            form_driver(my_form, REQ_UP_FIELD);
            form_driver(my_form, REQ_END_LINE);
          end;
        KEY_LEFT:
          if form_driver(my_form, REQ_LEFT_CHAR) <> E_OK then
          begin
            form_driver(my_form, REQ_VALIDATION);
            form_driver(my_form, REQ_LEFT_FIELD);
            form_driver(my_form, REQ_END_LINE);
          end;
        KEY_RIGHT:
          if form_driver(my_form, REQ_RIGHT_CHAR) <> E_OK then
          begin
            form_driver(my_form, REQ_VALIDATION);
            form_driver(my_form, REQ_RIGHT_FIELD);
          end;
        KEY_BACKSPACE: form_driver(my_form, REQ_DEL_PREV);
        10: { ENTER }
          begin
            form_driver(my_form, 10);
            if form_driver(my_form, REQ_NEXT_LINE) <> E_OK then
            begin
              form_driver(my_form, REQ_VALIDATION);
              form_driver(my_form, REQ_NEXT_FIELD);
              form_driver(my_form, REQ_END_LINE);
            end;
          end;
      else
          { If this is a normal character, it gets
            Printed }
        form_driver(my_form, ch);
      end;
      ch := getch();
    end;

  refresh();

  finally
    unpost_form(my_form);
    free_form(my_form);
    endwin();

    for i := 0 to 4 do
    begin
      if field_status(field[i]) then
      begin
        writeln;
        writeln('Value ', i,':');
        writeln(field_buffer(field[i], 0));
      end;
      free_field(field[i]);
    end
  end;
end.