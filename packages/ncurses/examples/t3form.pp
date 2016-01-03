{
   Author: Vitaliy Trifonov
}
program form_test_3;

{$MODE OBJFPC}

uses
<<<<<<< HEAD
<<<<<<< HEAD
  ncurses, form, initc;

procedure setlocale(cat : integer; p : pchar); cdecl; external clib;


const
  LC_ALL = 6;
=======
  ncurses, form, libc;

>>>>>>> graemeg/fixes_2_2
=======
  ncurses, form, libc;

>>>>>>> origin/fixes_2_2


function st_middle(scrlen, itemlen: Smallint): Smallint; inline;
begin
  st_middle := (scrlen - itemlen) div 2;
end;


procedure draw;

function randomchar: chtype;
var
  ch: Char = #0;
begin
  while not (ch in ['0'..'9','A'..'Z','a'..'z']) do
    ch := Char(Random(123));
  randomchar := chtype(ch);
end;

function randompair: longint;
var
  pair: longint = 0;
begin
  while not (pair in [1..5]) do
    pair := Random(6);
  randompair := pair;
end;

var
  y, x:  Smallint;
begin
  for y := 0 to LINES - 1 do
    for x := 0 to COLS - 1 do
      mvaddch(y, x, randomchar OR COLOR_PAIR(randompair));
end;

const
  enumval: array[0..2] of PChar = ('one', 'two', 'three');
  desc: array[0..5] of PChar =
              (
                'TYPE_ALPHA    Char data, a min width 8',
                'TYPE_ENUM      one, two, three',
                'TYPE_INTEGER     -300 .. 300',
                'TYPE_NUMERIC    -30.0 .. 30.0',
                'TYPE_REGEXP ^http://.+\.(ru|net|com)\s*$',
                'TYPE_IPV4     An IP Version 4 address.'
              );
var
  my_bg: Smallint = COLOR_BLACK;
  form_win: PWINDOW;

  pair: Smallint;
  field: array[0..6] of PFIELD;
  my_form: PFORM;
  i, frows, fcols, ch: Longint;
begin

try
  setlocale(LC_ALL, '');

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
     init_pair(4, COLOR_CYAN, my_bg);
     init_pair(5, COLOR_GREEN, my_bg);
     init_pair(6, COLOR_WHITE, COLOR_BLUE);
     init_pair(7, COLOR_BLACK, COLOR_CYAN);
   end;


   for i := 0 to 5 do
   begin
     field[i] := new_field(1, 30, 2 + i * 3, 10, 0, 0);
     field_opts_off(field[i], O_AUTOSKIP);
     if i AND 1 = 0 then
       pair := 7
     else
       pair := 6;
     set_field_fore(field[i], COLOR_PAIR(pair));
     set_field_back(field[i], A_UNDERLINE OR COLOR_PAIR(pair));
     //set_field_pad(field[i],chtype(' '));
   end;
   draw;
   refresh();

   field[6] := nil;

   set_field_type(field[0],TYPE_ALPHA,8);
   set_field_type(field[1],TYPE_ENUM,PPChar(enumval),0,0);
   set_field_type(field[2],TYPE_INTEGER,3,-300,300);
   set_field_type(field[3],TYPE_NUMERIC,8,-30.0,30.0);
   set_field_type(field[4],TYPE_REGEXP,'^http://.+\.(ru|net|com)\s*$');
   set_field_type(field[5],TYPE_IPV4);


   my_form := new_form(field);

(* Calculate the area required for the form *)
   scale_form(my_form, @frows, @fcols);

(* Create the window to be associated with the form *)
   //form_win := newwin(rows + 4, cols + 4, 4, 4);
   form_win := newwin(frows + 4, fcols + 4, st_middle(LINES,frows+4), st_middle(COLS,fcols+4));
   keypad(form_win, TRUE);

(* Set main window and sub window *)
   set_form_win(my_form, form_win);
   set_form_sub(my_form, derwin(form_win, frows, fcols, 2, 2));

(* Print a border around the main window and print a title *)
   box(form_win, 0, 0);
   //print_in_middle(my_form_win, 1, 0, cols + 4, "My Form", COLOR_PAIR(1));

   post_form(my_form);
   wrefresh(form_win);

   for i := 0 to 5 do
     mvwaddstr(form_win, 3 + i * 3, 1,desc[i]);
   wrefresh(form_win);

   //set_field_buffer(field[0], 0, 'Test Field');
   //refresh();

  (* Loop through to get user requests *)
    ch := wgetch(form_win);
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
          end;
        KEY_UP:
          if form_driver(my_form, REQ_UP_CHAR) <> E_OK then
          begin
            form_driver(my_form, REQ_VALIDATION);
            form_driver(my_form, REQ_UP_FIELD);
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
      ch := wgetch(form_win);
    end;

  finally

    unpost_form(my_form);
    free_form(my_form);
    delwin(form_win);
    endwin();

    for i := 0 to 5 do
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