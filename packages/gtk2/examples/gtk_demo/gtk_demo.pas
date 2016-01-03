(* GTK Demo for Pascal
 *
 * Welcome to GTK Demo for Pascal.
 *
 *
 * This demo is an adaption of the GTK Demo included in the GTK+-2.0 source.
 * A new feature is syntax highligting for pascal.
 *)

program gtk_demo;

{$mode objfpc} {$H+}
uses glib2, pango, gdk2, gtk2, gdk2pixbuf, strings, math;

var
  info_buffer   : PGtkTextBuffer;
  source_buffer : PGtkTextBuffer;
  current_file  : pgchar;



type
  TFileOfChar = file of char;

  TGDoDemoFunc = function : PGtkWidget;

  PDemo = ^TDemo;
  TDemo = record
            title    : pgchar;
            filename : pgchar;
            func     : TGDoDemoFunc;
            children : PDemo;
          end;

  PCallbackData = ^TCallbackData;
  TCallbackData = record
                    model : PGtkTreeModel;
                    path  : PGtkTreePath;
                  end;

const

  DEMO_DATA_DIR   = 'data';

  TITLE_COLUMN    = 0;
  FILENAME_COLUMN = 1;
  FUNC_COLUMN     = 2;
  ITALIC_COLUMN   = 3;
  NUM_COLUMNS     = 4;

  STATE_NORMAL     = 0;
  STATE_IN_COMMENT = 1;

  function demo_find_file (    base : pchar; err  : PPGError): pgchar; forward;

  (* file_is_valid
   *  a dirty little hack to find out if a file variable is assigned and the
   *  file is opened.
   *)
  function file_is_valid (var f: file): boolean;
  begin
    {$I-}
    if eof(f) then
      exit (TRUE);
    {$I+}
    if IOResult <> 0 then
      file_is_valid := FALSE
    else
      file_is_valid := TRUE;
  end;

  (* min, max
   *  these two functions of the math unit are overloaded to understand double
   *  values.
   *)
  function min (d1, d2: double): double;
  begin
    if d1 > d2 then  min := d2
    else min := d1;
  end;

  function max (d1, d2: double): double;
  begin
    if d1 < d2 then max := d2
    else max := d1;
  end;

  (* do_dummy
   *  creates a widget informing the user that the demo isn't implemented, yet
   *)

  procedure do_dummy (demo : pgchar);
  var
    dialog : PGtkWidget;
  begin
   dialog := gtk_message_dialog_new (NULL, 0,
                                       GTK_MESSAGE_INFO,
                                       GTK_BUTTONS_CLOSE,
                                       'Sorry, "%s" is''t implemented, yet.',
                                       [demo]);

    gtk_widget_show (dialog);

    g_signal_connect (dialog, 'response',
                        G_CALLBACK (@gtk_widget_destroy), NULL);
  end;

  (* include the modules here;
   * if you'd like to add one add the include command and
   * create a new entry in the testgtk_demos array
   *)

  {$include appwindow.inc}
  {$include button_box.inc}
  {$include colorsel.inc}
  {$include dialog.inc}
  {$include drawingarea.inc}
  {$include editable_cells.inc}
  {$include images.inc}
  {$include item_factory.inc}
  {$include list_store.inc}
  {$include menus.inc}
  {$include panes.inc}
  {$include pixbufs.inc}
  {$include sizegroup.inc}
  {$include stock_browser.inc}
  {$include textview.inc}
  {$include tree_store.inc}


const
  child0 : array [1..4] of TDemo  = (
      (title: 'Editable Cells'; filename: 'editable_cells.inc'; func: @do_editable_cells; children: nil),
      (title: 'List Store';     filename: 'list_store.inc';     func: @do_list_store;     children: nil),
      (title: 'Tree Store';     filename: 'tree_store.inc';     func: @do_tree_store;     children: nil),
      (title: nil;              filename: nil;                  func: nil;                children: nil));

  testgtk_demos: array [1..16] of TDemo  = (
      (title: '* This Application *';        filename: 'gtk_demo.pas';      func: nil;               children: nil),
      (title: 'Application main window';     filename: 'appwindow.inc';     func: @do_appwindow;     children: nil),
      (title: 'Button Boxes';                filename: 'button_box.inc';    func: @do_button_box;    children: nil),
      (title: 'Color Selector';              filename: 'colorsel.inc';      func: @do_colorsel;      children: nil),
      (title: 'Dialog and Message Boxes';    filename: 'dialog.inc';        func: @do_dialog;        children: nil),
      (title: 'Drawing Area';                filename: 'drawingarea.inc';   func: @do_drawingarea;   children: nil),
      (title: 'Images';                      filename: 'images.inc';        func: @do_images;        children: nil),
      (title: 'Item Factory';                filename: 'item_factory.inc';  func: @do_item_factory;  children: nil),
      (title: 'Menus';                       filename: 'menus.inc';         func: @do_menus;         children: nil),
      (title: 'Paned Widgets';               filename: 'panes.inc';         func: @do_panes;         children: nil),
      (title: 'Pixbufs';                     filename: 'pixbufs.inc';       func: @do_pixbufs;       children: nil),
      (title: 'Size Groups';                 filename: 'sizegroup.inc';     func: @do_sizegroup;     children: nil),
      (title: 'Stock Item and Icon Browser'; filename: 'stock_browser.inc'; func: @do_stock_browser; children: nil),
      (title: 'Text Widget';                 filename: 'textview.inc';      func: @do_textview;      children: nil),
      (title: 'Tree View';                   filename: nil;                 func: nil;               children: @child0),
      (title:  nil;                          filename: nil;                 func: nil;               children: nil));


function demo_find_file (    base : pchar;
                             err  : PPGError): pgchar;
var
  filename : pchar;

begin

  if g_file_test (base, G_FILE_TEST_EXISTS) then begin
    demo_find_file := g_strdup (base);
    exit;
  end else
  begin
    filename := g_build_filename (DEMO_DATA_DIR, [ base, NULL ]);

    if not (g_file_test (filename, G_FILE_TEST_EXISTS)) then
    begin
      g_set_error (err, G_FILE_ERROR, G_FILE_ERROR_NOENT,
                  'Cannot find demo data file "%s"', [base]);
      g_free (filename);
      demo_find_file := NULL;
    end else
      demo_find_file := filename;
  end;
end;


function create_text (var buffer : PGtkTextBuffer;
                      is_source  : gboolean): PGtkWidget;
var
  scrolled_window,
  text_view         : PGtkWidget;

  font_desc         : PPangoFontDescription;

begin
  scrolled_window := gtk_scrolled_window_new (NULL, NULL);

  gtk_scrolled_window_set_policy (GTK_SCROLLED_WINDOW (scrolled_window),
                                  GTK_POLICY_AUTOMATIC,
                                  GTK_POLICY_AUTOMATIC);

  gtk_scrolled_window_set_shadow_type (GTK_SCROLLED_WINDOW (scrolled_window),
                                       GTK_SHADOW_IN);

  text_view := gtk_text_view_new;

  buffer := gtk_text_buffer_new (NULL);

  gtk_text_view_set_buffer (GTK_TEXT_VIEW (text_view), buffer);
  gtk_text_view_set_editable (GTK_TEXT_VIEW (text_view), FALSE);
  gtk_text_view_set_cursor_visible (GTK_TEXT_VIEW (text_view), FALSE);

  gtk_container_add (GTK_CONTAINER (scrolled_window), text_view);

  if is_source then
  begin
      font_desc := pango_font_description_from_string ('Courier 12');
      gtk_widget_modify_font (text_view, font_desc);
      pango_font_description_free (font_desc);

      gtk_text_view_set_wrap_mode (GTK_TEXT_VIEW (text_view),
                                   GTK_WRAP_NONE);
  end else
  begin
        (* Make it a bit nicer for text. *)
      gtk_text_view_set_wrap_mode (GTK_TEXT_VIEW (text_view),
                                   GTK_WRAP_WORD);
      gtk_text_view_set_pixels_above_lines (GTK_TEXT_VIEW (text_view),  2);
      gtk_text_view_set_pixels_below_lines (GTK_TEXT_VIEW (text_view),  2);
  end;

  create_text := scrolled_window;
end;


const
  tokens: array [1..4] of pgchar =
      ('(*',
       '''',
       '{',
       '//');

  types: array [1..57] of pgchar =
      ('integer',
       'gchar',
       'pgchar',
       'char',
       'gfloat',
       'real',
       'gint8',
       'gint16',
       'gint32',
       'gint',
       'guint',
       'guint8',
       'guint16',
       'guint32',
       'guchar',
       'glong',
       'longint',
       'gboolean' ,
       'gshort',
       'gushort',
       'gulong',
       'gdouble',
       'double',
       'gldouble',
       'gpointer',
       'pointer',
       'NULL',
       'nil',
       'PGList',
       'TGList',
       'TGSList',
       'PGSList',
       'FALSE',
       'TRUE',
       'PGtkObject',
       'TGtkObject',
       'TGtkColorSelection',
       'PGtkColorSelection',
       'PGtkWidget',
       'TGtkWidget',
       'PGtkButton',
       'TGtkButton',
       'TGdkColor',
       'PGdkColor',
       'TGdkRectangle',
       'PGdkRectangle',
       'TGdkEventExpose',
       'PGdkEventExpose',
       'TGdkGC',
       'PGdkGC',
       'TGdkPixbufLoader',
       'PGdkPixbufLoader',
       'TGdkPixbuf',
       'PGdkPixbuf',
       'PPGError',
       'PGError',
       'array');

  control: array [1..23] of pgchar = (
       'if',
       'then',
       'case',
       'while',
       'else',
       'do',
       'for',
       'begin',
       'end',
       'exit',
       'goto',
       'program',
       'unit',
       'library',
       'procedure',
       'function',
       'type',
       'var',
       'const',
       'record',
       'uses',
       'of',
       'in');

procedure parse_chars ( text        :  pgchar;
                        var end_ptr :  pgchar;
                        var state   :  gint;
                        var tag     :  pgchar;
                        start       :  gboolean);
var
  i          : gint;
  next_token : pgchar;

  maybe_escape : boolean;
begin
 (* leave out leading spaces *)
  while (text^ <> #0) and (g_ascii_isspace (text^)) do
    inc (text);

  (* Handle comments first *)

  if state = STATE_IN_COMMENT then
  begin
    end_ptr    := StrPos (text, '*)');
    next_token := StrPos (text, '}');

    if next_token > end_ptr then begin
      end_ptr := next_token + 1;        // '}' comment type
      state   := STATE_NORMAL;
          tag     := 'comment';
    end else
      if end_ptr <> NULL then
      begin
            end_ptr := end_ptr + 2;         // '* )' comment type
            state   := STATE_NORMAL;
            tag     := 'comment';
      end;

    exit;
  end;

  tag := NULL;
  end_ptr := NULL;

  if text^ = #0 then
    exit;

  (* check for preprocessor defines *)

  if (((StrLComp (text, '(*', 2)) = 0) and (text[2] = '$') ) or
     (((StrLComp (text, '{', 1)) = 0) and (text[1] = '$') ) then
  begin
    end_ptr    := StrPos (text, '*)');
    next_token := StrPos (text, '}');

    if next_token > end_ptr then
      end_ptr := next_token + 1
    else
      if end_ptr <> NULL then
        end_ptr := end_ptr + 2;

    tag := 'preprocessor';
    exit;
  end;



  (* check for comment *)

  if ((StrLComp (text, '(*', 2)) = 0) or
     ((StrLComp (text, '{', 1)) = 0)  then
  begin
    end_ptr    := StrPos (text, '*)');
    next_token := StrPos (text, '}');

    if next_token > end_ptr then
      end_ptr := next_token+1
    else begin
      if end_ptr <> NULL then
        end_ptr := end_ptr + 2
      else
            state := STATE_IN_COMMENT;
    end;
    tag   := 'comment';
    exit;
  end;

  if (StrLComp (text, '//', 2)) = 0 then
  begin
    end_ptr := NULL;
    tag := 'comment';
    exit;
  end;

  (* check for types *)

  for i := 1 to high (types) do
    if ((StrLComp (text, types[i], strlen (types[i]))) = 0 )  and
        ((text+strlen(types[i]))^ in [#8, #32, #0, ';', #13, #10, ')', ']', ':']) then
    begin
          end_ptr := text + strlen (types[i]);
          tag := 'type';
      exit;
    end;

  (* check for control *)
  for i := 1 to  high (control) do begin
    if ((StrLComp (text, control[i], strlen (control[i]))) = 0) and
         ((text+strlen(control[i]))^ in [#8, #32, #0, ';', #13, #10, ')', ']', ':'])  then
    begin
          end_ptr := text + strlen (control[i]);
          tag := 'control';
      exit;
    end;
  end;

  (* check for string *)
  if text^= '''' then
  begin
    maybe_escape := FALSE;

    end_ptr := text + 1;
    tag := 'string';

    while end_ptr^ <> #0 do
    begin
          if (end_ptr^ = '''') and (maybe_escape = FALSE) then
      begin
        inc (end_ptr);
        exit;
      end;

          if end_ptr^ = '\' then
            maybe_escape := TRUE
          else
            maybe_escape := FALSE;

          inc (end_ptr);
    end;
    exit;
  end;


  (* not at the start of a tag.  Find the next one. *)
  for i := 1 to high(tokens) do
  begin
    next_token := StrPos (text, tokens[i]);
    if next_token <> NULL then
        begin
          if end_ptr <> NULL then
      begin
            if end_ptr > next_token then
          end_ptr := next_token;
      end else
            end_ptr := next_token;
    end;
  end;

  for i := 1 to high(types) do
  begin
    next_token := StrPos (text, types[i]);
    if next_token <> NULL then
      if ( (next_token+strlen(types[i]))^
          in [#8, #32, #0, ';', #13, #10, ')', ']', ':']) and
          g_ascii_isspace ((next_token-1)^) then
      begin
            if end_ptr <> NULL then
        begin
              if end_ptr > next_token then
            end_ptr := next_token;
        end else
              end_ptr := next_token;
      end;
  end;


  for i := 1 to high(control) do
  begin
    next_token := StrPos (text, control[i]);
    if next_token <> NULL then
      if ( (next_token+strlen(control[i]))^
          in [#8, #32, #0, ';', #13, #10, ')', ']', ':']) and
          g_ascii_isspace ((next_token-1)^) then
          begin
            if end_ptr <> NULL then
        begin
              if end_ptr > next_token then
            end_ptr := next_token;
        end else
              end_ptr := next_token;
      end;
  end;
end;


(* While not as cool as c-mode, this will do as a quick attempt at highlighting *)

procedure fontify;

var
  start_iter,
  next_iter,
  tmp_iter    : TGtkTextIter;

  state       : gint;
  text        : pgchar;
  start_ptr,

  end_ptr     : pgchar;
  tag         : pgchar;
  start       : gboolean;

begin
  state := STATE_NORMAL;

  gtk_text_buffer_get_iter_at_offset (source_buffer, @start_iter, 0);

  next_iter := start_iter;

  while (gtk_text_iter_forward_line (@next_iter)) do
  begin
    start := TRUE;
    text  := gtk_text_iter_get_text ( @start_iter, @next_iter);
    start_ptr := text;

    repeat
          parse_chars (start_ptr, end_ptr, state, tag, start);

          start := FALSE;
          if end_ptr <> NULL then begin
        tmp_iter := start_iter;
            gtk_text_iter_forward_chars (@tmp_iter, end_ptr - start_ptr);
      end else
        tmp_iter := next_iter;

          if tag <> NULL then
            gtk_text_buffer_apply_tag_by_name (source_buffer, tag, @start_iter, @tmp_iter);

          start_iter := tmp_iter;
          start_ptr  := end_ptr;
    until end_ptr = NULL;

    g_free (text);
    start_iter := next_iter;
  end;
end;

function read_line (var f: TFileOfChar; str: PGString): boolean;
var
  n_read : integer;
  c,
  next_c : char;

begin
  n_read := 0;

  g_string_truncate (str, 0);

  while not eof(f) do begin
    read (f, c);

        inc (n_read);

    if (c = #10) or (c = #13) then
    begin
      if not eof(f) then
      begin
        read (f, next_c);

        if not ((next_c in [#13, #10]) and (c <> next_c)) then
          seek(f, filepos(f)-1);
            break;
      end;
    end else
          g_string_append_c (str, c);
  end;

  read_line := n_read > 0;
end;



(* opens a textfile and reads it into the TGtkTextBuffer *)
procedure load_file (filename : pgchar);

var
  text_start,
  text_end     : TGtkTextIter;

  err          : PGError;
  buffer       : PGString;
  state,
  len_chars,
  len          : integer;

  in_para      : gboolean;
  f            : TFileOfChar;
  full_name    : pchar;
  p, q, r      : pgchar;

begin

  err     := NULL;
  buffer  := g_string_new (NULL);
  state   := 0;
  in_para := FALSE;

  if (current_file <> NULL) and  (StrComp (current_file, filename) = 0) then begin
    g_string_free (buffer, TRUE);
    exit;
  end;

  g_free (current_file);
  current_file := g_strdup (filename);

  gtk_text_buffer_get_bounds (info_buffer, @text_start, @text_end);
  gtk_text_buffer_delete (info_buffer, @text_start, @text_end);

  gtk_text_buffer_get_bounds (source_buffer, @text_start, @text_end);
  gtk_text_buffer_delete (source_buffer, @text_start, @text_end);

  full_name := demo_find_file (filename, @err);

  if full_name = NULL then begin
      g_warning ('%s', [err^.message]);
      g_error_free (err);
      exit;
  end;

  {$I-}
  assign (f, full_name);
  reset (f);
  {$I+}

  if IOResult <> 0 then
    g_print ('Cannot open %s:  file not found'#13#10, [full_name]);

  g_free (full_name);

  if IOResult <> 0 then
    exit;

  gtk_text_buffer_get_iter_at_offset (info_buffer, @text_start, 0);

  while read_line (f, buffer) do
  begin
    p := buffer^.str;

    case state of
          0 : begin (* Reading title *)

                while (((p^ = '(') or  (p^ = '*')) or (p^ = '{'))  or  g_ascii_isspace (p^)  do
                  inc (p);
                r := p;

                while (r^ <> ')')  and (strlen (r) > 0) do
                  inc (r);

                if strlen (r) > 0 then
                  p := r + 1;

                q := p + strlen (p);

                while (q > p)  and  g_ascii_isspace ((q - 1)^) do
                  dec(q);

                if q > p then
            begin
                  len_chars := g_utf8_pointer_to_offset (p, q);

                  text_end := text_start;

//                g_assert (strlen (p) >= (q - p));

                  gtk_text_buffer_insert (info_buffer, @text_end, p, q - p);
                  text_start := text_end;

                  gtk_text_iter_backward_chars (@text_start, len_chars);
                  gtk_text_buffer_apply_tag_by_name (info_buffer, 'title', @text_start, @text_end);

                  text_start := text_end;

                  inc (state);
             end; {of q > p }
              end; {of state = 0}


          1:  begin (* Reading body of info section *)
                while g_ascii_isspace (p^) do
                  inc(p);

                if (p^ = '*') and ((p + 1)^ = ')') then
                begin
              gtk_text_buffer_get_iter_at_offset (source_buffer, @text_start, 0);
                  inc(state);
            end else
            begin
              while (p^ = '*') or  g_ascii_isspace (p^) do
                        inc(p);

                  len := strlen (p);

                  while g_ascii_isspace ( (p + len - 1)^) do
                        dec (len);

                  if len > 0 then
              begin

                        if in_para then
                          gtk_text_buffer_insert (info_buffer, @text_start, ' ', 1);

//                      g_assert (strlen (p) >= len);

                        gtk_text_buffer_insert (info_buffer, @text_start, p, len);

                        in_para := TRUE;
              end else
              begin
                gtk_text_buffer_insert (info_buffer, @text_start, #10, 1);
                        in_para := FALSE;
              end; {else len <= 0}
            end;
              end;
          2: begin (* Skipping blank lines *)
               while g_ascii_isspace (p^) do
                 inc(p);

               if p^ <> #0 then
           begin
             p := buffer^.str;
             inc (state); (* Fall through *)

             (* Reading program body *)
             gtk_text_buffer_insert (source_buffer, @text_start, p, -1);
                 gtk_text_buffer_insert (source_buffer, @text_start, #10, 1);
           end;
         end;

          3: begin (* Reading program body *)
           gtk_text_buffer_insert (source_buffer, @text_start, p, -1);
               gtk_text_buffer_insert (source_buffer, @text_start, #10, 1);
         end;
    end;
  end;

 close (f);
 fontify ();

  g_string_free (buffer, TRUE);
end;

(* some callbacks *)

procedure window_closed_cb (window :  PGtkWidget;
                           data   :  gpointer); cdecl;
var
  cbdata   : PCallbackData;
  iter     : TGtkTreeIter;
  italic,
  nitalic  : gboolean;

begin
  cbdata := data;

  gtk_tree_model_get_iter (cbdata^.model, @iter, cbdata^.path);
  gtk_tree_model_get (GTK_TREE_MODEL (cbdata^.model), @iter,
                      [ ITALIC_COLUMN, @italic, -1] );
  nitalic := not italic;

  if italic then
    gtk_tree_store_set (GTK_TREE_STORE (cbdata^.model), @iter,
                        [ ITALIC_COLUMN, nitalic, -1] );

  gtk_tree_path_free (cbdata^.path);
  dispose (cbdata);

end;



procedure row_activated_cb (tree_view  : PGtkTreeView;
                            path       : PGtkTreePath;
                            column     : PGtkTreeViewColumn); cdecl;
var
  iter     : TGtkTreeIter;
  italic,
  nitalic  : gboolean;
  func     : TGDoDemoFunc;
  window   : PGtkWidget;
  model    : PGtkTreeModel;
  cbdata   : PCallbackData;

begin
  model := gtk_tree_view_get_model (tree_view);

  gtk_tree_model_get_iter (model, @iter, path);
  gtk_tree_model_get (GTK_TREE_MODEL (model),
                      @iter,
                    [ FUNC_COLUMN, @func,
                      ITALIC_COLUMN, @italic, -1 ]);

  if func <> NULL then
  begin
      nitalic := not italic;
      gtk_tree_store_set (GTK_TREE_STORE (model),
                          @iter,
                         [ ITALIC_COLUMN, nitalic, -1 ] );

      window := func();

      if window <> NULL then
          begin
        new (cbdata);
            cbdata^.model := model;

            cbdata^.path  := gtk_tree_path_copy (path);

            g_signal_connect (window, 'destroy',
                          G_CALLBACK (@window_closed_cb), cbdata );
          end;
  end;
end;


procedure selection_cb ( selection : PGtkTreeSelection;
                         model     : PGtkTreeModel); cdecl;
var
  iter   : TGtkTreeIter;
// value  : TGValue;
  str    : pgchar;

begin

(*  g_value_init(@value, G_TYPE_STRING);  //   added to test if TGValue works
                                          // -- its seems not as if it does *)

  if not gtk_tree_selection_get_selected (selection, NULL, @iter)  then
    exit;

(* The original code used TGValue but it seems not to work; check why *)

(*
  gtk_tree_model_get_value (model, @iter, FILENAME_COLUMN, @value);

  if (g_value_get_string (@value)) <> NULL then
    load_file (g_value_get_string (@value));

  g_value_unset (@value);
*)

  gtk_tree_model_get (model, @iter, [FILENAME_COLUMN, @str, -1]);

  if str <> NULL then
    load_file (str);

end;


function create_tree: PGtkWidget;

var
  selection  : PGtkTreeSelection;
  cell       : PGtkCellRenderer;
  tree_view  : PGtkWidget;
  column     : PGtkTreeViewColumn;
  model      : PGtkTreeStore;

  iter,
  child_iter : TGtkTreeIter;

  d,
  children   : PDemo;

begin
  d := @testgtk_demos;

  model := gtk_tree_store_new (NUM_COLUMNS, [G_TYPE_STRING, G_TYPE_STRING, G_TYPE_POINTER, G_TYPE_BOOLEAN]);

  tree_view := gtk_tree_view_new ();

  gtk_tree_view_set_model (GTK_TREE_VIEW (tree_view), GTK_TREE_MODEL (model));
  selection := gtk_tree_view_get_selection (GTK_TREE_VIEW (tree_view));

  gtk_tree_selection_set_mode (GTK_TREE_SELECTION (selection),
                               GTK_SELECTION_BROWSE);

  gtk_widget_set_size_request (tree_view, 200, -1);

  (* this code only supports 1 level of children. If we
   * want more we probably have to use a recursing function.
   *)


  while d^.title <> NULL do begin
    children := d^.children;

    gtk_tree_store_append (GTK_TREE_STORE (model), @iter, NULL);

    gtk_tree_store_set (GTK_TREE_STORE (model),
                          @iter,
                          [ TITLE_COLUMN, d^.title,
                            FILENAME_COLUMN, d^.filename,
                            FUNC_COLUMN, d^.func,
                            ITALIC_COLUMN, FALSE, -1 ] );

    inc(d);

    if children = NULL then
          continue;

    while children^.title <> NULL do begin

          gtk_tree_store_append (GTK_TREE_STORE (model), @child_iter, @iter);

          gtk_tree_store_set (GTK_TREE_STORE (model),
                              @child_iter,
                              [TITLE_COLUMN, children^.title,
                              FILENAME_COLUMN, children^.filename,
                              FUNC_COLUMN, children^.func,
                              ITALIC_COLUMN, FALSE,  -1]);
          inc (children);
    end;
  end;

  cell := gtk_cell_renderer_text_new ();

  g_object_set (G_OBJECT (cell),
                'style', [ PANGO_STYLE_ITALIC, NULL ]);


  column := gtk_tree_view_column_new_with_attributes ('Widget (double click for demo)',
                                                     cell,
                                                     [ 'text', TITLE_COLUMN,
                                                       'style_set', ITALIC_COLUMN, NULL ] );

  gtk_tree_view_append_column (GTK_TREE_VIEW (tree_view),
                               GTK_TREE_VIEW_COLUMN (column));

  g_signal_connect (selection, 'changed', G_CALLBACK (@selection_cb), model);
  g_signal_connect (tree_view, 'row_activated', G_CALLBACK (@row_activated_cb), model);

  gtk_tree_view_expand_all (GTK_TREE_VIEW (tree_view));

  create_tree := tree_view;
end;


procedure setup_default_icon;
var
  pixbuf      : PGdkPixbuf;
  filename    : pchar;
  err         : PGError;

  dialog      : PGtkWidget;

  list        : PGList;
  transparent : PGdkPixbuf;

begin
  err := NULL;
  pixbuf := NULL;
  dialog := NULL;

  filename := demo_find_file ('gtk-logo-rgb.gif', @err);

  if filename <> NULL then
  begin
    pixbuf := gdk_pixbuf_new_from_file (filename, @err);
    g_free (filename);
  end;

  (* Ignoring this error (passing NULL instead of &err above)
   * would probably be reasonable for most apps.  We're just
   * showing off.
   *)

  if err <> NULL then
  begin
    dialog := gtk_message_dialog_new (NULL, 0,
                                       GTK_MESSAGE_ERROR,
                                       GTK_BUTTONS_CLOSE,
                                       'Failed to read icon file: %s',
                                       [err^.message]);
    gtk_widget_show (dialog);
    g_error_free (err);

    g_signal_connect (dialog, 'response',
                        G_CALLBACK (@gtk_widget_destroy), NULL);
  end;

  if pixbuf <> NULL then
  begin
    (* The gtk-logo-rgb icon has a white background, make it transparent *)

    transparent := gdk_pixbuf_add_alpha (pixbuf, TRUE, $ff, $ff, $ff);

    list := NULL;
    list := g_list_append (list, transparent);

    gtk_window_set_default_icon_list (list);

    g_list_free (list);
    g_object_unref (G_OBJECT (pixbuf));
    g_object_unref (G_OBJECT (transparent));
  end;
end;



var
  window,
  notebook,
  hbox,
  tree       : PGtkWidget;

begin
  current_file := NULL;

  {$include init.inc}  (* contains all variable inits of the demos *)

  gtk_init (@argc, @argv);

  setup_default_icon ();

  window := gtk_window_new (GTK_WINDOW_TOPLEVEL);

  gtk_window_set_title (GTK_WINDOW (window), 'GTK+ Code Demos');

  g_signal_connect (window, 'destroy',
                            G_CALLBACK (@gtk_main_quit), NULL);

  hbox := gtk_hbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), hbox);

  tree := create_tree;

  gtk_box_pack_start (GTK_BOX (hbox), tree, FALSE, FALSE, 0);

  notebook := gtk_notebook_new;
  gtk_box_pack_start (GTK_BOX (hbox), notebook, TRUE, TRUE, 0);

  gtk_notebook_append_page (GTK_NOTEBOOK (notebook),
                            create_text (info_buffer, FALSE),
                            gtk_label_new_with_mnemonic ('_Info'));


  gtk_notebook_append_page (GTK_NOTEBOOK (notebook),
                            create_text (source_buffer, TRUE),
                            gtk_label_new_with_mnemonic ('_Source'));

  gtk_text_buffer_create_tag (info_buffer, 'title', 'font', ['Sans 18',  NULL ]);

  gtk_text_buffer_create_tag (source_buffer, 'comment', 'foreground', ['red', NULL]);

  gtk_text_buffer_create_tag (source_buffer, 'type', 'foreground', ['ForestGreen', NULL]);

  gtk_text_buffer_create_tag (source_buffer, 'string', 'foreground',
                                    ['RosyBrown', 'weight', PANGO_WEIGHT_BOLD, NULL]);

  gtk_text_buffer_create_tag (source_buffer, 'control', 'foreground', ['purple', NULL]);

  gtk_text_buffer_create_tag (source_buffer, 'preprocessor', 'style',
                                     [ PANGO_STYLE_OBLIQUE, 'foreground', 'blue', NULL] );

  gtk_text_buffer_create_tag (source_buffer, 'function', 'weight',
                                     [ PANGO_WEIGHT_BOLD, 'foreground', 'DarkGoldenrod4', NULL]);

  gtk_window_set_default_size (GTK_WINDOW (window), 600, 400);
  gtk_widget_show_all (window);

  gtk_main;
end.
