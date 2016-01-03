Program colbrowser;

uses xforms,strings;

Const  MAX_RGB = 3000;

var
  cl : PFL_FORM;
  rescol, dbobj, colbr, rs, gs, bs : PFL_OBJECT;
  dbname : string;
  infile : text;

{ the RGB data file does not have a standard location on unix. }
{ You may need to edit this }

const rgbfile = '/usr/lib/X11/rgb.txt';

type TRGBdb = record
       r, g, b : longint;
     end;

var
rgbdb : array [0..MAX_RGB] of TRGBdb;
numcol : longint;

procedure set_entry(i : longint);

var
    db : TRGBdb;

begin
  db := rgbdb[i-1];

    fl_freeze_form(cl);
    fl_mapcolor(FL_FREE_COL4+i, db.r, db.g, db.b);
    fl_mapcolor(FL_FREE_COL4, db.r, db.g, db.b);
    fl_set_slider_value(rs, db.r);
    fl_set_slider_value(gs, db.g);
    fl_set_slider_value(bs, db.b);
    fl_redraw_object(rescol);
    fl_unfreeze_form(cl);
end;

procedure br_cb(ob : PFL_OBJECT; q :longint);cdecl;

var r : longint;

begin
    r := fl_get_browser(ob);
    if (r <= 0) then exit;
    set_entry(r - 1);
end;

{ slow but straightforward }
function stripsp (s : string) : string;

var temp : string;
    i : longint;
begin
  temp:='';
  for i:=1 to length(s) do
    if pos(s[i],'0987654321')<>0 then temp:=temp+s[i];
  stripsp:=temp;
end;


function  read_entry(Var r,g,b : longint;var name : string) : longint;

var
    n : longint;
    buf,temp : string;
    code : word;


begin
    readln (infile,buf);
    if buf[1]='!' then exit(0);
    temp:=stripsp(copy(buf,1,4));delete(buf,1,4);
    val (temp,r,code);
    if code<>0 then exit(0);
    temp:=stripsp(copy(buf,1,4));delete(buf,1,4);
    val (temp,g,code);
    if code<>0 then exit(0);
    temp:=stripsp(copy(buf,1,4));delete(buf,1,4);
    val (temp,b,code);
    if code<>0 then exit(0);
    { strip leading spaces from name }
    while (buf[code+1]=' ') or (buf[code+1]=#9) do inc(code);
    if code<>0 then delete(buf,1,code);
    name:=buf+#0;
    read_entry:=1;
end;


function load_browser(fname : string) : longint;

var buf : string;
    r,g,b : Longint;
    rr,gg,bb : string[3];

begin
   assign (infile,fname);
{$i-}
  reset(infile);
{$i+}
  if ioresult<>0 then
    begin
      fname:=fname+#0;
        fl_show_alert('Load', @fname[1], 'Can''t open', 0);
        exit(0);
    end;

    fl_freeze_form(cl);
    numcol:=-1;
    while not eof(infile) do
      begin
      if read_entry(r, g, b, buf)<>0 then
        begin
        inc(numcol);
        rgbdb[numcol].r := r;
        rgbdb[numcol].g := g;
        rgbdb[numcol].b := b;
        str (r,rr); if length(rr)<3 then rr:=copy('   ',1,3-length(rr))+rr;
        str(g,gg);if length(gg)<3 then gg:=copy('   ',1,3-length(gg))+gg;
        str(b,bb);if length(bb)<3 then bb:=copy('   ',1,3-length(bb))+bb;
        buf:='('+rr+' '+gg+' '+bb+') '+buf;
        fl_addto_browser(colbr, @buf[1]);
        end;
      end;
    close(infile);
    fl_set_browser_topline(colbr, 1);
    fl_select_browser_line(colbr, 1);
    set_entry(0);
    fl_unfreeze_form(cl);
    load_browser:=1;
end;

function search_entry(r,g,b : Longint) : Longint;

var i, j, diffr, diffg, diffb,diff, mindiff : longint;

begin
    mindiff := 1 shl 25;
    J:=0;
    i:=0;
    for i:=0 to numcol do
      begin
       diffr := abs(r - rgbdb[i].r);
       diffg := abs(g - rgbdb[i].g);
       diffb := abs(b - rgbdb[i].b);
       diff := round((3.0 * diffr) +
               (5.9 * diffg) +
               (1.1 * diffb));
       if (mindiff > diff) then
         begin
         mindiff := diff;
         j := i;
         end;
      end;
    search_entry:= j;
end;

procedure search_rgb(ob : PFL_OBJECT; q : longint);cdecl;

var r, g, b, i,top : longint;

begin
    top  := fl_get_browser_topline(colbr);
    r := round(fl_get_slider_value(rs));
    g := round(fl_get_slider_value(gs));
    b := round(fl_get_slider_value(bs));

    fl_freeze_form(cl);
    fl_mapcolor(FL_FREE_COL4, r, g, b);
    fl_redraw_object(rescol);
    i := search_entry(r, g, b);
    { change topline only if necessary }
    if (i < top) or (i > (top+15)) then
       fl_set_browser_topline(colbr, i-8);
    fl_select_browser_line(colbr, i + 1);
    fl_unfreeze_form(cl);
end;

{ change database }
procedure db_cb(ob : PFL_OBJECT; q : longint);cdecl;

var p: pchar;
    buf : string;

begin
    p := fl_show_input('Enter New Database Name', @dbname[1]);
    buf:=strpas(p)+#0;
    if buf=dbname then exit;

    if (load_browser(buf)<>0) then
        dbname:=buf
    else
        fl_set_object_label(ob, @dbname[1]);
end;

procedure done_cb (ob : PFL_OBJECT; q :  longint);cdecl;
begin
    halt(0);
end;

procedure create_form_cl;
var
    obj : PFL_OBJECT;

begin
    if (cl<>nil) then exit;
    cl := fl_bgn_form(FL_NO_BOX, 330, 385);
    obj := fl_add_box(FL_UP_BOX, 0, 0, 330, 385, '');
    fl_set_object_color(obj, FL_INDIANRED, FL_COL1);
    obj := fl_add_box(FL_NO_BOX, 40, 10, 250, 30, 'Color Browser');
    fl_set_object_lcol(obj, FL_RED);
    fl_set_object_lsize(obj, FL_HUGE_SIZE);
    fl_set_object_lstyle(obj, FL_BOLD_STYLE + FL_SHADOW_STYLE);
    obj := fl_add_button(FL_NORMAL_BUTTON, 40, 50, 250, 25, '');
    dbobj := obj ;
    fl_set_object_boxtype(obj, FL_BORDER_BOX);
{    if fl_get_visual_depth()=1 then
      fl_set_object_color(obj, FL_WHITE,FL_INDIANRED)
    else
      fl_set_object_color(obj, FL_INDIANRED, FL_INDIANRED);
}
    fl_set_object_callback(obj, PFL_CALLBACKPTR(@db_cb), 0);

    obj:= fl_add_valslider(FL_VERT_FILL_SLIDER, 225, 130, 30, 200, '');
    rs := obj;
    fl_set_object_color(obj, FL_INDIANRED, FL_RED);
    fl_set_slider_bounds(obj, 0, 255);
    fl_set_slider_precision(obj, 0);
    fl_set_object_callback(obj, PFL_CALLBACKPTR(@search_rgb), 0);
    fl_set_slider_return(obj, 0);
    obj:= fl_add_valslider(FL_VERT_FILL_SLIDER, 255, 130, 30, 200, '');
    gs := obj ;
    fl_set_object_color(obj, FL_INDIANRED, FL_GREEN);
    fl_set_slider_bounds(obj, 0.0, 255.0);
    fl_set_slider_precision(obj, 0);
    fl_set_object_callback(obj, PFL_CALLBACKPTR(@search_rgb), 1);
    fl_set_slider_return(obj, 0);
    obj := fl_add_valslider(FL_VERT_FILL_SLIDER, 285, 130, 30, 200, '');
    bs := obj;
    fl_set_object_color(obj, FL_INDIANRED, FL_BLUE);
    fl_set_slider_bounds(obj, double(0.0), double(255.0));
    fl_set_slider_precision(obj, 0);
    fl_set_object_callback(obj, PFL_CALLBACKPTR(@search_rgb), 2);
    fl_set_slider_return(obj, 0);
    obj := fl_add_browser(FL_HOLD_BROWSER, 10, 90, 205, 240, '');
    colbr := obj ;
    fl_set_browser_fontstyle(obj, FL_FIXED_STYLE);
    fl_set_object_callback(obj, PFL_CALLBACKPTR(@br_cb), 0);

    obj := fl_add_button(FL_NORMAL_BUTTON, 135, 345, 80, 30, 'Done');
    fl_set_object_callback(obj, PFL_CALLBACKPTR(@done_cb), 0);
    obj := fl_add_box(FL_FLAT_BOX, 225, 90, 90, 35, '');
    rescol := obj;
    fl_set_object_color(obj, FL_FREE_COL4, FL_FREE_COL4);
    fl_set_object_boxtype(obj, FL_BORDER_BOX);

    fl_end_form();
    {fl_scale_form (cl, 1.1, 1.0);}
end;

begin
    fl_initialize(@argc, argv, 'FormDemo', nil, 0);
    cl:=nil;
    create_form_cl();
    dbname:= rgbfile+#0;
    if (load_browser(dbname)<>0) then
        fl_set_object_label(dbobj, @dbname[1])
    else
        fl_set_object_label(dbobj, 'None');

    fl_set_form_minsize(cl, cl^.w , cl^.h);
    fl_set_form_maxsize(cl, 2*cl^.w , 2*cl^.h);
    fl_show_form(cl, FL_PLACE_FREE, FL_TRANSIENT, 'RGB Browser');

    while (fl_do_forms()<>nil) do;
end.
