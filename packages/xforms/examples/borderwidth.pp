program borderwidth;

uses xforms;

type TFD_bwform = record
        bwform : PFL_FORM;
        vdata : pointer;
        ldata : longint;
        done,
        bw_choice,
        bwgroup,
        pmobj : PFL_OBJECT;
        end;
PFD_bwform = ^TFD_bwform;


{ callbacks for form bwform }
procedure done_callback(ob : PFL_OBJECT; data : longint);cdecl;
begin
  { fill-in code for callback }
   halt(0);
end;

var
fd_bwform : PFD_bwform;

const bws : array[0..7] of longint = (-3,-2,-1,1,2,3,4,5);

procedure bw_callback(ob : PFL_OBJECT; data : longint);cdecl;

var bw : longint;

begin
   bw := bws[fl_get_choice(ob)-1];

   fl_set_object_bw(fd_bwform^.bwgroup, bw);
   { since bwgroup includes the backface, it wipes out the done button}
   fl_redraw_object(fd_bwform^.done);
   fl_redraw_object(fd_bwform^.bw_choice);
end;

function create_form_bwform : PFD_bwform;
var
  obj : PFL_OBJECT;
  fdui : PFD_bwform;

begin

  new(fdui);
  fdui^.bwform := fl_bgn_form(FL_NO_BOX, 380, 340);

  fdui^.bwgroup := fl_bgn_group();
  obj := fl_add_box(FL_UP_BOX,0,0,380,340,'');
  obj := fl_add_frame(FL_EMBOSSED_FRAME,220,60,135,145,'');
  obj := fl_add_frame(FL_ENGRAVED_FRAME,15,60,185,145,'');
  obj := fl_add_slider(FL_HOR_SLIDER,25,75,160,25,'');
  obj := fl_add_pixmapbutton(FL_NORMAL_BUTTON,305,145,40,35,'');
  fdui^.pmobj := obj;
  obj := fl_add_positioner(FL_NORMAL_POSITIONER,30,225,100,80,'');
  obj := fl_add_counter(FL_NORMAL_COUNTER,25,160,160,25,'');
  obj := fl_add_lightbutton(FL_PUSH_BUTTON,230,100,100,30,'LightButton');
  obj := fl_add_roundbutton(FL_PUSH_BUTTON,230,130,80,33,'Button');
  obj := fl_add_round3dbutton(FL_PUSH_BUTTON,230,152,80,33,'Button');
    fl_set_object_color(obj,FL_COL1, FL_BLUE);
  obj := fl_add_checkbutton(FL_PUSH_BUTTON,230,174,80,33,'Button');
  obj := fl_add_input(FL_NORMAL_INPUT,195,240,160,28,'Input');
  obj := fl_add_valslider(FL_HOR_BROWSER_SLIDER,25,120,160,25,'');
  obj := fl_add_button(FL_NORMAL_BUTTON,230,65,100,30,'Button');
  fl_end_group();

  obj := fl_add_button(FL_NORMAL_BUTTON,270,290,75,30,'Done');
  fdui^.done := obj;
    fl_set_object_callback(obj,PFL_CALLBACKPTR(@done_callback),0);
  obj:= fl_add_choice(FL_NORMAL_CHOICE,105,20,80,25,'Border Width');
  fdui^.bw_choice := obj ;
    fl_set_object_callback(obj,PFL_CALLBACKPTR(@bw_callback),0);
  fl_end_form();
  fdui^.bwform^.fdui  := fdui;

  create_form_bwform:= fdui;
end;

begin
   { application default. Can be overriden by the command line options }
   fl_set_border_width(-2);

   fl_initialize(@argc, argv, 'FormDemo', nil, 0);
   fd_bwform := create_form_bwform;

   { fill-in form initialization code }
   fl_set_pixmapbutton_file(fd_bwform^.pmobj, 'crab.xpm');

   fl_addto_choice(fd_bwform^.bw_choice,'-3 Pixel|-2 Pixel|-1 Pixel');
   fl_addto_choice(fd_bwform^.bw_choice,' 1 Pixel| 2 Pixel| 3 Pixel');
   fl_addto_choice(fd_bwform^.bw_choice,' 4 Pixel| 5 Pixel');
   fl_set_choice_text(fd_bwform^.bw_choice, '-2 Pixel');

   { show the first form }
   fl_show_form(fd_bwform^.bwform,FL_PLACE_CENTER,FL_NOBORDER,'bwform');
   while (fl_do_forms<>nil) do;
end.
