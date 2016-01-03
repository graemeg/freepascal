Program yesno;
{
  A box with two buttons and a string. Simple boxes like this
   are very usefull for asking questions
}

Uses xforms;

var
    form   : PFL_FORM;
    yes,no : PFL_OBJECT;

begin
  fl_initialize(@argc, argv, 'FormDemo', nil, 0);
  form := fl_bgn_form(FL_UP_BOX,320,120);
    fl_add_box(FL_NO_BOX,160,40,0,0,'Do you want to quit ?');
    yes := fl_add_button(FL_NORMAL_BUTTON,40,70,80,30,'Yes');
    no  := fl_add_button(FL_NORMAL_BUTTON,200,70,80,30,'No');
  fl_end_form;
  fl_show_form(form,FL_PLACE_MOUSE,FL_TRANSIENT,'Question : ');
  while (fl_do_forms <> yes) do;
  fl_hide_form(form);
end.
