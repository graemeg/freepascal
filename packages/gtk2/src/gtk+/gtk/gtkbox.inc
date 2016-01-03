// included by gtk2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
   PGtkBox = ^TGtkBox;
   TGtkBox = record
        container : TGtkContainer;
        children : PGList;
        spacing : gint16;
        flag0 : word;
     end;

   PGtkBoxClass = ^TGtkBoxClass;
   TGtkBoxClass = record
        parent_class : TGtkContainerClass;
     end;

   PGtkBoxChild = ^TGtkBoxChild;
   TGtkBoxChild = record
        widget : PGtkWidget;
        padding : guint16;
        flag0 : word;
     end;
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}
const
   bm_TGtkBox_homogeneous = $1;
   bp_TGtkBox_homogeneous = 0;

   bm_TGtkBoxChild_expand = $1;
   bp_TGtkBoxChild_expand = 0;
   bm_TGtkBoxChild_fill = $2;
   bp_TGtkBoxChild_fill = 1;
   bm_TGtkBoxChild_pack = $4;
   bp_TGtkBoxChild_pack = 2;
   bm_TGtkBoxChild_is_secondary = $8;
   bp_TGtkBoxChild_is_secondary = 3;


function GTK_TYPE_BOX : GType;
function GTK_BOX(obj: pointer) : PGtkBox;
function GTK_BOX_CLASS(klass: pointer) : PGtkBoxClass;
function GTK_IS_BOX(obj: pointer) : boolean;
function GTK_IS_BOX_CLASS(klass: pointer) : boolean;
function GTK_BOX_GET_CLASS(obj: pointer) : PGtkBoxClass;


function homogeneous(var a : TGtkBox) : guint; overload;
procedure set_homogeneous(var a : TGtkBox; __homogeneous : guint); overload;

function expand(var a : TGtkBoxChild) : guint; overload;
procedure set_expand(var a : TGtkBoxChild; __expand : guint); overload;
function fill(var a : TGtkBoxChild) : guint;
procedure set_fill(var a : TGtkBoxChild; __fill : guint);
function pack(var a : TGtkBoxChild) : guint;
procedure set_pack(var a : TGtkBoxChild; __pack : guint);
function is_secondary(var a : TGtkBoxChild) : guint;
procedure set_is_secondary(var a : TGtkBoxChild; __is_secondary : guint);

function gtk_box_get_type:TGtkType; cdecl; external gtklib;
procedure gtk_box_pack_start(box:PGtkBox; child:PGtkWidget; expand:gboolean; fill:gboolean; padding:guint); cdecl; external gtklib;
procedure gtk_box_pack_end(box:PGtkBox; child:PGtkWidget; expand:gboolean; fill:gboolean; padding:guint); cdecl; external gtklib;
procedure gtk_box_pack_start_defaults(box:PGtkBox; widget:PGtkWidget); cdecl; external gtklib;
procedure gtk_box_pack_end_defaults(box:PGtkBox; widget:PGtkWidget); cdecl; external gtklib;
procedure gtk_box_set_homogeneous(box:PGtkBox; homogeneous:gboolean); cdecl; external gtklib;
function gtk_box_get_homogeneous(box:PGtkBox):gboolean; cdecl; external gtklib;
procedure gtk_box_set_spacing(box:PGtkBox; spacing:gint); cdecl; external gtklib;
function gtk_box_get_spacing(box:PGtkBox):gint; cdecl; external gtklib;
procedure gtk_box_reorder_child(box:PGtkBox; child:PGtkWidget; position:gint); cdecl; external gtklib;
procedure gtk_box_query_child_packing(box:PGtkBox; child:PGtkWidget; expand:Pgboolean; fill:Pgboolean; padding:Pguint;
            pack_type:PGtkPackType); cdecl; external gtklib;
procedure gtk_box_set_child_packing(box:PGtkBox; child:PGtkWidget; expand:gboolean; fill:gboolean; padding:guint;
            pack_type:TGtkPackType); cdecl; external gtklib;
{$ENDIF read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
function GTK_TYPE_BOX : GType;
begin
   GTK_TYPE_BOX:=gtk_box_get_type;
end;

function GTK_BOX(obj: pointer) : PGtkBox;
begin
   GTK_BOX:=PGtkBox(GTK_CHECK_CAST(obj,GTK_TYPE_BOX));
end;

function GTK_BOX_CLASS(klass: pointer) : PGtkBoxClass;
begin
   GTK_BOX_CLASS:=PGtkBoxClass(GTK_CHECK_CLASS_CAST(klass,GTK_TYPE_BOX));
end;

function GTK_IS_BOX(obj: pointer) : boolean;
begin
   GTK_IS_BOX:=GTK_CHECK_TYPE(obj,GTK_TYPE_BOX);
end;

function GTK_IS_BOX_CLASS(klass: pointer) : boolean;
begin
   GTK_IS_BOX_CLASS:=GTK_CHECK_CLASS_TYPE(klass,GTK_TYPE_BOX);
end;

function GTK_BOX_GET_CLASS(obj: pointer) : PGtkBoxClass;
begin
   GTK_BOX_GET_CLASS:=PGtkBoxClass(GTK_CHECK_GET_CLASS(obj,GTK_TYPE_BOX));
end;

function homogeneous(var a : TGtkBox) : guint;
begin
   homogeneous:=(a.flag0 and bm_TGtkBox_homogeneous) shr bp_TGtkBox_homogeneous;
end;

procedure set_homogeneous(var a : TGtkBox; __homogeneous : guint);
begin
   a.flag0:=a.flag0 or ((__homogeneous shl bp_TGtkBox_homogeneous) and bm_TGtkBox_homogeneous);
end;

function expand(var a : TGtkBoxChild) : guint;
begin
   expand:=(a.flag0 and bm_TGtkBoxChild_expand) shr bp_TGtkBoxChild_expand;
end;

procedure set_expand(var a : TGtkBoxChild; __expand : guint);
begin
   a.flag0:=a.flag0 or ((__expand shl bp_TGtkBoxChild_expand) and bm_TGtkBoxChild_expand);
end;

function fill(var a : TGtkBoxChild) : guint;
begin
   fill:=(a.flag0 and bm_TGtkBoxChild_fill) shr bp_TGtkBoxChild_fill;
end;

procedure set_fill(var a : TGtkBoxChild; __fill : guint);
begin
   a.flag0:=a.flag0 or ((__fill shl bp_TGtkBoxChild_fill) and bm_TGtkBoxChild_fill);
end;

function pack(var a : TGtkBoxChild) : guint;
begin
   pack:=(a.flag0 and bm_TGtkBoxChild_pack) shr bp_TGtkBoxChild_pack;
end;

procedure set_pack(var a : TGtkBoxChild; __pack : guint);
begin
   a.flag0:=a.flag0 or ((__pack shl bp_TGtkBoxChild_pack) and bm_TGtkBoxChild_pack);
end;

function is_secondary(var a : TGtkBoxChild) : guint;
begin
   is_secondary:=(a.flag0 and bm_TGtkBoxChild_is_secondary) shr bp_TGtkBoxChild_is_secondary;
end;

procedure set_is_secondary(var a : TGtkBoxChild; __is_secondary : guint);
begin
   a.flag0:=a.flag0 or ((__is_secondary shl bp_TGtkBoxChild_is_secondary) and bm_TGtkBoxChild_is_secondary);
end;

{$ENDIF read_implementation}
// included by gtk2.pas

