// included by gtk2.pas

{$IFDEF read_forward_definitions}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}

    PGtkCellRendererCombo = ^TGtkCellRendererCombo;
    TGtkCellRendererCombo = record
        parent : TGtkCellRendererText;
        model : PGtkTreeModel;
        text_column: gint;
        has_entry: gboolean;
        {< private >}
       focus_out_id: guint;
    end;

    PGtkCellRendererComboClass = ^TGtkCellRendererComboClass;
    TGtkCellRendererComboClass = record
       parent_class : TGtkCellRendererTextClass;
    end;

{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_rest}

    function GTK_TYPE_CELL_RENDERER_COMBO: GType;
    function GTK_CELL_RENDERER_COMBO(obj: Pointer): PGtkCellRendererCombo;
    function GTK_CELL_RENDERER_COMBO_CLASS(klass: pointer): PGtkCellRendererComboClass;
    function GTK_IS_CELL_RENDERER_COMBO(obj: pointer): boolean;
    function GTK_IS_CELL_RENDERER_COMBO_CLASS(klass: pointer):boolean;
    function GTK_CELL_RENDERER_COMBO_GET_CLASS(obj: pointer): PGtkCellRendererComboClass;

    function gtk_cell_renderer_combo_get_type:TGtkType; cdecl; external gtklib;
    function gtk_cell_renderer_combo_new: PGtkCellRendererCombo; cdecl; external gtklib;

{$ENDIF read_interface_rest}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}

    function GTK_TYPE_CELL_RENDERER_COMBO: GType;
    begin
      Result:= gtk_cell_renderer_combo_get_type ();
    end;
    
    function GTK_CELL_RENDERER_COMBO(obj: Pointer): PGtkCellRendererCombo;
    begin
      Result:= PGtkCellRendererCombo(GTK_CHECK_CAST(obj, GTK_TYPE_CELL_RENDERER_COMBO));
    end;
    
    function GTK_CELL_RENDERER_COMBO_CLASS(klass: pointer): PGtkCellRendererComboClass;
    begin
      Result:= PGtkCellRendererComboClass(GTK_CHECK_CLASS_CAST(klass, GTK_TYPE_CELL_RENDERER_COMBO));
    end;
     
    function GTK_IS_CELL_RENDERER_COMBO(obj: pointer): boolean;
    begin
      Result:= GTK_CHECK_TYPE(obj, GTK_TYPE_CELL_RENDERER_COMBO);
    end;
    
    function GTK_IS_CELL_RENDERER_COMBO_CLASS(klass: pointer):boolean;
    begin
      Result:= GTK_CHECK_CLASS_TYPE(klass, GTK_TYPE_CELL_RENDERER_COMBO);
    end;
    
    function GTK_CELL_RENDERER_COMBO_GET_CLASS(obj: pointer): PGtkCellRendererComboClass;
    begin
      Result:= PGtkCellRendererComboClass(GTK_CHECK_GET_CLASS(obj, GTK_TYPE_CELL_RENDERER_COMBO));
    end;


{$ENDIF read_implementation}
// included by gtk2.pas

