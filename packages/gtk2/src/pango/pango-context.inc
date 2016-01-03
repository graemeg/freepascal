// included by pango.pp
{$IFDEF read_interface_types}

{$ENDIF}

//------------------------------------------------------------------------------

{$IFDEF read_interface_functions}

{ Sort of like a GC - application set information about how
   to handle scripts
  }
{ PangoContext typedefed in pango-fontmap.h  }

function PANGO_TYPE_CONTEXT : GType;
function PANGO_CONTEXT(anObject : pointer) : PPangoContext;
function PANGO_CONTEXT_CLASS(klass : pointer) : PPangoContextClass;
function PANGO_IS_CONTEXT(anObject : pointer) : boolean;
function PANGO_IS_CONTEXT_CLASS(klass : pointer) : boolean;
function PANGO_CONTEXT_GET_CLASS(obj : PPangoContext) : PPangoContextClass;

{ The PangoContext and PangoContextClass structs are private; if you
   need to create a subclass of these, mail otaylor@redhat.com
  }
function pango_context_get_type:GType; cdecl; external pangolib;
{$ifdef PANGO_ENABLE_BACKEND}

function pango_context_new:PPangoContext; cdecl; external pangolib;
procedure pango_context_set_font_map(context:PPangoContext; font_map:PPangoFontMap); cdecl; external pangolib;
{$endif}
{ PANGO_ENABLE_BACKEND  }

procedure pango_context_list_families(context:PPangoContext; families:PPPPangoFontFamily; n_families:Plongint); cdecl; external pangolib;
function pango_context_load_font(context:PPangoContext; desc:PPangoFontDescription):PPangoFont; cdecl; external pangolib;
function pango_context_load_fontset(context:PPangoContext; desc:PPangoFontDescription; language:PPangoLanguage):PPangoFontset; cdecl; external pangolib;
function pango_context_get_metrics(context:PPangoContext; desc:PPangoFontDescription; language:PPangoLanguage):PPangoFontMetrics; cdecl; external pangolib;
procedure pango_context_set_font_description(context:PPangoContext; desc:PPangoFontDescription); cdecl; external pangolib;
function pango_context_get_font_description(context:PPangoContext):PPangoFontDescription; cdecl; external pangolib;
function pango_context_get_language(context:PPangoContext):PPangoLanguage; cdecl; external pangolib;
procedure pango_context_set_language(context:PPangoContext; language:PPangoLanguage); cdecl; external pangolib;
procedure pango_context_set_base_dir(context:PPangoContext; direction:TPangoDirection); cdecl; external pangolib;
function pango_context_get_base_dir(context:PPangoContext):TPangoDirection; cdecl; external pangolib;

{ Break a string of Unicode characters into segments with
   consistent shaping/language engine and bidrectional level.
   Returns a GList of PangoItem's
  }
function pango_itemize(context:PPangoContext; text:Pchar; start_index:longint; length:longint; attrs:PPangoAttrList;
           cached_iter:PPangoAttrIterator):PGList; cdecl; external pangolib;
{$endif read_interface_functions}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}

function PANGO_TYPE_CONTEXT : GType;
begin
  PANGO_TYPE_CONTEXT:=pango_context_get_type;
end;

function PANGO_CONTEXT(anObject : pointer) : PPangoContext;
begin
  PANGO_CONTEXT:=PPangoContext(G_TYPE_CHECK_INSTANCE_CAST(anObject,
                                                          PANGO_TYPE_CONTEXT));
end;

function PANGO_CONTEXT_CLASS(klass : pointer) : PPangoContextClass;
begin
  PANGO_CONTEXT_CLASS:=PPangoContextClass(G_TYPE_CHECK_CLASS_CAST(klass,
                                          PANGO_TYPE_CONTEXT));
end;

function PANGO_IS_CONTEXT(anObject : pointer) : boolean;
begin
  PANGO_IS_CONTEXT:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,PANGO_TYPE_CONTEXT);
end;

function PANGO_IS_CONTEXT_CLASS(klass : pointer) : boolean;
begin
  PANGO_IS_CONTEXT_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,PANGO_TYPE_CONTEXT);
end;

function PANGO_CONTEXT_GET_CLASS(obj : PPangoContext) : PPangoContextClass;
begin
  PANGO_CONTEXT_GET_CLASS:=PPangoContextClass(G_TYPE_INSTANCE_GET_CLASS(obj,
                                              PANGO_TYPE_CONTEXT));
end;
{$ENDIF read_implementation}

