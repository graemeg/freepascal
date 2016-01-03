// included by pango.pp
{$IFDEF read_forward_definitions}
{$ifdef PANGO_ENABLE_BACKEND}
   PPangoFont = ^TPangoFont;
   PPangoFontMetrics = ^TPangoFontMetrics;
{$ENDIF}
{$ENDIF read_forward_definitions}

//------------------------------------------------------------------------------

{$IFDEF read_interface_types}
   PPangoStyle = ^TPangoStyle;
   TPangoStyle = gint;


   PPangoVariant = ^TPangoVariant;
   TPangoVariant = gint;

   PPangoWeight = ^TPangoWeight;
   TPangoWeight = gint;

   PPangoStretch = ^TPangoStretch;
   TPangoStretch = gint;

   PPangoFontMask = ^TPangoFontMask;
   TPangoFontMask = longint; // see below for possible values

{$ifdef PANGO_ENABLE_BACKEND}
   TPangoFontMetrics = record
        ref_count               : guint;
        ascent                  : longint;
        descent                 : longint;
        approximate_char_width  : longint;
        approximate_digit_width : longint;
    end;

   PPangoFontFamily   = ^TPangoFontFamily;
   PPPangoFontFamily  = ^PPangoFontFamily;
   PPPPangoFontFamily = ^PPPangoFontFamily;
   TPangoFontFamily   = record
        parent_instance : TGObject;
     end;

{ Padding for future expansion  }
{}

   PPangoFontFace   = ^TPangoFontFace;
   PPPangoFontFace  = ^PPangoFontFace;
   PPPPangoFontFace = ^PPPangoFontFace;
   TPangoFontFace   = record
        parent_instance : TGObject;
     end;

   TPangoFontFamilyClassListFaces =
     procedure (family:PPangoFontFamily; faces:PPPPangoFontFace;
                                n_faces:Plongint); cdecl;
   TPangoFontFamilyClassGetName =
     function (family:PPangoFontFamily):Pchar; cdecl;

   PPangoFontFamilyClass = ^TPangoFontFamilyClass;
   TPangoFontFamilyClass = record
        parent_class : TGObjectClass;
        list_faces : TPangoFontFamilyClassListFaces;
        get_name : TPangoFontFamilyClassGetName;
        _pango_reserved1 : procedure; cdecl;
        _pango_reserved2 : procedure; cdecl;
        _pango_reserved3 : procedure; cdecl;
        _pango_reserved4 : procedure; cdecl;
     end;

{ Padding for future expansion  }
   PPangoFontFaceClass = ^TPangoFontFaceClass;
   TPangoFontFaceClass = record
        parent_class : TGObjectClass;
        get_face_name : function (face:PPangoFontFace):Pchar; cdecl;
        describe : function (face:PPangoFontFace):PPangoFontDescription; cdecl;
        _pango_reserved1 : procedure ; cdecl;
        _pango_reserved2 : procedure ; cdecl;
        _pango_reserved3 : procedure ; cdecl;
        _pango_reserved4 : procedure ; cdecl;
     end;

   TPangoFont = record
        parent_instance : TGObject;
     end;

   TPangoFontClassDescribe =
     function (font:PPangoFont):PPangoFontDescription; cdecl;
   TPangoFontClassGetCoverage =
     function (font:PPangoFont; lang:PPangoLanguage):PPangoCoverage;
   TPangoFontClassFindShaper =
     function (font:PPangoFont; lang:PPangoLanguage;
               ch:guint32):PPangoEngineShape;
   TPangoFontClassGetGlyphExtents =
     procedure (font:PPangoFont; glyph:TPangoGlyph; ink_rect:PPangoRectangle;
                logical_rect:PPangoRectangle);
   TPangoFontClassGetMetrics =
     function (font:PPangoFont; language:PPangoLanguage):PPangoFontMetrics;

   { Padding for future expansion  }
   PPangoFontClass = ^TPangoFontClass;
   TPangoFontClass = record
        parent_class : TGObjectClass;
        describe : TPangoFontClassDescribe;
        get_coverage : TPangoFontClassGetCoverage;
        find_shaper : TPangoFontClassFindShaper;
        get_glyph_extents : TPangoFontClassGetGlyphExtents;
        get_metrics : TPangoFontClassGetMetrics;
        _pango_reserved1 : procedure ; cdecl;
        _pango_reserved2 : procedure ; cdecl;
        _pango_reserved3 : procedure ; cdecl;
        _pango_reserved4 : procedure ; cdecl;
     end;

{$ENDIF PANGO_ENABLE_BACKEND}

{$ENDIF read_interface_types}

//-----------------------------------------------------------------------------

{$IFDEF read_interface_functions}
const
   // values for TPangoStyle
   PANGO_STYLE_NORMAL  = 0;  // the font is upright
   PANGO_STYLE_OBLIQUE = 1;  // the font is slanted, but in a roman style
   PANGO_STYLE_ITALIC  = 2;  // the font is slanted in an italic style

   //values for TPangoVariant
   PANGO_VARIANT_NORMAL      = 0;
   PANGO_VARIANT_SMALL_CAPS  = 1;

   // values for TPangoWeight
   PANGO_WEIGHT_ULTRALIGHT = 200;
   PANGO_WEIGHT_LIGHT      = 300;
   PANGO_WEIGHT_NORMAL     = 400;
   PANGO_WEIGHT_BOLD       = 700;
   PANGO_WEIGHT_ULTRABOLD  = 800;
   PANGO_WEIGHT_HEAVY      = 900;

   //values for TPangoStretch
   PANGO_STRETCH_ULTRA_CONDENSED = 0;
   PANGO_STRETCH_EXTRA_CONDENSED = 1;
   PANGO_STRETCH_CONDENSED       = 2;
   PANGO_STRETCH_SEMI_CONDENSED  = 3;
   PANGO_STRETCH_NORMAL          = 4;
   PANGO_STRETCH_SEMI_EXPANDED   = 5;
   PANGO_STRETCH_EXPANDED        = 6;
   PANGO_STRETCH_EXTRA_EXPANDED  = 7;
   PANGO_STRETCH_ULTRA_EXPANDED  = 8;


   // values for TPangoFontMask
   PANGO_FONT_MASK_FAMILY  = 1 shl 0;
   PANGO_FONT_MASK_STYLE   = 1 shl 1;
   PANGO_FONT_MASK_VARIANT = 1 shl 2;
   PANGO_FONT_MASK_WEIGHT  = 1 shl 3;
   PANGO_FONT_MASK_STRETCH = 1 shl 4;
   PANGO_FONT_MASK_SIZE    = 1 shl 5;

{ CSS scale factors (1.2 factor between each size)  }
   PANGO_SCALE_XX_SMALL = 0.5787037037037;
   PANGO_SCALE_X_SMALL  = 0.6444444444444;
   PANGO_SCALE_SMALL    = 0.8333333333333;
   PANGO_SCALE_MEDIUM   = 1.0;
   PANGO_SCALE_LARGE    = 1.2;
   PANGO_SCALE_X_LARGE  = 1.4399999999999;
   PANGO_SCALE_XX_LARGE = 1.728;

  {
   PangoFontDescription
  }

function PANGO_TYPE_FONT_DESCRIPTION : GType;
function pango_font_description_get_type:GType; cdecl; external pangolib;
function pango_font_description_new:PPangoFontDescription; cdecl; external pangolib;
function pango_font_description_copy(desc:PPangoFontDescription):PPangoFontDescription; cdecl; external pangolib;
function pango_font_description_copy_static(desc:PPangoFontDescription):PPangoFontDescription; cdecl; external pangolib;
function pango_font_description_hash(desc:PPangoFontDescription):guint; cdecl; external pangolib;
function pango_font_description_equal(desc1:PPangoFontDescription; desc2:PPangoFontDescription):gboolean; cdecl; external pangolib;
procedure pango_font_description_free(desc:PPangoFontDescription); cdecl; external pangolib;
procedure pango_font_descriptions_free(var descs:PPangoFontDescription; n_descs:longint); cdecl; external pangolib;
procedure pango_font_description_set_family(desc:PPangoFontDescription; family:Pchar); cdecl; external pangolib;
procedure pango_font_description_set_family_static(desc:PPangoFontDescription; family:Pchar); cdecl; external pangolib;
function pango_font_description_get_family(desc:PPangoFontDescription):Pchar; cdecl; external pangolib;
procedure pango_font_description_set_style(desc:PPangoFontDescription; style:TPangoStyle); cdecl; external pangolib;
function pango_font_description_get_style(desc:PPangoFontDescription):TPangoStyle; cdecl; external pangolib;
procedure pango_font_description_set_variant(desc:PPangoFontDescription; variant:TPangoVariant); cdecl; external pangolib;
function pango_font_description_get_variant(desc:PPangoFontDescription):TPangoVariant; cdecl; external pangolib;
procedure pango_font_description_set_weight(desc:PPangoFontDescription; weight:TPangoWeight); cdecl; external pangolib;
function pango_font_description_get_weight(desc:PPangoFontDescription):TPangoWeight; cdecl; external pangolib;
procedure pango_font_description_set_stretch(desc:PPangoFontDescription; stretch:TPangoStretch); cdecl; external pangolib;
function pango_font_description_get_stretch(desc:PPangoFontDescription):TPangoStretch; cdecl; external pangolib;
procedure pango_font_description_set_size(desc:PPangoFontDescription; size:gint); cdecl; external pangolib;
function pango_font_description_get_size(desc:PPangoFontDescription):gint; cdecl; external pangolib;
procedure pango_font_description_set_absolute_size(desc:PPangoFontDescription;size:double); cdecl; external pangolib;
function pango_font_description_get_size_is_absolute (desc:PPangoFontDescription):gboolean; cdecl; external pangolib;
function pango_font_description_get_set_fields(desc:PPangoFontDescription):TPangoFontMask; cdecl; external pangolib;
procedure pango_font_description_unset_fields(desc:PPangoFontDescription; to_unset:TPangoFontMask); cdecl; external pangolib;
procedure pango_font_description_merge(desc:PPangoFontDescription; desc_to_merge:PPangoFontDescription; replace_existing:gboolean); cdecl; external pangolib;
procedure pango_font_description_merge_static(desc:PPangoFontDescription; desc_to_merge:PPangoFontDescription; replace_existing:gboolean); cdecl; external pangolib;
function pango_font_description_better_match(desc:PPangoFontDescription; old_match:PPangoFontDescription; new_match:PPangoFontDescription):gboolean; cdecl; external pangolib;
function pango_font_description_from_string(str:Pchar):PPangoFontDescription; cdecl; external pangolib;
function pango_font_description_to_string(desc:PPangoFontDescription):Pchar; cdecl; external pangolib;
function pango_font_description_to_filename(desc:PPangoFontDescription):Pchar; cdecl; external pangolib;
{
   PangoFontMetrics
}
function PANGO_TYPE_FONT_METRICS:GType;
function pango_font_metrics_get_type:GType; cdecl; external pangolib;
function pango_font_metrics_ref(metrics:PPangoFontMetrics):PPangoFontMetrics; cdecl; external pangolib;
procedure pango_font_metrics_unref(metrics:PPangoFontMetrics); cdecl; external pangolib;
function pango_font_metrics_get_ascent(metrics:PPangoFontMetrics):longint; cdecl; external pangolib;
function pango_font_metrics_get_descent(metrics:PPangoFontMetrics):longint; cdecl; external pangolib;
function pango_font_metrics_get_approximate_char_width(metrics:PPangoFontMetrics):longint; cdecl; external pangolib;
function pango_font_metrics_get_approximate_digit_width(metrics:PPangoFontMetrics):longint; cdecl; external pangolib;


{$ifdef PANGO_ENABLE_BACKEND}

function pango_font_metrics_new:PPangoFontMetrics; cdecl; external pangolib;

{$endif PANGO_ENABLE_BACKEND}
{ PANGO_ENABLE_BACKEND  }
{
   PangoFontFamily
}

function PANGO_TYPE_FONT_FAMILY: GType;
function PANGO_FONT_FAMILY(anObject: Pointer): PPangoFontFamily;
function PANGO_IS_FONT_FAMILY(anObject : Pointer) : boolean;

function pango_font_family_get_type:GType; cdecl; external pangolib;
procedure pango_font_family_list_faces(family:PPangoFontFamily;
                     faces:PPPPangoFontFace; n_faces:Plongint); cdecl; external pangolib;
function pango_font_family_get_name(family:PPangoFontFamily):Pchar; cdecl; external pangolib;
<<<<<<< HEAD
<<<<<<< HEAD
function pango_font_family_is_monospace(family:PPangoFontFamily):gboolean; cdecl; external pangolib;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifdef PANGO_ENABLE_BACKEND}
function PANGO_FONT_FAMILY_CLASS(klass : pointer) : PPangoFontFamilyClass;
function PANGO_IS_FONT_FAMILY_CLASS(klass : pointer) : boolean;
function PANGO_FONT_FAMILY_GET_CLASS(obj : PPangoFontFamily) : PPangoFontFamilyClass;
{$endif}
{ PANGO_ENABLE_BACKEND  }


{
   PangoFontFace
}
function PANGO_TYPE_FONT_FACE : GType;
function PANGO_FONT_FACE(anObject : pointer) : PPangoFontFace;
function PANGO_IS_FONT_FACE(anObject : pointer) : boolean;

function pango_font_face_get_type:GType; cdecl; external pangolib;
function pango_font_face_describe(face:PPangoFontFace):PPangoFontDescription; cdecl; external pangolib;
function pango_font_face_get_face_name(face:PPangoFontFace):Pchar; cdecl; external pangolib;
{$ifdef PANGO_ENABLE_BACKEND}
function PANGO_FONT_FACE_CLASS(klass : pointer) : PPangoFontFaceClass;
function PANGO_IS_FONT_FACE_CLASS(klass : pointer) : boolean;
function PANGO_FONT_FACE_GET_CLASS(obj : pointer) : PPangoFontFaceClass;
{$endif}
{ PANGO_ENABLE_BACKEND  }

{
   PangoFont
}
function PANGO_TYPE_FONT : GType;
function PANGO_FONT(anObject : pointer) : PPangoFont;
function PANGO_IS_FONT(anObject : pointer) : boolean;

function pango_font_get_type:GType; cdecl; external pangolib;
function pango_font_describe(font:PPangoFont):PPangoFontDescription; cdecl; external pangolib;
function pango_font_get_coverage(font:PPangoFont; language:PPangoLanguage):PPangoCoverage; cdecl; external pangolib;
function pango_font_find_shaper(font:PPangoFont; language:PPangoLanguage; ch:guint32):PPangoEngineShape; cdecl; external pangolib;
function pango_font_get_metrics(font:PPangoFont; language:PPangoLanguage):PPangoFontMetrics; cdecl; external pangolib;
procedure pango_font_get_glyph_extents(font:PPangoFont; glyph:TPangoGlyph; ink_rect:PPangoRectangle; logical_rect:PPangoRectangle); cdecl; external pangolib;

{$ifdef PANGO_ENABLE_BACKEND}
function PANGO_FONT_CLASS(klass : pointer) : PPangoFontClass;
function PANGO_IS_FONT_CLASS(klass : pointer) : boolean;
function PANGO_FONT_GET_CLASS(obj : PPangoFont) : PPangoFontClass;
{$endif}
{ PANGO_ENABLE_BACKEND  }

{$ENDIF read_interface_functions}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}

function PANGO_TYPE_FONT_DESCRIPTION : GType;
begin
  PANGO_TYPE_FONT_DESCRIPTION:=pango_font_description_get_type;
end;

function PANGO_TYPE_FONT_METRICS : GType;
begin
  PANGO_TYPE_FONT_METRICS:=pango_font_metrics_get_type;
end;

function PANGO_TYPE_FONT_FAMILY : GType;
begin
  PANGO_TYPE_FONT_FAMILY:=pango_font_family_get_type;
end;

function PANGO_FONT_FAMILY(anObject : pointer) : PPangoFontFamily;
begin
  PANGO_FONT_FAMILY:=PPangoFontFamily(G_TYPE_CHECK_INSTANCE_CAST(anObject,
                                                       PANGO_TYPE_FONT_FAMILY));
end;

function PANGO_IS_FONT_FAMILY(anObject : Pointer) : boolean;
begin
  PANGO_IS_FONT_FAMILY:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,
                                                   PANGO_TYPE_FONT_FAMILY);
end;

function PANGO_FONT_FAMILY_CLASS(klass : Pointer) : PPangoFontFamilyClass;
begin
  PANGO_FONT_FAMILY_CLASS:=PPangoFontFamilyClass(G_TYPE_CHECK_CLASS_CAST(klass,
                                                       PANGO_TYPE_FONT_FAMILY));
end;

function PANGO_IS_FONT_FAMILY_CLASS(klass : Pointer) : boolean;
begin
  PANGO_IS_FONT_FAMILY_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,
                                                        PANGO_TYPE_FONT_FAMILY);
end;

function PANGO_FONT_FAMILY_GET_CLASS(obj : PPangoFontFamily) : PPangoFontFamilyClass;
begin
  PANGO_FONT_FAMILY_GET_CLASS:=PPangoFontFamilyClass(G_TYPE_INSTANCE_GET_CLASS(
                                                   obj,PANGO_TYPE_FONT_FAMILY));
end;

function PANGO_TYPE_FONT_FACE : GType;
begin
  PANGO_TYPE_FONT_FACE:=pango_font_face_get_type;
end;

function PANGO_FONT_FACE(anObject : Pointer) : PPangoFontFace;
begin
  PANGO_FONT_FACE:=PPangoFontFace(G_TYPE_CHECK_INSTANCE_CAST(anObject,
                                                         PANGO_TYPE_FONT_FACE));
end;

function PANGO_IS_FONT_FACE(anObject : Pointer) : boolean;
begin
  PANGO_IS_FONT_FACE:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,PANGO_TYPE_FONT_FACE);
end;

function PANGO_FONT_FACE_CLASS(klass : Pointer) : PPangoFontFaceClass;
begin
  PANGO_FONT_FACE_CLASS:=PPangoFontFaceClass(G_TYPE_CHECK_CLASS_CAST(klass,
                                                         PANGO_TYPE_FONT_FACE));
end;

function PANGO_IS_FONT_FACE_CLASS(klass : Pointer) : boolean;
begin
  PANGO_IS_FONT_FACE_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,PANGO_TYPE_FONT_FACE);
end;

function PANGO_FONT_FACE_GET_CLASS(obj : Pointer) : PPangoFontFaceClass;
begin
  PANGO_FONT_FACE_GET_CLASS:=PPangoFontFaceClass(G_TYPE_INSTANCE_GET_CLASS(obj,
                                                         PANGO_TYPE_FONT_FACE));
end;

function PANGO_TYPE_FONT : GType;
begin
  PANGO_TYPE_FONT:=pango_font_get_type;
end;

function PANGO_FONT(anObject : Pointer) : PPangoFont;
begin
  PANGO_FONT:=PPangoFont(G_TYPE_CHECK_INSTANCE_CAST(anObject,PANGO_TYPE_FONT));
end;

function PANGO_IS_FONT(anObject : Pointer) : boolean;
begin
  PANGO_IS_FONT:=G_TYPE_CHECK_INSTANCE_TYPE(anObject,PANGO_TYPE_FONT);
end;

function PANGO_FONT_CLASS(klass : Pointer) : PPangoFontClass;
begin
  PANGO_FONT_CLASS:=PPangoFontClass(G_TYPE_CHECK_CLASS_CAST(klass,
                                                            PANGO_TYPE_FONT));
end;

function PANGO_IS_FONT_CLASS(klass : Pointer) : boolean;
begin
  PANGO_IS_FONT_CLASS:=G_TYPE_CHECK_CLASS_TYPE(klass,PANGO_TYPE_FONT);
end;

function PANGO_FONT_GET_CLASS(obj : PPangoFont) : PPangoFontClass;
begin
  PANGO_FONT_GET_CLASS:=PPangoFontClass(G_TYPE_INSTANCE_GET_CLASS(obj,
                                                              PANGO_TYPE_FONT));
end;

{$ENDIF read_implementation}

