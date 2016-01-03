// included by pango.pp
{$IFDEF read_interface_types}
   PPangoLanguage = ^TPangoLanguage;
   TPangoLanguage = pointer;

   { An index of a glyph into a font. Rendering system dependent }
   PPangoGlyph = ^TPangoGlyph;
   TPangoGlyph = guint32;

   { A rectangle. Used to store logical and physical extents of glyphs,
     runs, strings, etc.
   }
   PPangoRectangle = ^TPangoRectangle;
   TPangoRectangle = record
        x : longint;
        y : longint;
        width : longint;
        height : longint;
     end;

   PPangoDirection = ^TPangoDirection;
   TPangoDirection = (
     PANGO_DIRECTION_LTR,
     PANGO_DIRECTION_RTL,
     PANGO_DIRECTION_TTB_LTR,
     PANGO_DIRECTION_TTB_RTL
   );
{$ENDIF read_interface_types}

//------------------------------------------------------------------------------

{$IFDEF read_interface_functions}
const
  PANGO_SCALE = 1024;

function PANGO_PIXELS(d: integer): integer;

{ Macros to translate from extents rectangles to ascent/descent/lbearing/rbearing
  }
function PANGO_ASCENT(rect : TPangoRectangle) : longint;
function PANGO_DESCENT(rect : TPangoRectangle) : longint;
function PANGO_LBEARING(rect : TPangoRectangle) : longint;
function PANGO_RBEARING(rect : TPangoRectangle) : longint;

{ Information about a segment of text with a consistent
   shaping/language engine and bidirectional level
}
function PANGO_TYPE_LANGUAGE : GType;
function pango_language_get_type:GType; cdecl; external pangolib;
function pango_language_from_string(language:Pchar):PPangoLanguage; cdecl; external pangolib;
function pango_language_to_string(language : PPangoLanguage) : Pchar;

function pango_language_matches(language:PPangoLanguage; range_list:Pchar):gboolean; cdecl; external pangolib;
{$endif read_interface_funtions}

//------------------------------------------------------------------------------

{$IFDEF read_implementation}
function PANGO_ASCENT(rect : TPangoRectangle) : longint;
begin
   PANGO_ASCENT:=-(rect.y);
end;

function PANGO_DESCENT(rect : TPangoRectangle) : longint;
begin
   PANGO_DESCENT:=(rect.y) + (rect.height);
end;

function PANGO_LBEARING(rect : TPangoRectangle) : longint;
begin
   PANGO_LBEARING:=rect.x;
end;

function PANGO_RBEARING(rect : TPangoRectangle) : longint;
begin
   PANGO_RBEARING:=(rect.x) + (rect.width);
end;

function PANGO_TYPE_LANGUAGE : GType;
begin
  PANGO_TYPE_LANGUAGE:=pango_language_get_type;
end;

function pango_language_to_string(language : PPangoLanguage) : Pchar;
begin
  pango_language_to_string:=Pchar(language);
end;

function PANGO_PIXELS(d: integer): integer;
begin
  if d>=0 then
    PANGO_PIXELS:=(d+(PANGO_SCALE div 2)) div PANGO_SCALE
  else
    PANGO_PIXELS:=(d-(PANGO_SCALE div 2)) div PANGO_SCALE;
end;
{$ENDIF}
