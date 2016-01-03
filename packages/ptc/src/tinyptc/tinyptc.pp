{todo: handle exceptions}

Unit TinyPTC;

{$MODE objfpc}

Interface

Function ptc_open(title : String; width, height : Integer) : Boolean;
Function ptc_update(buffer : Pointer) : Boolean;
Procedure ptc_close;

Implementation

Uses
  ptc;

Var
  console : TPTCConsole;
  format : TPTCFormat;
  palette : TPTCPalette;
  w, h : Integer;

Function ptc_open(title : String; width, height : Integer) : Boolean;

Begin
  If console = Nil Then
    console := TPTCConsole.Create;
  If format = Nil Then
    format := TPTCFormat.Create(32, $FF0000, $FF00, $FF);
  If palette = Nil Then
    palette := TPTCPalette.Create;
  console.open(title, width, height, format);
  w := width;
  h := height;
  ptc_open := True;
End;

Function ptc_update(buffer : Pointer) : Boolean;

Begin
  console.load(buffer, w, h, w*4, format, palette);
  ptc_update := True;
End;

Procedure ptc_close;

Begin
  If console <> Nil Then
    console.close;
  FreeAndNil(console);
  FreeAndNil(format);
  FreeAndNil(palette);
End;

Initialization
  console := Nil;
Finalization
  ptc_close;
End.
