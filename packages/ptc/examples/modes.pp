{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Modes example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program ModesExample;

{$MODE objfpc}

Uses
  ptc;

Procedure print(Const format : TPTCFormat);

Begin
  { check format type }
  If format.direct Then
    { check alpha }
    If format.a = 0 Then
      { direct color format without alpha }
      Write('Format(', format.bits:2, ',$', HexStr(format.r, 8), ',$', HexStr(format.g, 8), ',$', HexStr(format.b, 8), ')')
    Else
      { direct color format with alpha }
      Write('Format(', format.bits:2, ',$', HexStr(format.r, 8), ',$', HexStr(format.g, 8), ',$', HexStr(format.b, 8), ',$', HexStr(format.a, 8), ')')
  Else
    { indexed color format }
    Write('Format(', format.bits:2, ')');
End;

Procedure print(Const mode : TPTCMode);

Begin
  { print mode width and height }
  Write(' ', mode.width:4, ' x ', mode.height);
  If mode.height < 1000 Then
    Write(' ');
  If mode.height < 100 Then
    Write(' ');
  If mode.height < 10 Then
    Write(' ');
  Write(' x ');

  { print mode format }
  print(mode.format);

  { newline }
  Writeln;
End;

Var
  console : TPTCConsole;
  modes : PPTCMode;
  index : Integer;

Begin
  console := Nil;
  Try
    Try
      { create console }
      console := TPTCConsole.Create;

      { get list of console modes }
      modes := console.modes;

      { check for empty list }
      If Not modes[0].valid Then
        { the console mode list was empty }
        Writeln('[console mode list is not available]')
      Else
      Begin
        { print mode list header }
        Writeln('[console modes]');

        { mode index }
        index := 0;

        { iterate through all modes }
        While modes[index].valid Do
        Begin
          { print mode }
          print(modes[index]);

          { next mode }
          Inc(index);
        End;
      End;
    Finally
      console.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
