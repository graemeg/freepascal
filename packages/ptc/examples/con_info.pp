{
Ported to FPC by Nikolay Nikolov (nickysn@users.sourceforge.net)
}

{
 Info example for OpenPTC 1.0 C++ Implementation
 Copyright (c) Glenn Fiedler (ptc@gaffer.org)
 This source code is in the public domain
}

Program InfoExample;

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

Var
  console : TPTCConsole;

Begin
  console := Nil;
  Try
    Try
      Writeln('[ptc version]');
      { print ptc version string define }
      Writeln(PTC_VERSION);
      Writeln;

      { create console }
      console := TPTCConsole.Create;

      { open the console }
      console.open('Info example');

      { print console data }
      Writeln('[console data]');
      Writeln('name   = ', console.name);
      Writeln('title  = ', console.title);
      Writeln('width  = ', console.width);
      Writeln('height = ', console.height);
      Writeln('pages  = ', console.pages);
      Writeln('pitch  = ', console.pitch);
      Write('format = ');
      print(console.format);
      Writeln;
      Writeln;

      { print console information }
      Writeln('[console information]');
      Writeln(console.information);
    Finally
      console.close;
      console.Free;
    End;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
