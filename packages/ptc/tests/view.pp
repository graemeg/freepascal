{$MODE objfpc}

Uses
  SysUtils, ptc;

Var
  console : TPTCConsole;
  surface : TPTCSurface;
  format : TPTCFormat;
  pixels : Pint32;
  width, height : Integer;
  I : Integer;
  F : File;

Begin
  Try
    console := TPTCConsole.Create;

    format := TPTCFormat.Create(24, $00FF0000, $0000FF00, $000000FF);
    surface := TPTCSurface.Create(320, 200, format);

    console.open('Random example', surface.width, surface.height, format);

    format.Free;
    
    For I := 1 To 100 Do
    Begin
      Writeln('test', I, '.raw');
      ASSign(F, 'test' + IntToStr(I) + '.raw');
      Reset(F, 1);
      BlockRead(F, surface.lock^, surface.height * surface.pitch);
      surface.unlock;
      Close(F);
      surface.copy(console);
      console.update;
      console.read.Free;
    End;

    console.close;
    console.Free;
    surface.Free;
  Except
    On error : TPTCError Do
      { report error }
      error.report;
  End;
End.
