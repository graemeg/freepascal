program Math1;

{$mode delphi}{$H+}

uses
 {$IFDEF UNIX}
 cthreads,
 {$ENDIF}
 Classes,
 Math
 { you can add units after this };

var x:double;
begin
 SetExceptionMask([exInvalidOp,exDenormalized,exZeroDivide,exOverflow,exUnderflow,exPrecision]);
 x:=0;
 writeln('ln(x)');
 writeln(ln(x));
 writeln('1/x');
 writeln(1/x);
end.
