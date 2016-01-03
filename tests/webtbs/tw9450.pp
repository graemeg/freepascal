<<<<<<< HEAD
<<<<<<< HEAD
{ %skiptarget=android }

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{ This test checks whether the 'extradefines' from compiler/system/i_*.pas   }
{ are defined before the configuration file is parsed, together with tw9450a }

{$i+}
var
  t: text;
begin
  assign(t,'tw9450a.cfg');
  rewrite(t);
  writeln(t,'-vw');
{$ifdef unix}
  writeln(t,'#ifndef unix');
  writeln(t,'#error Unix not defined in config file');
  writeln(t,'#endif');
{$else}
{$ifdef windows}
  writeln(t,'#ifndef windows');
  writeln(t,'#error Windows not defined in config file');
  writeln(t,'#endif');
{$else}
{$ifdef dpmi}
  writeln(t,'#ifndef dpmi');
  writeln(t,'#error DPMI not defined in config file');
  writeln(t,'#endif');
{$endif}
{$endif}
{$endif}
  close(t);
end.
