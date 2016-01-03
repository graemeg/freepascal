{ %norun }

uses
  {$ifdef unix}
<<<<<<< HEAD
<<<<<<< HEAD
  {$ifdef darwin}iosxwstr{$else}cwstring{$endif},
=======
  cwstring,
>>>>>>> graemeg/fixes_2_2
=======
  cwstring,
>>>>>>> origin/fixes_2_2
  {$endif}
  sysutils;

{ just to make sure that no all wide->shortstring compile time conversions }
{ fail, but only those resulting in data loss                              }
const
  cw = widestring('abc');
  de = 'a'+shortstring(cw);
  wc = widechar('a');
  df = shortstring(wc)+'abcd';
  dg = char(wc)+'abcd';

begin
end.
