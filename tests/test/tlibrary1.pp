{ %NORUN }
{ %SKIPTARGET=macos,go32v2 }

{$ifdef darwin}
{$PIC+}
{$endif darwin}

{$ifdef darwin}
{$PIC+}
{$endif darwin}

{$ifdef darwin}
{$PIC+}
{$endif darwin}

{$ifdef CPUX86_64}
{$ifndef WINDOWS}
{$PIC+}
{$endif WINDOWS}
{$endif CPUX86_64}

{ The .so of the library needs to be in the current dir when
  testing the loading at runtime }

{$ifdef mswindows}
 {$define supported}
 {$define supportidx}
{$endif win32}
{$ifdef Unix}
 {$define supported}
{$endif Unix}
{$ifndef fpc}
   {$define supported}
{$endif}

{$ifdef supported}

library bug;

const
   publicname='TestName';
   publicindex = 1234;

procedure Test;export;

 begin
<<<<<<< HEAD
<<<<<<< HEAD
   if not islibrary then
     halt(1);
   if not moduleislib then
     halt(2);
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
   writeln('Hoi');
 end;

exports
  Test name publicname;
{$ifdef supportidx}
exports
  Test index publicindex;
{$endif}

begin
end.
{$else supported}
begin
  Writeln('No library for that target');
end.
{$endif supported}
