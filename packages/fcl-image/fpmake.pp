{$ifndef ALLPACKAGES}
{$mode objfpc}{$H+}
program fpmake;

uses fpmkunit;

Var
  T : TTarget;
  P : TPackage;
begin
  With Installer do
    begin
{$endif ALLPACKAGES}

    P:=AddPackage('fcl-image');
<<<<<<< HEAD
<<<<<<< HEAD
    P.ShortName:='fcli';
{$ifdef ALLPACKAGES}
    P.Directory:=ADirectory;
{$endif ALLPACKAGES}
    P.Version:='3.1.1';
    P.Dependencies.Add('pasjpeg');
    P.Dependencies.Add('hash');
    P.Dependencies.Add('paszlib');
    P.Dependencies.Add('fcl-base');

    P.Author := 'Michael Van Canneyt of the Free Pascal development team';
    P.License := 'LGPL with modification, ';
    P.HomepageURL := 'www.freepascal.org';
    P.Email := '';
    P.Description := 'Image loading and conversion parts of Free Component Libraries (FCL), FPC''s OOP library.';
    P.NeedLibC:= false;
    P.OSes := P.OSes - [embedded,nativent,msdos,win16];

=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef ALLPACKAGES}
    P.Directory:='fcl-image';
{$endif ALLPACKAGES}
    P.Version:='2.2.4';

    P.Dependencies.Add('pasjpeg');
    P.Dependencies.Add('paszlib');
    P.Dependencies.Add('fcl-base');

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    P.SourcePath.Add('src');
    P.IncludePath.Add('src');

    T:=P.Targets.AddUnit('bmpcomn.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimgcmn');
        end;
    T:=P.Targets.AddUnit('fptiffcmn.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('clipping.pp');
    T:=P.Targets.AddUnit('ellipses.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpcanvas');
        end;
    T:=P.Targets.AddUnit('extinterpolation.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpcanvas');
        end;
    T:=P.Targets.AddUnit('fpcanvas.pp');
      with T.Dependencies do
        begin
          AddInclude('fphelper.inc');
          AddInclude('fpfont.inc');
          AddInclude('fppen.inc');
          AddInclude('fpbrush.inc');
          AddInclude('fpinterpolation.inc');
          AddInclude('fpcanvas.inc');
          AddInclude('fpcdrawh.inc');
          AddUnit('fpimage');
          AddUnit('clipping');
        end;
    T:=P.Targets.AddUnit('fpcolhash.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('fpditherer.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpcolhash');
        end;
    T:=P.Targets.AddUnit('fpimage.pp');
      with T.Dependencies do
        begin
          AddInclude('fpcolors.inc');
          AddInclude('fpimage.inc');
          AddInclude('fphandler.inc');
          AddInclude('fppalette.inc');
          AddInclude('fpcolcnv.inc');
<<<<<<< HEAD
<<<<<<< HEAD
          AddInclude('fpcompactimg.inc');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
        end;
    T:=P.Targets.AddUnit('fpimgcanv.pp');
      with T.Dependencies do
        begin
          AddUnit('fppixlcanv');
          AddUnit('fpimage');
          AddUnit('clipping');
        end;
    T:=P.Targets.AddUnit('fpimgcmn.pp');
    T:=P.Targets.AddUnit('fppixlcanv.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpcanvas');
          AddUnit('pixtools');
          AddUnit('ellipses');
          AddUnit('clipping');
        end;
    T:=P.Targets.AddUnit('fpquantizer.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpcolhash');
        end;
    T:=P.Targets.AddUnit('fpreadbmp.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('bmpcomn');
        end;
    T:=P.Targets.AddUnit('fpreadjpeg.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('fpreadpcx.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('pcxcomn');
        end;
    T:=P.Targets.AddUnit('fpreadpng.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpimgcmn');
          AddUnit('pngcomn');
        end;
    T:=P.Targets.AddUnit('fpreadpnm.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('fpreadtga.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('targacmn');
        end;
    T:=P.Targets.AddUnit('fpreadtiff.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fptiffcmn');
        end;
    T:=P.Targets.AddUnit('fpreadxpm.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpreadgif.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('fpreadpsd.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('xwdfile.pp');
    T:=P.Targets.AddUnit('fpreadxwd.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('xwdfile');
        end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
    T:=P.Targets.AddUnit('fpwritebmp.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('bmpcomn');
        end;
    T:=P.Targets.AddUnit('fpwritejpeg.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpreadjpeg');
        end;
    T:=P.Targets.AddUnit('fpwritepcx.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('pcxcomn');
        end;
    T:=P.Targets.AddUnit('fpwritepng.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpimgcmn');
          AddUnit('pngcomn');
        end;
    T:=P.Targets.AddUnit('fpwritepnm.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('fpwritetga.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('targacmn');
        end;
    T:=P.Targets.AddUnit('fpwritetiff.pas');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fptiffcmn');
        end;
    T:=P.Targets.AddUnit('fpwritexpm.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('freetypeh.pp',[solaris,iphonesim,darwin,freebsd,openbsd,netbsd,linux,haiku,beos,win32,win64,aix,dragonfly]);
    T:=P.Targets.AddUnit('freetype.pp',[solaris,iphonesim,darwin,freebsd,openbsd,netbsd,linux,haiku,beos,win32,win64,aix,dragonfly]);
=======
    T:=P.Targets.AddUnit('freetypeh.pp');
    T:=P.Targets.AddUnit('freetype.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('freetypeh.pp');
    T:=P.Targets.AddUnit('freetype.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('freetypeh');
          AddUnit('fpimgcmn');
        end;
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('ftfont.pp',[solaris,iphonesim,darwin,freebsd,openbsd,netbsd,linux,haiku,beos,win32,win64,aix,dragonfly]);
=======
    T:=P.Targets.AddUnit('ftfont.pp');
>>>>>>> graemeg/fixes_2_2
=======
    T:=P.Targets.AddUnit('ftfont.pp');
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpcanvas');
          AddUnit('fpimgcmn');
          AddUnit('freetype');
          AddUnit('freetypeh');
          AddUnit('fpimage');
        end;
    T:=P.Targets.AddUnit('pcxcomn.pas');
    T:=P.Targets.AddUnit('pixtools.pp');
      with T.Dependencies do
        begin
          AddUnit('fpcanvas');
          AddUnit('fpimage');
          AddUnit('clipping');
          AddUnit('ellipses');
        end;
    T:=P.Targets.AddUnit('pngcomn.pp');
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpimgcmn');
        end;
    T:=P.Targets.AddUnit('pscanvas.pp');
<<<<<<< HEAD
<<<<<<< HEAD
      T.ResourceStrings:=true;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
      with T.Dependencies do
        begin
          AddUnit('fpimage');
          AddUnit('fpcanvas');
        end;
    T:=P.Targets.AddUnit('targacmn.pp');
<<<<<<< HEAD
<<<<<<< HEAD
    T:=P.Targets.AddUnit('fpimggauss.pp');
    With T.Dependencies do
      AddUnit('fpimage');

    P.ExamplePath.Add('examples');
    T:=P.Targets.AddExampleProgram('drawing.pp');
    T:=P.Targets.AddExampleProgram('imgconv.pp');
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

{$ifndef ALLPACKAGES}
    Run;
    end;
end.
{$endif ALLPACKAGES}
<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
