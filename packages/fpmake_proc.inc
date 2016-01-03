procedure add_a52(const ADirectory: string);
begin
  with Installer do
{$include a52/fpmake.pp}
end;

procedure add_ami_extra(const ADirectory: string);
begin
  with Installer do
{$include ami-extra/fpmake.pp}
end;

procedure add_amunits(const ADirectory: string);
begin
  with Installer do
{$include amunits/fpmake.pp}
end;

procedure add_arosunits(const ADirectory: string);
begin
  with Installer do
{$include arosunits/fpmake.pp}
end;

procedure add_aspell(const ADirectory: string);
begin
  with Installer do
{$include aspell/fpmake.pp}
end;

procedure add_bfd(const ADirectory: string);
begin
  with Installer do
{$include bfd/fpmake.pp}
end;

procedure add_bzip2(const ADirectory: string);
begin
  with Installer do
{$include bzip2/fpmake.pp}
end;

procedure add_cairo(const ADirectory: string);
begin
  with Installer do
{$include cairo/fpmake.pp}
end;

procedure add_cdrom(const ADirectory: string);
begin
  with Installer do
{$include cdrom/fpmake.pp}
end;

procedure add_chm(const ADirectory: string);
begin
  with Installer do
{$include chm/fpmake.pp}
end;

procedure add_cocoaint(const ADirectory: string);
begin
  with Installer do
{$include cocoaint/fpmake.pp}
end;

{$include dblib/fpmake.pp}

procedure add_dbus(const ADirectory: string);
begin
  with Installer do
{$include dbus/fpmake.pp}
end;

procedure add_dts(const ADirectory: string);
begin
  with Installer do
{$include dts/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_fastcgi(const ADirectory: string);
=======
procedure add_fastcgi;
>>>>>>> graemeg/cpstrnew
=======
procedure add_fastcgi;
>>>>>>> graemeg/cpstrnew
=======
procedure add_fastcgi;
>>>>>>> graemeg/cpstrnew
=======
procedure add_fastcgi;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include fastcgi/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_fcl_async(const ADirectory: string);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure add_fcl_async;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include fcl-async/fpmake.pp}
end;

procedure add_fcl_base(const ADirectory: string);
begin
  with Installer do
{$include fcl-base/fpmake.pp}
end;

{$include fcl-db/fpmake.pp}

procedure add_fcl_extra(const ADirectory: string);
begin
  with Installer do
{$include fcl-extra/fpmake.pp}
end;

<<<<<<< HEAD
procedure add_fcl_fpcunit(const ADirectory: string);
=======
procedure add_fcl_extra;
begin
  with Installer do
{$include fcl-extra/fpmake.pp}
end;

procedure add_fcl_extra;
begin
  with Installer do
{$include fcl-extra/fpmake.pp}
end;

procedure add_fcl_extra;
begin
  with Installer do
{$include fcl-extra/fpmake.pp}
end;

procedure add_fcl_extra;
begin
  with Installer do
{$include fcl-extra/fpmake.pp}
end;

procedure add_fcl_fpcunit;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include fcl-fpcunit/fpmake.pp}
end;

procedure add_fcl_image(const ADirectory: string);
begin
  with Installer do
{$include fcl-image/fpmake.pp}
end;

procedure add_fcl_js(const ADirectory: string);
begin
  with Installer do
{$include fcl-js/fpmake.pp}
end;

procedure add_fcl_json(const ADirectory: string);
begin
  with Installer do
{$include fcl-json/fpmake.pp}
end;

procedure add_fcl_net(const ADirectory: string);
begin
  with Installer do
{$include fcl-net/fpmake.pp}
end;

procedure add_fcl_passrc(const ADirectory: string);
begin
  with Installer do
{$include fcl-passrc/fpmake.pp}
end;

procedure add_fcl_process(const ADirectory: string);
begin
  with Installer do
{$include fcl-process/fpmake.pp}
end;

procedure add_fcl_registry(const ADirectory: string);
begin
  with Installer do
{$include fcl-registry/fpmake.pp}
end;

procedure add_fcl_res(const ADirectory: string);
begin
  with Installer do
{$include fcl-res/fpmake.pp}
end;

procedure add_fcl_sdo(const ADirectory: string);
begin
  with Installer do
{$include fcl-sdo/fpmake.pp}
end;

procedure add_fcl_sound(const ADirectory: string);
begin
  with Installer do
{$include fcl-sound/fpmake.pp}
end;

procedure add_fcl_stl(const ADirectory: string);
begin
  with Installer do
{$include fcl-stl/fpmake.pp}
end;

procedure add_fcl_web(const ADirectory: string);
begin
  with Installer do
{$include fcl-web/fpmake.pp}
end;

procedure add_fcl_xml(const ADirectory: string);
begin
  with Installer do
{$include fcl-xml/fpmake.pp}
end;

procedure add_fftw(const ADirectory: string);
begin
  with Installer do
{$include fftw/fpmake.pp}
end;

procedure add_fpgtk(const ADirectory: string);
begin
  with Installer do
{$include fpgtk/fpmake.pp}
end;

{$include fpindexer/fpmake.pp}

procedure add_fpmkunit(const ADirectory: string);
begin
  with Installer do
{$include fpmkunit/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$include fppkg/fpmake.pp}

procedure add_fv(const ADirectory: string);
=======
procedure add_fpvectorial;
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure add_fpvectorial;
begin
  with Installer do
{$include fpvectorial/fpmake.pp}
end;

procedure add_fv;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include fpvectorial/fpmake.pp}
end;

procedure add_fv;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include fv/fpmake.pp}
end;

{$include gdbint/fpmake.pp}

procedure add_gdbm(const ADirectory: string);
begin
  with Installer do
{$include gdbm/fpmake.pp}
end;

procedure add_ggi(const ADirectory: string);
begin
  with Installer do
{$include ggi/fpmake.pp}
end;

procedure add_gmp(const ADirectory: string);
begin
  with Installer do
{$include gmp/fpmake.pp}
end;

procedure add_gnome1(const ADirectory: string);
begin
  with Installer do
{$include gnome1/fpmake.pp}
end;

procedure add_graph(const ADirectory: string);
begin
  with Installer do
{$include graph/fpmake.pp}
end;

procedure add_gtk1(const ADirectory: string);
begin
  with Installer do
{$include gtk1/fpmake.pp}
end;

procedure add_gtk2(const ADirectory: string);
begin
  with Installer do
{$include gtk2/fpmake.pp}
end;

procedure add_hash(const ADirectory: string);
begin
  with Installer do
{$include hash/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_hermes(const ADirectory: string);
begin
  with Installer do
{$include hermes/fpmake.pp}
end;

procedure add_httpd13(const ADirectory: string);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure add_httpd13;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include httpd13/fpmake.pp}
end;

procedure add_httpd20(const ADirectory: string);
begin
  with Installer do
{$include httpd20/fpmake.pp}
end;

procedure add_httpd22(const ADirectory: string);
begin
  with Installer do
{$include httpd22/fpmake.pp}
end;

procedure add_httpd24(const ADirectory: string);
begin
  with Installer do
{$include httpd24/fpmake.pp}
end;

procedure add_ibase(const ADirectory: string);
begin
  with Installer do
{$include ibase/fpmake.pp}
end;

procedure add_iconvenc(const ADirectory: string);
begin
  with Installer do
{$include iconvenc/fpmake.pp}
end;

procedure add_imagemagick(const ADirectory: string);
begin
  with Installer do
{$include imagemagick/fpmake.pp}
end;

procedure add_imlib(const ADirectory: string);
begin
  with Installer do
{$include imlib/fpmake.pp}
end;

procedure add_iosxlocale(const ADirectory: string);
begin
  with Installer do
{$include iosxlocale/fpmake.pp}
end;

procedure add_jni(const ADirectory: string);
begin
  with Installer do
{$include jni/fpmake.pp}
end;

procedure add_ldap(const ADirectory: string);
begin
  with Installer do
{$include ldap/fpmake.pp}
end;

procedure add_libc(const ADirectory: string);
begin
  with Installer do
{$include libc/fpmake.pp}
end;

procedure add_libcurl(const ADirectory: string);
begin
  with Installer do
{$include libcurl/fpmake.pp}
end;

procedure add_libgbafpc(const ADirectory: string);
begin
  with Installer do
{$include libgbafpc/fpmake.pp}
end;

procedure add_libgd(const ADirectory: string);
begin
  with Installer do
{$include libgd/fpmake.pp}
end;

procedure add_libndsfpc(const ADirectory: string);
begin
  with Installer do
{$include libndsfpc/fpmake.pp}
end;

procedure add_libogcfpc(const ADirectory: string);
begin
  with Installer do
{$include libogcfpc/fpmake.pp}
end;

procedure add_libpng(const ADirectory: string);
begin
  with Installer do
{$include libpng/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_librsvg(const ADirectory: string);
=======
procedure add_librsvg;
>>>>>>> graemeg/cpstrnew
=======
procedure add_librsvg;
>>>>>>> graemeg/cpstrnew
=======
procedure add_librsvg;
>>>>>>> graemeg/cpstrnew
=======
procedure add_librsvg;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include librsvg/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_libsee(const ADirectory: string);
=======
procedure add_libsee;
>>>>>>> graemeg/cpstrnew
=======
procedure add_libsee;
>>>>>>> graemeg/cpstrnew
=======
procedure add_libsee;
>>>>>>> graemeg/cpstrnew
=======
procedure add_libsee;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include libsee/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_libtar(const ADirectory: string);
begin
  with Installer do
{$include libtar/fpmake.pp}
end;

procedure add_libvlc(const ADirectory: string);
begin
  with Installer do
{$include libvlc/fpmake.pp}
end;
procedure add_libmicrohttpd(const ADirectory: string);
begin
  with Installer do
{$include libmicrohttpd/fpmake.pp}
end;

procedure add_libxml(const ADirectory: string);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure add_libxml;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include libxml/fpmake.pp}
end;

procedure add_lua(const ADirectory: string);
begin
  with Installer do
{$include lua/fpmake.pp}
end;

procedure add_mad(const ADirectory: string);
begin
  with Installer do
{$include mad/fpmake.pp}
end;

procedure add_modplug(const ADirectory: string);
begin
  with Installer do
{$include modplug/fpmake.pp}
end;

procedure add_morphunits(const ADirectory: string);
begin
  with Installer do
{$include morphunits/fpmake.pp}
end;

procedure add_mysql(const ADirectory: string);
begin
  with Installer do
{$include mysql/fpmake.pp}
end;

procedure add_ncurses(const ADirectory: string);
begin
  with Installer do
{$include ncurses/fpmake.pp}
end;

procedure add_newt(const ADirectory: string);
begin
  with Installer do
{$include newt/fpmake.pp}
end;

procedure add_numlib(const ADirectory: string);
begin
  with Installer do
{$include numlib/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_nvapi(const ADirectory: string);
=======
procedure add_nvapi;
>>>>>>> graemeg/cpstrnew
=======
procedure add_nvapi;
>>>>>>> graemeg/cpstrnew
=======
procedure add_nvapi;
>>>>>>> graemeg/cpstrnew
=======
procedure add_nvapi;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include nvapi/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_objcrtl(const ADirectory: string);
=======
procedure add_objcrtl;
>>>>>>> graemeg/cpstrnew
=======
procedure add_objcrtl;
>>>>>>> graemeg/cpstrnew
=======
procedure add_objcrtl;
>>>>>>> graemeg/cpstrnew
=======
procedure add_objcrtl;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include objcrtl/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_odbc(const ADirectory: string);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure add_odbc;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include odbc/fpmake.pp}
end;

procedure add_oggvorbis(const ADirectory: string);
begin
  with Installer do
{$include oggvorbis/fpmake.pp}
end;

procedure add_openal(const ADirectory: string);
begin
  with Installer do
{$include openal/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_opencl(const ADirectory: string);
=======
procedure add_opencl;
>>>>>>> graemeg/cpstrnew
=======
procedure add_opencl;
>>>>>>> graemeg/cpstrnew
=======
procedure add_opencl;
>>>>>>> graemeg/cpstrnew
=======
procedure add_opencl;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include opencl/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_opengles(const ADirectory: string);
begin
  with Installer do
{$include opengles/fpmake.pp}
end;

procedure add_opengl(const ADirectory: string);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure add_opengl;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include opengl/fpmake.pp}
end;

procedure add_openssl(const ADirectory: string);
begin
  with Installer do
{$include openssl/fpmake.pp}
end;

procedure add_oracle(const ADirectory: string);
begin
  with Installer do
{$include oracle/fpmake.pp}
end;

procedure add_os2units(const ADirectory: string);
begin
  with Installer do
{$include os2units/fpmake.pp}
end;

procedure add_palmunits(const ADirectory: string);
begin
  with Installer do
{$include palmunits/fpmake.pp}
end;

procedure add_pasjpeg(const ADirectory: string);
begin
  with Installer do
{$include pasjpeg/fpmake.pp}
end;

procedure add_paszlib(const ADirectory: string);
begin
  with Installer do
{$include paszlib/fpmake.pp}
end;

procedure add_pcap(const ADirectory: string);
begin
  with Installer do
{$include pcap/fpmake.pp}
end;

procedure add_postgres(const ADirectory: string);
begin
  with Installer do
{$include postgres/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_proj4(const ADirectory: string);
=======
procedure add_proj4;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include proj4/fpmake.pp}
end;

procedure add_ptc(const ADirectory: string);
=======
procedure add_proj4;
>>>>>>> graemeg/cpstrnew
=======
procedure add_proj4;
>>>>>>> graemeg/cpstrnew
=======
procedure add_proj4;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include proj4/fpmake.pp}
end;

procedure add_pthreads(const ADirectory: string);
begin
  with Installer do
{$include pthreads/fpmake.pp}
end;

procedure add_pxlib(const ADirectory: string);
begin
  with Installer do
{$include pxlib/fpmake.pp}
end;

procedure add_regexpr(const ADirectory: string);
begin
  with Installer do
{$include regexpr/fpmake.pp}
end;

procedure add_rexx(const ADirectory: string);
begin
  with Installer do
{$include rexx/fpmake.pp}
end;

{$include rtl-console/fpmake.pp}

<<<<<<< HEAD
{$include rtl-extra/fpmake.pp}

<<<<<<< HEAD
<<<<<<< HEAD
{$include rtl-objpas/fpmake.pp}

<<<<<<< HEAD
{$include rtl-unicode/fpmake.pp}

procedure add_sdl(const ADirectory: string);
begin
  with Installer do
{$include sdl/fpmake.pp}
end;

procedure add_sqlite(const ADirectory: string);
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
procedure add_sqlite;
>>>>>>> graemeg/cpstrnew
begin
  with Installer do
{$include sqlite/fpmake.pp}
end;

procedure add_svgalib(const ADirectory: string);
begin
  with Installer do
{$include svgalib/fpmake.pp}
end;

procedure add_symbolic(const ADirectory: string);
begin
  with Installer do
{$include symbolic/fpmake.pp}
end;

procedure add_syslog(const ADirectory: string);
begin
  with Installer do
{$include syslog/fpmake.pp}
end;

procedure add_tcl(const ADirectory: string);
begin
  with Installer do
{$include tcl/fpmake.pp}
end;

procedure add_univint(const ADirectory: string);
begin
  with Installer do
{$include univint/fpmake.pp}
end;

procedure add_unixutil(const ADirectory: string);
begin
  with Installer do
{$include unixutil/fpmake.pp}
end;

procedure add_unzip(const ADirectory: string);
begin
  with Installer do
{$include unzip/fpmake.pp}
end;

procedure add_users(const ADirectory: string);
begin
  with Installer do
{$include users/fpmake.pp}
end;

procedure add_utmp(const ADirectory: string);
begin
  with Installer do
{$include utmp/fpmake.pp}
end;

procedure add_uuid(const ADirectory: string);
begin
  with Installer do
{$include uuid/fpmake.pp}
end;

procedure add_winceunits(const ADirectory: string);
begin
  with Installer do
{$include winceunits/fpmake.pp}
end;

procedure add_winunits_base(const ADirectory: string);
begin
  with Installer do
{$include winunits-base/fpmake.pp}
end;

procedure add_winunits_jedi(const ADirectory: string);
begin
  with Installer do
{$include winunits-jedi/fpmake.pp}
end;

procedure add_x11(const ADirectory: string);
begin
  with Installer do
{$include x11/fpmake.pp}
end;

procedure add_xforms(const ADirectory: string);
begin
  with Installer do
{$include xforms/fpmake.pp}
end;

procedure add_zlib(const ADirectory: string);
begin
  with Installer do
{$include zlib/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
procedure add_libenet(const ADirectory: string);
begin
  with Installer do
{$include libenet/fpmake.pp}
end;

procedure add_zorba(const ADirectory: string);
=======
procedure add_zorba;
>>>>>>> graemeg/cpstrnew
=======
procedure add_zorba;
>>>>>>> graemeg/cpstrnew
=======
procedure add_zorba;
>>>>>>> graemeg/cpstrnew
=======
procedure add_zorba;
>>>>>>> origin/cpstrnew
begin
  with Installer do
{$include zorba/fpmake.pp}
end;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
{$include googleapi/fpmake.pp}

=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
