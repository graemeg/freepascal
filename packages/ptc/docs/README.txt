<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
PTCPas 0.99.14
=======
PTCPas 0.99.11
>>>>>>> graemeg/cpstrnew
=======
PTCPas 0.99.11
>>>>>>> graemeg/cpstrnew
=======
PTCPas 0.99.11
>>>>>>> graemeg/cpstrnew
=======
PTCPas 0.99.11
>>>>>>> origin/cpstrnew
Nikolay Nikolov (nickysn@users.sourceforge.net)

PTCPas is a free, portable framebuffer library, written in Free Pascal. It is
distributed under the the terms of a modified version of the GNU LGPL (see
modified_lgpl.txt).

The latest version can be found at http://ptcpas.sourceforge.net

Basically it provides an abstraction layer for high-speed low-level graphics
access. It is OOP and supports multiple platforms. (tested on Linux, DOS and
Windows, more will be added in the future)

Since version 0.99.13 it is also possible to create OpenGL applications with
PTCPas. See the ptcgl.pp and ptcgl2.pp examples in the 'examples' directory.

PTCPas initially started out as a complete Object Pascal translation of the
OpenPTC C++ library. Since then, OpenPTC development has stalled and PTCPas
lives on as a fully independent Object Pascal project.

PTCPas initially started out as a complete Object Pascal translation of the
OpenPTC C++ library. Since then, OpenPTC development has stalled and PTCPas
lives on as a fully independent Object Pascal project.

PTCPas initially started out as a complete Object Pascal translation of the
OpenPTC C++ library. Since then, OpenPTC development has stalled and PTCPas
lives on as a fully independent Object Pascal project.

PTCPas initially started out as a complete Object Pascal translation of the
OpenPTC C++ library. Since then, OpenPTC development has stalled and PTCPas
lives on as a fully independent Object Pascal project.

PTCPas initially started out as a complete Object Pascal translation of the
OpenPTC C++ library. Since then, OpenPTC development has stalled and PTCPas
lives on as a fully independent Object Pascal project.

Supported consoles:
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  DirectX 3+ (should work on all x86 and x64 Windows versions since Windows 95,
              except Windows CE. This currently means 95/98/ME/NT4/2000/XP/2003/
              Vista/2008/7. It is compatible with the x64 editions of XP, 2003,
              Vista, 2008 and 7 (both as a 32-bit application and native win64).
              On NT4 you need SP3 or later. Ancient versions of Windows 95 come
              without any DirectX version by default, so you may have to install
              it.)
=======
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  DirectX 3+ (should work on all x86 Windows versions since Windows 95, except
              Windows CE. This currently means 95/98/ME/NT4/2000/XP/2003/Vista.
              It is compatible with the x64 editions of XP and 2003 (although
              only as a 32-bit application, no native win64 yet). On NT4 you
              need SP3 or later. Also some very ancient versions of Windows 95
              do not have any DirectX preinstalled, so it has to be installed
              separately.)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> graemeg/cpstrnew
=======
>>>>>>> origin/cpstrnew
  Win32 GDI (no fullscreen support. Slower than DirectX, but maybe more
             compatible.)
  X11 (on linux and other unix-like OSes, supports XRandR, XF86VidMode, XShm
       and xf86dga extensions)
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  Vesa 1.0+ (DOS. Supports LFB and banked video memory access)
=======
  Vesa 1.0+ (DOS. Supports LFB and banked video memory access. Video pages not
             yet supported)
>>>>>>> graemeg/cpstrnew
=======
  Vesa 1.0+ (DOS. Supports LFB and banked video memory access. Video pages not
             yet supported)
>>>>>>> graemeg/cpstrnew
=======
  Vesa 1.0+ (DOS. Supports LFB and banked video memory access. Video pages not
             yet supported)
>>>>>>> graemeg/cpstrnew
=======
  Vesa 1.0+ (DOS. Supports LFB and banked video memory access. Video pages not
             yet supported)
>>>>>>> origin/cpstrnew
  VGA (DOS, fakemodes, mode13h, etc...)
  CGA (DOS, added by me just for fun ... and maybe some day I'll even add
       EGA :-) )
  Text (DOS, 80x50 - 16 colours, should work even in the most buggy dos boxes
        (2000,XP) and IMHO looks better than AALib ;-) )
  WinCE GAPI (Windows CE 3.0 and later on devices that support GAPI. Needs
              testing on more devices.)
  WinCE GDI (Windows CE 3.0 and later. Slow. Needs testing on more devices.)

All programs using PTCPas look (at runtime) for a config file that may contain
various (platform specific) options, so you can try different consoles, etc,
without the need to recompile. It is called ptcpas.cfg and is searched in the
current directory on DOS and Windows. On unix it is .ptcpas.conf in the user's
HOME directory. There's an example ptcpas.cfg file with all supported options,
prefixed with #. If you want to try an option just remove the # and put it in
the same directory as the .exe (or copy to ~/.ptcpas.conf on unix :) )

--------------------------------------------------------------------------------
The original copyrights from the C++ version:
The X11 classes are Copyright (c) 1998/99 Christian Nentwich
(christian@systemwire.com, old mail brn@eleet.mcb.at no longer works?)
The OpenPTC 1.0 C++ API is (c) 1998/99 Glenn Fiedler (ptc@gaffer.org)

The OpenPTC C++ library can be found at http://sourceforge.net/projects/openptc/
The Hermes C library can be found at http://www.clanlib.org/download/legacy/
