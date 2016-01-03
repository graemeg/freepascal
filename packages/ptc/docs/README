PTCPas 0.99.5
Nikolay Nikolov (nickysn@users.sourceforge.net)

This is a FPC port of the OpenPTC C++ library. It is distributed under the
the terms of the GNU LGPL (see lgpl.txt).

The latest version can be found at http://ptcpas.sourceforge.net

Basically it provides an abstraction layer for high-speed low-level graphics
access. It is OOP and supports multiple platforms. (tested on Linux, DOS and
Windows, more will be added in the future)
3d acceleration isn't supported, nor planned. If you need that, you should use
something like OpenGL instead. :-)

Supported consoles:
  DirectX 3+ (should work on all Windows versions since Windows 95, except
              Windows CE. This currently means 95/98/ME/NT4/2000/XP/2003.
	      On NT4 you need SP3 or later. Also some very ancient versions of
	      Windows 95 do not have any DirectX preinstalled, so it has to be
	      installed separately.)
  X11 (on linux, maybe also other unix-like OSes, supports dga and XShm
       extensions)
  Vesa 1.2+ (DOS. LFB and video pages not yet supported)
  VGA (DOS, fakemodes, mode13h, etc...)
  CGA (DOS, added by me just for fun ... and maybe some day I'll even add
       EGA :-) )
  Text (DOS, 80x50 - 16 colours, should work even in the most buggy dos boxes
        (2000,XP) and IMHO looks better than AALib ;-) )

All programs using OpenPTC look (at runtime) for a config file that may contain
various (platform specific) options, so you can try different consoles, etc,
without the need to recompile. It is called ptc.cfg and is searched in the
current directory on DOS and Windows. On unix it is .ptc.conf in the user's
HOME directory. There's an example ptc.cfg file with all supported options,
prefixed with #. If you want to try an option just remove the # and put it in
the same directory as the .exe (or copy to ~/.ptc.conf on unix :) )

--------------------------------------------------------------------------------
The original copyrights from the C++ version:
The X11 classes are Copyright (c) 1998/99 Christian Nentwich (brn@eleet.mcb.at)
The OpenPTC 1.0 C++ API is (c) 1998/99 Glenn Fiedler (ptc@gaffer.org)

The OpenPTC C++ library can be found at http://www.gaffer.org/ptc
The Hermes C library can be found at http://hermes.terminal.at
