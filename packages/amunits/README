
Readme for the m68k compiler FPC 0.99.5c
----------------------------------------

Updated since last version:
  - alignment problems bugfixed -- now it works for real (I hope!)
  - CRC loading on big endian machines was wrong

1) m68k binaries
2) Cross-compiler binaries
3) General information and porting tips

1) m68k Binaries information
----------------------------

  Amiga binary version notes:
  ---------------------------

  Requirements:
    - 2 Megabytes RAM (Chip or Fast) minimum to produce assembler files
    - 1 Megabyte hard drive space
    - 128K stack space (set it via 'stack')
    - AmigaOS v2.04 or higher

  Further information:
    - Because the heap can become fragmented when using ld and as, sometimes
      you will no longer be able to run ppc after calling these programs,
      use avail flush or reboot your computer to solve this.
    - Because how the os works with files and because of a bug in the
      compiler, if an error occurs or a break signal is issued. some files
      might remain opened. This only happens with non units and non source
      code files such as ppc.cfg, the only way to fix this is to reboot
      the computer. As soon as i have the time, I will implement an exit
      procedure in the system unit which close all files automatically on
      any program termination.
    - gdb support is not implemented, as it requires an interface
      to ixemul.library
    - as 2.5.2 and ld 2.8.1 are used in this package, since as 2.8.1 seems
      to be buggy (at least the version i downloaded) with gdb info
    - A good debugging tool to use is barfly available from Aminet

  Atari binary version notes:
  ---------------------------

  Requirements:
    - 2 Megabytes RAM minimum to produce assembler files
    - 1 Megabyte hard drive space
    - GemDOS 0.15 or higher
    - Atari Extended Argument Specification compatible shell


2) Cross-compiler binaries
--------------------------

   PC/MS-DOS version
   ------------------
    - 2 Megabytes RAM minimum to produce assembler files
    - 1 Megabyte hard drive space
    - DOS 3.3 and higher
    - 16-bit dpmi server (one is supplied with the binary)

    A default configuration can be found in ppc.cfg, and the
    sysatari library source code is in the ./src/ directory.

    This can be used as a template for embedded processor
    development, you just need to replace all sysatari
    routines by emtpy ones (for example), most other routines
    in the include file should be kept (some of them are internal)

   PC/Linux version
   -----------------
    - 2 Megabytes RAM minimum to produce Amiga binaries
    - 2 Megabyte hard drive space with all binaries,
      compiler, assembler and linker + rtl.

    - amigaas, gnu as, a crossversion, included
    - amigald, gnu ld, a crossversion, included

    The compiler will read pp68k.cfg for configuration the
    proper place for this is in /etc. A default pp68k.cfg is
    in /bin/.

    Just make sure that pp68k, amigaas and amigald is in
    your path, why not /usr/local/bin

3) General information and porting tips
---------------------------------------

  - Alignment output is supposedely correct even though i can't
    personally test this. (Someone else tested for me)
  - Some tips to port some general code from i386 FPC to m68k FPC,
    you should limit your local variables and pushed variables
    in a routine 32K, this is a displacement limit of older m68k
    processors, and it has been kept.
  - If you use PACKED records anywhere, make sure that non-byte fields
    are aligned on even addresses, otherwise this will cause
    alignment errors on older m68k processors (68000/68010), if
    you don't use packed , disregard this remark, as everything
    will be automatically aligned. The compiler takes care of
    of aligning all local and global simple type variables on
    at least word boundaries (for the m68k only). pointer are always
    at least aligned on dword boundaries.
  - PPU files (PP? files) are portable across big-endian and little
    endian systems, EXCEPT in the case where the unit references
    floating point values, as these are not saved in the correct
    endian for the moment.
  - GNU assembler (gas) syntax acceptance varies widely between gas versions
    ,therefore the -Ai and -Agas switches are your friend here. If you
    still get trouble try, -TPALMOS as a target, this changes to more
    standard assembler. Finally in any case you can always the
    --register-prefix-optional options in any GNU assembler version if it
    still does not work.
  - To compile a system unit use these switches:
      TARGET -dm68k -Sg -Us mysystem.pp
          where TARGET can be:
            -TAMIGA, -TATARI , -TLINUX or -TPALMOS
            mysystem.pp should be replaced by the system unit name
            for the platform:
             amiga: sysamiga.pas
             atari: sysatari.pas
             linux: syslinux.pp
             palmos: syspalm.pp
  - BIG sets (with more then different 32 values) are stored in little
    endian format. This can cause BIG problems if you use exotic set
    functions like an array of byte typecast to a set, the values should
    be byteswapped first to conform to little (intel) endian format. If
    you use normal set functions such as addition, subtraction, in operator
    you should not get any problems. I'm not sure if this is worth fixing
    or not :(...
  - Because of how everything works now, BYTE pushes are stored in byte
    reversed format in a word on the stack. This behavior should not
    be noticeable unless you do very low level stuff. The downside of this
    is that linking with external routines which expect bytes as parameters
    will probably not work. This will be fixed, just need to find the
    time to do it.

Enjoy! BTW: I still need help in porting to Mac, Linux and Atari and
also someone to do a peephole optimizer for the m68k code output.

You can get general Free Pascal information at:
 http://www.brain.uni-freibrug.de/~klaus/fpc/fpc.html
Developer mailing list:
 fpc-devel@mail.terrasoft.hu

You can contact me at:
 codc01@gel.usherb.ca
 http://www-edu.gel.usherb.ca/codc01
Amiga inlucdes/units and Amiga specific stuff:
 nils.sjoholm@mailbox.swipnet.se

Thanks:
  That is apart from the FPC development team (which i am part)....:

    Nils Sjoholm - AMIGA porter and tester, 68000 tester,
      the most dedicated person for the m68k port found so far!

                                    Enjoy!
                                    Carl Eric Codere


