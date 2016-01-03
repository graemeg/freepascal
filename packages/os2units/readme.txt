OS/2 Multimedia Toolkit
=======================

OS/2 Multimedia toolkit is set of units to manage various parts of multimedia
subsystem of OS/2. Not a part of standard OS/2 Multimedia:

a) WarpOverlay! library interface (hwvideo.pas)

MMPM/2 Documentation
--------------------

OS/2 Multimedia Subsystem Programming Guide
Provides guidelines for developing multimedia subsystems. Each subsystem
component is described in detail in individual chapters. Models are used
to complement the information provided by component sample program templates.

OS/2 Multimedia Application Programming Guide
Provides advisory information on application interfaces to help you select
and implement functions for your OS/2 multimedia applications. Code examples
from fully documented sample programs accompany the descriptions of the
functions.

OS/2 Multimedia Programming Reference
Provides detailed information on multimedia functions, messages, and data
structures to enable you to write code for your multimedia application
programs and subsystems.

The MMPM/2 Device Driver Reference for OS/2
It is for subsystem developers who want to write their own physical
device drivers (and associated virtual device drivers) to support audio
and video adapters in the Multimedia Presentation Manager/2 system.

Multimedia REXX
Describes REXX functions that enable media control interface string commands
to be sent from an OS/2 command file to control multimedia devices.  This
online book is provided with OS/2 multimedia.

Guide to Multimedia User Interface Design - (41G2922)
Describes design concepts to be considered when designing a CUA multimedia
interface that is consistent within a particular multimedia product and
across other products.

WarpOverlay!
------------

WarpOverlay! is video acceleration driver. This is original readme.txt
(BEWARE! Most probably, Valery don't know about this port):

-Begin-
Short overview:
hwvideo.h is a main header for use WarpOverlay! video acceleration driver.
There are no more or less suitable SDK yet, I am to lazy :(


DEMO - small stupid example of using WarpOverlay!, currently it does not sup-
port many features of WarpOverlay!, only basic functionality.
it just tried to load file demo.mpg and cyclically play it.
Used mpeg1 decoder pretty unstable and very unoptimal. This is just example.
Many of MPEG files can crush decoder. I am not very fimilar with different
MPEG aspects and do not want to mess with it, sorry.
I just included one small MPEG1 file, which work well.

RGB - example of using RGB (FOURCC_R565) overlay. Attention! RGB overlay
not supported for NVidia chips and for Matrox G200.
Known worked RGB overlay: Rage128, Radeon, i740, Savages.


Please contact me, if you have questions/troubles with this example or with
WarpOverlay! usage.

Valery Gaynullin

MPEG  decoding engine based on the Berkeley MPEG (mpegplay).
-End-

Longterm plans
--------------

a) Fix lot of bugs ;)
b) Add support for IBM Multimedia Classes and CWMM Classes for WPS (as fast as
   SOM and WPS toolkits will be presented)
c) SEAL library interface (seal.pas) www.netlabs.org/hobbes.nmsu.edu
   SEAL is module player library. At the present time SEAL not developing
   anymore (for all platforms), but still very usefull for mudule music fans.
d) DualMode Library (dual.pas) www.netlabs.org
   DualMode library allows to use common access method as for fullscreen (MGL)
   as for windowed (DIVE) video modes.
e) Examples, examples, examples (tests as well)

CU!
Yuri & Andry
