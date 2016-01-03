<<<<<<< HEAD
<<<<<<< HEAD
(*---------------------------------------------------------------------------------

	libnds Copyright (C) 2005
		Michael Noland (joat)
		Jason Rogers (dovoto)
		Dave Murphy (WinterMute)

	This software is provided 'as-is', without any express or implied
	warranty.  In no event will the authors be held liable for any
	damages arising from the use of this software.

	Permission is granted to anyone to use this software for any
	purpose, including commercial applications, and to alter it and
	redistribute it freely, subject to the following restrictions:


	1.	The origin of this software must not be misrepresented; you
		must not claim that you wrote the original software. If you use
		this software in a product, an acknowledgment in the product
		documentation would be appreciated but is not required.

	2.	Altered source versions must be plainly marked as such, and
		must not be misrepresented as being the original software.

	3.	This notice may not be removed or altered from any source
		distribution.

  ----------------------------------------------------------------------------
   Free Pascal porting by Francesco Lombardi.
---------------------------------------------------------------------------------*)


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
unit nds7;
{$mode objfpc} 
{$apptype arm7}
{$define arm7}

{$J+}
{$INLINE ON}
{$MACRO ON}
{$PACKRECORDS C}

interface

uses
  ctypes;

{$linklib nds7}

{$linklib c}
{$linklib gcc}
<<<<<<< HEAD
<<<<<<< HEAD
{$linklib g}
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$linklib sysbase}

{$define NDS_INTERFACE}
{$include nds/ndsinclude.inc}
{$undef NDS_INTERFACE}

implementation

{$define NDS_IMPLEMENTATION}
{$include nds/ndsinclude.inc}
{$undef NDS_IMPLEMENTATION}

end.
