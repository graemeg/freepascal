unit fat;
{$mode objfpc} 
{$apptype arm9}
{$define arm9}

{$J+}
{$INLINE ON}
{$MACRO ON}
{$PACKRECORDS C}

interface

uses
  ctypes, nds9;

<<<<<<< HEAD
{$linklib fat}
{$linklib nds9}
=======
{$linklib nds9}
{$linklib libfat.a}
>>>>>>> graemeg/fixes_2_2

{$linklib c}
{$linklib gcc}
{$linklib sysbase}

{$define NDS_INTERFACE}
<<<<<<< HEAD
{ $include fathelper.inc}
{$include ../nds/disc_io.inc}
=======
>>>>>>> graemeg/fixes_2_2
{$include fat.inc}
{$undef NDS_INTERFACE}

implementation
{$define NDS_IMPLEMENTATION}
<<<<<<< HEAD
{ $include fathelper.inc}
{$include ../nds/disc_io.inc}
{$include fat.inc}
{$undef NDS_IMPLEMENTATION}

initialization

finalization

=======
{$include fat.inc}
{$undef NDS_IMPLEMENTATION}

>>>>>>> graemeg/fixes_2_2
end.
