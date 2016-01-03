unit dswifi9;
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

{$linklib nds9}
{$linklib dswifi9}

{$linklib c}
{$linklib gcc}
{$linklib sysbase}

{$define NDS_INTERFACE}
<<<<<<< HEAD
<<<<<<< HEAD
{$include inc/dswifi9.inc}
=======
{$include dswifi9.inc}
>>>>>>> graemeg/fixes_2_2
=======
{$include dswifi9.inc}
>>>>>>> origin/fixes_2_2
{$undef NDS_INTERFACE}

implementation
{$define NDS_IMPLEMENTATION}
<<<<<<< HEAD
<<<<<<< HEAD
{$include inc/dswifi9.inc}
=======
{$include dswifi9.inc}
>>>>>>> graemeg/fixes_2_2
=======
{$include dswifi9.inc}
>>>>>>> origin/fixes_2_2
{$undef NDS_IMPLEMENTATION}

end.
