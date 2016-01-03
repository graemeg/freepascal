unit dswifi7;
{$mode objfpc} 
{$apptype arm7}
{$define arm7}

{$J+}
{$INLINE ON}
{$MACRO ON}
{$PACKRECORDS C}

interface

uses
  ctypes, nds7;

{$linklib nds7}
{$linklib dswifi7}

{$linklib c}
{$linklib gcc}
{$linklib sysbase}

{$define NDS_INTERFACE}
<<<<<<< HEAD
<<<<<<< HEAD
{$include inc/dswifi7.inc}
=======
{$include dswifi7.inc}
>>>>>>> graemeg/fixes_2_2
=======
{$include dswifi7.inc}
>>>>>>> origin/fixes_2_2
{$undef NDS_INTERFACE}

implementation
{$define NDS_IMPLEMENTATION}
<<<<<<< HEAD
<<<<<<< HEAD
{$include inc/dswifi7.inc}
=======
{$include dswifi7.inc}
>>>>>>> graemeg/fixes_2_2
=======
{$include dswifi7.inc}
>>>>>>> origin/fixes_2_2
{$undef NDS_IMPLEMENTATION}

end.
