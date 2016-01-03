unit matrix;
{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2004 by Daniel Mantione

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{This unit provides some common matrix and vector operations on vectors and
 matrixes with dimensions of 2, 3 and 4 which are the most common ones in
 computer graphics, and all routines provided in variations with single,
 double and extended precision.

 The goal of this unit is also to invite some standardisation
 between libraries, i.e. a vector from library x can be passed to library y
 without conversion routines in between.

 It would be nice to have some diehard assembler optimized versions of
 these routines, however the Free Pascal team wishes to concentrate on the
 compiler. Contributions from the community are very welcome.}

{*****************************************************************************}

interface

{*****************************************************************************}
{$mode fpc}

{$ifndef FPUNONE}
{$MACRO on}

type    Tvector2_single_data=array[0..1] of single;
        Tvector2_double_data=array[0..1] of double;
        Tvector2_extended_data=array[0..1] of extended;

        Tvector3_single_data=array[0..2] of single;
        Tvector3_double_data=array[0..2] of double;
        Tvector3_extended_data=array[0..2] of extended;

        Tvector4_single_data=array[0..3] of single;
        Tvector4_double_data=array[0..3] of double;
        Tvector4_extended_data=array[0..3] of extended;

        Tmatrix2_single_data=array[0..1,0..1] of single;
        Tmatrix2_double_data=array[0..1,0..1] of double;
        Tmatrix2_extended_data=array[0..1,0..1] of extended;

        Tmatrix3_single_data=array[0..2,0..2] of single;
        Tmatrix3_double_data=array[0..2,0..2] of double;
        Tmatrix3_extended_data=array[0..2,0..2] of extended;

        Tmatrix4_single_data=array[0..3,0..3] of single;
        Tmatrix4_double_data=array[0..3,0..3] of double;
        Tmatrix4_extended_data=array[0..3,0..3] of extended;

        Tvector2_single=object
            data:Tvector2_single_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b:single);
            function length:single;
            function squared_length:single;
        end;

        Tvector2_double=object
            data:Tvector2_double_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b:double);
            function length:double;
            function squared_length:double;
        end;

        Tvector2_extended=object
            data:Tvector2_extended_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b:extended);
            function length:extended;
            function squared_length:extended;
        end;

        Tvector3_single=object
            data:Tvector3_single_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b,c:single);
            function length:single;
            function squared_length:single;
        end;

        Tvector3_double=object
            data:Tvector3_double_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b,c:double);
            function length:double;
            function squared_length:double;
        end;

        Tvector3_extended=object
            data:Tvector3_extended_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b,c:extended);
            function length:extended;
            function squared_length:extended;
        end;

        Tvector4_single=object
            data:Tvector4_single_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b,c,d:single);
            function length:single;
            function squared_length:single;
        end;

        Tvector4_double=object
            data:Tvector4_double_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b,c,d:double);
            function length:double;
            function squared_length:double;
        end;

        Tvector4_extended=object
            data:Tvector4_extended_data;
            constructor init_zero;
            constructor init_one;
            constructor init(a,b,c,d:extended);
            function length:extended;
            function squared_length:extended;
        end;

        Tmatrix2_single=object
            data:Tmatrix2_single_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ba,bb:single);
            function get_column(c:byte):Tvector2_single;
            function get_row(r:byte):Tvector2_single;
            procedure set_column(c:byte;const v:Tvector2_single);
            procedure set_row(r:byte;const v:Tvector2_single);
            function determinant:single;
            function inverse(Adeterminant:single):Tmatrix2_single;
            function transpose:Tmatrix2_single;
        end;

        Tmatrix2_double=object
            data:Tmatrix2_double_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ba,bb:double);
            function get_column(c:byte):Tvector2_double;
            function get_row(r:byte):Tvector2_double;
            procedure set_column(c:byte;const v:Tvector2_double);
            procedure set_row(r:byte;const v:Tvector2_double);
            function determinant:double;
            function inverse(Adeterminant:double):Tmatrix2_double;
            function transpose:Tmatrix2_double;
        end;

        Tmatrix2_extended=object
            data:Tmatrix2_extended_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ba,bb:extended);
            function get_column(c:byte):Tvector2_extended;
            function get_row(r:byte):Tvector2_extended;
            procedure set_column(c:byte;const v:Tvector2_extended);
            procedure set_row(r:byte;const v:Tvector2_extended);
            function determinant:extended;
            function inverse(Adeterminant:extended):Tmatrix2_extended;
            function transpose:Tmatrix2_extended;
        end;

        Tmatrix3_single=object
            data:Tmatrix3_single_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ac,ba,bb,bc,ca,cb,cc:single);
            function get_column(c:byte):Tvector3_single;
            function get_row(r:byte):Tvector3_single;
            procedure set_column(c:byte;const v:Tvector3_single);
            procedure set_row(r:byte;const v:Tvector3_single);
            function determinant:single;
            function inverse(Adeterminant:single):Tmatrix3_single;
            function transpose:Tmatrix3_single;
        end;

        Tmatrix3_double=object
            data:Tmatrix3_double_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ac,ba,bb,bc,ca,cb,cc:double);
            function get_column(c:byte):Tvector3_double;
            function get_row(r:byte):Tvector3_double;
            procedure set_column(c:byte;const v:Tvector3_double);
            procedure set_row(r:byte;const v:Tvector3_double);
            function determinant:double;
            function inverse(Adeterminant:double):Tmatrix3_double;
            function transpose:Tmatrix3_double;
        end;

        Tmatrix3_extended=object
            data:Tmatrix3_extended_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ac,ba,bb,bc,ca,cb,cc:extended);
            function get_column(c:byte):Tvector3_extended;
            function get_row(r:byte):Tvector3_extended;
            procedure set_column(c:byte;const v:Tvector3_extended);
            procedure set_row(r:byte;const v:Tvector3_extended);
            function determinant:extended;
            function inverse(Adeterminant:extended):Tmatrix3_extended;
            function transpose:Tmatrix3_extended;
        end;

        Tmatrix4_single=object
            data:Tmatrix4_single_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ac,ad,ba,bb,bc,bd,ca,cb,cc,cd,da,db,dc,dd:single);
            function get_column(c:byte):Tvector4_single;
            function get_row(r:byte):Tvector4_single;
            procedure set_column(c:byte;const v:Tvector4_single);
            procedure set_row(r:byte;const v:Tvector4_single);
            function determinant:single;
            function inverse(Adeterminant:single):Tmatrix4_single;
            function transpose:Tmatrix4_single;
        end;

        Tmatrix4_double=object
            data:Tmatrix4_double_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ac,ad,ba,bb,bc,bd,ca,cb,cc,cd,da,db,dc,dd:double);
            function get_column(c:byte):Tvector4_double;
            function get_row(r:byte):Tvector4_double;
            procedure set_column(c:byte;const v:Tvector4_double);
            procedure set_row(r:byte;const v:Tvector4_double);
            function determinant:double;
            function inverse(Adeterminant:double):Tmatrix4_double;
            function transpose:Tmatrix4_double;
        end;

        Tmatrix4_extended=object
            data:Tmatrix4_extended_data;
            constructor init_zero;
            constructor init_identity;
            constructor init(aa,ab,ac,ad,ba,bb,bc,bd,ca,cb,cc,cd,da,db,dc,dd:extended);
            function get_column(c:byte):Tvector4_extended;
            function get_row(r:byte):Tvector4_extended;
            procedure set_column(c:byte;const v:Tvector4_extended);
            procedure set_row(r:byte;const v:Tvector4_extended);
            function determinant:extended;
            function inverse(Adeterminant:extended):Tmatrix4_extended;
            function transpose:Tmatrix4_extended;
        end;


{Operators to make different vectors assignable to each other}
operator := (const v:Tvector2_single) result:Tvector2_double;
operator := (const v:Tvector2_single) result:Tvector2_extended;
operator := (const v:Tvector2_double) result:Tvector2_single;
operator := (const v:Tvector2_double) result:Tvector2_extended;
operator := (const v:Tvector2_extended) result:Tvector2_single;
operator := (const v:Tvector2_extended) result:Tvector2_double;

operator := (const v:Tvector2_single) result:Tvector3_single;
operator := (const v:Tvector2_single) result:Tvector3_double;
operator := (const v:Tvector2_single) result:Tvector3_extended;
operator := (const v:Tvector2_double) result:Tvector3_single;
operator := (const v:Tvector2_double) result:Tvector3_double;
operator := (const v:Tvector2_double) result:Tvector3_extended;
operator := (const v:Tvector2_extended) result:Tvector3_single;
operator := (const v:Tvector2_extended) result:Tvector3_double;
operator := (const v:Tvector2_extended) result:Tvector3_extended;

operator := (const v:Tvector2_single) result:Tvector4_single;
operator := (const v:Tvector2_single) result:Tvector4_double;
operator := (const v:Tvector2_single) result:Tvector4_extended;
operator := (const v:Tvector2_double) result:Tvector4_single;
operator := (const v:Tvector2_double) result:Tvector4_double;
operator := (const v:Tvector2_double) result:Tvector4_extended;
operator := (const v:Tvector2_extended) result:Tvector4_single;
operator := (const v:Tvector2_extended) result:Tvector4_double;
operator := (const v:Tvector2_extended) result:Tvector4_extended;

operator := (const v:Tvector3_single) result:Tvector2_single;
operator := (const v:Tvector3_single) result:Tvector2_double;
operator := (const v:Tvector3_single) result:Tvector2_extended;
operator := (const v:Tvector3_double) result:Tvector2_single;
operator := (const v:Tvector3_double) result:Tvector2_double;
operator := (const v:Tvector3_double) result:Tvector2_extended;
operator := (const v:Tvector3_extended) result:Tvector2_single;
operator := (const v:Tvector3_extended) result:Tvector2_double;
operator := (const v:Tvector3_extended) result:Tvector2_extended;

operator := (const v:Tvector3_single) result:Tvector3_double;
operator := (const v:Tvector3_single) result:Tvector3_extended;
operator := (const v:Tvector3_double) result:Tvector3_single;
operator := (const v:Tvector3_double) result:Tvector3_extended;
operator := (const v:Tvector3_extended) result:Tvector3_single;
operator := (const v:Tvector3_extended) result:Tvector3_double;

operator := (const v:Tvector3_single) result:Tvector4_single;
operator := (const v:Tvector3_single) result:Tvector4_double;
operator := (const v:Tvector3_single) result:Tvector4_extended;
operator := (const v:Tvector3_double) result:Tvector4_single;
operator := (const v:Tvector3_double) result:Tvector4_double;
operator := (const v:Tvector3_double) result:Tvector4_extended;
operator := (const v:Tvector3_extended) result:Tvector4_single;
operator := (const v:Tvector3_extended) result:Tvector4_double;
operator := (const v:Tvector3_extended) result:Tvector4_extended;

operator := (const v:Tvector4_single) result:Tvector2_single;
operator := (const v:Tvector4_single) result:Tvector2_double;
operator := (const v:Tvector4_single) result:Tvector2_extended;
operator := (const v:Tvector4_double) result:Tvector2_single;
operator := (const v:Tvector4_double) result:Tvector2_double;
operator := (const v:Tvector4_double) result:Tvector2_extended;
operator := (const v:Tvector4_extended) result:Tvector2_single;
operator := (const v:Tvector4_extended) result:Tvector2_double;
operator := (const v:Tvector4_extended) result:Tvector2_extended;

operator := (const v:Tvector4_single) result:Tvector3_single;
operator := (const v:Tvector4_single) result:Tvector3_double;
operator := (const v:Tvector4_single) result:Tvector3_extended;
operator := (const v:Tvector4_double) result:Tvector3_single;
operator := (const v:Tvector4_double) result:Tvector3_double;
operator := (const v:Tvector4_double) result:Tvector3_extended;
operator := (const v:Tvector4_extended) result:Tvector3_single;
operator := (const v:Tvector4_extended) result:Tvector3_double;
operator := (const v:Tvector4_extended) result:Tvector3_extended;

operator := (const v:Tvector4_single) result:Tvector4_double;
operator := (const v:Tvector4_single) result:Tvector4_extended;
operator := (const v:Tvector4_double) result:Tvector4_single;
operator := (const v:Tvector4_double) result:Tvector4_extended;
operator := (const v:Tvector4_extended) result:Tvector4_single;
operator := (const v:Tvector4_extended) result:Tvector4_double;

{Vector to vector operations.}
operator + (const x,y:Tvector2_single) result:Tvector2_single;
operator + (const x,y:Tvector2_double) result:Tvector2_double;
operator + (const x,y:Tvector2_extended) result:Tvector2_extended;
operator + (const x,y:Tvector3_single) result:Tvector3_single;
operator + (const x,y:Tvector3_double) result:Tvector3_double;
operator + (const x,y:Tvector3_extended) result:Tvector3_extended;
operator + (const x,y:Tvector4_single) result:Tvector4_single;
operator + (const x,y:Tvector4_double) result:Tvector4_double;
operator + (const x,y:Tvector4_extended) result:Tvector4_extended;

operator - (const x,y:Tvector2_single) result:Tvector2_single;
operator - (const x,y:Tvector2_double) result:Tvector2_double;
operator - (const x,y:Tvector2_extended) result:Tvector2_extended;
operator - (const x,y:Tvector3_single) result:Tvector3_single;
operator - (const x,y:Tvector3_double) result:Tvector3_double;
operator - (const x,y:Tvector3_extended) result:Tvector3_extended;
operator - (const x,y:Tvector4_single) result:Tvector4_single;
operator - (const x,y:Tvector4_double) result:Tvector4_double;
operator - (const x,y:Tvector4_extended) result:Tvector4_extended;

operator - (const x:Tvector2_single) result:Tvector2_single;
operator - (const x:Tvector2_double) result:Tvector2_double;
operator - (const x:Tvector2_extended) result:Tvector2_extended;
operator - (const x:Tvector3_single) result:Tvector3_single;
operator - (const x:Tvector3_double) result:Tvector3_double;
operator - (const x:Tvector3_extended) result:Tvector3_extended;
operator - (const x:Tvector4_single) result:Tvector4_single;
operator - (const x:Tvector4_double) result:Tvector4_double;
operator - (const x:Tvector4_extended) result:Tvector4_extended;

operator * (const x,y:Tvector2_single) result:Tvector2_single;
operator * (const x,y:Tvector2_double) result:Tvector2_double;
operator * (const x,y:Tvector2_extended) result:Tvector2_extended;
operator * (const x,y:Tvector3_single) result:Tvector3_single;
operator * (const x,y:Tvector3_double) result:Tvector3_double;
operator * (const x,y:Tvector3_extended) result:Tvector3_extended;
operator * (const x,y:Tvector4_single) result:Tvector4_single;
operator * (const x,y:Tvector4_double) result:Tvector4_double;
operator * (const x,y:Tvector4_extended) result:Tvector4_extended;

operator ** (const x,y:Tvector2_single) result:single;
operator ** (const x,y:Tvector2_double) result:double;
operator ** (const x,y:Tvector2_extended) result:extended;
operator ** (const x,y:Tvector3_single) result:single;
operator ** (const x,y:Tvector3_double) result:double;
operator ** (const x,y:Tvector3_extended) result:extended;
operator ** (const x,y:Tvector4_single) result:single;
operator ** (const x,y:Tvector4_double) result:double;
operator ** (const x,y:Tvector4_extended) result:extended;

operator >< (const x,y:Tvector3_single) result:Tvector3_single;
operator >< (const x,y:Tvector3_double) result:Tvector3_double;
operator >< (const x,y:Tvector3_extended) result:Tvector3_extended;

{Vector/scalar operations.}
operator + (const x:Tvector2_single;y:single) result:Tvector2_single;
operator + (const x:Tvector2_double;y:double) result:Tvector2_double;
operator + (const x:Tvector2_extended;y:extended) result:Tvector2_extended;
operator + (const x:Tvector3_single;y:single) result:Tvector3_single;
operator + (const x:Tvector3_double;y:double) result:Tvector3_double;
operator + (const x:Tvector3_extended;y:extended) result:Tvector3_extended;
operator + (const x:Tvector4_single;y:single) result:Tvector4_single;
operator + (const x:Tvector4_double;y:double) result:Tvector4_double;
operator + (const x:Tvector4_extended;y:extended) result:Tvector4_extended;

operator - (const x:Tvector2_single;y:single) result:Tvector2_single;
operator - (const x:Tvector2_double;y:double) result:Tvector2_double;
operator - (const x:Tvector2_extended;y:extended) result:Tvector2_extended;
operator - (const x:Tvector3_single;y:single) result:Tvector3_single;
operator - (const x:Tvector3_double;y:double) result:Tvector3_double;
operator - (const x:Tvector3_extended;y:extended) result:Tvector3_extended;
operator - (const x:Tvector4_single;y:single) result:Tvector4_single;
operator - (const x:Tvector4_double;y:double) result:Tvector4_double;
operator - (const x:Tvector4_extended;y:extended) result:Tvector4_extended;

operator * (const x:Tvector2_single;y:single) result:Tvector2_single;
operator * (const x:Tvector2_double;y:double) result:Tvector2_double;
operator * (const x:Tvector2_extended;y:extended) result:Tvector2_extended;
operator * (const x:Tvector3_single;y:single) result:Tvector3_single;
operator * (const x:Tvector3_double;y:double) result:Tvector3_double;
operator * (const x:Tvector3_extended;y:extended) result:Tvector3_extended;
operator * (const x:Tvector4_single;y:single) result:Tvector4_single;
operator * (const x:Tvector4_double;y:double) result:Tvector4_double;
operator * (const x:Tvector4_extended;y:extended) result:Tvector4_extended;

operator / (const x:Tvector2_single;y:single) result:Tvector2_single;
operator / (const x:Tvector2_double;y:double) result:Tvector2_double;
operator / (const x:Tvector2_extended;y:extended) result:Tvector2_extended;
operator / (const x:Tvector3_single;y:single) result:Tvector3_single;
operator / (const x:Tvector3_double;y:double) result:Tvector3_double;
operator / (const x:Tvector3_extended;y:extended) result:Tvector3_extended;
operator / (const x:Tvector4_single;y:single) result:Tvector4_single;
operator / (const x:Tvector4_double;y:double) result:Tvector4_double;
operator / (const x:Tvector4_extended;y:extended) result:Tvector4_extended;

{Operators to make different matrixes assignable to each other}
operator := (const v:Tmatrix2_single) result:Tmatrix2_double;
operator := (const v:Tmatrix2_single) result:Tmatrix2_extended;
operator := (const v:Tmatrix2_double) result:Tmatrix2_single;
operator := (const v:Tmatrix2_double) result:Tmatrix2_extended;
operator := (const v:Tmatrix2_extended) result:Tmatrix2_single;
operator := (const v:Tmatrix2_extended) result:Tmatrix2_double;

operator := (const v:Tmatrix2_single) result:Tmatrix3_single;
operator := (const v:Tmatrix2_single) result:Tmatrix3_double;
operator := (const v:Tmatrix2_single) result:Tmatrix3_extended;
operator := (const v:Tmatrix2_double) result:Tmatrix3_single;
operator := (const v:Tmatrix2_double) result:Tmatrix3_double;
operator := (const v:Tmatrix2_double) result:Tmatrix3_extended;
operator := (const v:Tmatrix2_extended) result:Tmatrix3_single;
operator := (const v:Tmatrix2_extended) result:Tmatrix3_double;
operator := (const v:Tmatrix2_extended) result:Tmatrix3_extended;

operator := (const v:Tmatrix2_single) result:Tmatrix4_single;
operator := (const v:Tmatrix2_single) result:Tmatrix4_double;
operator := (const v:Tmatrix2_single) result:Tmatrix4_extended;
operator := (const v:Tmatrix2_double) result:Tmatrix4_single;
operator := (const v:Tmatrix2_double) result:Tmatrix4_double;
operator := (const v:Tmatrix2_double) result:Tmatrix4_extended;
operator := (const v:Tmatrix2_extended) result:Tmatrix4_single;
operator := (const v:Tmatrix2_extended) result:Tmatrix4_double;
operator := (const v:Tmatrix2_extended) result:Tmatrix4_extended;

operator := (const v:Tmatrix3_single) result:Tmatrix2_single;
operator := (const v:Tmatrix3_single) result:Tmatrix2_double;
operator := (const v:Tmatrix3_single) result:Tmatrix2_extended;
operator := (const v:Tmatrix3_double) result:Tmatrix2_single;
operator := (const v:Tmatrix3_double) result:Tmatrix2_double;
operator := (const v:Tmatrix3_double) result:Tmatrix2_extended;
operator := (const v:Tmatrix3_extended) result:Tmatrix2_single;
operator := (const v:Tmatrix3_extended) result:Tmatrix2_double;
operator := (const v:Tmatrix3_extended) result:Tmatrix2_extended;

operator := (const v:Tmatrix3_single) result:Tmatrix3_double;
operator := (const v:Tmatrix3_single) result:Tmatrix3_extended;
operator := (const v:Tmatrix3_double) result:Tmatrix3_single;
operator := (const v:Tmatrix3_double) result:Tmatrix3_extended;
operator := (const v:Tmatrix3_extended) result:Tmatrix3_single;
operator := (const v:Tmatrix3_extended) result:Tmatrix3_double;

operator := (const v:Tmatrix3_single) result:Tmatrix4_single;
operator := (const v:Tmatrix3_single) result:Tmatrix4_double;
operator := (const v:Tmatrix3_single) result:Tmatrix4_extended;
operator := (const v:Tmatrix3_double) result:Tmatrix4_single;
operator := (const v:Tmatrix3_double) result:Tmatrix4_double;
operator := (const v:Tmatrix3_double) result:Tmatrix4_extended;
operator := (const v:Tmatrix3_extended) result:Tmatrix4_single;
operator := (const v:Tmatrix3_extended) result:Tmatrix4_double;
operator := (const v:Tmatrix3_extended) result:Tmatrix4_extended;

operator := (const v:Tmatrix4_single) result:Tmatrix2_single;
operator := (const v:Tmatrix4_single) result:Tmatrix2_double;
operator := (const v:Tmatrix4_single) result:Tmatrix2_extended;
operator := (const v:Tmatrix4_double) result:Tmatrix2_single;
operator := (const v:Tmatrix4_double) result:Tmatrix2_double;
operator := (const v:Tmatrix4_double) result:Tmatrix2_extended;
operator := (const v:Tmatrix4_extended) result:Tmatrix2_single;
operator := (const v:Tmatrix4_extended) result:Tmatrix2_double;
operator := (const v:Tmatrix4_extended) result:Tmatrix2_extended;

operator := (const v:Tmatrix4_single) result:Tmatrix3_single;
operator := (const v:Tmatrix4_single) result:Tmatrix3_double;
operator := (const v:Tmatrix4_single) result:Tmatrix3_extended;
operator := (const v:Tmatrix4_double) result:Tmatrix3_single;
operator := (const v:Tmatrix4_double) result:Tmatrix3_double;
operator := (const v:Tmatrix4_double) result:Tmatrix3_extended;
operator := (const v:Tmatrix4_extended) result:Tmatrix3_single;
operator := (const v:Tmatrix4_extended) result:Tmatrix3_double;
operator := (const v:Tmatrix4_extended) result:Tmatrix3_extended;

operator := (const v:Tmatrix4_single) result:Tmatrix4_double;
operator := (const v:Tmatrix4_single) result:Tmatrix4_extended;
operator := (const v:Tmatrix4_double) result:Tmatrix4_single;
operator := (const v:Tmatrix4_double) result:Tmatrix4_extended;
operator := (const v:Tmatrix4_extended) result:Tmatrix4_single;
operator := (const v:Tmatrix4_extended) result:Tmatrix4_double;

{Matrix to matrix operatons.}
operator + (const m1,m2:Tmatrix2_single) result:Tmatrix2_single;
operator + (const m1,m2:Tmatrix2_double) result:Tmatrix2_double;
operator + (const m1,m2:Tmatrix2_extended) result:Tmatrix2_extended;
operator + (const m1,m2:Tmatrix3_single) result:Tmatrix3_single;
operator + (const m1,m2:Tmatrix3_double) result:Tmatrix3_double;
operator + (const m1,m2:Tmatrix3_extended) result:Tmatrix3_extended;
operator + (const m1,m2:Tmatrix4_single) result:Tmatrix4_single;
operator + (const m1,m2:Tmatrix4_double) result:Tmatrix4_double;
operator + (const m1,m2:Tmatrix4_extended) result:Tmatrix4_extended;

operator - (const m1,m2:Tmatrix2_single) result:Tmatrix2_single;
operator - (const m1,m2:Tmatrix2_double) result:Tmatrix2_double;
operator - (const m1,m2:Tmatrix2_extended) result:Tmatrix2_extended;
operator - (const m1,m2:Tmatrix3_single) result:Tmatrix3_single;
operator - (const m1,m2:Tmatrix3_double) result:Tmatrix3_double;
operator - (const m1,m2:Tmatrix3_extended) result:Tmatrix3_extended;
operator - (const m1,m2:Tmatrix4_single) result:Tmatrix4_single;
operator - (const m1,m2:Tmatrix4_double) result:Tmatrix4_double;
operator - (const m1,m2:Tmatrix4_extended) result:Tmatrix4_extended;

operator - (const m1:Tmatrix2_single) result:Tmatrix2_single;
operator - (const m1:Tmatrix2_double) result:Tmatrix2_double;
operator - (const m1:Tmatrix2_extended) result:Tmatrix2_extended;
operator - (const m1:Tmatrix3_single) result:Tmatrix3_single;
operator - (const m1:Tmatrix3_double) result:Tmatrix3_double;
operator - (const m1:Tmatrix3_extended) result:Tmatrix3_extended;
operator - (const m1:Tmatrix4_single) result:Tmatrix4_single;
operator - (const m1:Tmatrix4_double) result:Tmatrix4_double;
operator - (const m1:Tmatrix4_extended) result:Tmatrix4_extended;

operator * (const m1,m2:Tmatrix2_single) result:Tmatrix2_single;
operator * (const m1,m2:Tmatrix2_double) result:Tmatrix2_double;
operator * (const m1,m2:Tmatrix2_extended) result:Tmatrix2_extended;
operator * (const m1,m2:Tmatrix3_single) result:Tmatrix3_single;
operator * (const m1,m2:Tmatrix3_double) result:Tmatrix3_double;
operator * (const m1,m2:Tmatrix3_extended) result:Tmatrix3_extended;
operator * (const m1,m2:Tmatrix4_single) result:Tmatrix4_single;
operator * (const m1,m2:Tmatrix4_double) result:Tmatrix4_double;
operator * (const m1,m2:Tmatrix4_extended) result:Tmatrix4_extended;

{Matrix/vector operations}
operator * (const m:Tmatrix2_single;const v:Tvector2_single) result:Tvector2_single;
operator * (const m:Tmatrix2_double;const v:Tvector2_double) result:Tvector2_double;
operator * (const m:Tmatrix2_extended;const v:Tvector2_extended) result:Tvector2_extended;
operator * (const m:Tmatrix3_single;const v:Tvector3_single) result:Tvector3_single;
operator * (const m:Tmatrix3_double;const v:Tvector3_double) result:Tvector3_double;
operator * (const m:Tmatrix3_extended;const v:Tvector3_extended) result:Tvector3_extended;
operator * (const m:Tmatrix4_single;const v:Tvector4_single) result:Tvector4_single;
operator * (const m:Tmatrix4_double;const v:Tvector4_double) result:Tvector4_double;
operator * (const m:Tmatrix4_extended;const v:Tvector4_extended) result:Tvector4_extended;

{Matrix/scalar operations}
operator + (const m:Tmatrix2_single;const x:single) result:Tmatrix2_single;
operator + (const m:Tmatrix2_double;const x:double) result:Tmatrix2_double;
operator + (const m:Tmatrix2_extended;const x:extended) result:Tmatrix2_extended;
operator + (const m:Tmatrix3_single;const x:single) result:Tmatrix3_single;
operator + (const m:Tmatrix3_double;const x:double) result:Tmatrix3_double;
operator + (const m:Tmatrix3_extended;const x:extended) result:Tmatrix3_extended;
operator + (const m:Tmatrix4_single;const x:single) result:Tmatrix4_single;
operator + (const m:Tmatrix4_double;const x:double) result:Tmatrix4_double;
operator + (const m:Tmatrix4_extended;const x:extended) result:Tmatrix4_extended;

operator - (const m:Tmatrix2_single;const x:single) result:Tmatrix2_single;
operator - (const m:Tmatrix2_double;const x:double) result:Tmatrix2_double;
operator - (const m:Tmatrix2_extended;const x:extended) result:Tmatrix2_extended;
operator - (const m:Tmatrix3_single;const x:single) result:Tmatrix3_single;
operator - (const m:Tmatrix3_double;const x:double) result:Tmatrix3_double;
operator - (const m:Tmatrix3_extended;const x:extended) result:Tmatrix3_extended;
operator - (const m:Tmatrix4_single;const x:single) result:Tmatrix4_single;
operator - (const m:Tmatrix4_double;const x:double) result:Tmatrix4_double;
operator - (const m:Tmatrix4_extended;const x:extended) result:Tmatrix4_extended;

operator * (const m:Tmatrix2_single;const x:single) result:Tmatrix2_single;
operator * (const m:Tmatrix2_double;const x:double) result:Tmatrix2_double;
operator * (const m:Tmatrix2_extended;const x:extended) result:Tmatrix2_extended;
operator * (const m:Tmatrix3_single;const x:single) result:Tmatrix3_single;
operator * (const m:Tmatrix3_double;const x:double) result:Tmatrix3_double;
operator * (const m:Tmatrix3_extended;const x:extended) result:Tmatrix3_extended;
operator * (const m:Tmatrix4_single;const x:single) result:Tmatrix4_single;
operator * (const m:Tmatrix4_double;const x:double) result:Tmatrix4_double;
operator * (const m:Tmatrix4_extended;const x:extended) result:Tmatrix4_extended;

operator / (const m:Tmatrix2_single;const x:single) result:Tmatrix2_single;
operator / (const m:Tmatrix2_double;const x:double) result:Tmatrix2_double;
operator / (const m:Tmatrix2_extended;const x:extended) result:Tmatrix2_extended;
operator / (const m:Tmatrix3_single;const x:single) result:Tmatrix3_single;
operator / (const m:Tmatrix3_double;const x:double) result:Tmatrix3_double;
operator / (const m:Tmatrix3_extended;const x:extended) result:Tmatrix3_extended;
operator / (const m:Tmatrix4_single;const x:single) result:Tmatrix4_single;
operator / (const m:Tmatrix4_double;const x:double) result:Tmatrix4_double;
operator / (const m:Tmatrix4_extended;const x:extended) result:Tmatrix4_extended;

{*****************************************************************************}

implementation

{*****************************************************************************}

{******************************************************************************
                                Tvector2_single
******************************************************************************}

{Need to use capitals due to bug in FPC. Bug was fixed in FPC 1.9.3 on
 10 Feb. 2004}
{$DEFINE datatype:=SINGLE}
{$DEFINE objectname:=Tvector2_single}
{$DEFINE vecsize:=2}
{$INFO Compile mvecimp.inc for Tvector2_single}
{$i mvecimp.inc}

{******************************************************************************
                                Tvector2_double
******************************************************************************}

{$DEFINE datatype:=DOUBLE}
{$DEFINE objectname:=Tvector2_double}
{$DEFINE vecsize:=2}
{$INFO Compile mvecimp.inc for Tvector2_double}
{$i mvecimp.inc}

{******************************************************************************
                               Tvector2_extended
******************************************************************************}

{$DEFINE datatype:=EXTENDED}
{$DEFINE objectname:=Tvector2_extended}
{$DEFINE vecsize:=2}
{$INFO Compile mvecimp.inc for Tvector2_extended}
{$i mvecimp.inc}

{******************************************************************************
                                Tvector3_single
******************************************************************************}

{Need to use capitals due to bug in FPC. Bug was fixed in FPC 1.9.3 on
 10 Feb. 2004}
{$DEFINE datatype:=SINGLE}
{$DEFINE objectname:=Tvector3_single}
{$DEFINE vecsize:=3}
{$INFO Compile mvecimp.inc for Tvector3_single}
{$i mvecimp.inc}

{******************************************************************************
                                Tvector3_double
******************************************************************************}

{$DEFINE datatype:=DOUBLE}
{$DEFINE objectname:=Tvector3_double}
{$DEFINE vecsize:=3}
{$INFO Compile mvecimp.inc for Tvector3_double}
{$i mvecimp.inc}

{******************************************************************************
                               Tvector3_extended
******************************************************************************}

{$DEFINE datatype:=EXTENDED}
{$DEFINE objectname:=Tvector3_extended}
{$DEFINE vecsize:=3}
{$INFO Compile mvecimp.inc for Tvector3_extended}
{$i mvecimp.inc}


{******************************************************************************
                                Tvector4_single
******************************************************************************}

{Need to use capitals due to bug in FPC. Bug was fixed in FPC 1.9.3 on
 10 Feb. 2004}
{$DEFINE datatype:=SINGLE}
{$DEFINE objectname:=Tvector4_single}
{$DEFINE vecsize:=4}
{$INFO Compile mvecimp.inc for Tvector4_single}
{$i mvecimp.inc}

{******************************************************************************
                                Tvector4_double
******************************************************************************}

{$DEFINE datatype:=DOUBLE}
{$DEFINE objectname:=Tvector4_double}
{$DEFINE vecsize:=4}
{$INFO Compile mvecimp.inc for Tvector4_double}
{$i mvecimp.inc}

{******************************************************************************
                               Tvector4_extended
******************************************************************************}

{$DEFINE datatype:=EXTENDED}
{$DEFINE objectname:=Tvector4_extended}
{$DEFINE vecsize:=4}
{$INFO Compile mvecimp.inc for Tvector4_extended}
{$i mvecimp.inc}

{******************************************************************************
                                Tmatrix2_single
******************************************************************************}

{$DEFINE datatype:=SINGLE}
{$DEFINE objectname:=Tmatrix2_single}
{$DEFINE vectorcompanion:=Tvector2_single}
{$DEFINE matsize:=2}
{$INFO Compile mmatimp.inc for Tmatrix2_single}
{$i mmatimp.inc}

{******************************************************************************
                                Tmatrix2_double
******************************************************************************}

{$DEFINE datatype:=DOUBLE}
{$DEFINE objectname:=Tmatrix2_double}
{$DEFINE vectorcompanion:=Tvector2_double}
{$DEFINE matsize:=2}
{$INFO Compile mmatimp.inc for Tmatrix2_double}
{$i mmatimp.inc}

{******************************************************************************
                               Tmatrix2_extended
******************************************************************************}

{$DEFINE datatype:=EXTENDED}
{$DEFINE objectname:=Tmatrix2_extended}
{$DEFINE vectorcompanion:=Tvector2_extended}
{$DEFINE matsize:=2}
{$INFO Compile mmatimp.inc for Tmatrix2_extended}
{$i mmatimp.inc}

{******************************************************************************
                                Tmatrix3_single
******************************************************************************}

{$DEFINE datatype:=SINGLE}
{$DEFINE objectname:=Tmatrix3_single}
{$DEFINE vectorcompanion:=Tvector3_single}
{$DEFINE matsize:=3}
{$INFO Compile mmatimp.inc for Tmatrix3_single}
{$i mmatimp.inc}

{******************************************************************************
                                Tmatrix3_double
******************************************************************************}

{$DEFINE datatype:=DOUBLE}
{$DEFINE objectname:=Tmatrix3_double}
{$DEFINE vectorcompanion:=Tvector3_double}
{$DEFINE matsize:=3}
{$INFO Compile mmatimp.inc for Tmatrix3_double}
{$i mmatimp.inc}

{******************************************************************************
                                Tmatrix3_extended
******************************************************************************}

{$DEFINE datatype:=EXTENDED}
{$DEFINE objectname:=Tmatrix3_extended}
{$DEFINE vectorcompanion:=Tvector3_extended}
{$DEFINE matsize:=3}
{$INFO Compile mmatimp.inc for Tmatrix3_extended}
{$i mmatimp.inc}

{******************************************************************************
                                Tmatrix4_single
******************************************************************************}

{$DEFINE datatype:=SINGLE}
{$DEFINE objectname:=Tmatrix4_single}
{$DEFINE vectorcompanion:=Tvector4_single}
{$DEFINE matsize:=4}
{$INFO Compile mmatimp.inc for Tmatrix4_single}
{$i mmatimp.inc}

{******************************************************************************
                                Tmatrix4_double
******************************************************************************}

{$DEFINE datatype:=DOUBLE}
{$DEFINE objectname:=Tmatrix4_double}
{$DEFINE vectorcompanion:=Tvector4_double}
{$DEFINE matsize:=4}
{$INFO Compile mmatimp.inc for Tmatrix4_double}
{$i mmatimp.inc}

{******************************************************************************
                               Tmatrix4_extended
******************************************************************************}

{$DEFINE datatype:=EXTENDED}
{$DEFINE objectname:=Tmatrix4_extended}
{$DEFINE vectorcompanion:=Tvector4_extended}
{$DEFINE matsize:=4}
{$INFO Compile mmatimp.inc for Tmatrix4_extended}
{$i mmatimp.inc}

{$else}
implementation
{$endif FPUNONE}
end.
