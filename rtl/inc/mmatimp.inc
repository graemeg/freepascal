{*****************************************************************************
                           Methods of the matrix object
*****************************************************************************}
constructor objectname.init_zero;

begin
  fillchar(data,sizeof(data),0);
end;

constructor objectname.init_identity;

begin
  fillchar(data,sizeof(data),0);
  data[0,0]:=1;
  data[1,1]:=1;
{$if matsize>=3}
  data[2,2]:=1;
{$endif}
{$if matsize>=4}
  data[3,3]:=1;
{$endif}
end;

constructor objectname.init(aa
                           ,ab
           {$if matsize>=3},ac{$endif}
           {$if matsize>=4},ad{$endif}
                           ,ba
                           ,bb
           {$if matsize>=3},bc{$endif}
           {$if matsize>=4},bd{$endif}
{$if matsize>=3}
                           ,ca
                           ,cb
                           ,cc
           {$if matsize>=4},cd{$endif}
{$endif}
{$if matsize>=4}
                           ,da
                           ,db
                           ,dc
                           ,dd
{$endif}:datatype);

begin
    data[0,0]:=aa;
    data[0,1]:=ab;
    {$if matsize>=3}data[0,2]:=ac;{$endif}
    {$if matsize>=4}data[0,3]:=ad;{$endif}
    data[1,0]:=ba;
    data[1,1]:=bb;
    {$if matsize>=3}data[1,2]:=bc;{$endif}
    {$if matsize>=4}data[1,3]:=bd;{$endif}
{$if matsize>=3}
    data[2,0]:=ca;
    data[2,1]:=cb;
    data[2,2]:=cc;
    {$if matsize>=4}data[2,3]:=cd;{$endif}
{$endif}
{$if matsize>=4}
    data[3,0]:=da;
    data[3,1]:=db;
    data[3,2]:=dc;
    data[3,3]:=dd;
{$endif}
end;

function objectname.get_column(c:byte):vectorcompanion;

begin
    get_column.data[0]:=data[0,c];
    get_column.data[1]:=data[1,c];
{$if matsize>=3}
    get_column.data[2]:=data[2,c];
{$endif}
{$if matsize>=4}
    get_column.data[3]:=data[3,c];
{$endif}
end;

function objectname.get_row(r:byte):vectorcompanion;

begin
    get_row.data:=data[r];
end;

procedure objectname.set_column(c:byte;const v:vectorcompanion);

begin
    data[0,c]:=v.data[0];
    data[1,c]:=v.data[1];
{$if matsize>=3}
    data[2,c]:=v.data[2];
{$endif}
{$if matsize>=4}
    data[3,c]:=v.data[3];
{$endif}
end;

procedure objectname.set_row(r:byte;const v:vectorcompanion);

begin
    data[r]:=v.data;
end;

function objectname.transpose:objectname;

begin
    transpose.data[0,0]:=data[0,0];
    transpose.data[0,1]:=data[1,0];
{$if matsize>=3}
    transpose.data[0,2]:=data[2,0];
{$endif}
{$if matsize>=4}
    transpose.data[0,3]:=data[3,0];
{$endif}
    transpose.data[1,0]:=data[0,1];
    transpose.data[1,1]:=data[1,1];
{$if matsize>=3}
    transpose.data[1,2]:=data[2,1];
{$endif}
{$if matsize>=4}
    transpose.data[1,3]:=data[3,1];
{$endif}
{$if matsize>=3}
    transpose.data[2,0]:=data[0,2];
    transpose.data[2,1]:=data[1,2];
    transpose.data[2,2]:=data[2,2];
{$endif}
{$if matsize>=4}
    transpose.data[2,3]:=data[3,2];
    transpose.data[3,0]:=data[0,3];
    transpose.data[3,1]:=data[1,3];
    transpose.data[3,2]:=data[2,3];
    transpose.data[3,3]:=data[3,3];
{$endif}
end;

{$if matsize=2}
function objectname.determinant:datatype;

begin
    determinant:=data[0,0]*data[1,1]-data[0,1]*data[1,0];
end;
{$endif}

{$if matsize=3}
function objectname.determinant:datatype;

begin
    determinant:=data[0,0]*(data[1,1]*data[2,2]-data[1,2]*data[2,1])-
                 data[0,1]*(data[1,0]*data[2,2]-data[1,2]*data[2,0])+
                 data[0,2]*(data[1,0]*data[2,1]-data[1,1]*data[2,0]);
end;
{$endif}

{$if matsize=4}
function objectname.determinant:datatype;

begin
    determinant:=(data[0,0]*data[1,1]-data[0,1]*data[1,0])*(data[2,2]*data[3,3]-data[2,3]*data[3,2])-
                 (data[0,0]*data[1,2]-data[0,2]*data[1,0])*(data[2,1]*data[3,3]-data[2,3]*data[3,1])+
                 (data[0,0]*data[1,3]-data[0,3]*data[1,0])*(data[2,1]*data[3,2]-data[2,2]*data[3,1])+
                 (data[0,1]*data[1,2]-data[0,2]*data[1,1])*(data[2,0]*data[3,3]-data[2,3]*data[3,0])-
                 (data[0,1]*data[1,3]-data[0,3]*data[1,1])*(data[2,0]*data[3,2]-data[2,2]*data[3,0])+
                 (data[0,2]*data[1,3]-data[0,3]*data[1,2])*(data[2,0]*data[3,1]-data[2,1]*data[3,0]);
end;
{$endif}

{$if matsize=2}
function objectname.inverse(Adeterminant:datatype):objectname;

begin
    Adeterminant:=1/Adeterminant;
    inverse.data[0,0]:=data[1,1]*Adeterminant;
    inverse.data[0,1]:=-data[0,1]*Adeterminant;
    inverse.data[1,0]:=-data[1,0]*Adeterminant;
    inverse.data[1,1]:=data[0,0]*Adeterminant;
end;
{$endif}

{$if matsize=3}
function objectname.inverse(Adeterminant:datatype):objectname;

begin
    Adeterminant:=1/Adeterminant;
    inverse.data[0,0]:=(data[1,1]*data[2,2]-data[2,1]*data[1,2])*Adeterminant;
    inverse.data[0,1]:=-(data[1,0]*data[2,2]-data[2,0]*data[1,2])*Adeterminant;
    inverse.data[0,2]:=(data[1,0]*data[2,1]-data[2,0]*data[1,1])*Adeterminant;
    inverse.data[1,0]:=-(data[0,1]*data[2,2]-data[2,1]*data[0,2])*Adeterminant;
    inverse.data[1,1]:=(data[0,0]*data[2,2]-data[2,0]*data[0,2])*Adeterminant;
    inverse.data[1,2]:=-(data[0,0]*data[2,1]-data[2,0]*data[0,1])*Adeterminant;
    inverse.data[2,0]:=(data[0,1]*data[1,2]-data[1,1]*data[0,2])*Adeterminant;
    inverse.data[2,1]:=-(data[0,0]*data[1,2]-data[1,0]*data[0,2])*Adeterminant;
    inverse.data[2,2]:=(data[0,0]*data[1,1]-data[1,0]*data[0,1])*Adeterminant;
end;
{$endif}

{$if matsize=4}
function objectname.inverse(Adeterminant:datatype):objectname;

begin
    Adeterminant:=1/Adeterminant;
    inverse.data[0,0]:=Adeterminant*(data[1,1]*(data[2,2]*data[3,3]-data[2,3]*data[3,2])+
                                     data[1,2]*(data[2,3]*data[3,1]-data[2,1]*data[3,3])+
                                     data[1,3]*(data[2,1]*data[3,2]-data[2,2]*data[3,1]));
    inverse.data[0,1]:=Adeterminant*(data[2,1]*(data[0,2]*data[3,3]-data[0,3]*data[3,2])+
                                     data[2,2]*(data[0,3]*data[3,1]-data[0,1]*data[3,3])+
                                     data[2,3]*(data[0,1]*data[3,2]-data[0,2]*data[3,1]));
    inverse.data[0,2]:=Adeterminant*(data[3,1]*(data[0,2]*data[1,3]-data[0,3]*data[1,2])+
                                     data[3,2]*(data[0,3]*data[1,1]-data[0,1]*data[1,3])+
                                     data[3,3]*(data[0,1]*data[1,2]-data[0,2]*data[1,1]));
    inverse.data[0,3]:=Adeterminant*(data[0,1]*(data[1,3]*data[2,2]-data[1,2]*data[2,3])+
                                     data[0,2]*(data[1,1]*data[2,3]-data[1,3]*data[2,1])+
                                     data[0,3]*(data[1,2]*data[2,1]-data[1,1]*data[2,2]));
    inverse.data[1,0]:=Adeterminant*(data[1,2]*(data[2,0]*data[3,3]-data[2,3]*data[3,0])+
                                     data[1,3]*(data[2,2]*data[3,0]-data[2,0]*data[3,2])+
                                     data[1,0]*(data[2,3]*data[3,2]-data[2,2]*data[3,3]));
    inverse.data[1,1]:=Adeterminant*(data[2,2]*(data[0,0]*data[3,3]-data[0,3]*data[3,0])+
                                     data[2,3]*(data[0,2]*data[3,0]-data[0,0]*data[3,2])+
                                     data[2,0]*(data[0,3]*data[3,2]-data[0,2]*data[3,3]));
    inverse.data[1,2]:=Adeterminant*(data[3,2]*(data[0,0]*data[1,3]-data[0,3]*data[1,0])+
                                     data[3,3]*(data[0,2]*data[1,0]-data[0,0]*data[1,2])+
                                     data[3,0]*(data[0,3]*data[1,2]-data[0,2]*data[1,3]));
    inverse.data[1,3]:=Adeterminant*(data[0,2]*(data[1,3]*data[2,0]-data[1,0]*data[2,3])+
                                     data[0,3]*(data[1,0]*data[2,2]-data[1,2]*data[2,0])+
                                     data[0,0]*(data[1,2]*data[2,3]-data[1,3]*data[2,2]));
    inverse.data[2,0]:=Adeterminant*(data[1,3]*(data[2,0]*data[3,1]-data[2,1]*data[3,0])+
                                     data[1,0]*(data[2,1]*data[3,3]-data[2,3]*data[3,1])+
                                     data[1,1]*(data[2,3]*data[3,0]-data[2,0]*data[3,3]));
    inverse.data[2,1]:=Adeterminant*(data[2,3]*(data[0,0]*data[3,1]-data[0,1]*data[3,0])+
                                     data[2,0]*(data[0,1]*data[3,3]-data[0,3]*data[3,1])+
                                     data[2,1]*(data[0,3]*data[3,0]-data[0,0]*data[3,3]));
    inverse.data[2,2]:=Adeterminant*(data[3,3]*(data[0,0]*data[1,1]-data[0,1]*data[1,0])+
                                     data[3,0]*(data[0,1]*data[1,3]-data[0,3]*data[1,1])+
                                     data[3,1]*(data[0,3]*data[1,0]-data[0,0]*data[1,3]));
    inverse.data[2,3]:=Adeterminant*(data[0,3]*(data[1,1]*data[2,0]-data[1,0]*data[2,1])+
                                     data[0,0]*(data[1,3]*data[2,1]-data[1,1]*data[2,3])+
                                     data[0,1]*(data[1,0]*data[2,3]-data[1,3]*data[2,0]));
    inverse.data[3,0]:=Adeterminant*(data[1,0]*(data[2,2]*data[3,1]-data[2,1]*data[3,2])+
                                     data[1,1]*(data[2,0]*data[3,2]-data[2,2]*data[3,0])+
                                     data[1,2]*(data[2,1]*data[3,0]-data[2,0]*data[3,1]));
    inverse.data[3,1]:=Adeterminant*(data[2,0]*(data[0,2]*data[3,1]-data[0,1]*data[3,2])+
                                     data[2,1]*(data[0,0]*data[3,2]-data[0,2]*data[3,0])+
                                     data[2,2]*(data[0,1]*data[3,0]-data[0,0]*data[3,1]));
    inverse.data[3,2]:=Adeterminant*(data[3,0]*(data[0,2]*data[1,1]-data[0,1]*data[1,2])+
                                     data[3,1]*(data[0,0]*data[1,2]-data[0,2]*data[1,0])+
                                     data[3,2]*(data[0,1]*data[1,0]-data[0,0]*data[1,1]));
    inverse.data[3,3]:=Adeterminant*(data[0,0]*(data[1,1]*data[2,2]-data[1,2]*data[2,1])+
                                     data[0,1]*(data[1,2]*data[2,0]-data[1,0]*data[2,2])+
                                     data[0,2]*(data[1,0]*data[2,1]-data[1,1]*data[2,0]));
end;
{$endif}

{*****************************************************************************
                            Conversion from matrix2
*****************************************************************************}


{$if (matsize<>2) or (datatype<>single)}
operator := (const v:Tmatrix2_single) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[0,3]:=0;
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[1,3]:=0;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=0;
  result.data[2,1]:=0;
  result.data[2,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[2,3]:=0;
  result.data[3,0]:=0;
  result.data[3,1]:=0;
  result.data[3,2]:=0;
  result.data[3,3]:=0;
{$endif}
end;
{$endif}

{$if (matsize<>2) or (datatype<>double)}
operator := (const v:Tmatrix2_double) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[0,3]:=0;
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[1,3]:=0;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=0;
  result.data[2,1]:=0;
  result.data[2,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[2,3]:=0;
  result.data[3,0]:=0;
  result.data[3,1]:=0;
  result.data[3,2]:=0;
  result.data[3,3]:=0;
{$endif}
end;
{$endif}

{$if (matsize<>2) or (datatype<>extended)}
operator := (const v:Tmatrix2_extended) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[0,3]:=0;
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[1,3]:=0;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=0;
  result.data[2,1]:=0;
  result.data[2,2]:=0;
{$endif}
{$if matsize>=4}
  result.data[2,3]:=0;
  result.data[3,0]:=0;
  result.data[3,1]:=0;
  result.data[3,2]:=0;
  result.data[3,3]:=0;
{$endif}
end;
{$endif}

{*****************************************************************************
                            Conversion from matrix3
*****************************************************************************}

{$if (matsize<>3) or (datatype<>single)}
operator := (const v:Tmatrix3_single) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=v.data[0,2];
{$endif}
{$if matsize>=4}
  result.data[0,3]:=0;
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=v.data[1,2];
{$endif}
{$if matsize>=4}
  result.data[1,3]:=0;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=v.data[2,0];
  result.data[2,1]:=v.data[2,1];
  result.data[2,2]:=v.data[2,2];
{$endif}
{$if matsize>=4}
  result.data[2,3]:=0;
  result.data[3,0]:=0;
  result.data[3,1]:=0;
  result.data[3,2]:=0;
  result.data[3,3]:=0;
{$endif}
end;
{$endif}

{$if (matsize<>3) or (datatype<>double)}
operator := (const v:Tmatrix3_double) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=v.data[0,2];
{$endif}
{$if matsize>=4}
  result.data[0,3]:=0;
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=v.data[1,2];
{$endif}
{$if matsize>=4}
  result.data[1,3]:=0;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=v.data[2,0];
  result.data[2,1]:=v.data[2,1];
  result.data[2,2]:=v.data[2,2];
{$endif}
{$if matsize>=4}
  result.data[2,3]:=0;
  result.data[3,0]:=0;
  result.data[3,1]:=0;
  result.data[3,2]:=0;
  result.data[3,3]:=0;
{$endif}
end;
{$endif}

{$if (matsize<>3) or (datatype<>extended)}
operator := (const v:Tmatrix3_extended) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=v.data[0,2];
{$endif}
{$if matsize>=4}
  result.data[0,3]:=0;
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=v.data[1,2];
{$endif}
{$if matsize>=4}
  result.data[1,3]:=0;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=v.data[2,0];
  result.data[2,1]:=v.data[2,1];
  result.data[2,2]:=v.data[2,2];
{$endif}
{$if matsize>=4}
  result.data[2,3]:=0;
  result.data[3,0]:=0;
  result.data[3,1]:=0;
  result.data[3,2]:=0;
  result.data[3,3]:=0;
{$endif}
end;
{$endif}

{*****************************************************************************
                            Conversion from matrix4
*****************************************************************************}

{$if (matsize<>4) or (datatype<>single)}
operator := (const v:Tmatrix4_single) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=v.data[0,2];
{$endif}
{$if matsize>=4}
  result.data[0,3]:=v.data[0,3];
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=v.data[1,2];
{$endif}
{$if matsize>=4}
  result.data[1,3]:=v.data[1,3];
{$endif}
{$if matsize>=3}
  result.data[2,0]:=v.data[2,0];
  result.data[2,1]:=v.data[2,1];
  result.data[2,2]:=v.data[2,2];
{$endif}
{$if matsize>=4}
  result.data[2,3]:=v.data[2,3];
  result.data[3,0]:=v.data[3,0];
  result.data[3,1]:=v.data[3,1];
  result.data[3,2]:=v.data[3,2];
  result.data[3,3]:=v.data[3,3];
{$endif}
end;
{$endif}

{$if (matsize<>4) or (datatype<>double)}
operator := (const v:Tmatrix4_double) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=v.data[0,2];
{$endif}
{$if matsize>=4}
  result.data[0,3]:=v.data[0,3];
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=v.data[1,2];
{$endif}
{$if matsize>=4}
  result.data[1,3]:=v.data[1,3];
{$endif}
{$if matsize>=3}
  result.data[2,0]:=v.data[2,0];
  result.data[2,1]:=v.data[2,1];
  result.data[2,2]:=v.data[2,2];
{$endif}
{$if matsize>=4}
  result.data[2,3]:=v.data[2,3];
  result.data[3,0]:=v.data[3,0];
  result.data[3,1]:=v.data[3,1];
  result.data[3,2]:=v.data[3,2];
  result.data[3,3]:=v.data[3,3];
{$endif}
end;
{$endif}

{$if (matsize<>4) or (datatype<>extended)}
operator := (const v:Tmatrix4_extended) result:objectname;

begin
  result.data[0,0]:=v.data[0,0];
  result.data[0,1]:=v.data[0,1];
{$if matsize>=3}
  result.data[0,2]:=v.data[0,2];
{$endif}
{$if matsize>=4}
  result.data[0,3]:=v.data[0,3];
{$endif}
  result.data[1,0]:=v.data[1,0];
  result.data[1,1]:=v.data[1,1];
{$if matsize>=3}
  result.data[1,2]:=v.data[1,2];
{$endif}
{$if matsize>=4}
  result.data[1,3]:=v.data[1,3];
{$endif}
{$if matsize>=3}
  result.data[2,0]:=v.data[2,0];
  result.data[2,1]:=v.data[2,1];
  result.data[2,2]:=v.data[2,2];
{$endif}
{$if matsize>=4}
  result.data[2,3]:=v.data[2,3];
  result.data[3,0]:=v.data[3,0];
  result.data[3,1]:=v.data[3,1];
  result.data[3,2]:=v.data[3,2];
  result.data[3,3]:=v.data[3,3];
{$endif}
end;
{$endif}

{*****************************************************************************
                           Matrix to matrix operations
*****************************************************************************}

operator + (const m1,m2:objectname) result:objectname;

{Add the elements of a matrix to each other.}

begin
    result.data[0,0]:=m1.data[0,0]+m2.data[0,0];
    result.data[0,1]:=m1.data[0,1]+m2.data[0,1];
{$if matsize>=3}
    result.data[0,2]:=m1.data[0,2]+m2.data[0,2];
{$endif}
{$if matsize>=4}
    result.data[0,3]:=m1.data[0,3]+m2.data[0,3];
{$endif}
    result.data[1,0]:=m1.data[1,0]+m2.data[1,0];
    result.data[1,1]:=m1.data[1,1]+m2.data[1,1];
{$if matsize>=3}
    result.data[1,2]:=m1.data[1,2]+m2.data[1,2];
{$endif}
{$if matsize>=4}
    result.data[1,3]:=m1.data[1,3]+m2.data[1,3];
{$endif}
{$if matsize>=3}
    result.data[2,0]:=m1.data[2,0]+m2.data[2,0];
    result.data[2,1]:=m1.data[2,1]+m2.data[2,1];
    result.data[2,2]:=m1.data[2,2]+m2.data[2,2];
{$endif}
{$if matsize>=4}
    result.data[2,3]:=m1.data[2,3]+m2.data[2,3];
    result.data[3,0]:=m1.data[3,0]+m2.data[3,0];
    result.data[3,1]:=m1.data[3,1]+m2.data[3,1];
    result.data[3,2]:=m1.data[3,2]+m2.data[3,2];
    result.data[3,3]:=m1.data[3,3]+m2.data[3,3];
{$endif}
end;

operator - (const m1,m2:objectname) result:objectname;

{Subtract the elements of two matrixes from each other.}

begin
    result.data[0,0]:=m1.data[0,0]-m2.data[0,0];
    result.data[0,1]:=m1.data[0,1]-m2.data[0,1];
{$if matsize>=3}
    result.data[0,2]:=m1.data[0,2]-m2.data[0,2];
{$endif}
{$if matsize>=4}
    result.data[0,3]:=m1.data[0,3]-m2.data[0,3];
{$endif}
    result.data[1,0]:=m1.data[1,0]-m2.data[1,0];
    result.data[1,1]:=m1.data[1,1]-m2.data[1,1];
{$if matsize>=3}
    result.data[1,2]:=m1.data[1,2]-m2.data[1,2];
{$endif}
{$if matsize>=4}
    result.data[1,3]:=m1.data[1,3]-m2.data[1,3];
{$endif}
{$if matsize>=3}
    result.data[2,0]:=m1.data[2,0]-m2.data[2,0];
    result.data[2,1]:=m1.data[2,1]-m2.data[2,1];
    result.data[2,2]:=m1.data[2,2]-m2.data[2,2];
{$endif}
{$if matsize>=4}
    result.data[2,3]:=m1.data[2,3]-m2.data[2,3];
    result.data[3,0]:=m1.data[3,0]-m2.data[3,0];
    result.data[3,1]:=m1.data[3,1]-m2.data[3,1];
    result.data[3,2]:=m1.data[3,2]-m2.data[3,2];
    result.data[3,3]:=m1.data[3,3]-m2.data[3,3];
{$endif}
end;

operator - (const m1:objectname) result:objectname;

{Negate the elements of a matrix.}

begin
    result.data[0,0]:=-m1.data[0,0];
    result.data[0,1]:=-m1.data[0,1];
{$if matsize>=3}
    result.data[0,2]:=-m1.data[0,2];
{$endif}
{$if matsize>=4}
    result.data[0,3]:=-m1.data[0,3];
{$endif}
    result.data[1,0]:=-m1.data[1,0];
    result.data[1,1]:=-m1.data[1,1];
{$if matsize>=3}
    result.data[1,2]:=-m1.data[1,2];
{$endif}
{$if matsize>=4}
    result.data[1,3]:=-m1.data[1,3];
{$endif}
{$if matsize>=3}
    result.data[2,0]:=-m1.data[2,0];
    result.data[2,1]:=-m1.data[2,1];
    result.data[2,2]:=-m1.data[2,2];
{$endif}
{$if matsize>=4}
    result.data[2,3]:=-m1.data[2,3];
    result.data[3,0]:=-m1.data[3,0];
    result.data[3,1]:=-m1.data[3,1];
    result.data[3,2]:=-m1.data[3,2];
    result.data[3,3]:=-m1.data[3,3];
{$endif}
end;

operator * (const m1,m2:objectname) result:objectname;

{Multiply two matrixes.}

var r:array[0..matsize-1] of datatype;
    i:byte;

begin
    for i:=0 to matsize-1 do
        begin
            r:=m1.data[i];
            result.data[i,0]:=r[0]*m2.data[0,0]
                             +r[1]*m2.data[1,0]
             {$if matsize>=3}+r[2]*m2.data[2,0]{$endif}
             {$if matsize>=4}+r[3]*m2.data[3,0]{$endif};
            result.data[i,1]:=r[0]*m2.data[0,1]
                             +r[1]*m2.data[1,1]
             {$if matsize>=3}+r[2]*m2.data[2,1]{$endif}
             {$if matsize>=4}+r[3]*m2.data[3,1]{$endif};
        {$if matsize>=3}
            result.data[i,2]:=r[0]*m2.data[0,2]
                             +r[1]*m2.data[1,2]
                             +r[2]*m2.data[2,2]
             {$if matsize>=4}+r[3]*m2.data[3,2]{$endif};
        {$endif}
        {$if matsize>=4}
            result.data[i,3]:=r[0]*m2.data[0,3]
                             +r[1]*m2.data[1,3]
                             +r[2]*m2.data[2,3]
                             +r[3]*m2.data[3,3];
        {$endif}
        end;
end;

{*****************************************************************************
                           Vector/matrix operations
*****************************************************************************}

operator * (const m:objectname;const v:vectorcompanion) result:vectorcompanion;

{Multiplies a matrix with a vector.}

begin
    result.data[0]:=m.data[0,0]*v.data[0]
                   +m.data[0,1]*v.data[1]
   {$if matsize>=3}+m.data[0,2]*v.data[2]{$endif}
   {$if matsize>=4}+m.data[0,3]*v.data[3]{$endif};
    result.data[1]:=m.data[1,0]*v.data[0]
                   +m.data[1,1]*v.data[1]
   {$if matsize>=3}+m.data[1,2]*v.data[2]{$endif}
   {$if matsize>=4}+m.data[1,3]*v.data[3]{$endif};
{$if matsize>=3}
    result.data[2]:=m.data[2,0]*v.data[0]
                   +m.data[2,1]*v.data[1]
                   +m.data[2,2]*v.data[2]
   {$if matsize>=4}+m.data[2,3]*v.data[3]{$endif};
{$endif}
{$if matsize>=4}
    result.data[3]:=m.data[3,0]*v.data[0]
                   +m.data[3,1]*v.data[1]
                   +m.data[3,2]*v.data[2]
                   +m.data[3,3]*v.data[3];
{$endif}
end;

{*****************************************************************************
                           Matrix/scalar operations
*****************************************************************************}

operator + (const m:objectname;const x:datatype) result:objectname;

{Adds to the elements of a matrix.}

begin
  result.data[0,0]:=m.data[0,0]+x;
  result.data[0,1]:=m.data[0,1]+x;
{$if matsize>=3}
  result.data[0,2]:=m.data[0,2]+x;
{$endif}
{$if matsize>=4}
  result.data[0,3]:=m.data[0,3]+x;
{$endif}
  result.data[1,0]:=m.data[1,0]+x;
  result.data[1,1]:=m.data[1,1]+x;
{$if matsize>=3}
  result.data[1,2]:=m.data[1,2]+x;
{$endif}
{$if matsize>=4}
  result.data[1,3]:=m.data[1,3]+x;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=m.data[2,0]+x;
  result.data[2,1]:=m.data[2,1]+x;
  result.data[2,2]:=m.data[2,2]+x;
{$endif}
{$if matsize>=4}
  result.data[2,3]:=m.data[2,3]+x;
  result.data[3,0]:=m.data[3,0]+x;
  result.data[3,1]:=m.data[3,1]+x;
  result.data[3,2]:=m.data[3,2]+x;
  result.data[3,3]:=m.data[3,3]+x;
{$endif}
end;

operator - (const m:objectname;const x:datatype) result:objectname;

{Subtracts from the elements of a matrix.}

begin
  result.data[0,0]:=m.data[0,0]-x;
  result.data[0,1]:=m.data[0,1]-x;
{$if matsize>=3}
  result.data[0,2]:=m.data[0,2]-x;
{$endif}
{$if matsize>=4}
  result.data[0,3]:=m.data[0,3]-x;
{$endif}
  result.data[1,0]:=m.data[1,0]-x;
  result.data[1,1]:=m.data[1,1]-x;
{$if matsize>=3}
  result.data[1,2]:=m.data[1,2]-x;
{$endif}
{$if matsize>=4}
  result.data[1,3]:=m.data[1,3]-x;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=m.data[2,0]-x;
  result.data[2,1]:=m.data[2,1]-x;
  result.data[2,2]:=m.data[2,2]-x;
{$endif}
{$if matsize>=4}
  result.data[2,3]:=m.data[2,3]-x;
  result.data[3,0]:=m.data[3,0]-x;
  result.data[3,1]:=m.data[3,1]-x;
  result.data[3,2]:=m.data[3,2]-x;
  result.data[3,3]:=m.data[3,3]-x;
{$endif}
end;

operator * (const m:objectname;const x:datatype) result:objectname;

{Multiplies the elements of a matrix.}

begin
  result.data[0,0]:=m.data[0,0]*x;
  result.data[0,1]:=m.data[0,1]*x;
{$if matsize>=3}
  result.data[0,2]:=m.data[0,2]*x;
{$endif}
{$if matsize>=4}
  result.data[0,3]:=m.data[0,3]*x;
{$endif}
  result.data[1,0]:=m.data[1,0]*x;
  result.data[1,1]:=m.data[1,1]*x;
{$if matsize>=3}
  result.data[1,2]:=m.data[1,2]*x;
{$endif}
{$if matsize>=4}
  result.data[1,3]:=m.data[1,3]*x;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=m.data[2,0]*x;
  result.data[2,1]:=m.data[2,1]*x;
  result.data[2,2]:=m.data[2,2]*x;
{$endif}
{$if matsize>=4}
  result.data[2,3]:=m.data[2,3]*x;
  result.data[3,0]:=m.data[3,0]*x;
  result.data[3,1]:=m.data[3,1]*x;
  result.data[3,2]:=m.data[3,2]*x;
  result.data[3,3]:=m.data[3,3]*x;
{$endif}
end;

operator / (const m:objectname;const x:datatype) result:objectname;

{Divides the elements of a matrix.

 In most cases, you will want to avoid this and multiply by the inverse.
 In case you need to preserve accuracy, dividing might be better though.}

begin
  result.data[0,0]:=m.data[0,0]/x;
  result.data[0,1]:=m.data[0,1]/x;
{$if matsize>=3}
  result.data[0,2]:=m.data[0,2]/x;
{$endif}
{$if matsize>=4}
  result.data[0,3]:=m.data[0,3]/x;
{$endif}
  result.data[1,0]:=m.data[1,0]/x;
  result.data[1,1]:=m.data[1,1]/x;
{$if matsize>=3}
  result.data[1,2]:=m.data[1,2]/x;
{$endif}
{$if matsize>=4}
  result.data[1,3]:=m.data[1,3]/x;
{$endif}
{$if matsize>=3}
  result.data[2,0]:=m.data[2,0]/x;
  result.data[2,1]:=m.data[2,1]/x;
  result.data[2,2]:=m.data[2,2]/x;
{$endif}
{$if matsize>=4}
  result.data[2,3]:=m.data[2,3]/x;
  result.data[3,0]:=m.data[3,0]/x;
  result.data[3,1]:=m.data[3,1]/x;
  result.data[3,2]:=m.data[3,2]/x;
  result.data[3,3]:=m.data[3,3]/x;
{$endif}
end;

