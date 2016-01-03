<<<<<<< HEAD
<<<<<<< HEAD
=======
const
  err: boolean = false;
>>>>>>> graemeg/fixes_2_2
=======
const
  err: boolean = false;
>>>>>>> origin/fixes_2_2

var
  a, b: array[0..515] of byte;
  bw: array[0..258] of word absolute b;
  bd: array[0..129] of dword absolute b;

procedure test_index;
type
  pword = ^word;
  pdword = ^cardinal;
var
  i, j, k: longint;
  index: longint;
  l: dword;
begin
  for i := 0 to 512 do
    a[i] := byte(i);
  for i := 0 to 256 do
    for j := 0 to 31 do
      for k := 0 to 31 do
        begin
          fillchar(b,sizeof(b),0);
          move(a[j],b[k+4],i);
          { for i = 256, every element appears in the array }
          if (byte(i)>0) then
            index:=i-1
          else
            index:=0;
          if indexbyte(b[k+4],index,a[j+i])<>-1 then
            begin
              writeln(indexbyte(b[k+4],index,a[j+1]),' <> -1');
              writeln('indexbyte error 1 for (',i,',',j,',',k,')');
              halt(1);
            end;
          if b[k+4]=0 then
            index:=0
          else if (b[k+4]+i>=256) then
            index:=256-b[k+4]
          else
            index:=i;
          if indexbyte(b[k+4],i+1,0)<>index then
            begin
              writeln(indexbyte(b[k+4],i+1,0),' <> ',index);
              writeln('indexbyte error 2 for (',i,',',j,',',k,')');
              halt(2);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          {same for length=-1}
          if indexbyte(b[k+4],-1,0)<>index then
            begin
              writeln(indexbyte(b[k+4],-1,0),' <> ',index);
              writeln('indexbyte error 2a for (',i,',',j,',',k,')');
              halt(22);
            end;
  
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

          if indexbyte(b[k+4],i,b[k+4+i-1])<>i-1 then
            begin
              writeln('indexbyte error 3 for (',i,',',j,',',k,')');
              halt(3);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          {same for length=-1}  
          if i<>0 then   // previous test will be no-op when i=0
            if indexbyte(b[k+4],-1,b[k+4+i-1])<>i-1 then
              begin
                writeln('indexbyte error 3a for (',i,',',j,',',k,')');
                halt(23);
              end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          if (i<1) then
            index:=-1
          else
            index:=i shr 1;
          if indexbyte(b[k+4],i,b[k+4+i shr 1])<>index then
            begin
              writeln(indexbyte(b[k+4],i,b[k+4+i shr 1]),' <> ',index);
              writeln('indexbyte error 4 for (',i,',',j,',',k,')');
              halt(4);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          {same for length=-1}
          if i<>0 then  // previous test will be no-op when i=0
            if indexbyte(b[k+4],-1,b[k+4+i shr 1])<>index then
              begin
                writeln(indexbyte(b[k+4],-1,b[k+4+i shr 1]),' <> ',index);
                writeln('indexbyte error 4a for (',i,',',j,',',k,')');
                halt(24);
              end;


=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
          if (i=0) then
            index:=-1
          else
            index:=0;
          if indexbyte(b[k+4],i,b[k+4])<>index then
            begin
              writeln('indexbyte error 5 for (',i,',',j,',',k,')');
<<<<<<< HEAD
<<<<<<< HEAD
              halt(5);
            end;
          {same for length=-1}
          if i<>0 then
            if indexbyte(b[k+4],-1,b[k+4])<>index then
              begin
                writeln('indexbyte error 5a for (',i,',',j,',',k,')');
                halt(25);
              end;
=======
              halt(3);
            end;
>>>>>>> graemeg/fixes_2_2
=======
              halt(3);
            end;
>>>>>>> origin/fixes_2_2


          if indexword(b[k+4],i shr 1,0)<>-1 then
            begin
              writeln('indexword error 6 for (',i,',',j,',',k,')');
              halt(6);
            end;

          if (unaligned(pword(@b[k+4])^)=0) then
            index:=0
          else if (i=0) then
            index:=-1
          else if (b[k+4+i-1] = 0) and
                  odd(i) then
            index:=((i+1) shr 1)-1
          else
            index:=((i+1) shr 1);
          if indexword(b[k+4],(i+1) shr 1 + 1,0)<>index then
            begin
              writeln(indexword(b[k+4],(i+1) shr 1 + 1,0),' <> ',index);
              writeln('indexword error 7 for (',i,',',j,',',k,')');
              halt(7);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          {same for length=-1}
          if indexword(b[k+4],-1,0)<>index then
            begin
              writeln(indexword(b[k+4],-1,0),' <> ',index);
              writeln('indexword error 7a for (',i,',',j,',',k,')');
              halt(27);
            end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

          if (i=0) then
            index:=0
          else
            index:=i shr 1;
          l:=unaligned(pword(@(b[k+4+(i and not 1)]))^);
          if indexword(b[k+4],i shr 1+1,l)<>index then
            begin
              writeln(indexword(b[k+4],((i and not 1)+1) shr 1+1,l),' <> ',index);
              writeln('indexword error 8 for (',i,',',j,',',k,')');
              halt(8);
            end;
<<<<<<< HEAD
<<<<<<< HEAD
          {same for length=-1}
          if indexword(b[k+4],-1,l)<>index then
            begin
              writeln(indexword(b[k+4],-1,l),' <> ',index);
              writeln('indexword error 8a for (',i,',',j,',',k,')');
              halt(28);
            end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

           l:=unaligned(pword(@(b[k+4+((i shr 2) and not 1)-2]))^);
           if (i>=8) then
             index:=((i shr 2) and not 1) shr 1 - 1
           else
             index:=-1;
           if indexword(b[k+4],i shr 1,l)<>index then
             begin
               writeln(indexword(b[k+4],i shr 1,l),' <> ',index);
               writeln('indexword error 9 for (',i,',',j,',',k,')');
               halt(9);
             end;
<<<<<<< HEAD
<<<<<<< HEAD
           if (i>1) and (index<>-1) then
             if indexword(b[k+4],-1,l)<>index then
               begin
                 writeln(indexword(b[k+4],-1,l),' <> ',index);
                 writeln('indexword error 9a for (',i,',',j,',',k,')');
                 halt(29);
               end;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
           l:=unaligned(pword(@(b[k+4]))^);
           if (i<2) then
             index:=-1
           else
             index:=0;
           if indexword(b[k+4],i shr 1,l)<>index then
             begin
               writeln('indexword error 10 for (',i,',',j,',',k,')');
               halt(10);
             end;
<<<<<<< HEAD
<<<<<<< HEAD
           if i>1 then
             if indexword(b[k+4],-1,l)<>index then
               begin
                 writeln('indexword error 10a for (',i,',',j,',',k,')');
                 halt(30);
               end;
=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2


           if (unaligned(pdword(@b[k+4])^)=0) then
             index:=0
           else if (i=0) then
             index:=-1
           else if (b[k+4+i-1] = 0) and
                   ((i mod 4) = 1) then
             index:=((i+3) shr 2)-1
           else
             index:=((i+3) shr 2);
           if indexdword(b[k+4],(i+3) shr 2 + 1,0)<>index then
             begin
               writeln(indexdword(b[k+4],(i+3) shr 2 + 1,0),' <> ',index);
               writeln('indexdword error 11 for (',i,',',j,',',k,')');
               halt(11);
             end;

           if (i=0) then
             index:=0
           else
             index:=i shr 2;
           l:=unaligned(pdword(@(b[k+4+(i and not 3)]))^);
           if indexdword(b[k+4],i shr 2+1,l)<>index then
             begin
               writeln('indexdword error 12 for (',i,',',j,',',k,')');
               halt(12);
             end;

           l:=unaligned(pdword(@(b[k+4+((i shr 3) and not 3)-4]))^);
           if (i>=32) then
             index:=((i shr 3) and not 3) shr 2 - 1
           else
             index:=-1;
           if indexdword(b[k+4],i shr 2,l)<>index then
             begin
               writeln(indexdword(b[k+4],i shr 2,l),' <> ',index);
               writeln('indexword error 13 for (',i,',',j,',',k,')');
               halt(13);
             end;
          l:=unaligned(pword(@(b[k+4]))^);
          if (i<4) then
            index:=-1
          else
            index:=0;
          if indexword(b[k+4],i shr 2,l)<>index then
            begin
              writeln('indexdword error 14 for (',i,',',j,',',k,')');
              halt(14);
            end;

        end;
end;


begin
  test_index;
end.
