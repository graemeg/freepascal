program example;

uses fftw;

const s=128;

var i,o:Pcomplex_single;
    p:fftw_plan_single;
    a:cardinal;

begin
  fftw_getmem(i,s*sizeof(complex_single));
     
  fftw_getmem(o,s*sizeof(complex_single));
  p:=fftw_plan_dft_1d(128,i,o,fftw_forward,[fftw_estimate]);
  for a:=0 to 127 do
     begin
         i[a].re:=(single(a)-64);
         i[a].im:=0;
     end;
  writeln('input:');
  for a:=0 to 127 do
    writeln('(',i[a].re:8:4,',',i[a].im:8:4,')');
  fftw_execute(p);
  writeln('output:');
  for a:=0 to 127 do
    writeln('(',o[a].re:8:4,',',o[a].im:8:4,')');
  fftw_destroy_plan(p);
  fftw_freemem(i);
  fftw_freemem(o);
end.