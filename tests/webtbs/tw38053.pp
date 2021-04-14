{ %opt=-Sg }

program fsz;
// fpc -OoREGVAR az.pas; ./az
// fpc -OoNoREGVAR az.pas; ./az

// {$mode delphi}
{$mode objfpc}

Type
  ByteRA0 = array [0..0] of byte;
  Bytep0 = ^ByteRA0;
TNIFTIhdr =  record //Next: analyze Format Header structure
   HdrSz : longint; //MUST BE 348
end;

function readTiff(fnm: string; nhdr: TNIFTIhdr; img: byteP0): string;
label
  555;
type
  TTIFFhdr =  record
        Compression, ImageHeight, ImageWidth,
        NewSubfileType: uint32;
  end;
const
  kMaxIFD = 2200;
  kVal = 1050090;
var
  fsz, i, ok1: integer{int64};
  jj,w, nTag, nIFD: uint32 {uint64};
  hdr: array[1..kMaxIFD] of TTIFFhdr;
begin
	result := '';
	fsz := kVal;
	ok1 := kVal;
        jj  := kVal;
        w := kVal;
        i := kVal;
        nTag := kVal;
        nIFD := kVal;
	writeln('Value ', fsz,' ',ok1,' ',jj,' ',w,' ',i,' ',nTag,' ',nIFD,' -- ', kVal);
        if fsz <> kVal then
          halt(1);
        if ok1 <> kVal then
          halt(2);
        if jj <> kVal then
          halt(3);
        if w <> kVal then
          halt(4);
        if i <> kVal then
          halt(5);
        if nTag <> kVal then
          halt(6);
        if nIFD <> kVal then
          halt(7);
	555:
end;

procedure ReportTiff();
var
	img: byteP0;
    nhdr: TNIFTIhdr;
begin
  readTiff('xxx', nhdr, img);
end;

begin
	ReportTiff();
end.
