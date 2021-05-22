{
    Copyright (c) 2021 Karoly Balogh

    Test system record/structure sizes on a Sinclair QL
    A test program for Free Pascal's Sinclair QL support

    This test program is in the Public Domain under the terms of
    Unlicense: http://unlicense.org/

 **********************************************************************}

program trecsize;

uses
  qdos;

type
  size_test = record
    name: string[16];
    size: longint;
    size_of: longint;
  end;

const
  record_sizes: array of size_test = (
    { extend with more, as needed }
    ( name: 'TQDOS_QUEUE'; size: QDOSQUEUE_SIZE; size_of: sizeof(Tqdos_queue) ),
    ( name: 'TCHAN_DEFB'; size: CHAN_DEFBSIZE; size_of: sizeof(Tchan_defb) ),
    ( name: 'TSER_CDEFB'; size: SER_CDEFBSIZE; size_of: sizeof(Tser_cdefb) ),
    ( name: 'TNET_CDEFB'; size: NET_CDEFBSIZE; size_of: sizeof(Tnet_cdefb) ),
    ( name: 'TSCRN_INFO'; size: SCRN_INFOSIZE; size_of: sizeof(Tscrn_info) ),
    ( name: 'TSCR_CDEFB'; size: SCR_CDEFBSIZE; size_of: sizeof(Tscr_cdefb) ),
    ( name: 'TCON_CDEFB'; size: CON_CDEFBSIZE; size_of: sizeof(Tcon_cdefb) ),
    ( name: 'TFS_CDEFB'; size: FS_CDEFBSIZE; size_of: sizeof(Tfs_cdefb) )
  );

function test_record_sizes: boolean;
var
  i: longint;
begin
  test_record_sizes:=false;
  for i:=low(record_sizes) to high(record_sizes) do
    begin
      with record_sizes[i] do
        begin
          writeln(name,' is ',size_of,' bytes, expected: ',size);
          if size_of <> size then
            exit;
        end;
    end;
  test_record_sizes:=true;
end;

begin
  if test_record_sizes then
    writeln('All OK!')
  else
    writeln('Error! Wrong size!');
end.
