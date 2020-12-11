{$mode objfpc}

{ Tests proper setting of fc_no_direct_exit in flowcontrol when the exit statement jumps
  to an extra code instead of immediately finishing execution of the current routine. }

type
  TSymtable = class
  public
    name      : pshortstring;
    realname  : pshortstring;
    DefList   : TObject;
    SymList   : TObject;
    refcount  : smallint;
    destructor  destroy;override;
    procedure clear;virtual;
    procedure freeinstance;override;
    procedure test_inline_with_exit;virtual;
  end;

var
  st: TSymtable;

  procedure stringdispose(var p : pshortstring); inline;
  begin
   if assigned(p) then
     begin
       freemem(p);
       p:=nil;
     end;
  end;

  procedure cproc(a,b,c: longint); cdecl;
  begin
  end;

  procedure inline_with_exit(a,b,c: longint); inline;
  begin
    if a = 12345 then
      exit;
    cproc(a,b,c);
  end;

  procedure inline_error;
  begin
    writeln('Inline with exit error.');
    halt(3);
  end;

  procedure TSymtable.test_inline_with_exit;
  var
    i,j: integer;
  begin
    i:=12345;
    j:=1;
    stringdispose(name);
    inline_with_exit(i,j,i+j);
    if i<>12345 then
      inline_error;
    Inc(i);
    Inc(j);
    stringdispose(name);
  end;

  procedure TSymtable.clear;
  begin
  end;

  destructor TSymtable.destroy;
    var i: longint;
    begin
      i:=1;
      if refcount=0 then
        exit;
      Clear;
      DefList.Free;
      SymList.Free;
      stringdispose(name);
      stringdispose(realname);
      refcount:=i;
      { freeinstance is implicitly called here even when 'exit' is executed }
    end;

    procedure TSymtable.freeinstance;
      begin
        writeln('freeinstance');
        if Self <> st then
          begin
            writeln('Incorrect self.');
            Halt(1);
          end;

        inherited freeinstance;
        st:=nil;
      end;

begin
  st:=TSymtable.Create;
  st.test_inline_with_exit;
  st.Free;
  if st <> nil then
    begin
      writeln('freeinstance has not called.');
      Halt(1);
    end;
  writeln('OK');
end.
