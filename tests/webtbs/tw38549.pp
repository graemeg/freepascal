type

{$ifdef SET_39}
  {$define SET_31}
{$endif}
{$ifdef SET_31}
  {$define SET_25}
{$endif}
{$ifdef SET_25}
  {$define SET_23}
{$endif}
{$ifdef SET_23}
  {$define SET_17}
{$endif}
{$ifdef SET_17}
  {$define SET_15}
{$endif}
{$ifdef SET_15}
  {$define SET_9}
{$endif}

  { options for symtables }
  tsymtableoption = (
    sto_has_helper,       { contains at least one helper symbol }
    sto_has_generic,      { contains at least one generic symbol }
    sto_has_operator,     { contains at least one operator overload }
    sto_needs_init_final, { the symtable needs initialization and/or
                            finalization of variables/constants }
    sto_has_non_trivial_init, { contains at least on managed type that is not
                               initialized to zero (e.g. a record with management
                               operators }
    sto_above
{$ifdef SET_9}
    ,sto_6
    ,sto_7
    ,sto_8
    ,sto_9
{$endif}
{$ifdef SET_15}
    ,sto_10
    ,sto_11
    ,sto_12
    ,sto_13
    ,sto_14
    ,sto_15
{$endif}
{$ifdef SET_17}
    ,sto_16
    ,sto_17
{$endif}
{$ifdef SET_23}
    ,sto_18
    ,sto_19
    ,sto_20
    ,sto_21
    ,sto_22
    ,sto_23
{$endif}
{$ifdef SET_25}
    ,sto_24
    ,sto_25
{$endif}
{$ifdef SET_31}
    ,sto_26
    ,sto_27
    ,sto_28
    ,sto_29
    ,sto_30
    ,sto_31
{$endif}
{$ifdef SET_39}
    ,sto_32
    ,sto_33
    ,sto_34
    ,sto_35
    ,sto_36
    ,sto_37
    ,sto_38
    ,sto_39
{$endif}
  );
  tsymtableoptions = set of tsymtableoption;

const
  ok_count : longint = 0;
  error_count : longint = 0;

procedure add_error;
begin
  writeln('New error');
  inc(error_count);
end; 

procedure test(tableoptions : tsymtableoptions; expected : boolean);
begin
 if [sto_needs_init_final,sto_has_non_trivial_init] <= tableoptions then
   begin
     if expected then
       begin
         writeln('Ok');
         inc(ok_count);
       end
     else
       add_error;
   end
 else
   begin
     if not expected then
       begin
         writeln('Ok');
         inc(ok_count);
       end
     else
       add_error;
   end;
 if tableoptions >= [sto_needs_init_final,sto_has_non_trivial_init] then
   begin
     if expected then
       begin
         writeln('Ok');
         inc(ok_count);
       end
     else
       add_error;
   end
 else
   begin
     if not expected then
       begin
         writeln('Ok');
         inc(ok_count);
       end
     else
       add_error;
   end
end;

procedure test2(tableoptions1, tableoptions2 : tsymtableoptions; expected : boolean);
begin
 if tableoptions1 <= tableoptions2 then
   begin
     if expected then
       begin
         writeln('Ok');
         inc(ok_count);
       end
     else
       add_error;
   end
 else
   begin
     if not expected then
       begin
         writeln('Ok');
         inc(ok_count);
       end
     else
       add_error;
   end
end;

var
  tableoptions1, tableoptions2 : tsymtableoptions;

begin
  tableoptions1:=[];
  test(tableoptions1,false);

  tableoptions1:=[sto_has_helper];
  test(tableoptions1,false);

  tableoptions1:=[sto_needs_init_final];
  test(tableoptions1,false);

  tableoptions1:=[sto_has_non_trivial_init];
  test(tableoptions1,false);

  tableoptions1:=[sto_needs_init_final,sto_has_non_trivial_init];
  test(tableoptions1,true);

  tableoptions1:=[sto_has_helper,sto_needs_init_final,sto_has_non_trivial_init];
  test(tableoptions1,true);

  tableoptions1:=[sto_has_helper,sto_needs_init_final,sto_has_non_trivial_init,sto_above];
  test(tableoptions1,true);

  tableoptions1:=[sto_has_helper,sto_has_non_trivial_init,sto_above];
  test(tableoptions1,false);

  tableoptions1:=[];
  tableoptions2:=[];
  test2(tableoptions1,tableoptions2,true);
  test2(tableoptions2,tableoptions1,true);

  tableoptions2:=[sto_has_helper];
  test2(tableoptions1,tableoptions2,true);
  test2(tableoptions2,tableoptions1,false);

  tableoptions1:=[sto_needs_init_final,sto_has_non_trivial_init];
  tableoptions2:=[sto_needs_init_final,sto_has_non_trivial_init,sto_has_helper];
  test2(tableoptions1,tableoptions2,true);
  test2(tableoptions2,tableoptions1,false);
  test2(tableoptions1,tableoptions1,true);
  test2(tableoptions2,tableoptions2,true);

  tableoptions1:=[sto_needs_init_final,sto_has_non_trivial_init];
  tableoptions2:=[sto_has_helper,sto_needs_init_final,sto_has_non_trivial_init];
  test2(tableoptions1,tableoptions2,true);
  test2(tableoptions2,tableoptions1,false);

  tableoptions1:=[sto_has_helper,sto_needs_init_final,sto_has_non_trivial_init];
  tableoptions2:=[sto_needs_init_final,sto_has_non_trivial_init,sto_above];
  test2(tableoptions1,tableoptions2,false);
  test2(tableoptions2,tableoptions1,false);

  writeln('Test for sets of size : ',sizeof(tableoptions1));
  if error_count > 0 then
    begin
      writeln(error_count,' test(s) failed');
      writeln(ok_count,' test(s) OK');
      halt(1);
    end
  else
    writeln('Test OK: ',ok_count);
end.

