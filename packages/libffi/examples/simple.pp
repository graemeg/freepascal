program simple;

uses
  ffi;

function WritePChar(s: PChar): LongInt;{$ifdef windows}stdcall;{$else}cdecl;{$endif}
begin
  Writeln(s);
  WritePChar := StrLen(s);
end;

var
  cif: ffi_cif;
  args: array[0..0] of pffi_type;
  values: array[0..0] of Pointer;
  s: PChar;
  rc: ffi_arg;
begin
  args[0] := @ffi_type_pointer;
  values[0] := @s;

  if ffi_prep_cif(@cif, FFI_DEFAULT_ABI, 1, @ffi_type_sint, @args[0]) = FFI_OK then begin
    s := 'Hello World';
    ffi_call(@cif, ffi_fn(@WritePChar), @rc, @values[0]);
    Writeln('Length: ', rc);

    s := 'This is cool!';
    ffi_call(@cif, ffi_fn(@WritePChar), @rc, @values[0]);
    Writeln('Length: ', rc);
  end else
    Writeln('ffi_prep_cif failed');

end.
