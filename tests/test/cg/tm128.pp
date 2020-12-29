{$ASSERTIONS ON}

type
  tm128 = record
    case byte of
      1 : (m128_f32 : array[0..3] of single;)
  end align 16;

type
  tm128_unaligned = record
    case byte of
      1 : (m128_f32 : array[0..3] of single;)
  end;


var
  tr1 : record
    b : byte;
    m128 : tm128;
  end;
  tr1_unaligned : record
    b : byte;
    m128_unaligned : tm128_unaligned;
  end;
  d : double;

begin
  assert((sizeof(tm128))=16);
  assert((ptruint(@tr1.m128) mod 16)=0);
  assert((ptruint(@tr1_unaligned.m128_unaligned)-ptruint(@tr1_unaligned.b))=4);
end.
