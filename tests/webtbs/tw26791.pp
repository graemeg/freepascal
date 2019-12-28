{ %opt=-vw -Sew }
{ %cpu=x86_64,i386 }
Function Fast_ReciprocalSquareRoot(number: Single): Single;
    Begin
        Asm
            RSQRTSS number, %xmm0
            MOVLPS %xmm0, __Result
        End ['xmm0'];
    End;
    
begin
end.
