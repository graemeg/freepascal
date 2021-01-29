{ %norun }
type
    measure = (short := 1, long := 2);
    generic bar<const x: measure> = object
            public
                const
                    myMeasure = ord(x);
        end;
begin
end.
