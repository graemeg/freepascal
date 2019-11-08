program RoundFunctionTest(output);

{$MODE ISO}
                                                                 {                      Expected result       }
                                                                 { FPC result     in accordance with ISO 7185 }
                                                                 { ----------     --------------------------- }
begin
  writeln('Testing the round() function with positive numbers:');
  writeln('round(0.5)   = ', round(0.5));                        {     0                          1           }
  if round(0.5)<>1 then
    halt(1);
  writeln('round(1.5)   = ', round(1.5));                        {     2                          2           }
  if round(1.5)<>2 then
    halt(1);
  writeln('round(2.5)   = ', round(2.5));                        {     2                          3           }
  if round(2.5)<>3 then
    halt(1);
  writeln('round(3.5)   = ', round(3.5));                        {     4                          4           }
  if round(3.5)<>4 then
    halt(1);
  writeln('round(4.5)   = ', round(4.5));                        {     4                          5           }
  if round(4.5)<>5 then
    halt(1);
  writeln('round(5.5)   = ', round(5.5));                        {     6                          6           }
  if round(5.5)<>6 then
    halt(1);
  writeln('round(10.5)  = ', round(10.5));                       {    10                         11           }
  if round(10.5)<>11 then
    halt(1);
  writeln('round(11.5)  = ', round(11.5));                       {    12                         12           }
  if round(11.5)<>12 then
    halt(1);
  writeln('round(12.5)  = ', round(12.5));                       {    12                         13           }
  if round(12.5)<>13 then
    halt(1);
  writeln;
  writeln('Testing the round() function with negative numbers:');
  writeln('round(-0.5)  = ', round(-0.5));                       {     0                         -1           }
  if round(-0.5)<>-1 then
    halt(1);
  writeln('round(-1.5)  = ', round(-1.5));                       {    -2                         -2           }
  if round(-1.5)<>-2 then
    halt(1);
  writeln('round(-2.5)  = ', round(-2.5));                       {    -2                         -3           }
  if round(-2.5)<>-3 then
    halt(1);
  writeln('round(-3.5)  = ', round(-3.5));                       {    -4                         -4           }
  if round(-3.5)<>-4 then
    halt(1);
  writeln('round(-4.5)  = ', round(-4.5));                       {    -4                         -5           }
  if round(-4.5)<>-5 then
    halt(1);
  writeln('round(-5.5)  = ', round(-5.5));                       {    -6                         -6           }
  if round(-5.5)<>-6 then
    halt(1);
  writeln('round(-10.5) = ', round(-10.5));                      {   -10                        -11           }
  if round(-10.5)<>-11 then
    halt(1);
  writeln('round-(11.5) = ', round(-11.5));                      {   -12                        -12           }
  if round(-11.5)<>-12 then
    halt(1);
  writeln('round(-12.5) = ', round(-12.5));                      {   -12                        -13           }
  if round(-12.5)<>-13 then
    halt(1);
  writeln
end.
