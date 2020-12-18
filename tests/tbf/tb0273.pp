{ %FAIL }

// EXPECTED: 'Error: Illegal function result type'
// ACTUAL: gets compiled
type M = function : file;

begin
end.
