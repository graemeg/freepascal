{ %opt=-O3 -Sg }
{$mode objfpc} {$longstrings+}
label start1, end1, start2, end2, start3, end3, start4, end4;

var
	s: string;

begin
	writeln('31 concatenated string literals, completely folded:');
start1:
	s :=
		'Once like a Great House' + LineEnding +
		'founded on sand,' + LineEnding +
		'Stood our Temple' + LineEnding +
		'whose pillars on troubles were based.' + LineEnding +
		'Now mischievous spirits, bound,' + LineEnding +
		'in dim corners stand,' + LineEnding +
		'Rotted columns, but' + LineEnding +
		'with iron-bound bands embraced' + LineEnding +
		'Cracked, crumbling marble,' + LineEnding +
		'tempered on every hand,' + LineEnding +
		'By strong steel' + LineEnding +
		'forged in fire and faith.' + LineEnding +
		'Shackled, these wayward servants' + LineEnding +
		'serve the land,' + LineEnding +
		'The Temple secured' + LineEnding +
		'by the Builder’s grace.';
end1:
    writeln(Copy(s, 1, 0), PtrUint(CodePointer(@end1) - CodePointer(@start1)), ' b of code');
    { more than 100 bytes of code might point out that the constants are not folded }
    if PtrUint(CodePointer(@end1) - CodePointer(@start1))>100 then
      halt(1);
	writeln;

	writeln('1 dynamic string concatenated with 31 literals, they could fold but didn''t at all:');
start2:
	s := Copy('', 1, 0) +
		'Once like a Great House' + LineEnding +
		'founded on sand,' + LineEnding +
		'Stood our Temple' + LineEnding +
		'whose pillars on troubles were based.' + LineEnding +
		'Now mischievous spirits, bound,' + LineEnding +
		'in dim corners stand,' + LineEnding +
		'Rotted columns, but' + LineEnding +
		'with iron-bound bands embraced' + LineEnding +
		'Cracked, crumbling marble,' + LineEnding +
		'tempered on every hand,' + LineEnding +
		'By strong steel' + LineEnding +
		'forged in fire and faith.' + LineEnding +
		'Shackled, these wayward servants' + LineEnding +
		'serve the land,' + LineEnding +
		'The Temple secured' + LineEnding +
		'by the Builder’s grace.';
end2:
    writeln(Copy(s, 1, 0), PtrUint(CodePointer(@end2) - CodePointer(@start2)), ' b of code');
    { more than 100 bytes of code might point out that the constants are not folded,
      example x86_64-linux: not folded: 639 bytes; folded: 76 bytes
    }
    if PtrUint(CodePointer(@end2) - CodePointer(@start2))>100 then
      halt(2);
	writeln;

	writeln('16 literals concatenated with 1 dynamic string and 15 more literals, first 16 folded but last 15 did not:');
start3:
	s :=
		'Once like a Great House' + LineEnding +
		'founded on sand,' + LineEnding +
		'Stood our Temple' + LineEnding +
		'whose pillars on troubles were based.' + LineEnding +
		'Now mischievous spirits, bound,' + LineEnding +
		'in dim corners stand,' + LineEnding +
		'Rotted columns, but' + LineEnding +
		'with iron-bound bands embraced' + LineEnding +
		Copy('', 1, 0) +
		'Cracked, crumbling marble,' + LineEnding +
		'tempered on every hand,' + LineEnding +
		'By strong steel' + LineEnding +
		'forged in fire and faith.' + LineEnding +
		'Shackled, these wayward servants' + LineEnding +
		'serve the land,' + LineEnding +
		'The Temple secured' + LineEnding +
		'by the Builder’s grace.';
end3:
	writeln(Copy(s, 1, 0), PtrUint(CodePointer(@end3) - CodePointer(@start3)), ' b of code');
    { more than 300 bytes of code might point out that the constants are not folded,
      example x86_64-linux: not folded: 369 bytes; folded: 120 bytes
    }
    if PtrUint(CodePointer(@end3) - CodePointer(@start3))>300 then
      halt(3);
    writeln;

	writeln('31 literals concatenated with 1 dynamic string, they could fold but didn''t at all:');
start4:
	s := 'Once like a Great House' + (LineEnding +
		('founded on sand,' + (LineEnding +
		('Stood our Temple' + (LineEnding +
		('whose pillars on troubles were based.' + (LineEnding +
		('Now mischievous spirits, bound,' + (LineEnding +
		('in dim corners stand,' + (LineEnding +
		('Rotted columns, but' + (LineEnding +
		('with iron-bound bands embraced' + (LineEnding +
		('Cracked, crumbling marble,' + (LineEnding +
		('tempered on every hand,' + (LineEnding +
		('By strong steel' + (LineEnding +
		('forged in fire and faith.' + (LineEnding +
		('Shackled, these wayward servants' + (LineEnding +
		('serve the land,' + (LineEnding +
		('The Temple secured' + (LineEnding +
		('by the Builder’s grace.' +
		Copy('', 1, 0)))))))))))))))))))))))))))))));
end4:
    writeln(Copy(s, 1, 0), PtrUint(CodePointer(@end4) - CodePointer(@start4)), ' b of code');
    { more than 100 bytes of code might point out that the constants are not folded,
      example x86_64-linux: not folded: 1384 bytes; folded: 108 bytes
    }
    if PtrUint(CodePointer(@end4) - CodePointer(@start4))>300 then
      halt(4);


    writeln('ok');
end.
