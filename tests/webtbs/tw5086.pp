{
TASK:water
LANG:PASCAL
}
program water;

const
	MAXV					= 200;

type
	Byte					= Integer;
	TMove					= record
		a, b, c				: Byte;
	end;

var
	used					: Array [0 .. MAXV, 0 .. MAXV, 0 .. MAXV] of Boolean;
	v, k, a1, a2, b1, b2, c1, c2		: Byte;
	answ					: LongInt;

procedure readInput;
begin
        assign(input,'tw5086.tmp');
        reset(input);
	readln(v, k);
	readln(a1, a2, b1, b2, c1, c2);
  close(input);
  erase(input);
end;

var
  p : pbyte;

procedure dfs(a, b, c, k1: Byte);
var
	x					: Byte;
	d					: Array [1 .. 100] of TMove;
	dn, i					: Byte;

procedure add(a, b, c: Byte);
var
	i					: Integer;
	fnd					: Boolean;
begin
	fnd := False;
	for i := 1 to dn do
		if (d[i].a = a) and (d[i].b = b) and (d[i].c = c) then begin
			fnd := True;
			break;
		end;
	if (not fnd) then begin
		dn := dn + 1;
		d[dn].a := a; d[dn].b := b; d[dn].c := c;
	end;
end;

begin
//    p:=@a;

	if (k1 > k) then Exit;
	if (a = 1) or (b = 1) or (c = 1) then begin
		answ := answ + 1;
		Exit;
	end;
	used[a, b, c] := True;
	dn := 0;
	if (a > 0) then begin
		if (b+a <= v) and (not used[0, b+a, c]) then
			add(0, b+a, c);
		if (c+a <= v) and (not used[0, b, c+a]) then
			add(0, b, c+a);
		if (a-a1 > 0) then begin
			x := a-a1;
			if (b+x <= v) and (not used[a1, b+x, c]) then
				add(a1, b+x, c);
			if (c+x <= v) and (not used[a1, b, c+x]) then
				add(a1, b, c+x);
		end;
		if (a-a2 > 0) then begin
			x := a-a2;
			if (b+x <= v) and (not used[a2, b+x, c]) then
				add(a2, b+x, c);
			if (c+x <= v) and (not used[a2, b, c+x]) then
				add(a2, b, c+x);
		end;
		if (b1-b > 0) then begin
			x := b1-b;
			if (a-x > 0) and (not used[a-x, b1, c]) then
				add(a-x, b1, c);
		end;
		if (b2-b > 0) then begin
			x := b2-b;
			if (a-x > 0) and (not used[a-x, b2, c]) then
				add(a-x, b2, c);
		end;
		if (c1-c > 0) then begin
			x := c1-c;
			if (a-x > 0) and (not used[a-x, b, c1]) then
				add(a-x, b, c1);
		end;
		if (c2-c > 0) then begin
			x := c2-c;
			if (a-x > 0) and (not used[a-x, b, c2]) then
				add(a-x, b, c2);
		end;
	end;
	for i := 1 to dn do
		dfs(d[i].a, d[i].b, d[i].c, k1+1);
	dn := 0;
	if (b > 0) then begin
		if (b+a <= v) and (not used[a+b, 0, c]) then
			add(a+b, 0, c);
		if (c+b <= v) and (not used[a, 0, c+b]) then
			add(a, 0, c+b);
		if (b-b1 > 0) then begin
			x := b-b1;
			if (a+x <= v) and (not used[a+x, b1, c]) then
				add(a+x, b1, c);
			if (c+x <= v) and (not used[a, b1, c+x]) then
				add(a, b1, c+x);
		end;
		if (b-b2 > 0) then begin
			x := b-b2;
			if (a+x <= v) and (not used[a+x, b2, c]) then
				add(a+x, b2, c);
			if (c+x <= v) and (not used[a, b2, c+x]) then
				add(a, b2, c+x);
		end;
		if (a1-a > 0) then begin
			x := a1-a;
			if (b-x > 0) and (not used[a1, b-x, c]) then
				add(a1, b-x, c);
		end;
		if (a2-a > 0) then begin
			x := a2-a;
			if (b-x > 0) and (not used[a2, b-x, c]) then
				add(a2, b-x, c);
		end;
		if (c1-c > 0) then begin
			x := c1-c;
			if (b-x > 0) and (not used[a, b-x, c1]) then
				add(a, b-x, c1);
		end;
		if (c2-c > 0) then begin
			x := c2-c;
			if (b-x > 0) and (not used[a, b-x, c2]) then
				add(a, b-x, c2);
		end;
	end;
	for i := 1 to dn do
		dfs(d[i].a, d[i].b, d[i].c, k1+1);
	dn := 0;
	if (c > 0) then begin
		if (b+c <= v) and (not used[a, b+c, 0]) then
			add(a, b+c, 0);
		if (c+a <= v) and (not used[a+c, b, 0]) then
			add(a+c, b, 0);
		if (c-c1 > 0) then begin
			x := c-c1;
			if (a+x <= v) and (not used[a+x, b, c1]) then
				add(a+x, b, c1);
			if (b+x <= v) and (not used[a, b+x, c1]) then
				add(a, b+x, c1);
		end;
		if (c-c2 > 0) then begin
			x := c-c2;
			if (a+x <= v) and (not used[a+x, b, c2]) then
				add(a+x, b, c2);
			if (b+x <= v) and (not used[a, b+x, c2]) then
				add(a, b+x, c2);
		end;
		if (b1-b > 0) then begin
			x := b1-b;
			if (c-x > 0) and (not used[a, b1, c-x]) then
				add(a, b1, c-x);
		end;
		if (b2-b > 0) then begin
			x := b2-b;
			if (c-x > 0) and (not used[a, b2, c-x]) then
				add(a, b2, c-x);
		end;
		if (a1-a > 0) then begin
			x := a1-a;
			if (c-x > 0) and (not used[a1, b, c-x]) then
				add(a1, b, c-x);
		end;
		if (a2-a > 0) then begin
			x := a2-a;
			if (c-x > 0) and (not used[a2, b, c-x]) then
				add(a2, b, c-x);
		end;
	end;
	for i := 1 to dn do
		dfs(d[i].a, d[i].b, d[i].c, k1+1);
	used[a, b, c] := False;
end;

procedure Solve;
var
	i,j,k					: Byte;
begin
	for i := 0 to v do
		for j := 0 to v do
			for k := 0 to v do
				used[i, j, k] := False;
	answ := 0;
	dfs(v, 0, 0, 0);
	writeln(answ);
        if (answ <> 2216) then
          halt(1);
end;

var
  t : text;
begin
assign(t,'tw5086.tmp');
rewrite(t);
writeln(t,'180 6');
writeln(t,'37 78 59 100 64 128');
close(t);
	readInput;
	Solve;
end.
