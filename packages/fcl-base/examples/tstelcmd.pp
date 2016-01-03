{$mode objfpc}
{$h+}

program testelcmd;

uses eventlog;

Var
  E : TEventType;

begin
  With TEventLog.Create(Nil) do
    Try
      Identification:='Test eventlog class';
      RegisterMessageFile('');
      Active:=True;
      For E:=etInfo to etDebug do
        Log(E,'An event log message of type '+EventTypeToString(E));
    finally
      Free;
    end;
end.
