{
  Contains the non-external functions for MySQL v4.0
}

function IS_PRI_KEY(n : longint) : Boolean;
begin
   IS_PRI_KEY:=(n and PRI_KEY_FLAG)<>0;
end;


function IS_NOT_NULL(n : longint) : Boolean;
begin
   IS_NOT_NULL:=(n and NOT_NULL_FLAG)<>0;
end;


function IS_BLOB(n : longint) : Boolean;
begin
   IS_BLOB:=(n and BLOB_FLAG)<>0;
end;

// This function is not used?!?!
{function IS_NUM_FIELD(f : Pst_mysql_field) : Boolean;
begin
   IS_NUM_FIELD:=((f^.flags) and NUM_FLAG)<>0;
end;}


function MYSQL_COUNT_ERROR : longint;
begin
  MYSQL_COUNT_ERROR:= not (my_ulonglong(0));
end;


function mysql_reload(mysql : pmysql) : longint;
begin
  mysql_reload:=mysql_refresh(mysql,REFRESH_GRANT);
end;
