(module
  (func $add (result i64)
    i64.const 13
  )
  (func $sub (result i64)
    i64.const 4
  )
  (table 0 anyfunc)
  (elem 0 (i32.const 0) $sub $add)
)
