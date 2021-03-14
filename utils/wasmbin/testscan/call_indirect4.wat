(module
  (func $add (result i64)
    i64.const 13
  )
  (func $sub (result i64)
    i64.const 4
  )
  (table 0 anyfunc)
  (func $test (result i64)
    i32.const $sub     ;; calling $add
    call_indirect (type 0) ;; type 0 (the only type used in this function)
  )
)
