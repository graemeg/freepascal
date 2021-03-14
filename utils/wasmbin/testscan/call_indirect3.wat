(module
  (func $add (result i64)
    i64.const 13
  )
  (table 0 anyfunc)
  (func $test (result i64)
    i32.const $add     ;; calling $add
    call_indirect (type 0) ;; type 0 (the only type used in this function)
  )
)
