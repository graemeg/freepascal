(module
  (import "console" "log" (func $log (param i32)))
  (func $add 
    i32.const 13
    call $log)
)
