(module
  ;;.local
  ;;.weak
  (func $anycall (param $i i32)
    unreachable
  )

  ;;.name hello
  ;;.forhost
  (func $add (param $lhs i32) (param $rhs i32) (result i32)
    get_local $lhs
    call $anycall
    get_local $lhs
    get_local $rhs
    i32.add)
  (export "add" (func $add))
)