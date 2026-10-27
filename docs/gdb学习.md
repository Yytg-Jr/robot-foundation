1. gdb --args ./build/robot_cli          # 启动 GDB，指定要调试的程序
2. break robot::parse_and_validate       # 在函数入口下断点
3. run < fixtures/10_single_joint.csv    # 启动程序，喂输入，停在第 1 个断点
   -> 停在 validator.cpp 函数入口，帧 #0 = parse_and_validate

4. catch throw                           # 设异常捕获点
5. continue                              # 继续，直到抛出异常才停
   -> 停在 __cxa_throw，此时栈是 #0..throw #1..fmt #2..at #3..parse #4..main

6. bt                                    # 看整条调用栈，确认是 at() 越界
7. frame 3                               # 切到 parse_and_validate 帧（关键一步）
8. print i                               # i=5
9. print arm.size()                      # size=5  -> 越界成立
10. print arm                            # {0.5,0,0,0,0}
11. list 95,99                           # 看源码确认 i <= arm.size()

//cmake 的指令

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j

