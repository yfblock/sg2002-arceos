# RISC-V M 态跳转到 S 态的最简示例代码

.section .text
.global _start_m_mode

_start_m_mode:
    # 1. 设置 PMP，允许 S 态访问所有内存
    li      t0, 0x3fffffffffffff
    csrw    pmpaddr0, t0
    li      t0, 0xf                 # A=TOR, R=1, W=1, X=1
    csrw    pmpcfg0, t0

    # 2. 设置 mstatus.MPP = S-mode (01)
    #    MPP 位于 mstatus[12:11]
    li      t0, (1 << 11)
    csrw    mstatus, t0

    # 3. 设置返回地址 (mret 后跳转到这个地址)
    la      t0, _start_s_mode
    csrw    mepc, t0

    # 4. 关闭 S 态分页 (使用物理地址)
    csrw    satp, zero

    # 5. 可选：委托异常和中断给 S 态
    li      t0, 0xffff
    csrw    medeleg, t0
    csrw    mideleg, t0

    # 6. 执行 mret 跳转到 S 态
    mret

.global _start_s_mode
_start_s_mode:
    # 现在运行在 S 态了
    # 在这里添加 S 态的初始化代码
    
    # 示例：设置栈指针
    # la      sp, _stack_top
    
    # 示例：跳转到 C 代码入口
    # call    main

    # 死循环 (占位)
1:
    wfi
    j       1b
