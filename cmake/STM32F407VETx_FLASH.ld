/*
 * STM32F407VETx 链接脚本（GCC / GNU ld）
 *
 * 与 Keil 侧 PRJ/Listings/STM32F4.sct、EIDE 侧 build/V1.0/STM32_CAN_CMD.sct 等价：
 *   IROM : 0x08000000, 512K   IRAM : 0x20000000, 128K
 *   CCMRAM 未使用（与两条既有链路一致），保留声明以便后续把栈或 .bss 迁入。
 *
 * Heap / Stack 尺寸与 ARMCC 启动文件保持一致：Heap 0x200、Stack 0x400。
 */

ENTRY(Reset_Handler)

/* 栈顶位于 SRAM 最高地址；GCC 启动文件用 _estack 设置 SP */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* _Min_Heap_Size / _Min_Stack_Size 仅用于 ._user_heap_stack 的容量校验，
 * 实际栈指针由启动文件设为 _estack（即 RAM 顶部），向下生长。 */
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
  CCMRAM (xrw) : ORIGIN = 0x10000000, LENGTH = 64K
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
}

SECTIONS
{
  /* 中断向量表必须落在 Flash 起始处 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* .data 的运行地址在 RAM，装载地址紧接 .text/.rodata 之后 */
  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);

  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* 堆栈容量下限校验：若 RAM 放不下会在此报错 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* 此处不写 ST 模板的 /DISCARD/ { libc.a(*) libm.a(*) libgcc.a(*) }：
     那条会把 libm 整个丢弃，导致 nav 层的 sinf/cosf 直接 undefined reference。
     未用代码由 --gc-sections 回收，不需要它。 */

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
