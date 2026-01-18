/* Указываем конец стека (начало RAM + размер RAM) */
_estack = 0x20020000;    /* Конец 128KB RAM */

_Min_Stack_Size = 0x400;
_Min_Heap_Size  = 0x200;


MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  /* 1. Код и константы во Flash */
  .text :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Вектора прерываний из startup файла */
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
    _etext = .;        
  } > FLASH

  /* 2. Инициализированные переменные (из Flash копируются в RAM) */
  _sidata = LOADADDR(.data);
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;        
  } > RAM AT > FLASH

  /* 3. Неинициализированные переменные (обнуляются при старте) */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;         
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;         
    __bss_end__ = _ebss;
  } > RAM

  /* 4. Динамическая память (Heap) - именно тут нужна метка end */
  .user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + 0x200;    /* Резервируем немного места под кучу */
    . = . + 0x400;    /* Резервируем под стек (минимум) */
    . = ALIGN(8);
  } > RAM
}