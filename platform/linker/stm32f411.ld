/* Указываем конец стека (начало RAM + размер RAM) */
_estack = 0x20020000;    /* 0x20000000 + 128KB (0x20000) */

MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
}

SECTIONS
{
  /* Сюда пойдут ваши секции .text, .data, .bss */
}

MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  /* Секция кода во Flash */
  .text :
  {
    KEEP(*(.isr_vector)) /* Вектора прерываний */
    *(.text*)
    /* ... */
  } >FLASH

  /* Метка начала данных во Flash для копирования в RAM */
  _sidata = LOADADDR(.data);

  .data : 
  {
    _sdata = .;        /* Начало данных в RAM */
    *(.data*)
    _edata = .;        /* Конец данных в RAM */
  } >RAM AT> FLASH

  .bss :
  {
    _sbss = .;         /* Начало BSS */
    *(.bss*)
    *(COMMON)
    _ebss = .;         /* Конец BSS */
  } >RAM
}