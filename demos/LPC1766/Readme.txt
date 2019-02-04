Catatan :

Board : Larantuka_3.1 (LPC1766)
ChibiOS : 2.5.2

4-Feb-2019
-Mengganti konfigurasi clock pada file mcuconf.h agar sesuai dengan board larantuka_3.1. Pada board larantuka_3.1 tidak terdapat MAINOSC (EXTAL), melainkan hanya IRCOSC (Internal RC Oscillator) saja.
-Blink berhasil dicoba untuk P2.0
-Tambah deklarasi UART2 pin P2.8 da P2.9 pada file board.c
