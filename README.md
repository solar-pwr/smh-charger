# smh-charger
Custom firmware for EASUN ISolar-SMH-II-7K solar inverter charger

**For educational/experimental use only. This is very far from any "production" release, but at this point seems to work fine for me.**

Only AC charging supported. I've been developing and using this since 09/2025 as a high-efficiency PFC charger in my dual-conversion solar setup.

For the main part of the code, see:
- [offgrid_f28034.c](offgrid_f28034/offgrid_f28034.c)
- [hw.h](offgrid_f28034/hw.h)

Also uses:
- [sevenseg.h library by David Madison](github.com/dmadison/Segmented-LED-Display-ASCII)
- [C2000Ware header files and libraries from Texas Instruments](https://github.com/TexasInstruments/c2000ware-core-sdk/)
