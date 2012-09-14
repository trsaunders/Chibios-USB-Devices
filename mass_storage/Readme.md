Mass Storage Device
===================

This driver implements a USB mass storage device. It requires a Chibios block device (e.g mmc_spi or SDC)

Example usage:
--------------
```c
mmcObjectInit(&MMCD1);
mmcStart(&MMCD1, &mmccfg);
mmcConnect(&MMCD1);

msdInit(&USBD1, &MMCD1);
```