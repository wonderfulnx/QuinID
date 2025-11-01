# QuinReader

This folder contains source files for generating the FPGA image of QuinReader, targeting an ADRV9375+ZC706 platform.

The main entrance is `QuinReader_HW.slx`. The default RFID BLF is 40kHz. Using higher BLFs requires changing the `BLF` and `Tari` variables in the initialization function, as well as the filter chain inside `Lib_QuinIDRxDDC.slx` and the RFIDRx processing inside `Lib_RFIDSystem.slx`. Note that you can also build your own models using the libraries we provide, especially `Lib_RFIDSystem.slx` and `RFIDRx_xxxkHz.slxp`.