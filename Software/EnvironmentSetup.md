# VS Code Environment Setup
## Prerequisites
* NodeJS
* NPM
* Git
* Python
* VS Code
## pyocd install
* `pip install pyocd --upgrade`
* `pyocd pack update`
* `pyocd pack find stm32g0b1`
* `pyocd pack install STM32G0B1CEUx`
## SVD file
download from https://github.com/posborne/cmsis-svd/tree/master/data/STMicro
## Open OCD
* `npm install --global xpm@latest`
* `xpm install --global @xpack-dev-tools/openocd`
* Set `XDG_CACHE_HOME` env variable to `%USERPROFILE%\AppData\Local\Temp` to prevent symbol cache path error (Windows)
## GNU Arm Embedded Toolchain
* https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
* Set `MBED_GCC_ARM_PATH` env variable to `C:\Program Files (x86)\GNU Arm Embedded Toolchain\11 2021-q2\bin` (Windows latest path)
## Required NPM packages
* `npm install -g cppbuild`
* `npm install -g shx`
* `npm install -g @serialport/terminal`
## Required VS Code plug-ins
* `C/C++` (Microsoft)
* `Cortex-Debug` (marcus25) See: https://github.com/Marus/cortex-debug/wiki
* `LinkerScript` (Zixuan Wang)
* `Arm Assembly` (dan-c-underwood)
* `Code Spell Checker` (Street Side Software)