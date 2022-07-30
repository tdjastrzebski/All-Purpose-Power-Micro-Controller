# VS Code Environment Setup
## Prerequisites
* NodeJS + NPM
* Python + PIP
* Git
* VS Code
## pyOCD
* `pip install pyocd --upgrade`
* `pyocd pack update`
* `pyocd pack find stm32g0b1`
* `pyocd pack install STM32G0B1CEUx`
## OpenOCD
* Download the latest version from https://github.com/xpack-dev-tools/openocd-xpack/releases  
or build yourself following [this guide](https://github.com/Marus/cortex-debug/wiki/How-to-build-current-OpenOCD-version-on-Windows)
* Unzip it to `OpenOCD` folder in `C:\Program Files`
* Add `C:\Program Files\OpenOCD\bin` to system PATH variable
> Note: xpm utility does not really support global installations (yet), hence I suggest the above approach.
## SVD file - MPU specific
Download from https://github.com/posborne/cmsis-svd/tree/master/data/STMicro and place in the Software root folder.
## GNU Arm Embedded Toolchain
* required version: **arm-none-eabi** (bare-metal target)
* https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/
* Set `MBED_GCC_ARM_PATH` env variable to `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\11.2 2022.02\bin` (latest version Windows path).
* Add `MBED_GCC_ARM_PATH` env variable to Windows `Path` env variable (`%MBED_GCC_ARM_PATH%`) - if not set by the installer.
> Note: the most official build can be found [here](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/downloads) but I do not recomment using it. `arm-none-eabi-gdb` ver. 11.2 has a buggy dependency on unsupported and retired Python 2.7.
## Required NPM packages
* `npm install -g cppbuild`
* `npm install -g shx`
* `npm install -g @serialport/terminal`
## Required VS Code plug-ins
* `C/C++` (Microsoft)
* `Cortex-Debug` (marcus25) See: https://github.com/Marus/cortex-debug/wiki
## VS Code config
* Set the default VS Code terminal to `Git Bash`. Otherwise, VS Code may try to execute task NPM packages as (e.g.) PowerShell scripts.
## Recommended VS Code plug-ins
* `LinkerScript` (Zixuan Wang)
* `Arm Assembly` (dan-c-underwood)
* `Code Spell Checker` (Street Side Software)
* `Build++` (Tomasz Jastrzębski)
## ST Utilities
* [STSW-LINK009](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html) ST-LINK USB Driver
* [STSW-LINK007](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-programmers/stsw-link007.html) ST-LINK boards firmware upgrade
