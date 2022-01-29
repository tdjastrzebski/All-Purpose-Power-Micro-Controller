# mbed-os
pip install mbed-cli **as admin**
python -m pip install mbed-tools **as admin**
python -m pip install mbed-tools --upgrade
mbed-tools configure -m STM32L053x8 -t GCC_ARM

# pyocd
pip install pyocd --upgrade
pyocd pack update
pyocd pack find stm32g0b1
pyocd pack install STM32G0B1CEUx

# Obtain svd file
from https://github.com/posborne/cmsis-svd/tree/master/data/STMicro

# SH1106 Libs
* https://github.com/afiskon/stm32-ssd1306
* https://github.com/desertkun/SH1106
* https://picaxeforum.co.uk/threads/fyi-comparing-the-ssd1306-and-sh1106-oled-drivers.32387
* https://www.espruino.com/SH1107
* https://www.waveshare.com/wiki/1.3inch_OLED_Module_(C)
* https://www.waveshare.com/w/upload/5/58/SH1107Datasheet.pdf

# FONTS
* https://github.com/liberationfonts/liberation-fonts (Liberation Mono)
* https://www.1001fonts.com/liberation-mono-font.html
* Consolas (TTF)
* https://github.com/fcambus/spleen
* https://oleddisplay.squix.ch (library + converter !)
* https://www.freetype.org/ + https://gitlab.freedesktop.org/freetype
* http://www.angelcode.com/products/bmfont
* https://sourceforge.net/projects/terminus-font (Terminus font)
* https://github.com/lvgl/lv_font_conv (font converter as npm package!)
* https://lvgl.io/tools/fontconverter (online font converter)

`npm i lv_font_conv -g`
`lv_font_conv --font ./fonts/liberation-mono.regular.ttf --symbols 0123456789X-?V --range 0x20 --size 64 --format lvgl --bpp 1 --no-compress -o ./Fonts/liberation_mono.c`
`lv_font_conv --font ./fonts/liberation-mono.regular.ttf --symbols 0123456789▲▼°☼↑↓• --size 64 --format lvgl --bpp 1 --no-compress -o ./Fonts/liberation_mono.c`
`lv_font_conv --font ./fonts/liberation-mono.regular.ttf --symbols 0123456789 --size 64 --format dump --bpp 1 --no-compress -o ./dump`

https://www.acodersjourney.com/top-20-c-pointer-mistakes

# encoder
https://community.st.com/s/question/0D53W00000805z2SAA/when-tim4-is-set-to-encoder-mode-what-is-the-minimum-pulse-width-it-can-capture
https://petoknm.wordpress.com/2015/01/05/rotary-encoder-and-stm32
https://stackoverflow.com/questions/39450610/stm32f4-encoder-count-is-changing-when-it-should-not (!!)
Note: do not use prescaler for encoder

# adac+dma
http://e673.com/index.php/a-detailed-tutorial-on-stm32-adc-multiple-channel
http://www.emcu.eu/how-to-use-3-channels-of-the-adc-in-dma-mode-using-cube-mx-and-atollic
https://msalamon.pl/adc-w-stm32-na-kilka-sposobow-joystick
https://www.youtube.com/watch?v=pLsAhJ8umJk (excellent video)
https://www.youtube.com/c/stmicroelectronics/videos

ADC on STM32G0: https://microgeek.eu/viewtopic.php?t=2231
STM32G0 and STM32CubeMX 5.0: https://www.st.com/content/st_com/en/about/events/events.html/stm32g0-with-stm32cubemx-webinar.html
DS13560, 5.3.17 Analog-to-digital converter characteristics
5.3.18 Digital-to-analog converter characteristics
5.3.19 Voltage reference buffer characteristics
5.3.21 Temperature sensor characteristics
5.3.22 VBAT monitoring characteristics

STM32G0 Workshop: https://www.youtube.com/playlist?list=PLnMKNibPkDnG1xN9JSrrwMWt5ngQQchAa

# printf
http://www.nerdkits.com/videos/printf_and_scanf
