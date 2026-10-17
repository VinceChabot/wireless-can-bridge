# Dot-source this to get a working NCS v3.1.0 `west` in any PowerShell:  . .\ncs-env.ps1
$tc = 'C:\ncs\toolchains\b8b84efebd'
$env:PATH = "$tc;$tc\mingw64\bin;$tc\bin;$tc\opt\bin;$tc\opt\bin\Scripts;$tc\nrfutil\bin;$tc\opt\zephyr-sdk\arm-zephyr-eabi\bin;$env:PATH"
$env:PYTHONPATH = "$tc\opt\bin;$tc\opt\bin\Lib;$tc\opt\bin\Lib\site-packages"
$env:ZEPHYR_TOOLCHAIN_VARIANT = 'zephyr'
$env:ZEPHYR_SDK_INSTALL_DIR = "$tc\opt\zephyr-sdk"
$env:ZEPHYR_BASE = 'C:\ncs\v3.1.0\zephyr'
$env:NRFUTIL_HOME = "$tc\nrfutil\home"
