openocd.exe -f interface/cmsis-dap.cfg -f target/stm32f3x.cfg -c "init; program $1; reset run; exit;"
