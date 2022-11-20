#openocd.exe -f interface/cmsis-dap.cfg -f target/stm32g4x.cfg -c "init; program $1; reset run; exit;"
