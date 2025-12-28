### read flash

`~/.platformio/packages/tool-minichlink/minichlink.exe -r test.bin flash $(ls -l .pio/build/ch592f/firmware.bin | awk '{print $5}')`

### openocd

`~/.platformio/packages/OpenOCD/bin/openocd.exe -f ~/.platformio/packages/OpenOCD/bin/wch-riscv.cfg -c "chip_id CH59x"`

if having issues:

`~/.platformio/packages/OpenOCD/bin/openocd.exe -f ~/.platformio/packages/OpenOCD/bin/wch-riscv.cfg -c "chip_id CH59x" -c "adapter speed 1000" -c init -c halt -c "flash erase_sector wch_riscv 0 last"`

### copy fresh framework

`rm -rf ~/.platformio/packages/framework-n-able-arduino-riscv/ && clear && pio run`

### inspect heap usage

`~/.platformio/packages/toolchain-riscv/bin/riscv-wch-elf-nm -n .pio/build/ch592f/firmware.elf | grep -E '_end|_heap_end|__stack_size'`

### useful gdb commands

- check free heap `p xFreeBytesRemaining`