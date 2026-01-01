## Setup

Prerequisites:
Either one of them will work, no need for both, Clion works with the PlatformIO plugin as well.
- PlatformIO Core: https://platformio.org/install/cli
- PlatformIO IDE (includes Core): https://platformio.org/install/ide?install=vscode


You need 3 things installed:
- Platform: https://github.com/kaminaris/platform-n-able
- Core: https://github.com/kaminaris/n-able-arduino
- This example repo https://github.com/kaminaris/ch592-nimble-test

Platform has some hardcoded paths to core so make sure to change it 
(as well as in this repo's platformio.ini) to your actual user path.

## Most important files
Those files were heavily modified to work with CH59x:
- `lib/nimble/syscfg/devcfg/ch5xxcfg.h` - main NimBLE config for CH59x (reference `syscfg.h` for all options)
- `lib/nimble/nimble/nimble/drivers/ch5xx/src/ble_phy.c` - **MOST IMPORTANT** - low level radio driver for CH59x
- `lib/nimble/nimble/nimble/drivers/ch5xx/src/ble_hw.c` - remaining low level hardware driver for CH59x (RNG, whitelist, etc)
- `lib/nimble/nimble/porting/nimble/src/hal_timer.c` - timer implementation for CH59x (not sure if i did it right)
- `lib/nimble/nimble/porting/nimble/src/nimble_port.c` - main nimble port initialization for CH59x
- `src/FreeRTOSConfig.h` - FreeRTOS config for CH59x (mainly heap size)3 

## Debugging

minichlink does work but requires new version from discord.

1. Clean -> debug build (will fail, ignore errors)
2. Flash using minichlink
   `~/.platformio/packages/tool-minichlink/minichlink.exe -w /a/Projects/Electronic/CH59/CH592FNimbleTest/.pio/build/ch592f/firmware.bin 0x0 -b`
3. open up another terminal start minichlink gdb server
   `./minichlink.exe -b -a -G`
4. Hit debug again, it should connect to gdb server now.
5. Restart MCU FROM gdb (in clion its R button) and you should be good to go.

**OLD INFO** - 

minichlink does not work, has bug when writing flash and that bricks flash.
Have to use `wch-link` for now which is extremely janky.

If built in OpenOCD in platform does not work, use the one from MounRiver Studio.

you may need a custom openocd config file for CH59x if the one 
in platform does not work.
Create a file `wch-riscv.cfg` with the following content:

```cfg
#interface wlink
adapter driver wlinke
adapter speed 6000
transport select sdi

wlink_set_address 0x00000000
set _CHIPNAME wch_riscv
sdi newtap $_CHIPNAME cpu -irlen 5 -expected-id 0x00001

set _TARGETNAME $_CHIPNAME.cpu

target create $_TARGETNAME.0 wch_riscv -chain-position $_TARGETNAME
$_TARGETNAME.0 configure  -work-area-phys 0x20000000 -work-area-size 10000 -work-area-backup 1
set _FLASHNAME $_CHIPNAME.flash

flash bank $_FLASHNAME wch_riscv 0x00000000 0 0 0 $_TARGETNAME.0

echo "Ready for Remote Connections"
```

## Commands

### debug verbose

`pio debug  --interface=gdb -v`

### read flash

`~/.platformio/packages/tool-minichlink/minichlink.exe -r test.bin flash $(ls -l .pio/build/ch592f/firmware.bin | awk '{print $5}')`

### write flash

`~/.platformio/packages/tool-minichlink/minichlink.exe -w /a/Projects/Electronic/CH59/CH592FNimbleTest/.pio/build/ch592f/firmware.bin 0x0 -b`

### openocd

`~/.platformio/packages/tool-openocd-riscv-wch/bin/openocd.exe -f ~/.platformio/packages/tool-openocd-riscv-wch/bin/wch-riscv.cfg -c "chip_id CH59x"`

if having issues:

`~/.platformio/packages/tool-openocd-riscv-wch/bin/openocd.exe -f ~/.platformio/packages/tool-openocd-riscv-wch/bin/wch-riscv.cfg -c "chip_id CH59x" -c "adapter speed 1000" -c init -c halt -c "flash erase_sector wch_riscv 0 last"`

### copy fresh framework

`rm -rf ~/.platformio/packages/framework-n-able-arduino-riscv/ && clear && pio run`

### inspect heap usage

`~/.platformio/packages/toolchain-riscv/bin/riscv-wch-elf-nm -n .pio/build/ch592f/firmware.elf | grep -E '_end|_heap_end|__stack_size'`

### useful gdb commands

- check free heap `p xFreeBytesRemaining`