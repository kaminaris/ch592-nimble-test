#!/bin/bash
pio run -t clean -e ch592f && pio debug && ~/.platformio/packages/tool-minichlink/minichlink.exe -w /a/Projects/Electronic/CH59/CH592FNimbleTest/.pio/build/ch592f/firmware.bin 0x0 -b