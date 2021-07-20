DebugConfigHelp.md

arm-none-eabi-gdb --interpreter=mi


target extended-remote ${COM_PORT}
monitor swdp_scan
attach 1
set mem inaccessible-by-default off

