# FATFS files.
FATFSSRC = ${CHIBIOS}/os/various/fatfs_bindings/fatfs_diskio.c \
           ${CHIBIOS}/os/various/fatfs_bindings/fatfs_syscall.c \
           ${CHIBIOS_EXT}/ext/fatfs/src/ff.c \
           ${CHIBIOS_EXT}/ext/fatfs/src/option/unicode.c
 
FATFSINC = ${CHIBIOS_EXT}/ext/fatfs/src
