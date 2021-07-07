#include <math.h>
#include <string.h>
#include <stdio.h>

#include "hal.h"
#include "test.h"
#include "chprintf.h"
#include "shell.h"
#include "usbcfg.h"
#include "chtm.h"
#include "common/types.h"
#include "vm/natives.h"
#include "audio/audio_thread.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "camera/dcmi_camera.h"
#include "sensors/battery_level.h"
#include "config_flash_storage.h"
#include "leds.h"
#include <main.h>
#include "motors.h"
#include <fat.h>
#include <audio/play_sound_file.h>
#include <audio/play_melody.h>

#define TEST_WA_SIZE        THD_WORKING_AREA_SIZE(256)
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

/*
 * SDC related variables and definitions.
 */
#define SDC_BURST_SIZE      16
/* Buffer for block read/write operations, note that extra bytes are
   allocated in order to support unaligned operations.*/
static uint8_t buf[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE + 4];
/* Additional buffer for sdcErase() test */
static uint8_t buf2[MMCSD_BLOCK_SIZE * SDC_BURST_SIZE ];

/*
 * Camera related variables.
 */
static format_t cmd_fmt;
static subsampling_t cmd_subx, cmd_suby;
static uint16_t cmd_x1, cmd_y1, cmd_width, cmd_height;

/*
*   fatFS related variables
*/
/* Generic large buffer.*/
#define SIZE_GENERIC_BUFFER 256
static char fbuff[SIZE_GENERIC_BUFFER];

void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    /*
     * Attempt to mount the drive.
     */
    if (!mountSDCard()) {
        chprintf(chp, "FS: f_mount() failed. Is the SD card inserted?\r\n");
        return;
    }
    chprintf(chp, "FS: f_mount() succeeded\r\n");
}

void cmd_unmount(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    if (!unmountSDCard()) {
        chprintf(chp, "FS: f_mount() unmount failed\r\n");
        return;
    }
}

void cmd_free(BaseSequentialStream *chp, int argc, char *argv[]) {
    FRESULT err;
    uint32_t clusters;
    FATFS *fsp;
    BYTE cluster_size;
    (void)argc;
    (void)argv;

    err = f_getfree("/", &clusters, &fsp);
    if (err != FR_OK) {
        chprintf(chp, "FS: f_getfree() failed\r\n");
        return;
    }

    cluster_size = getSDCardClusterSize();
    /*
     * Print the number of free clusters and size free in B, KiB and MiB.
     */
    chprintf(chp,"FS: %lu free clusters\r\n    %lu sectors per cluster\r\n",
        clusters, (uint32_t)cluster_size);
    chprintf(chp,"%lu B free\r\n",
        clusters * (uint32_t)cluster_size * (uint32_t)MMCSD_BLOCK_SIZE);
    chprintf(chp,"%lu KB free\r\n",
        (clusters * (uint32_t)cluster_size * (uint32_t)MMCSD_BLOCK_SIZE)/(1024));
    chprintf(chp,"%lu MB free\r\n",
        (clusters * (uint32_t)cluster_size * (uint32_t)MMCSD_BLOCK_SIZE)/(1024*1024));
}

void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void)argv;
    (void)argc;
    
    /*
    * Set the file path buffer to 0. Also means we will open the root directory
    */
    memset(fbuff,0,sizeof(fbuff));

    scan_files(chp, fbuff);
}

//create a file called hello.txt and write "Hello World" in it
void cmd_hello(BaseSequentialStream *chp, int argc, char *argv[]) {
    FIL fsrc;   /* file object */
    FRESULT err;
    int written;
    (void)argv;
    /*
     * Print the input arguments.
     */
    if (argc > 0) {
        chprintf(chp, "Usage: hello\r\n");
        chprintf(chp, "       Creates hello.txt with 'Hello World'\r\n");
        return;
    }
    /*
     * Open the text file
     */
    err = f_open(&fsrc, "hello.txt", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    if (err != FR_OK) {
        chprintf(chp, "FS: f_open(\"hello.txt\") failed.\r\n");
        fverbose_error(chp, err);
        return;
    } else {
        chprintf(chp, "FS: f_open(\"hello.txt\") succeeded\r\n");
    }
    /*
     * Write text to the file.
     */
    written = f_puts("Hello World", &fsrc);
    if (written == -1) {
        chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") failed\r\n");
    } else {
        chprintf(chp, "FS: f_puts(\"Hello World\",\"hello.txt\") succeeded\r\n");
    }
    /*
     * Close the file
     */
    f_close(&fsrc);
}

void cmd_mkdir(BaseSequentialStream *chp, int argc, char *argv[]) {
    FRESULT err;
    if (argc != 1) {
        chprintf(chp, "Usage: mkdir dirName\r\n");
        chprintf(chp, "       Creates directory with dirName (no spaces)\r\n");
        return;
    }
    /*
     * Attempt to make the directory with the name given in argv[0]
     */
    err=f_mkdir(argv[0]);
    if (err != FR_OK) {
        /*
         * Display failure message and reason.
         */
        chprintf(chp, "FS: f_mkdir(%s) failed\r\n",argv[0]);
        fverbose_error(chp, err);
        return;
    } else {
        chprintf(chp, "FS: f_mkdir(%s) succeeded\r\n",argv[0]);
    }
    return;
}

void cmd_setlabel(BaseSequentialStream *chp, int argc, char *argv[]) {
    FRESULT err;
    if (argc != 1) {
        chprintf(chp, "Usage: setlabel label\r\n");
        chprintf(chp, "       Sets FAT label (no spaces)\r\n");
        return;
    }
    /*
     * Attempt to set the label with the name given in argv[0].
     */
    err=f_setlabel(argv[0]);
    if (err != FR_OK) {
        chprintf(chp, "FS: f_setlabel(%s) failed.\r\n");
        fverbose_error(chp, err);
        return;
    } else {
        chprintf(chp, "FS: f_setlabel(%s) succeeded.\r\n");
    }
    return;
}

void cmd_getlabel(BaseSequentialStream *chp, int argc, char *argv[]) {
    FRESULT err;
    char lbl[12];
    DWORD sn;
    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: getlabel\r\n");
        chprintf(chp, "       Gets and prints FAT label\r\n");
        return;
    }
    memset(lbl,0,sizeof(lbl));
    /*
     * Get volume label & serial of the default drive
     */
    err = f_getlabel("", lbl, &sn);
    if (err != FR_OK) {
        chprintf(chp, "FS: f_getlabel failed.\r\n");
        fverbose_error(chp, err);
        return;
    }
    /*
     * Print the label and serial number
     */
    chprintf(chp, "LABEL: %s\r\n",lbl);
    chprintf(chp, "  S/N: 0x%X\r\n",sn);
    return;
}

/*
 * Print a text file to screen
 */
void cmd_cat(BaseSequentialStream *chp, int argc, char *argv[]) {
    FRESULT err;
    FIL fsrc;   /* file object */
    char Buffer[255];
    UINT ByteToRead=sizeof(Buffer);
    UINT ByteRead;
    /*
     * Print usage
     */
    if (argc != 1) {
        chprintf(chp, "Usage: cat filename\r\n");
        chprintf(chp, "       Echos filename (no spaces)\r\n");
        return;
    }
    /*
     * Attempt to open the file, error out if it fails.
     */
    err=f_open(&fsrc, argv[0], FA_READ);
    if (err != FR_OK) {
        chprintf(chp, "FS: f_open(%s) failed.\r\n",argv[0]);
        fverbose_error(chp, err);
        return;
    }
    /*
     * Do while the number of bytes read is equal to the number of bytes to read
     * (the buffer is filled)
     */
    do {
        /*
         * Clear the buffer.
         */
        memset(Buffer,0,sizeof(Buffer));
        /*
         * Read the file.
         */
        err=f_read(&fsrc,Buffer,ByteToRead,&ByteRead);
        if (err != FR_OK) {
            chprintf(chp, "FS: f_read() failed\r\n");
            fverbose_error(chp, err);
            f_close(&fsrc);
            return;
        }
        chprintf(chp, "%s", Buffer);
    } while (ByteRead>=ByteToRead);
    chprintf(chp,"\r\n");
    /*
     * Close the file.
     */
    f_close(&fsrc);
    return;
}

static void cmd_sound_file_play(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        chprintf(chp, "Usage: sf_play pathToTheMusic\r\n");
        return;
    }
    
    playSoundFile(argv[0],SF_FORCE_CHANGE);

}

static void cmd_sound_file_stop(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    (void)chp;
    
    stopCurrentSoundFile();
}

static void cmd_sound_file_volume(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        chprintf(chp, "Usage: sf_volume value\r\n");
        return;
    }

    setSoundFileVolume(atoi(argv[0]));
}

static void cmd_melody_play(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argv;
    if (argc != 1) {
        chprintf(chp, "Usage: ml_play numberOfTheMelody\r\n");
        chprintf(chp, "melodies availables :\r\n");
        chprintf(chp, "1) IMPOSSIBLE_MISSION,\r\n");
        chprintf(chp, "2) WE_ARE_THE_CHAMPIONS,\r\n");
        chprintf(chp, "3) RUSSIA,\r\n");
        chprintf(chp, "4) MARIO,\r\n");
        chprintf(chp, "5) UNDERWORLD,\r\n");
        chprintf(chp, "6) MARIO_START,\r\n");
        chprintf(chp, "7) MARIO_DEATH,\r\n");
        chprintf(chp, "8) MARIO_FLAG,\r\n");
        chprintf(chp, "9) WALKING,\r\n");
        chprintf(chp, "10) PIRATES_OF_THE_CARIBBEAN,\r\n");
        chprintf(chp, "11) SIMPSON,\r\n");
        chprintf(chp, "12) STARWARS,\r\n");
        chprintf(chp, "13) SANDSTORMS,\r\n");
        chprintf(chp, "14) SEVEN_NATION_ARMY\r\n");
        return;
    }
    playMelody(atoi(argv[0])-1, ML_FORCE_CHANGE, NULL);
}

static void cmd_melody_stop(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    (void)chp;
    
    stopCurrentMelody();
}

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[])
{
    size_t n, size;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: mem\r\n");
        return;
    }
    n = chHeapStatus(NULL, &size);
    chprintf(chp, "core free memory : %u bytes\r\n", chCoreGetStatusX());
    chprintf(chp, "heap fragments     : %u\r\n", n);
    chprintf(chp, "heap free total    : %u bytes\r\n", size);
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[])
{
    static const char *states[] = {CH_STATE_NAMES};
    thread_t *tp;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: threads\r\n");
        return;
    }
    chprintf(chp, "        addr        stack prio refs         state\r\n");
    tp = chRegFirstThread();
    do {
        chprintf(chp, "%08lx %08lx %4lu %4lu %9s\r\n",
                 (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
                 (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
                 states[tp->p_state]);
        tp = chRegNextThread(tp);
    } while (tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[])
{
    thread_t *tp;

    (void)argv;
    if (argc > 0) {
        chprintf(chp, "Usage: test\r\n");
        return;
    }
    tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriorityX(),
                             TestThread, chp);
    if (tp == NULL) {
        chprintf(chp, "out of memory\r\n");
        return;
    }
    chThdWait(tp);
}

static void cmd_readclock(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;
    chprintf(chp, "SYSCLK: %i \n HCLK: %i \n PCLK1  %i \n PCLK2 %i \n",
             STM32_SYSCLK, STM32_HCLK, STM32_PCLK1, STM32_PCLK2);
}


extern sint16 aseba_sqrt(sint16 num);

static void cmd_sqrt(BaseSequentialStream *chp, int argc, char *argv[])
{
    uint16_t input, result;
    float x;
    time_measurement_t tmp;
    chTMObjectInit(&tmp);

    if (argc != 2) {
        chprintf(chp,
                 "Usage: sqrt mode int\r\nModes: a (aseba), b (math), c (assembler) is default mode\r\n");
    } else {
        input = (uint16_t) atoi(argv[1]);

        if (!strcmp(argv[0], "a")) {
            chSysLock();
            chTMStartMeasurementX(&tmp);
            result = aseba_sqrt(input);
            chTMStopMeasurementX(&tmp);
            chSysUnlock();
        } else if (!strcmp(argv[0], "b")) {
            chSysLock();
            chTMStartMeasurementX(&tmp);
            result = sqrtf(input);
            chTMStopMeasurementX(&tmp);
            chSysUnlock();
        } else {
            chSysLock();
            chTMStartMeasurementX(&tmp);
            x = (float) input;
            __asm__ volatile (
                "vsqrt.f32 %[var], %[var]"
                : [var] "+t" (x)
                );
            result = (uint16_t) x;
            chTMStopMeasurementX(&tmp);
            chSysUnlock();
        }

        chprintf(chp, "sqrt(%u) = %u \r\n", input, result);
        chprintf(chp, "time: %u \r\n", tmp.last);
    }
}

extern sint16 aseba_atan2(sint16 y, sint16 x);

static void cmd_atan2(BaseSequentialStream *chp, int argc, char *argv[])
{
    int16_t a, b, result;
    time_measurement_t tmp;
    chTMObjectInit(&tmp);

    if (argc != 3) {
        chprintf(chp, "Usage: atan2 mode a b\r\nModes: a (aseba), b (math) is default mode\r\n");
    } else {
        a = (int16_t) atoi(argv[1]);
        b = (int16_t) atoi(argv[2]);

        if (!strcmp(argv[0], "a")) {
            chSysLock();
            chTMStartMeasurementX(&tmp);
            result = aseba_atan2(a, b);
            chTMStopMeasurementX(&tmp);
            chSysUnlock();
        } else {
            chSysLock();
            chTMStartMeasurementX(&tmp);
            result = (int16_t)(atan2f(a, b) * 32768 / M_PI);
            chTMStopMeasurementX(&tmp);
            chSysUnlock();
        }


        chprintf(chp, "atan2(%d, %d) = %d \r\n", a, b, result);
        chprintf(chp, "time: %u \r\n", tmp.last);
    }
}

static void tree_indent(BaseSequentialStream *out, int indent)
{
    int i;
    for (i = 0; i < indent; ++i) {
        chprintf(out, "  ");
    }
}

static void show_config_tree(BaseSequentialStream *out, parameter_namespace_t *ns, int indent)
{
    parameter_t *p;
    char string_buf[64];

    tree_indent(out, indent);
    chprintf(out, "%s:\r\n", ns->id);

    for (p = ns->parameter_list; p != NULL; p = p->next) {
        tree_indent(out, indent + 1);
        if (parameter_defined(p)) {
            switch (p->type) {
                case _PARAM_TYPE_SCALAR:
                    chprintf(out, "%s: %f\r\n", p->id, parameter_scalar_get(p));
                    break;

                case _PARAM_TYPE_INTEGER:
                    chprintf(out, "%s: %d\r\n", p->id, parameter_integer_get(p));
                    break;

                case _PARAM_TYPE_BOOLEAN:
                    chprintf(out, "%s: %s\r\n", p->id, parameter_boolean_get(p) ? "true" : "false");
                    break;

                case _PARAM_TYPE_STRING:
                    parameter_string_get(p, string_buf, sizeof(string_buf));
                    chprintf(out, "%s: %s\r\n", p->id, string_buf);
                    break;

                default:
                    chprintf(out, "%s: unknown type %d\r\n", p->id, p->type);
                    break;
            }
        } else {
            chprintf(out, "%s: [not set]\r\n", p->id);
        }
    }

    if (ns->subspaces) {
        show_config_tree(out, ns->subspaces, indent + 1);
    }

    if (ns->next) {
        show_config_tree(out, ns->next, indent);
    }
}

static void cmd_config_tree(BaseSequentialStream *chp, int argc, char **argv)
{
    parameter_namespace_t *ns;
    if (argc != 1) {
        ns = &parameter_root;
    } else {
        ns = parameter_namespace_find(&parameter_root, argv[0]);
        if (ns == NULL) {
            chprintf(chp, "Cannot find subtree.\r\n");
            return;
        }
    }

    show_config_tree(chp, ns, 0);
}

static void cmd_config_set(BaseSequentialStream *chp, int argc, char **argv)
{
    parameter_t *param;
    int value_i;

    if (argc != 2) {
        chprintf(chp, "Usage: config_set /parameter/url value.\r\n");
        return;
    }

    param = parameter_find(&parameter_root, argv[0]);

    if (param == NULL) {
        chprintf(chp, "Could not find parameter \"%s\"\r\n", argv[0]);
        return;
    }

    switch (param->type) {
        case _PARAM_TYPE_INTEGER:
            if (sscanf(argv[1], "%d", &value_i) == 1) {
                parameter_integer_set(param, value_i);
            } else {
                chprintf(chp, "Invalid value for integer parameter.\r\n");
            }
            break;

        case _PARAM_TYPE_BOOLEAN:
            if (!strcmp(argv[1], "true")) {
                parameter_boolean_set(param, true);
            } else if (!strcmp(argv[1], "false")) {
                parameter_boolean_set(param, false);
            } else {
                chprintf(chp, "Invalid value for boolean parameter, must be true or false.\r\n");
            }
            break;

        case _PARAM_TYPE_STRING:
            if (argc == 2) {
                parameter_string_set(param, argv[1]);
            } else {
                chprintf(chp, "Invalid value for string parameter, must not use spaces.\r\n");
            }
            break;

        default:
            chprintf(chp, "%s: unknown type %d\r\n", param->id, param->type);
            break;
    }
}

static void cmd_config_erase(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;
    (void) chp;
    extern uint8_t _config_start;

    config_erase(&_config_start);
}

static void cmd_config_save(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;
    extern uint8_t _config_start, _config_end;
    size_t len = (size_t)(&_config_end - &_config_start);
    bool success;

    // First write the config to flash
    config_save(&_config_start, len, &parameter_root);

    // Second try to read it back, see if we failed
    success = config_load(&parameter_root, &_config_start);

    if (success) {
        chprintf(chp, "OK.\r\n");
    } else {
        chprintf(chp, "Save failed.\r\n");
    }
}

static void cmd_config_load(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;
    extern uint8_t _config_start;
    bool success;

    success = config_load(&parameter_root, &_config_start);

    if (success) {
        chprintf(chp, "OK.\r\n");
    } else {
        chprintf(chp, "Load failed.\r\n");
    }
}

static void cmd_cam_set_brightness(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t value, err;

    if (argc != 1) {
        chprintf(chp,
                 "Usage: cam_brightness value.\r\nDefault=0, max=127, min=-128.\r\n");
    } else {
        value = (int8_t) atoi(argv[0]);
        err = po8030_set_brightness(value);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot write register (%d)\r\n", err);
        } else {
            chprintf(chp, "Register written correctly\r\n");
        }
    }
}

static void cmd_cam_set_contrast(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t value, err;

    if (argc != 1) {
        chprintf(chp,
                 "Usage: cam_contrast value.\r\nDefault=64, max=255, min=0.\r\n");
    } else {
        value = (int8_t) atoi(argv[0]);
        err = po8030_set_contrast(value);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot write register (%d)\r\n", err);
        } else {
            chprintf(chp, "Register written correctly\r\n");
        }
    }
}

static void cmd_cam_set_conf1(BaseSequentialStream *chp, int argc, char *argv[])
{
    uint16_t f;
    uint8_t sx, sy;

    if (argc != 3) {
        chprintf(chp,
                 "Usage: cam_conf1 format subsampling_x subsampling_y\r\nformat: 0=color, 1=grey\r\nsubsampling: 1, 2, 4\r\n");
    } else {
        f = (uint8_t) atoi(argv[0]);
        sx = (uint8_t) atoi(argv[1]);
        sy = (uint8_t) atoi(argv[2]);

        if(f==0) {
            cmd_fmt = FORMAT_YCBYCR;
            chprintf(chp, "Registered color format\r\n");
        } else {
            cmd_fmt = FORMAT_YYYY;
            chprintf(chp, "Registered greyscale format\r\n");
        }

        if(sx == 1) {
            cmd_subx = SUBSAMPLING_X1;
            chprintf(chp, "Registered x subsampling x1\r\n");
        } else if(sx == 2) {
        	cmd_subx = SUBSAMPLING_X2;
            chprintf(chp, "Registered x subsampling x2\r\n");
        } else {
        	cmd_subx = SUBSAMPLING_X4;
            chprintf(chp, "Registered x subsampling x4\r\n");
        }
        if(sy == 1) {
        	cmd_suby = SUBSAMPLING_X1;
            chprintf(chp, "Registered y subsampling x1\r\n");
        } else if(sy == 2) {
        	cmd_suby = SUBSAMPLING_X2;
            chprintf(chp, "Registered y subsampling x2\r\n");
        } else {
        	cmd_suby = SUBSAMPLING_X4;
            chprintf(chp, "Registered y subsampling x4\r\n");
        }
    }
}

static void cmd_cam_set_conf2(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t err;

    if (argc != 4) {
        chprintf(chp,"Usage: cam_conf2 x1 y1 width height\r\n");
    } else {
    	cmd_x1 = (uint16_t) atoi(argv[0]);
    	cmd_y1 = (uint16_t) atoi(argv[1]);
    	cmd_width = (uint16_t) atoi(argv[2]);
    	cmd_height = (uint16_t) atoi(argv[3]);

        err = po8030_advanced_config(cmd_fmt, cmd_x1, cmd_y1, cmd_width, cmd_height, cmd_subx, cmd_suby);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot set configuration (%d)\r\n", err);
        } else {
            chprintf(chp, "Configuration set correctly\r\n");
        }
    }
}

static void cmd_cam_set_mirror(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t err;
    uint8_t v, h;

    if (argc != 2) {
        chprintf(chp,
                 "Usage: cam_mirror vertical_en horizontal_en\r\n1=enabled, 0=disabled\r\n");
    } else {
        v = (uint8_t) atoi(argv[0]);
        h = (uint8_t) atoi(argv[1]);

        err = po8030_set_mirror(v, h);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot set mirroring (%d)\r\n", err);
        } else {
            chprintf(chp, "Mirroring set correctly\r\n");
        }
    }
}

static void cmd_cam_set_gain(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t err;
    uint8_t r, g, b;

    if (argc != 3) {
        chprintf(chp,
                 "Usage: cam_gain red_gain green_gain blue_gain\r\nThis command disable auto white balance.\r\nDefault: r=94, g=64, b=93\r\n");
    } else {
        r = (uint8_t) atoi(argv[0]);
        g = (uint8_t) atoi(argv[1]);
        b = (uint8_t) atoi(argv[2]);

        err = po8030_set_rgb_gain(r, g, b);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot set gain (%d)\r\n", err);
        } else {
            chprintf(chp, "Gain set correctly\r\n");
        }
    }
}

static void cmd_cam_set_awb(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t err;
    uint8_t awb;

    if (argc != 1) {
        chprintf(chp,
                 "Usage: cam_awb awb_en.\r\n1=enabled, 0=disabled\r\n");
    } else {
        awb = (uint8_t) atoi(argv[0]);

        err = po8030_set_awb(awb);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot set white balance (%d)\r\n", err);
        } else {
            chprintf(chp, "White balance set correctly\r\n");
        }
    }
}

static void cmd_cam_set_ae(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t err;
    uint8_t ae;

    if (argc != 1) {
        chprintf(chp,
                 "Usage: cam_ae ae_en.\r\n1=enabled, 0=disabled\r\n");
    } else {
        ae = (uint8_t) atoi(argv[0]);

        err = po8030_set_ae(ae);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot set auto exposure (%d)\r\n", err);
        } else {
            chprintf(chp, "Auto exposure set correctly\r\n");
        }
    }
}

static void cmd_cam_set_exposure(BaseSequentialStream *chp, int argc, char *argv[])
{
    int8_t err;
    uint16_t integral;
    uint8_t fractional;

    if (argc != 2) {
        chprintf(chp,
                 "Usage: cam_exposure integral fractional\r\nUnit is line time; total integration time = (integral + fractional/256) line time.\r\nDefault: integral=128, fractional=0\r\n");
    } else {
        integral = (uint16_t) atoi(argv[0]);
        fractional = (uint8_t) atoi(argv[1]);

        err = po8030_set_exposure(integral, fractional);
        if(err != MSG_OK) {
            chprintf(chp, "Cannot set exposure time (%d)\r\n", err);
        } else {
            chprintf(chp, "Exposure time set correctly\r\n");
        }
    }
}

static void cmd_cam_dcmi_prepare(BaseSequentialStream *chp, int argc, char **argv)
{
	uint8_t capture_mode = 0;

    if (argc != 1) {
        chprintf(chp, "Usage: cam_dcmi_prepare capture_mode\r\ncapture_mode: 0=oneshot, 1=continuous\r\n");
    } else {
        capture_mode = (uint8_t) atoi(argv[0]);

        dcmi_set_capture_mode(capture_mode);

        if(capture_mode == CAPTURE_ONE_SHOT) {
        	dcmi_disable_double_buffering();
        } else {
        	dcmi_enable_double_buffering();
        }

        if(dcmi_prepare() == 0) {
        	chprintf(chp, "DCMI prepared.\r\n");
        } else {
        	chprintf(chp, "Cannot prepare dcmi, image size too big.\r\n");
        	return;
        }

    }
}

static void cmd_cam_dcmi_unprepare(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;

    dcmi_unprepare();

    chprintf(chp, "DCMI released correctly\r\n");

}

static void cmd_cam_capture(BaseSequentialStream *chp, int argc, char **argv)
{
	(void) chp;
    (void) argc;
    (void) argv;
    dcmi_capture_start();
}

static void cmd_cam_send(BaseSequentialStream *chp, int argc, char **argv)
{
    (void) argc;
    (void) argv;
    uint8_t* img_buff_ptr;

    if(dcmi_capture_stop() != MSG_OK) {
    	chprintf(chp, "DCMI stop stream error\r\n");
    } else {
    	if(dcmi_double_buffering_enabled()) { // Send both images.
    		chprintf(chp, "The 2 images will be sent within 5 seconds\r\n");
    		chThdSleepMilliseconds(5000);
    		img_buff_ptr = dcmi_get_first_buffer_ptr();
    		chnWrite((BaseSequentialStream *)&SDU1, img_buff_ptr, po8030_get_image_size());
    		chThdSleepMilliseconds(3000);
    		img_buff_ptr = dcmi_get_second_buffer_ptr();
    		chnWrite((BaseSequentialStream *)&SDU1, img_buff_ptr, po8030_get_image_size());
    	} else {
    		chprintf(chp, "The image will be sent within 5 seconds\r\n");
    		chThdSleepMilliseconds(5000);
    		img_buff_ptr = dcmi_get_first_buffer_ptr();
    		chnWrite((BaseSequentialStream *)&SDU1, img_buff_ptr, po8030_get_image_size());
    	}
    }
}

static void cmd_set_led(BaseSequentialStream *chp, int argc, char **argv)
{
    uint8_t led_num = 0;
	uint8_t led_value = 0;

    if (argc != 2) {
        chprintf(chp, "Usage: set_led led_num led_value\r\nled_num: 0-3=small leds, 4=body led, 5=front led\r\nvalue: 0=off, 1=on, 2=toggle\r\n");
    } else {
        led_num = (uint8_t) atoi(argv[0]);
        led_value = (uint8_t) atoi(argv[1]);
		
		if(led_num <= 3) {
			set_led(led_num, led_value);
		} else if(led_num == 4) {
			set_body_led(led_value);
		} else if(led_num == 5) {
			set_front_led(led_value);
		}
    }
}

static void cmd_set_speed(BaseSequentialStream *chp, int argc, char **argv)
{
    int16_t speed_left = 0;
	int16_t speed_right = 0;

    if (argc != 2) {
        chprintf(chp, "Usage: set_speed left right\r\nspeed: %d..%d\r\n", -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
    } else {
        speed_left = (int16_t) atoi(argv[0]);
        speed_right = (int16_t) atoi(argv[1]);
		chprintf(chp, "lspeed=%d, rspeed=%d\r\n", speed_left, speed_right);
		right_motor_set_speed(speed_right);
		left_motor_set_speed(speed_left);
    }
}

static void cmd_get_battery(BaseSequentialStream *chp, int argc, char **argv)
{
	(void) chp;
    (void) argc;
    (void) argv;
    chprintf(chp, "Battery raw value = %d\r\n", get_battery_raw());
}

static void cmd_audio_play(BaseSequentialStream *chp, int argc, char *argv[])
{
    uint16_t freq;
    if (argc != 1) {
        chprintf(chp,
                 "Usage: audio_play freq\r\nfreq=100..20000 Hz\r\n");
    } else {
    	freq = (uint16_t) atoi(argv[0]);
        dac_play(freq);
    }
}

static void cmd_audio_stop(BaseSequentialStream *chp, int argc, char **argv)
{
	(void) chp;
    (void) argc;
    (void) argv;
    dac_stop();
}

static void cmd_volume(BaseSequentialStream *chp, int argc, char *argv[])
{
    uint8_t mic;
    if (argc != 1) {
        chprintf(chp, "Usage: volume mic_num.\r\nmic_num=0..3\r\n");
    } else {
        mic = (uint8_t) atoi(argv[0]);
        chprintf(chp, "%d\r\n", mic_get_volume(mic));
    }
}

static void cmd_mic_data(BaseSequentialStream *chp, int argc, char *argv[])
{
	(void) chp;
    (void) argc;
    (void) argv;
	int16_t* mic_data;
	uint16_t data_len = mic_buffer_get_size();
	uint8_t header[4] = {0xAA, 0x55, data_len>>8, data_len&0xFF};

	chnWrite((BaseSequentialStream *)&SDU1, (uint8_t*)header, 4);
	mic_buffer_ready_reset();
	while(!mic_buffer_is_ready());
	mic_data = mic_get_buffer_ptr();
	chnWrite((BaseSequentialStream *)&SDU1, (uint8_t*)mic_data, data_len);
}

void cmd_sdc(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *mode[] = {"SDV11", "SDV20", "MMC", NULL};
  systime_t start, end;
  uint32_t n, startblk;

  if (argc != 1) {
    chprintf(chp, "Usage: sdc read|write|erase|all\r\n");
    return;
  }

  /* Card presence check.*/
  if (!blkIsInserted(&SDCD1)) {
    chprintf(chp, "Card not inserted, aborting.\r\n");
    return;
  }

  /* Connection to the card.*/
  chprintf(chp, "Connecting... ");
  if (sdcConnect(&SDCD1)) {
    chprintf(chp, "failed\r\n");
    return;
  }

  chprintf(chp, "OK\r\n\r\nCard Info\r\n");
  chprintf(chp, "CSD      : %08X %8X %08X %08X \r\n",
           SDCD1.csd[3], SDCD1.csd[2], SDCD1.csd[1], SDCD1.csd[0]);
  chprintf(chp, "CID      : %08X %8X %08X %08X \r\n",
           SDCD1.cid[3], SDCD1.cid[2], SDCD1.cid[1], SDCD1.cid[0]);
  chprintf(chp, "Mode     : %s\r\n", mode[SDCD1.cardmode & 3U]);
  chprintf(chp, "Capacity : %DMB\r\n", SDCD1.capacity / 2048);

  /* The test is performed in the middle of the flash area.*/
  startblk = (SDCD1.capacity / MMCSD_BLOCK_SIZE) / 2;

  if ((strcmp(argv[0], "read") == 0) ||
      (strcmp(argv[0], "all") == 0)) {

    /* Single block read performance, aligned.*/
    chprintf(chp, "Single block aligned read performance:           ");
    start = chVTGetSystemTime();
    end = start + MS2ST(1000);
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf, 1)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n++;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

    /* Multiple sequential blocks read performance, aligned.*/
    chprintf(chp, "16 sequential blocks aligned read performance:   ");
    start = chVTGetSystemTime();
    end = start + MS2ST(1000);
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf, SDC_BURST_SIZE)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n += SDC_BURST_SIZE;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

#if STM32_SDC_SDIO_UNALIGNED_SUPPORT
    /* Single block read performance, unaligned.*/
    chprintf(chp, "Single block unaligned read performance:         ");
    start = chVTGetSystemTime();
    end = start + MS2ST(1000);
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf + 1, 1)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n++;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);

    /* Multiple sequential blocks read performance, unaligned.*/
    chprintf(chp, "16 sequential blocks unaligned read performance: ");
    start = chVTGetSystemTime();
    end = start + MS2ST(1000);
    n = 0;
    do {
      if (blkRead(&SDCD1, startblk, buf + 1, SDC_BURST_SIZE)) {
        chprintf(chp, "failed\r\n");
        goto exittest;
      }
      n += SDC_BURST_SIZE;
    } while (chVTIsSystemTimeWithin(start, end));
    chprintf(chp, "%D blocks/S, %D bytes/S\r\n", n, n * MMCSD_BLOCK_SIZE);
#endif /* STM32_SDC_SDIO_UNALIGNED_SUPPORT */
  }

  if ((strcmp(argv[0], "write") == 0) ||
      (strcmp(argv[0], "all") == 0)) {
    unsigned i;

    memset(buf, 0xAA, MMCSD_BLOCK_SIZE * 2);
    chprintf(chp, "Writing...");
    if(sdcWrite(&SDCD1, startblk, buf, 2)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");

    memset(buf, 0x55, MMCSD_BLOCK_SIZE * 2);
    chprintf(chp, "Reading...");
    if (blkRead(&SDCD1, startblk, buf, 1)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");

    for (i = 0; i < MMCSD_BLOCK_SIZE; i++)
      buf[i] = i + 8;
    chprintf(chp, "Writing...");
    if(sdcWrite(&SDCD1, startblk, buf, 2)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");

    memset(buf, 0, MMCSD_BLOCK_SIZE * 2);
    chprintf(chp, "Reading...");
    if (blkRead(&SDCD1, startblk, buf, 1)) {
      chprintf(chp, "failed\r\n");
      goto exittest;
    }
    chprintf(chp, "OK\r\n");
  }

  if ((strcmp(argv[0], "erase") == 0) ||
      (strcmp(argv[0], "all") == 0)) {
    /**
     * Test sdcErase()
     * Strategy:
     *   1. Fill two blocks with non-constant data
     *   2. Write two blocks starting at startblk
     *   3. Erase the second of the two blocks
     *      3.1. First block should be equal to the data written
     *      3.2. Second block should NOT be equal too the data written (i.e. erased).
     *   4. Erase both first and second block
     *      4.1 Both blocks should not be equal to the data initially written
     * Precondition: SDC_BURST_SIZE >= 2
     */
    memset(buf, 0, MMCSD_BLOCK_SIZE * 2);
    memset(buf2, 0, MMCSD_BLOCK_SIZE * 2);
    /* 1. */
    unsigned int i = 0;
    for (; i < MMCSD_BLOCK_SIZE * 2; ++i) {
      buf[i] = (i + 7) % 'T'; //Ensure block 1/2 are not equal
    }
    /* 2. */
    if(sdcWrite(&SDCD1, startblk, buf, 2)) {
      chprintf(chp, "sdcErase() test write failed\r\n");
      goto exittest;
    }
    /* 3. (erase) */
    if(sdcErase(&SDCD1, startblk + 1, startblk + 2)) {
      chprintf(chp, "sdcErase() failed\r\n");
      goto exittest;
    }
    sdcflags_t errflags = sdcGetAndClearErrors(&SDCD1);
    if(errflags) {
      chprintf(chp, "sdcErase() yielded error flags: %d\r\n", errflags);
      goto exittest;
    }
    if(sdcRead(&SDCD1, startblk, buf2, 2)) {
      chprintf(chp, "single-block sdcErase() failed\r\n");
      goto exittest;
    }
    /* 3.1. */
    if(memcmp(buf, buf2, MMCSD_BLOCK_SIZE) != 0) {
      chprintf(chp, "sdcErase() non-erased block compare failed\r\n");
      goto exittest;
    }
    /* 3.2. */
    if(memcmp(buf + MMCSD_BLOCK_SIZE,
              buf2 + MMCSD_BLOCK_SIZE, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(chp, "sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    /* 4. */
    if(sdcErase(&SDCD1, startblk, startblk + 2)) {
      chprintf(chp, "multi-block sdcErase() failed\r\n");
      goto exittest;
    }
    if(sdcRead(&SDCD1, startblk, buf2, 2)) {
      chprintf(chp, "single-block sdcErase() failed\r\n");
      goto exittest;
    }
    /* 4.1 */
    if(memcmp(buf, buf2, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(chp, "multi-block sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    if(memcmp(buf + MMCSD_BLOCK_SIZE,
              buf2 + MMCSD_BLOCK_SIZE, MMCSD_BLOCK_SIZE) == 0) {
      chprintf(chp, "multi-block sdcErase() erased block compare failed\r\n");
      goto exittest;
    }
    /* END of sdcErase() test */
  }

  /* Card disconnect and command end.*/
exittest:
  sdcDisconnect(&SDCD1);
}

const ShellCommand shell_commands[] = {
    {"mount", cmd_mount},
    {"unmount", cmd_unmount},
    {"getlabel", cmd_getlabel},
    {"setlabel", cmd_setlabel},
    {"tree", cmd_tree},
    {"free", cmd_free},
    {"mkdir", cmd_mkdir},
    {"hello", cmd_hello},
    {"cat", cmd_cat},
    {"sf_play",cmd_sound_file_play},
    {"sf_stop",cmd_sound_file_stop},
    {"sf_volume",cmd_sound_file_volume},
    {"ml_play",cmd_melody_play},
    {"ml_stop",cmd_melody_stop},
    {"mem", cmd_mem},
    {"threads", cmd_threads},
    {"mem", cmd_mem},
    {"threads", cmd_threads},
    {"test", cmd_test},
    {"clock", cmd_readclock},
    {"sqrt", cmd_sqrt},
    {"atan2", cmd_atan2},
    {"config_tree", cmd_config_tree},
    {"config_set", cmd_config_set},
    {"config_save", cmd_config_save},
    {"config_load", cmd_config_load},
    {"config_erase", cmd_config_erase},
    {"cam_brightness", cmd_cam_set_brightness},
    {"cam_contrast", cmd_cam_set_contrast},
    {"cam_conf1", cmd_cam_set_conf1},
    {"cam_conf2", cmd_cam_set_conf2},
    {"cam_mirror", cmd_cam_set_mirror},
    {"cam_gain", cmd_cam_set_gain},
    {"cam_awb", cmd_cam_set_awb},
    {"cam_ae", cmd_cam_set_ae},
    {"cam_exposure", cmd_cam_set_exposure},
    {"cam_dcmi_prepare", cmd_cam_dcmi_prepare},
    {"cam_dcmi_unprepare", cmd_cam_dcmi_unprepare},
	{"cam_capture", cmd_cam_capture},
	{"cam_send", cmd_cam_send},
	{"set_led", cmd_set_led},
	{"set_speed", cmd_set_speed},
	{"batt", cmd_get_battery},
	{"audio_play", cmd_audio_play},
	{"audio_stop", cmd_audio_stop},
	{"volume", cmd_volume},
	{"mic_data", cmd_mic_data},
	{"sdc", cmd_sdc},
    {NULL, NULL}
};

static THD_FUNCTION(shell_spawn_thd, p)
{
    (void) p;
    thread_t *shelltp = NULL;

    static const ShellConfig shell_cfg = {
        (BaseSequentialStream *)&SDU1,
        shell_commands
    };

    shellInit();

    while (TRUE) {
        if (!shelltp) {
            if (SDU1.config->usbp->state == USB_ACTIVE) {
                shelltp = shellCreate(&shell_cfg, SHELL_WA_SIZE, NORMALPRIO);
            }
        } else {
            if (chThdTerminatedX(shelltp)) {
                chThdRelease(shelltp);
                shelltp = NULL;
            }
        }
        chThdSleepMilliseconds(500);
    }
}


void shell_start(void)
{
    static THD_WORKING_AREA(wa, 2048);

    chThdCreateStatic(wa, sizeof(wa), NORMALPRIO, shell_spawn_thd, NULL);
}
