/*

File    : fat.c
Author  : Eliot Ferragni
Date    : 9 may 2018
REV 1.0

Some functions to use fatFS library

Adapted from the code written by Jed Frey
taken at https://github.com/jed-frey/ARMCM4-STM32F407-STF4BB-FATFS
*/


#include <ch.h>
#include <hal.h>

#include <chprintf.h>
#include <string.h>

#include <fat.h>
#include <sdio.h>

#define SIZE_ERROR_BUFFER	255


static FATFS SDC_FS;
static bool sd_card_mounted = false;

//////////////////////////////PRIVATE FUNCTIONS////////////////////////////

/**
 * @brief Returns a complete string depending on the error given
 * 
 * @param stat 	The error to interprete
 * @return 		A pointer to a string containing the text of the error
 */
char* fresult_str(FRESULT stat) {
    // static char str[SIZE_ERROR_BUFFER];
    // memset(str,0,sizeof(str));

    switch (stat) {
        case FR_OK:
            return "Succeeded";
        case FR_DISK_ERR:
            return "A hard error occurred in the low level disk I/O layer";
        case FR_INT_ERR:
            return "Assertion failed";
        case FR_NOT_READY:
            return "The physical drive cannot work";
        case FR_NO_FILE:
            return "Could not find the file";
        case FR_NO_PATH:
            return "Could not find the path";
        case FR_INVALID_NAME:
            return "The path name format is invalid";
        case FR_DENIED:
            return "Access denied due to prohibited access or directory full";
        case FR_EXIST:
            return "Access denied due to prohibited access";
        case FR_INVALID_OBJECT:
            return "The file/directory object is invalid";
        case FR_WRITE_PROTECTED:
            return "The physical drive is write protected";
        case FR_INVALID_DRIVE:
            return "The logical drive number is invalid";
        case FR_NOT_ENABLED:
            return "The volume has no work area";
        case FR_NO_FILESYSTEM:
            return "There is no valid FAT volume";
        case FR_MKFS_ABORTED:
            return "The f_mkfs() aborted due to any parameter error";
        case FR_TIMEOUT:
            return "Could not get a grant to access the volume within defined period";
        case FR_LOCKED:
            return "The operation is rejected according to the file sharing policy";
        case FR_NOT_ENOUGH_CORE:
            return "LFN working buffer could not be allocated";
        case FR_TOO_MANY_OPEN_FILES:
            return "Number of open files > _FS_SHARE";
        case FR_INVALID_PARAMETER:
            return "Given parameter is invalid";
        default:
            return "Unknown";
    }
    return "";
}

//////////////////////////////PUBLIC FUNCTIONS////////////////////////////
/**
 * @brief Mount the sd card
 */
bool mountSDCard(void){

    if(!sd_card_mounted){
        /*
        * Attempt to mount the drive.
        */
        if (sdio_connect() != HAL_SUCCESS) {
            return false;
        }
        if(f_mount(&SDC_FS,"",0) != FR_OK){
            return false;
        }
        sd_card_mounted = true;
    }
    return true;
}

/**
 * @brief Unmount the sd card
 */
bool unmountSDCard(void){
    if(sd_card_mounted){
        /*
        * Attempt to mount the drive.
        */
        if(sdio_disconnect() != HAL_SUCCESS) {
            return false;
        }
        if(f_mount(NULL,"",0) != FR_OK){
            return false;
        }
        sd_card_mounted = false;
    }
    
    return true;
}

/**
 * @brief   Returns if the sd card is mounted or not
 */
bool isSDCardMounted(void){
    return sd_card_mounted;
}

/*
 * Scan Files in a path and print them to the given stream.
 */
FRESULT scan_files(BaseSequentialStream *chp, char *path) {
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int fyear,fmonth,fday,fhour,fminute,fsecond;

    int i;
    char *fn;

#if _USE_LFN
    fno.lfname = 0;
    fno.lfsize = 0;
#endif
    /*
     * Open the Directory.
     */
    res = f_opendir(&dir, path);
    if (res == FR_OK) {
        /*
         * If the path opened successfully.
         */
        i = strlen(path);
        while (true) {
            /*
             * Read the Directory.
             */
            res = f_readdir(&dir, &fno);
            /*
             * If the directory read failed or end of dir
             */
            if (res != FR_OK || fno.fname[0] == 0) {
                break;
            }
            /*
             * If the directory or file begins with a '.' (hidden), continue
             */
            if (fno.fname[0] == '.') {
                continue;
            }
            fn = fno.fname;
            /*
             * Extract the date.
             */
            fyear = ((0b1111111000000000&fno.fdate) >> 9)+1980;
            fmonth= (0b0000000111100000&fno.fdate) >> 5;
            fday  = (0b0000000000011111&fno.fdate);
            /*
             * Extract the time.
             */
            fhour   = (0b1111100000000000&fno.ftime) >> 11;
            fminute = (0b0000011111100000&fno.ftime) >> 5;
            fsecond = (0b0000000000011111&fno.ftime)*2;
            /*
             * Print date and time of the file.
             */
            chprintf(chp, "%4d-%02d-%02d %02d:%02d:%02d ", fyear, fmonth, fday, fhour, fminute, fsecond);
            /*
             * If the 'file' is a directory.
             */
            if (fno.fattrib & AM_DIR) {
                /*
                 * Add a slash to the end of the path
                 */
                path[i++] = '/';
                strcpy(&path[i], fn);
                /*
                 * Print that it is a directory and the path.
                 */
                chprintf(chp, "<DIR> %s/\r\n", path);
                /*
                 * Recursive call to scan the files.
                 */
                res = scan_files(chp, path);
                if (res != FR_OK) {
                    break;
                }
                path[--i] = 0;
            } else {
                /*
                 * Otherwise print the path as a file.
                 */
                chprintf(chp, "      %s/%s\r\n", path, fn);
            }
        }
    } else {
        chprintf(chp, "FS: f_opendir() failed\r\n");
    }
    return res;
}

/**
 * @brief   Returns the size of the clusters of the sd card mounted
 * @return  number of elements per cluster
 */
BYTE getSDCardClusterSize(void){
    return SDC_FS.csize;
}
/*
*	Prints a complete error string depending on the error given
*/
void fverbose_error(BaseSequentialStream *chp, FRESULT err) {
    chprintf(chp, "\t%s.\r\n",fresult_str(err));
}