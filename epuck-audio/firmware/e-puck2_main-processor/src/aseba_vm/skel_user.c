#include <string.h>
#include <stdio.h>

#include "ch.h"
#include "hal.h"

#include "aseba_node.h"
#include "skel_user.h"

#include "vm/natives.h"
#include "common/productids.h"
#include "common/consts.h"
#include <main.h>
#include "config_flash_storage.h"

/* Struct used to share Aseba parameters between C-style API and Aseba. */
static parameter_t aseba_settings[SETTINGS_COUNT];
static char aseba_settings_name[SETTINGS_COUNT][10];

struct _vmVariables vmVariables;


const AsebaVMDescription vmDescription = {
    BOARD_NAME,
    {
     // {Number of element in array, Name displayed in aseba studio}
     {1, "_id"},
     {1, "event.source"},
     {VM_VARIABLES_ARG_SIZE, "event.args"},
     {2, "_fwversion"},
     {1, "_productId"},

     {1, "led1"},
     {NUM_COLOR_LED, "led2"},
     {1, "led3"},
     {NUM_COLOR_LED, "led4"},
     {1, "led5"},
     {NUM_COLOR_LED, "led6"},
     {1, "led7"},
     {NUM_COLOR_LED, "led8"},
     
     {NB_AXIS, "acc"},
     {NB_AXIS, "gyro"},

     {0, NULL}
}
};

// Event descriptions
const AsebaLocalEventDescription localEvents[] = {
    {"new_imu", "New imu measurement"},
    {"button", "User button clicked"},
    {NULL, NULL}
};

void aseba_variables_init(parameter_namespace_t *aseba_ns)
{
    /* Initializes constant variables. */
    memset(&vmVariables, 0, sizeof(vmVariables));

    vmVariables.productId = ASEBA_PID_UNDEFINED;
    vmVariables.fwversion[0] = 0;
    vmVariables.fwversion[1] = 1;

    /* Registers all Aseba settings in global namespace. */
    int i;

    /* Must be in descending order to keep them sorted on display. */
    for (i = SETTINGS_COUNT - 1; i >= 0; i--) {
        sprintf(aseba_settings_name[i], "%d", i);
        parameter_integer_declare_with_default(&aseba_settings[i],
                                               aseba_ns,
                                               aseba_settings_name[i],
                                               0);
    }
}

void aseba_read_variables_from_system(AsebaVMState *vm)
{
    vmVariables.id = vm->nodeId;
}

void aseba_write_variables_to_system(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);

    imu_cb();
    leds_cb();
}

// This function must update the accelerometer variables
void imu_cb(void)
{
    vmVariables.acc[X_AXIS] = (sint16) (1000 * get_acceleration(X_AXIS));
    vmVariables.acc[Y_AXIS] = (sint16) (1000 * get_acceleration(Y_AXIS));
    vmVariables.acc[Z_AXIS] = (sint16) (1000 * get_acceleration(Z_AXIS));

    vmVariables.gyro[X_AXIS] = (sint16) get_gyro(X_AXIS);
    vmVariables.gyro[Y_AXIS] = (sint16) get_gyro(Y_AXIS);
    vmVariables.gyro[Z_AXIS] = (sint16) get_gyro(Z_AXIS);

    SET_EVENT(EVENT_IMU);
}

void leds_cb(void){
    
    set_led(LED1, vmVariables.led1);
    set_rgb_led(LED2, vmVariables.led2[RED_LED], vmVariables.led2[GREEN_LED], vmVariables.led2[BLUE_LED]);
    set_led(LED3, vmVariables.led3);
    set_rgb_led(LED4, vmVariables.led4[RED_LED], vmVariables.led4[GREEN_LED], vmVariables.led4[BLUE_LED]);
    set_led(LED5, vmVariables.led5);
    set_rgb_led(LED6, vmVariables.led6[RED_LED], vmVariables.led6[GREEN_LED], vmVariables.led6[BLUE_LED]);
    set_led(LED7, vmVariables.led7);
    set_rgb_led(LED8, vmVariables.led8[RED_LED], vmVariables.led8[GREEN_LED], vmVariables.led8[BLUE_LED]);
}

void button_cb(void)
{
    SET_EVENT(EVENT_BUTTON);
}


// Native functions
static AsebaNativeFunctionDescription AsebaNativeDescription__system_reboot =
{
    "_system.reboot",
    "Reboot the microcontroller",
    {
     {0, 0}
}
};

void AsebaNative__system_reboot(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);
    NVIC_SystemReset();
}

static AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_read =
{
    "_system.settings.read",
    "Read a setting",
    {
     { 1, "address"},
     { 1, "value"},
     { 0, 0 }
}
};

static void AsebaNative__system_settings_read(AsebaVMState *vm)
{
    uint16 address = vm->variables[AsebaNativePopArg(vm)];
    uint16 destidx = AsebaNativePopArg(vm);
    if (address < SETTINGS_COUNT) {
        vm->variables[destidx] = parameter_integer_get(&aseba_settings[address]);
    } else {
        AsebaVMEmitNodeSpecificError(vm, "Invalid settings address.");
    }
}

static AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_write =
{
    "_system.settings.write",
    "Write a setting",
    {
     { 1, "address"},
     { 1, "value"},
     { 0, 0 }
}
};

static void AsebaNative__system_settings_write(AsebaVMState *vm)
{
    uint16 address = vm->variables[AsebaNativePopArg(vm)];
    uint16 value = vm->variables[AsebaNativePopArg(vm)];

    if (address < SETTINGS_COUNT) {
        parameter_integer_set(&aseba_settings[address], value);
    } else {
        AsebaVMEmitNodeSpecificError(vm, "Invalid settings address.");
    }
}




static AsebaNativeFunctionDescription AsebaNativeDescription_settings_save =
{
    "_system.settings.flash",
    "Save settings into flash",
    {
     {0, 0}
}
};

void AsebaNative_settings_save(AsebaVMState *vm)
{
    extern uint32_t _config_start, _config_end;
    size_t len = (size_t)(&_config_end - &_config_start);
    bool success;

    // First write the config to flash
    config_save(&_config_start, len, &parameter_root);

    // Second try to read it back, see if we failed
    success = config_load(&parameter_root, &_config_start);

    if (!success) {
        AsebaVMEmitNodeSpecificError(vm, "Config save failed!");
    }
}

static AsebaNativeFunctionDescription AsebaNativeDescription_settings_erase =
{
    "_system.settings.erase",
    "Restore settings to default value (erases flash)",
    {
     {0, 0}
}
};


void AsebaNative_settings_erase(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);

    extern uint32_t _config_start;

    config_erase(&_config_start);
}

AsebaNativeFunctionDescription AsebaNativeDescription_clear_all_leds = {
    "leds.clear_all",
    "Clear all the LEDs",
    {
     {0, 0}
}
};


void clear_all_leds(AsebaVMState *vm)
{
    ASEBA_UNUSED(vm);
    
    vmVariables.led1 = 0;
    vmVariables.led2[RED_LED] = 0;
    vmVariables.led2[GREEN_LED] = 0;
    vmVariables.led2[BLUE_LED] = 0;
    vmVariables.led3 = 0;
    vmVariables.led4[RED_LED] = 0;
    vmVariables.led4[GREEN_LED] = 0;
    vmVariables.led4[BLUE_LED] = 0;
    vmVariables.led5 = 0;
    vmVariables.led6[RED_LED] = 0;
    vmVariables.led6[GREEN_LED] = 0;
    vmVariables.led6[BLUE_LED] = 0;
    vmVariables.led7 = 0;
    vmVariables.led8[RED_LED] = 0;
    vmVariables.led8[GREEN_LED] = 0;
    vmVariables.led8[BLUE_LED] = 0;

}



// Native function descriptions
const AsebaNativeFunctionDescription* nativeFunctionsDescription[] = {
    &AsebaNativeDescription__system_reboot,
    &AsebaNativeDescription__system_settings_read,
    &AsebaNativeDescription__system_settings_write,
    &AsebaNativeDescription_settings_save,
    &AsebaNativeDescription_settings_erase,
    &AsebaNativeDescription_clear_all_leds,
    ASEBA_NATIVES_STD_DESCRIPTIONS,
    0
};

// Native function pointers
AsebaNativeFunctionPointer nativeFunctions[] = {
    AsebaNative__system_reboot,
    AsebaNative__system_settings_read,
    AsebaNative__system_settings_write,
    AsebaNative_settings_save,
    AsebaNative_settings_erase,
    clear_all_leds,
    ASEBA_NATIVES_STD_FUNCTIONS,
};

const int nativeFunctions_length = sizeof(nativeFunctions) / sizeof(nativeFunctions[0]);
