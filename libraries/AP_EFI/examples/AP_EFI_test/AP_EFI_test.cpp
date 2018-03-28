/*
 *  EFI test code
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_EFI/AP_EFI.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_SerialManager serial_manager;
static AP_EFI efi(serial_manager);

void setup()
{
    // print welcome message
    hal.console->printf("EFI library test\n");

    // setup for analog pin 13
    AP_Param::set_object_value(&efi, efi.var_info, "_ENABLED", 1.0f);
    AP_Param::set_object_value(&efi, efi.var_info, "_TYPE", AP_EFI::EFI_COMMUNICATION_TYPE_SERIAL_MS);

    // initialise sensor, delaying to make debug easier
    hal.scheduler->delay(2000);
    efi.init();
    hal.console->printf("EFI: %d devices detected\n", efi.num_instances());
}

void loop()
{
    // Delay between reads
    hal.scheduler->delay(100);
    efi.update();

    bool had_data = false;
    for (uint8_t i=0; i<efi.num_instances(); i++) {
        //TODO
    }
    if (!had_data) {
        hal.console->printf("All: no data on any sensor\n");
    }

}
AP_HAL_MAIN();
