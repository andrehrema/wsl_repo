#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_cntl_reg.h"


    .bss
    .global read_counter
read_counter:
    .long 0

    .global gpio_values
gpio_values:
    .long 0

    .text
    .global entry
entry:
    /*R2 and R3 set to read_counter and gpio_values addresses, respectively*/
    move R2, read_counter
    move R3, gpio_values
    /*load data stored in R2 (read_counter) into R1*/
    ld R1, R2, 0
    /*reading data from gpio and storing in r0*/
    READ_RTC_REG(RTC_GPIO_IN_REG, 14, 16)
    /*adding read_counter*/
    add R1, R1, 1
    /*storing new value of read_counter in its address*/
    st R1, R2, 0
    /*loading content of gpio_values into R1*/
    ld R1, R3, 0
    /*subtract old gpio_value (r0) and new gpio_value (r1). Store the result at r0*/
    sub R0, R0, R1
    /*comparing old gpio values with recently read values*/
    jumpr no_new_gpio_values, 0, EQ

new_gpio_values:
    /*storing new values in gpio_values' address*/
    ld R3, R0, 0
    /*waking up main processor*/
    wake
    /*stop ULP wake up timer (clear RTC_CNTL_ULP_CP_SLP_TIMER_EN)*/
    REG_WR 0x006, 24, 24, 0
    /*halting ULP processor*/
    halt


no_new_gpio_values:
    halt