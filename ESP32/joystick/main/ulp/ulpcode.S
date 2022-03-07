
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
    ; r2 and r3 set to read_counter and gpio_values addresses, respectively
    move r2, read_counter
    move r3, gpio_values
    ; load data stored in r2 (read_counter) into r1
    ld r1, r2, 0
    ; reading data from gpio and storing in r0
    READ_RTC_REG(RTC_GPIO_IN_REG, 14, 29)
    ; adding read_counter
    add r1, r1, 1
    ; storing new value of read_counter in its address
    st r2, r1, 0
    ; loading content of gpio_values into r1
    ld r1, r3, 0
    ; comparing old gpio values with recently read values
    jumpr no_new_gpio_values, r1, EQ

new_gpio_command:
    ; storing new values in gpio_values' address
    ld r3, r0
    ; waking up main processor
    wake
    ; stop ULP wake up timer (clear RTC_CNTL_ULP_CP_SLP_TIMER_EN)
    REG_WR 0x006, 24, 24, 0
    ; halting ULP processor
    halt


no_new_gpio_command:
    halt


