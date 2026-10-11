#include <assert.h>
#include <stdio.h>
#include "../User/measure_math.h"

int main(void)
{
    unsigned int frequency = 123;
    unsigned char alarms;
    unsigned char scratchpad[] = {0x50,0x05,0x4b,0x46,0x7f,0xff,0x0c,0x10};
    assert(freg_calc(1000, 0, &frequency) && frequency == 5000);
    assert(freg_calc(0, 0, &frequency) && frequency == 0);
    assert(!freg_calc(0, -100, &frequency));
    assert(freg_calc(20, -100, &frequency) && frequency == 0);
    assert(freg_calc(11999, 0, &frequency) && frequency == 59995);
    assert(!freg_calc(12000, 0, &frequency));
    assert(freg_calc(12179, -900, &frequency) && frequency == 59995);
    assert(!freg_calc(12180, -900, &frequency));
    assert(!freg_calc(65536, 0, &frequency));
    assert(dac_calc(0,1,2000)==51);
    assert(dac_calc(499,1,2000)==51);
    assert(dac_calc(500,1,2000)==51);
    assert(dac_calc(1250,1,2000)==153);
    assert(dac_calc(2000,1,2000)==255);
    assert(dac_calc(3000,1,2000)==255);
    assert(dac_calc(5000,0,2000)==0);
    assert(dac_calc(500,1,500)==0);
    assert(alarm_compare(0, 300, 300, 20) == 0);
    assert(alarm_compare(0, 301, 300, 20) == 1);
    assert(alarm_compare(1, 300, 300, 20) == 1);
    assert(alarm_compare(1, 281, 300, 20) == 1);
    assert(alarm_compare(1, 280, 300, 20) == 0);
    assert(alarm_compare(1, 0, 0, 10) == 0);
    assert(alarm_compare(0, 1, 0, 10) == 1);
    assert(alarm_calc(0, 400, 500, 6000, 0, 0, 0, 0, 30, 30, 50) == 0);
    assert(alarm_calc(0, 250, 200, 1000, 1, 1, 1, 1, 30, 30, 50) == 0);
    alarms = alarm_calc(0, 301, 301, 5001, 1, 1, 1, 1, 30, 30, 50);
    assert(alarms == 7);
    assert(alarm_calc(alarms, 290, 295, 4950, 1, 1, 1, 1, 30, 30, 50) == 7);
    assert(alarm_calc(alarms, 280, 290, 4900, 1, 1, 1, 1, 30, 30, 50) == 0);
    assert(alarm_calc(0, 250, 200, 1000, 0, 1, 1, 1, 30, 30, 50) == 8);
    assert(alarm_calc(0, 250, 200, 1000, 1, 0, 1, 1, 30, 30, 50) == 16);
    assert(alarm_calc(0, 250, 200, 1000, 1, 1, 0, 1, 30, 30, 50) == 0);
    assert(relay_calc(0, 8, 0) == 0);
    assert(relay_calc(1, 1, 1) == 1);
    assert(relay_calc(1, 2, 1) == 0);
    assert(relay_calc(1, 4, 1) == 0);
    assert(relay_calc(1, 8, 0) == 1);
    assert(relay_calc(1, 0, 1) == 0);
    assert(mute_calc(1, 1, 1) == 1);
    assert(mute_calc(1, 1, 3) == 0);
    assert(mute_calc(1, 8, 24) == 0);
    assert(mute_calc(1, 1, 0) == 0);
    assert(crc_read(scratchpad, 8) == 0x1c);
    scratchpad[0] ^= 1;
    assert(crc_read(scratchpad, 8) != 0x1c);
    puts("44 logic checks passed: frequency boundaries, DAC, hysteresis, cooling, mute and CRC");
    return 0;
}
