#include "nes_cpu.h"

int main(void)
{
    nes_cpu cpu;
    cpu.LOAD_TEST_ROM("nestest.nes");

    cpu.step();

    return 0;
}
