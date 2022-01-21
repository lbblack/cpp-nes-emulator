#include "nes_cpu.h"

int main(void)
{
    nes_cpu cpu;
    cpu.LOAD_TEST_ROM("nestest.nes");

    // cpu.print_test_rom();

    // cout << cpu.step() << endl;
    // cout << cpu.step() << endl;
    // cpu.print_registers();

    for (int i = 0; i < 100; i++) {
        cpu.step();
    }

    return 0;
}
