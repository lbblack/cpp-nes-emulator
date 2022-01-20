#ifndef NES_CPU_H
#define NES_CPU_H

#include <string>
#include <fstream>
#include <vector>
#include <iterator>
#include <iostream>
#include <cinttypes>
#include <bitset>

using namespace std;

class nes_cpu {
private:
    // Program counter stored in 16-bit
    uint16_t PC;
    // (8-bit Registers) Accumulator, X Index, Y index, Flag(P), Stack Pointer
    // Flag(P) Register looks like this:
    // 7  bit  0
    // ---- ----
    // NVss DIZC
    // |||| ||||
    // |||| |||+- Carry
    // |||| ||+-- Zero
    // |||| |+--- Interrupt Disable
    // |||| +---- Decimal
    // ||++------ No CPU effect, see: the B flag
    // |+-------- Overflow
    // +--------- Negative

    uint8_t A, X, Y, P, SP;
    unsigned char MEM[2 << 16];
    uint8_t cycles;
public:
    nes_cpu();

    bool LOAD_TEST_ROM(string input_filename);
    void print_test_rom();
    void print_registers();

    uint8_t read_mem_from_address(uint16_t addr);
    void write_mem_to_address(uint8_t value, uint16_t destination);

    void push_value_to_stack(uint8_t value);
    uint8_t pop_value_from_stack();

    // immediate - this is the address right after the opcode (PC + 1)
    uint16_t get_immediate(uint16_t addr);
    // zeropage - this is the address of the value at PC+1 (00nn)
    uint16_t get_zeropage(uint16_t addr);
    // zeropage, x-indexed - same as zeropage, but offset with register X
    uint16_t get_zeropage_x(uint16_t addr);
    // zeropage, y-indexed - same as zeropage, but offset with register Y
    uint16_t get_zeropage_y(uint16_t addr);
    uint16_t get_indirect(uint16_t addr);
    // indirect, x-indexed - read the value of the immediate byte.
    // use this value + X (low nibble), and this value + X + 1 (high nibble) as an address
    uint16_t get_indirect_x(uint16_t addr);
    // indirect, y-indexed - read the value of the immediate byte.
    // use this value + Y (low nibble), and this value + Y + 1 (high nibble) as an address.
    // add y to this address
    uint16_t get_indirect_y(uint16_t addr);
    // absolute - this gives a complete address with the next 2 bytes
    // (little Endian, so low nibble comes first)
    uint16_t get_absolute(uint16_t addr);
    // absolute, x-indexed - same as absolute, but offset with register X
    uint16_t get_absolute_x(uint16_t addr);
    // absolute, y-indexed - same as absolute, but offset with register Y
    uint16_t get_absolute_y(uint16_t addr);

    // CPU operations
    void ADC(uint16_t addr, uint8_t cycle_count);
    void AND(uint16_t addr, uint8_t cycle_count);
    void ASL(uint16_t addr, uint8_t cycle_count, bool accumulator);

    void BCC(uint16_t addr, uint8_t cycle_count);
    void BCS(uint16_t addr, uint8_t cycle_count);
    void BEQ(uint16_t addr, uint8_t cycle_count);
    void BIT(uint16_t addr, uint8_t cycle_count);
    void BMI(uint16_t addr, uint8_t cycle_count);
    void BNE(uint16_t addr, uint8_t cycle_count);
    void BPL(uint16_t addr, uint8_t cycle_count);
    void BRK(uint8_t cycle_count);
    void BVC(uint16_t addr, uint8_t cycle_count);
    void BVS(uint16_t addr, uint8_t cycle_count);

    void CLC(uint8_t cycle_count);
    void CLD(uint8_t cycle_count);
    void CLI(uint8_t cycle_count);
    void CLV(uint8_t cycle_count);
    void CMP(uint16_t addr, uint8_t cycle_count);
    void CPX(uint16_t addr, uint8_t cycle_count);
    void CPY(uint16_t addr, uint8_t cycle_count);

    void DEC(uint16_t addr, uint8_t cycle_count);
    void DEX(uint8_t cycle_count);
    void DEY(uint8_t cycle_count);

    void EOR(uint16_t addr, uint8_t cycle_count);

    void INC(uint16_t addr, uint8_t cycle_count);
    void INX(uint8_t cycle_count);
    void INY(uint8_t cycle_count);

    void JMP(uint16_t addr, uint8_t cycle_count, bool is_indirect);
    void JSR(uint16_t addr, uint8_t cycle_count);

    void LDA(uint16_t addr, uint8_t cycle_count);
    void LDX(uint16_t addr, uint8_t cycle_count);
    void LDY(uint16_t addr, uint8_t cycle_count);
    void LSR(uint16_t addr, uint8_t cycle_count, bool accumulator);

    void NOP(uint8_t cycle_count);

    void ORA(uint16_t addr, uint8_t cycle_count);

    void PHA(uint8_t cycle_count);
    void PHP(uint8_t cycle_count);
    void PLA(uint8_t cycle_count);
    void PLP(uint8_t cycle_count);

    void ROL(uint16_t addr, uint8_t cycle_count, bool accumulator);
    void ROR(uint16_t addr, uint8_t cycle_count, bool accumulator);
    void RTI(uint8_t cycle_count);
    void RTS(uint8_t cycle_count);

    void SBC(uint16_t addr, uint8_t cycle_count);
    void SEC(uint8_t cycle_count);
    void SED(uint8_t cycle_count);
    void SEI(uint8_t cycle_count);
    void STA(uint16_t addr, uint8_t cycle_count);
    void STX(uint16_t addr, uint8_t cycle_count);
    void STY(uint16_t addr, uint8_t cycle_count);

    void TAX(uint8_t cycle_count);
    void TAY(uint8_t cycle_count);
    void TSX(uint8_t cycle_count);
    void TXA(uint8_t cycle_count);
    void TXS(uint8_t cycle_count);
    void TYA(uint8_t cycle_count);

    void execute_implied_instruction(
        void (nes_cpu::*opcode_function)(uint8_t),
        uint8_t cycle_count
    );

    void execute_relative_instruction(
        void(nes_cpu::*opcode_function)(uint16_t, uint8_t),
        uint8_t instruction_length,
        uint8_t cycle_count
    );

    void execute_instruction_with_bool(
        void(nes_cpu::*opcode_function)(uint16_t, uint8_t, bool),
        uint8_t instruction_length,
        uint8_t cycle_count,
        bool parameter_flag
    );

    void execute_instruction_with_bool_and_address(
        void(nes_cpu::*opcode_function)(uint16_t, uint8_t, bool),
        uint16_t(nes_cpu::*addressing_function)(uint16_t),
        uint8_t instruction_length,
        uint8_t cycle_count,
        bool parameter_flag
    );

    void execute_instruction(
        void(nes_cpu::*opcode_function)(uint16_t, uint8_t),
        uint16_t(nes_cpu::*addressing_function)(uint16_t),
        uint8_t instruction_length,
        uint8_t cycle_count
    );

    // step through loaded CPU instructions
    bool step();    
};

#endif
