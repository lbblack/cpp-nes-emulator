#include "nes_cpu.h"

nes_cpu::nes_cpu()
{
    PC = 0x4020;
    P = 0x34;
    A = 0x00;
    X = 0x00;
    Y = 0x00;
    SP = 0xFD;
    MEM[0x4017] = 0x00;
    MEM[0x4015] = 0x00;
    for (uint8_t i = 0; i < 16; i++) {
        MEM[0x4000 + i] = 0x00;
        if (i < 4) {
            MEM[0x4010 + i] = 0x00;
        }
    }

    // todo: all 15 bits of noise channel LFST = 0x0000
}

bool nes_cpu::LOAD_TEST_ROM(string input_filename)
{
    ifstream test_rom;
    test_rom.open(input_filename, ios::binary);

    vector<char> bytes(
        (istreambuf_iterator<char>(test_rom)),
        (istreambuf_iterator<char>()) );

    uint16_t start_address = 0x4020;
    uint16_t target_offset = 0;

    for (char byte : bytes) {
        MEM[start_address + target_offset] = byte;
        target_offset++;
    }

    test_rom.close();
    return true;
}

void nes_cpu::print_test_rom()
{
    for (int i = 0x4020; i < 0x4100; i++) {
        cout << setfill('0') << setw(2) << hex << (0xFF & (unsigned int)MEM[i]) << endl;
    }
}

void nes_cpu::print_registers()
{
    cout << setfill('0') << setw(2) << "A: " << hex << (0xFF & A);
    cout << setfill('0') << setw(2) << " X: " << hex << (0xFF & X);
    cout << setfill('0') << setw(2) << " Y: " << hex << (0xFF & Y);
    cout << setfill('0') << setw(2) << " P: " << hex << (0xFF & P);
    cout << setfill('0') << setw(2) << " SP: " << hex << (0xFF & SP);
    cout << setfill('0') << setw(2) << " PC: " << hex << (0xFFFF & PC) << endl;
}

void nes_cpu::write_mem_to_address(uint8_t value, uint16_t addr)
{
    MEM[addr] = value;
}

uint8_t nes_cpu::read_mem_from_address(uint16_t addr)
{
    return MEM[addr];
}

void nes_cpu::push_value_to_stack(uint8_t value)
{
    write_mem_to_address(value, 0x0100 + SP);
    SP++;
}

uint8_t nes_cpu::pop_value_from_stack()
{
    uint8_t target_offset = SP;
    SP--;
    return read_mem_from_address(0x0100 + target_offset);
}

uint16_t nes_cpu::get_immediate(uint16_t addr)
{
    return addr;
}

uint16_t nes_cpu::get_zeropage(uint16_t addr)
{
    return addr & 0xFF;
}

uint16_t nes_cpu::get_zeropage_x(uint16_t addr)
{
    return (addr + X) & 0xFF;
}

uint16_t nes_cpu::get_zeropage_y(uint16_t addr)
{
    return (addr + Y) & 0xFF;
}

uint16_t nes_cpu::get_indirect(uint16_t addr)
{
    uint16_t value = (uint16_t) read_mem_from_address(addr);
    uint16_t high_nibble = value;
    value = (uint16_t) read_mem_from_address(addr +1 1);
    uint16_t low_nibble = value;

    return (low_nibble << 8) & high_nibble;
}

uint16_t nes_cpu::get_indirect_x(uint16_t addr)
{
    return (addr + X) & 0xFF00 + (addr + X + 1) & 0xFF;
}

uint16_t nes_cpu::get_indirect_y(uint16_t addr)
{
    return (addr & 0xFF) + (addr + 1) & 0xFF00 + Y;
}

uint16_t nes_cpu::get_absolute(uint16_t addr)
{
    return addr;
}

uint16_t nes_cpu::get_absolute_x(uint16_t addr)
{
    return addr + X;
}

uint16_t nes_cpu::get_absolute_y(uint16_t addr)
{
    return addr + Y;
}

void nes_cpu::ADC(uint16_t addr, uint8_t cycle_count)
{

}

void nes_cpu::AND(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // bitwise AND the accumulator
    A &= read_mem_from_address(addr);
    // check the negative and zero flag
    P |= A & 0x80;
    P |= (A == 0) << 1;
}

void nes_cpu::ASL(uint16_t addr, uint8_t cycle_count, bool accumulator)
{
    cycles += cycle_count;

    uint8_t carry = 0;
    // shift left the accumulator by 1 bit  
    if (accumulator) {
        carry = (A & 0x80) >> 7; 
        A <<= 1;
        // check the negative and zero flag
        P |= A & 0x80;
        P |= (A == 0) << 1; 
    // shift left the value at the address by 1 bit
    } else {
        uint8_t value = read_mem_from_address(addr);
        carry = (value & 0x80) >> 7;
        value <<= 1;
        write_mem_to_address(value, addr);  

        // check the negative and zero flag
        P |= value & 0x80;
        P |= (value == 0) << 1;
    }

    // set the negative and zero flag on, set carry flag to the
    // most significant bit of the value before shift left
    P |= carry;
}

void nes_cpu::BCC(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // carry flag is off
    if (!(P & 0x1)) {
        PC += addr;
        cycles++;
    }
}

void nes_cpu::BCS(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // carry flag is on
    if (P & 0x1) {
        PC += addr;
        cycles++;
    }
}

void nes_cpu::BEQ(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // zero flag is on
    if (P & 0x2) {
        PC += addr;
        cycles++;
    }
}

void nes_cpu::BIT(uint16_t addr, uint8_t cycle_count)
{
    // zero flag is the the result of the operand AND accumulator
    AND(addr, cycle_count);

    // set the negative and overflow flag to the same bits as the value
    // located at the address
    uint8_t value = read_mem_from_address(addr);
    P |= value & 0xC0;
}

void nes_cpu::BMI(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // negative flag is on
    if (P & 0x80) {
        P += addr;
        cycles++;
        // todo : add one more if crosses page boundry
    }
}

void nes_cpu::BNE(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // zero flag is off
    if (!(P & 0x2)) {
        PC += addr;
        cycles++;
    }
}

void nes_cpu::BPL(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // negative flag is clear
    if (!(P & 0x80)) {
        PC += addr;
        cycles++;
        // todo: add one more cycle if branch crosses a page boundry    
    }
}

void nes_cpu::BRK(uint8_t cycle_count)
{
    cycles += cycle_count;
    // set the interrupt flag
    P |= 0x4;

    // push Program Counter to the stack
    // low nibble then high nibble (little Endian)
    push_value_to_stack((uint8_t) (PC & 0xFF00) >> 8);
    push_value_to_stack((uint8_t) (PC & 0xFF));

    // push the Status Register to the stack
    push_value_to_stack(P);
}

void nes_cpu::BVC(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // if the overflow flag is off
    if (!(P & 0x40)) {
        PC += addr;
        cycles++;
    }
}

void nes_cpu::BVS(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // if the overflow flag is on
    if (P & 0x40) {
        PC += addr;
        cycles++;
    }
}

void nes_cpu::CLC(uint8_t cycle_count)
{
    cycles += cycle_count;
    // clear the carry flag
    P &= 0xFE;
}

void nes_cpu::CLD(uint8_t cycle_count)
{
    cycles += cycle_count;
    // clear the decimal flag
    P &= 0xF8;
}

void nes_cpu::CLI(uint8_t cycle_count)
{
    cycles += cycle_count;
    // clear the interrupt disable flag
    P &= 0xFB;
}

void nes_cpu::CLV(uint8_t cycle_count)
{
    cycles += cycle_count;
    // clear the overflow flag
    P &= 0xBF;
}

void nes_cpu::CMP(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    uint8_t value = read_mem_from_address(addr);
    bool compare = (A >= value);

    // set the carry flag
    P |= compare;
    // check the zero flag
    if (A == value) {
        P |= 0x2;
    }
    // check the negative flag
    if (A >= 0x80) {
        P |= 0x80;
    }
}

void nes_cpu::CPX(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    uint8_t value = read_mem_from_address(addr);
    bool compare = (X >= value);

    // set the carry flag
    P |= compare;
    // check the zero flag
    if (X == value) {
        P |= 0x2;
    }
    // check the negative flag
    if (X >= 0x80) {
        P |= 0x80;
    }
}

void nes_cpu::CPY(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    uint8_t value = read_mem_from_address(addr);
    bool compare = (Y >= value);

    // set the carry flag
    P |= compare;
    // check the zero flag
    P |= (Y == value) << 1;
    // check the negative flag
    P |= (Y & 0x80);
}

void nes_cpu::DEC(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    // decrement value located at address by one
    uint8_t value = read_mem_from_address(addr);
    write_mem_to_address(value--, addr);

    // check the zero flag
    P |= (value == 0) << 7;
    // check the negative flag
    P |= (value & 0x80);
}

void nes_cpu::DEX(uint8_t cycle_count)
{
    cycles += cycle_count;

    // decrement X by one
    X--;

    // check the zero flag
    P |= (X == 0) << 7;
    // check the negative flag
    P |= (X & 0x80);
}

void nes_cpu::DEY(uint8_t cycle_count)
{
    cycles += cycle_count;

    // decrement Y by one
    Y--;

    // check the zero flag
    P |= (Y == 0) << 7;
    // check the negative flag
    P |= (Y & 0x80);
}

void nes_cpu::EOR(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    // load the value from memory address
    uint8_t value = read_mem_from_address(addr);
    // A = A XOR value
    A ^= value;
    // check the zero and negative flag
    P |= (A == 0) << 1;
    P |= (A & 0x80);
}

void nes_cpu::INC(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    // increment value located at address by one
    uint8_t value = read_mem_from_address(addr);
    write_mem_to_address(value++, addr);

    // check the zero flag
    P |= (value == 0) << 1;
    // check the negative flag
    P |= (value & 0x80);
}
void nes_cpu::INX(uint8_t cycle_count)
{
    cycles += cycle_count;

    // increment X by one
    X++;

    // check the zero flag
    P |= (X == 0) << 1;
    // check the negative flag
    P |= (X & 0x80);
}

void nes_cpu::INY(uint8_t cycle_count)
{
    cycles += cycle_count;

    // increment Y by one
    Y++;

    // check the zero flag
    P |= (Y == 0) << 1;
    // check the negative flag
    P |= (Y & 0x80);
}

void nes_cpu::JMP(uint16_t addr, uint8_t cycle_count, bool is_indirect)
{
    cycles += cycle_count;

    if (is_indirect) {
        uint16_t pointer_address = get_indirect(addr);
        uint16_t indirect_address = get_indirect(pointer_address);
        PC = indirect_address;
    } else {
        uint16_t jump_address = get_indirect(addr);
        PC = jump_address;
    }
}

// todo : JPM must NEVER use a vector beginning on the last byte of a page.
void nes_cpu::JSR(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    uint16_t next_instruction = PC + 1; 

    push_value_to_stack(next_instruction & 0xFF00);
    push_value_to_stack(next_instruction & 0xFF - 1);

    // will this work? cast value to 16-bit
    // read high nibble from source address
    // read low nibble from source address - 1
    // Program Counter = (xxxx xxxx 0000 0000) & (0000 0000 xxxx xxxx)
    uint16_t jump_address = get_indirect(addr);
    PC = jump_address;
}

void nes_cpu::LDA(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    // load value from memory
    uint8_t value = read_mem_from_address(addr);
    // load accumulator with value
    A = value;

    // check zero flag
    P |= (A == 0) << 1;
    // check negative flag
    P |= (A & 0x80);
}

void nes_cpu::LDX(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    // load value from memory
    uint8_t value = read_mem_from_address(addr);
    // load X with value
    X = value;

    // check zero flag
    P |= (X == 0) << 1;
    // check negative flag
    P |= (X & 0x80);
}

void nes_cpu::LDY(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;

    // load value from memory
    uint8_t value = read_mem_from_address(addr);
    // load Y with value
    Y = value;

    // check zero flag
    P |= (Y == 0) << 1;
    // check negative flag
    P |= (Y & 0x80);
}

void nes_cpu::LSR(uint16_t addr, uint8_t cycle_count, bool accumulator)
{
    cycles += cycle_count;

    uint8_t carry = 0;
    if (accumulator) {
        // load temp variable with first bit of accumulator
        carry = A & 0x1;
        // logical bit shift right
        A >>= 1;
        // check the zero flag
        P |= (A == 0) << 1;
    } else {
        uint8_t value = read_mem_from_address(addr);
        // load temp variable with the first bit of target value 
        carry = value & 0x1;
        // logical bit shift right and store mutated value into memory
        value >>= 1;
        write_mem_to_address(value, addr);
        // check the zero flag
        P |= (A == 0) << 1;
    }

    // insert the temp variable into carry flag
    P |= carry;
    // set the negative flag to zero
    P &= 0x7F;
}

void nes_cpu::NOP(uint8_t cycle_count)
{
    cycles += cycle_count;
}

void nes_cpu::ORA(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // bitwise OR the accumulator
    A |= read_mem_from_address(addr);
    // check the negative and zero flag
    P |= A & 0x80;
    P |= (A == 0) << 1;
}

void nes_cpu::PHA(uint8_t cycle_count)
{
    cycles += cycle_count;

    // push the accumulator to the stack
    push_value_to_stack(A);
}

void nes_cpu::PHP(uint8_t cycle_count)
{
    cycles += cycle_count;

    // push the status-register onto the stack
    push_value_to_stack(P);
//  write_mem_to_address(value, SP);
//  increment_stack_pointer();
}

void nes_cpu::PLA(uint8_t cycle_count)
{
    cycles += cycle_count;

    // load the accumulator from the stack
    A = pop_value_from_stack();
}

void nes_cpu::PLP(uint8_t cycle_count)
{
    cycles += cycle_count;

    // load the status register from the stack
    P = pop_value_from_stack();
}

void nes_cpu::ROL(uint16_t addr, uint8_t cycle_count, bool accumulator)
{
    cycles += cycle_count; 

    uint8_t carry = 0;
    // rotate the accumulator by 1 bit into the carry flag 
    if (accumulator) {
        carry = (A & 0x80) >> 7;
        A = (P & 0x1) | (A << 1);

        // check the negative and zero flag
        P |= A & 0x80;
        P |= (A == 0) << 1;
    // rotate the value from the address by 1 bit into the carry flag
    } else {
        uint8_t value = read_mem_from_address(addr);
        carry = (value & 0x80) >> 7;
        value = (P & 0x1) | (value << 1);
        write_mem_to_address(value, addr);

        // check the negative and zero flag
        P |= value & 0x80;
        P |= (value == 0) << 1; 
    }

    // set the carry flag
    P |= carry;
}

void nes_cpu::ROR(uint16_t addr, uint8_t cycle_count, bool accumulator)
{
    cycles += cycle_count; 

    uint8_t carry = 0;
    // rotate the accumulator by 1 bit into the carry flag 
    if (accumulator) {
        carry = (A & 0x1);
        A = (P & 0x1) << 7 | (A >> 1);

        // check the negative and zero flag
        P |= A & 0x80;
        P |= (A == 0) << 1;
    // rotate the value from the address by 1 bit into the carry flag
    } else {
        uint8_t value = read_mem_from_address(addr);
        carry = (value & 0x1);
        value = (P & 0x1) << 7 | (value >> 1);
        write_mem_to_address(value, addr);

        // check the negative and zero flag
        P |= value & 0x80;
        P |= (value == 0) << 1; 
    }

    // set the carry flag
    P |= carry;
}

void nes_cpu::RTI(uint8_t cycle_count)
{
    cycles += cycle_count;

    // read the status register from the stack
    P = pop_value_from_stack();
    // read the PC from the stack
    uint16_t high_nibble = (uint16_t) pop_value_from_stack();
    uint16_t low_nibble  = (uint16_t) pop_value_from_stack();

    PC = (low_nibble << 8) & high_nibble;
}

void nes_cpu::RTS(uint8_t cycle_count)
{
    cycles += cycle_count;

    // read the PC from the stack
    uint16_t high_nibble = (uint16_t) pop_value_from_stack();
    uint16_t low_nibble  = (uint16_t) pop_value_from_stack();

    PC = (low_nibble << 8) & high_nibble;
}

void nes_cpu::SBC(uint16_t addr, uint8_t cycle_count)
{

}

void nes_cpu::SEC(uint8_t cycle_count)
{
    cycles += cycle_count;
    // set carry flag on
    P |= 0x1;
}

void nes_cpu::SED(uint8_t cycle_count)
{
    cycles += cycle_count;
    // set the decimal flag on
    P |= 0x8;
}

void nes_cpu::SEI(uint8_t cycle_count)
{
    cycles += cycle_count;
    // set the interrupt flag on
    P |= 0x4;
}

void nes_cpu::STA(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // write the accumulator to memory
    write_mem_to_address(A, addr);
}

void nes_cpu::STX(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // write X to memory
    write_mem_to_address(X, addr);
}

void nes_cpu::STY(uint16_t addr, uint8_t cycle_count)
{
    cycles += cycle_count;
    // write the accumulator to memory
    write_mem_to_address(Y, addr);
}

void nes_cpu::TAX(uint8_t cycle_count)
{
    cycles += cycle_count;
    // transfer the accumulator to X
    X = A;
}

void nes_cpu::TAY(uint8_t cycle_count)
{
    cycles += cycle_count;
    // transfer the accumulator to Y
    Y = A;
}

void nes_cpu::TSX(uint8_t cycle_count)
{
    cycles += cycle_count;
    // transfer the stack pointer to X
    X = SP;
}

void nes_cpu::TXA(uint8_t cycle_count)
{
    cycles += cycle_count;
    // transfer X to the accumulator
    A = X;
}

void nes_cpu::TXS(uint8_t cycle_count)
{
    cycles += cycle_count;
    // transfer X to the stack pointer
    SP = X;
}

void nes_cpu::TYA(uint8_t cycle_count)
{
    cycles += cycle_count;
    // transfer Y to the accumulator
    A = Y;
}

void execute_implied_instruction(void (*opcode_function)(uint8_t),
    uint8_t cycle_count)
{
    (*opcode_function)(cycle_count);
    PC++;
}

void execute_instruction_with_bool(
    void(*opcode_function)(uint16_t, uint8_t, bool),
    uint8_t instruction_length,
    uint8_t cycle_count,
    bool parameter_flag)
{
    (*opcode_function)(-1, cycle_count, parameter_flag);
    PC += instruction_length;
}

void execute_instruction_with_bool_and_address(
    void(*opcode_function)(uint16_t, uint8_t, bool),
    void(*addressing_function)(uint16_t),
    uint8_t instruction_length,
    uint8_t cycle_count,
    bool parameter_flag)
{
    (*opcode_function)((*addressing_function)(PC + 1), cycle_count, parameter_flag);
    PC += instruction_length;
}

void execute_instruction(
    void(*opcode_function)(uint16_t, uint8_t, bool),
    void(*addressing_function)(uint16_t),
    uint8_t instruction_length,
    uint8_t cycle_count)
{
    (*opcode_function)((*addressing_function)(PC + 1), cycle_count);
    PC += instruction_length;
}

// todo: check every instruction has the correct number of cycles
// todo: check every instruction moves the Program Counter
//       the correct number of bytes
// todo: check every instruction is using the correct addressing mode
// todo: check every instruction has the correct HEX opcode
bool nes_cpu::step()
{
    switch(read_mem_from_address(PC)) {
        case 0x69:
            ADC(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0x65:
            ADC(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0x75:
            ADC(get_zeropage_x(PC + 1), 4);
            PC += 2;
            break;
        case 0x6D:
            ADC(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0x7D:
            // todo: add another cycle on page boundry cross
            ADC(get_absolute_x(PC + 1), 4);
            PC += 3;
            break;
        case 0x79:
            // todo: add another cycle on page boundry cross
            ADC(get_absolute_y(PC + 1), 4);
            PC += 3;
            break;
        case 0x61:
            ADC(get_indirect_x(PC + 1), 6);
            PC += 2;
            break;
        case 0x71:
            // todo: add another cycle on page boundry cross
            ADC(get_indirect_y(PC), 5);
            PC += 2;
            break;

        case 0x29:
            AND(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0x25:
            AND(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0x35:
            AND(get_zeropage_x(PC + 1), 4);
            PC += 2;
            break;
        case 0x2D:
            AND(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0x3D:
            // todo: add another cycle on page boundry cross
            AND(get_absolute_x(PC + 1), 4);
            PC += 3;
            break;
        case 0x39:
            // todo: add another cycle on page boundry cross
            AND(get_absolute_y(PC + 1), 4);
            PC += 3;
            break;
        case 0x21:
            AND(get_indirect_x(PC + 1), 6);
            PC += 2;
            break;
        case 0x31:
            // todo: add another cycle on page boundry cross
            AND(get_indirect_y(PC + 1), 5);
            PC += 2;
            break;

        case 0x0A:
            ASL(-1, 2, true);
            PC++;
            break;
        case 0x06:
            ASL(get_zeropage(PC + 1), 5, false);
            PC += 2;
            break;
        case 0x16:
            ASL(get_zeropage_x(PC + 1), 6, false);
            PC += 2;
            break;
        case 0x0E:
            ASL(get_absolute(PC + 1), 6, false);
            PC += 3;
            break;
        case 0x1E:
            ASL(get_absolute_x(PC + 1), 7, false);
            PC += 3;
            break;

        case 0x90:
            // todo: add another cycle on page boundry cross
            BCC(PC + 1, 2);
            PC += 2;
            break;

        case 0xB0:
            // todo: add another cycle on page boundry cross
            BCS(PC + 1, 2);
            PC += 2;
            break;

        case 0xF0:
            // todo: add another cycles on page boundry cross
            BEQ(PC + 1, 2);
            PC += 2;
            break;

        case 0x24:
            BIT(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0x2C:
            BIT(get_absolute(PC + 1), 4);
            PC += 3;
            break;

        case 0x30:
            // todo: add another cycle on page boundry cross
            BMI(PC + 1, 2);
            PC += 2;
            break;

        case 0xD0:
            // todo: add another cycle on page boundry cross
            BNE(PC + 1, 2);
            PC += 2;
            break;

        case 0x10:
            // todo: add another cycle on page boundry cross
            BPL(PC + 1, 2);
            PC += 2;
            break;

        case 0x00:
            BRK(7);
            PC++;
            break;

        case 0x50:
            // todo: add another cycle on page boundry cross
            BVC(PC + 1, 2);
            PC += 2;
            break;

        case 0x70:
            // todo: add another cycle on page boundry cross
            BVS(PC + 1, 2);
            PC += 2;
            break;

        case 0x18:
            CLC(2);
            PC++;
            break;

        case 0xD8:
            CLD(2);
            PC++;
            break;

        case 0x58:
            CLI(2);
            PC++;
            break;

        case 0xB8:
            CLV(2);
            PC++;
            break;

        case 0xC9:
            CMP(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0xC5:
            CMP(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0xD5:
            CMP(get_zeropage_x(PC + 1), 4);
            PC += 2;
            break;
        case 0xCD:
            CMP(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0xDD:
            // todo: add another cycle on page boundry cross
            CMP(get_absolute_x(PC + 1), 4);
            PC += 3;
            break;
        case 0xD9:
            // todo: add another cycle on page boundry cross
            CMP(get_absolute_y(PC + 1), 4);
            PC += 3;
            break;
        case 0xC1:
            CMP(get_indirect_x(PC + 1), 6);
            PC += 2;
            break;
        case 0xD1:
            // todo: add another cycle on page boundry cross
            CMP(get_indirect_y(PC + 1), 5);
            PC += 2;
            break;

        case 0xE0:
            CPX(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0xE4:
            CPX(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0xEC:
            CPX(get_absolute(PC + 1), 4);
            PC += 3;
            break;

        case 0xC0:
            CPY(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0xC4:
            CPY(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0xCC:
            CPY(get_absolute(PC + 1), 4);
            PC += 3;
            break;

        case 0xC6:
            DEC(get_zeropage(PC + 1), 5);
            PC += 2;
            break;
        case 0xD6:
            DEC(get_zeropage_x(PC + 1), 6);
            PC += 2;
            break;
        case 0xCE:
            DEC(get_absolute(PC + 1), 6);
            PC += 3;
            break;
        case 0xDE:
            DEC(get_absolute_x(PC + 1), 7);
            PC += 3;
            break;

        case 0xCA:
            DEX(2);
            PC++;
            break;

        case 0x88:
            DEY(2);
            PC++;
            break;

        case 0x49:
            EOR(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0x45:
            EOR(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0x55:
            EOR(get_zeropage_x(PC + 1), 4);
            PC += 2;
            break;
        case 0x4D:
            EOR(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0x5D:
            // todo: add another cycle on page boundry cross
            EOR(get_absolute_x(PC + 1), 4);
            PC += 3;
            break;
        case 0x59:
            // todo: add another cycle on page boundry cross
            EOR(get_absolute_y(PC + 1), 4);
            PC += 3;
            break;
        case 0x41:
            EOR(get_indirect_x(PC + 1), 6);
            PC += 2;
            break;
        case 0x51:
            // todo: add another cycle on page boundry cross
            EOR(get_indirect_y(PC + 1), 5);
            PC += 2;
            break;

        case 0xE6:
            INC(get_zeropage(PC + 1), 5);
            PC += 2;
            break;
        case 0xF6:
            INC(get_zeropage_x(PC + 1), 6);
            PC += 2;
            break;
        case 0xEE:
            INC(get_absolute(PC + 1), 6);
            PC += 3;
            break;
        case 0xFE:
            INC(get_absolute_x(PC + 1), 7);
            PC += 3;
            break;

        case 0xE8:
            INX(2);
            PC++;
            break;

        case 0xC8:
            INY(2);
            PC++;
            break;

        case 0x4C:
            JMP(get_absolute(PC + 1), 3, false);
            PC += 3;
            break;
        case 0x6C:
            JMP(get_absolute(PC + 1), 5, true);
            PC += 3;
            break;

        case 0x20:
            JSR(get_absolute(PC + 1), 6);
            PC += 3;
            break;

        case 0xA9:
            LDA(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0xA5:
            LDA(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0xB5:
            LDA(get_zeropage_x(PC + 1), 4);
            PC += 2;
            break;
        case 0xAD:
            LDA(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0xBD:
            // todo: add another cycle on page boundry cross
            LDA(get_absolute_x(PC + 1), 4);
            PC += 3;
            break;
        case 0xB9:
            // todo: add another cycle on page boundry cross
            LDA(get_absolute_y(PC + 1), 4);
            PC += 3;
            break;
        case 0xA1:
            LDA(get_indirect_x(PC + 1), 6);
            PC += 2;
            break;
        case 0xB1:
            // todo: add another cycle on page boundry cross
            LDA(get_indirect_y(PC + 1), 5);
            PC += 2;
            break;

        case 0xA2:
            LDX(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0xA6:
            LDX(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0xB6:
            LDX(get_zeropage_y(PC + 1), 4);
            PC += 2;
            break;
        case 0xAE:
            LDX(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0xBE:
            // todo: add another cycle on page boundry cross
            LDX(get_absolute_y(PC + 1), 4);
            PC += 3;
            break;

        case 0xA0:
            LDY(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0xA4:
            LDY(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0xB4:
            LDY(get_zeropage_x(PC + 1), 4);
            PC += 2;
            break;
        case 0xAC:
            LDY(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0xBC:
            // todo: add another cycle on page boundry cross
            LDY(get_absolute_x(PC + 1), 4);
            PC += 3;
            break;

        case 0x4A:
            LSR(-1, 2, true);
            PC++;
            break;
        case 0x46:
            LSR(get_zeropage(PC + 1), 5, false);
            PC += 2;
            break;
        case 0x56:
            LSR(get_zeropage_x(PC + 1), 6, false);
            PC += 2;
            break;
        case 0x4E:
            LSR(get_absolute(PC + 1), 6, false);
            PC += 3;
            break;
        case 0x5E:
            LSR(get_absolute_x(PC), 7, false);
            PC += 3;
            break;

        case 0xEA:
            NOP(2);
            PC++;
            break;

        case 0x09:
            ORA(get_immediate(PC + 1), 2);
            PC += 2;
            break;
        case 0x05:
            ORA(get_zeropage(PC + 1), 3);
            PC += 2;
            break;
        case 0x15:
            ORA(get_zeropage_x(PC), 4);
            PC += 2;
            break;
        case 0x0D:
            ORA(get_absolute(PC + 1), 4);
            PC += 3;
            break;
        case 0x1D:
            // todo: add another cycle on page boundry cross
            ORA(get_absolute_x(PC), 4);
            PC += 3;
            break;
        case 0x19:
            // todo: add another cycle on page boundry cross
            ORA(get_absolute_y(PC), 4);
            PC += 3;
            break;
        case 0x01:
            ORA(get_indirect_x(PC + 1), 6);
            PC += 2;
            break;
        case 0x11:
            // todo: add another cycle on page boundry cross
            ORA(get_indirect_y(PC + 1), 5);
            PC += 2;

            // detect_and_process_page_cross(PC + 1, get_indirect_y(PC + 1));
            break;

        case 0x48:
            PHA(3);
            PC++;
            break;

        case 0x08:
            PHP(3);
            PC++;
            break;

        case 0x68:
            PLA(4);
            PC++;
            break;

        case 0x28:
            PLP(4);
            PC++;
            break;

        case 0x2A:
            ROL(-1, 2, true);
            PC++;
            break;
        case 0x26:
            ROL(get_zeropage(PC), 5, false);
            PC += 2;
            break;
        case 0x36:
            ROL(get_zeropage_x(PC), 6, false);
            PC += 2;
            break;
        case 0x2E:
            ROL(get_absolute(PC), 6, false);
            PC += 3;
            break;
        case 0x3E:
            ROL(get_absolute_x(PC), 7, false);
            PC += 3;
            break;

        case 0x6A:
            ROR(-1, 2, true);
            PC++;
            break;
        case 0x66:
            ROR(get_zeropage(PC), 5, false);
            PC += 2;
            break;
        case 0x76:
            ROR(get_zeropage_x(PC), 6, false);
            PC += 2;
            break;
        case 0x6E:
            ROR(get_absolute(PC), 6, false);
            PC += 3;
            break;
        case 0x7E:
            ROR(get_absolute_x(PC), 7, false);
            PC += 3;
            break;

        case 0x40:
            RTI(6);
            PC++;
            break;

        case 0x60:
            RTS(6);
            PC++;
            break;

        case 0xE9:
            SBC(get_immediate(PC), 2);
            PC += 2;
            break;
        case 0xE5:
            SBC(get_zeropage(PC), 3);
            PC += 2;
            break;
        case 0xF5:
            SBC(get_zeropage_x(PC), 4);
            PC += 2;
            break;
        case 0xED:
            SBC(get_absolute(PC), 4);
            PC += 3;
            break;
        case 0xFD:
            // todo: add another cycles on page boundry cross
            SBC(get_absolute_x(PC), 4);
            PC += 3;
            break;
        case 0xF9:
            // todo: add another cycles on page boundry cross
            SBC(get_absolute_y(PC), 4);
            PC += 3;
            break;
        case 0xE1:
            SBC(get_indirect_x(PC), 6);
            PC += 2;
            break;
        case 0xF1:
            // todo: add another cycles on page boundry cross
            SBC(get_indirect_y(PC), 5);
            PC += 2;
            break;

        case 0x38:
            SEC(2);
            PC++;
            break;

        case 0xF8:
            SED(2);
            PC++;
            break;

        case 0x78:
            SEI(2);
            PC++;
            break;

        case 0x85:
            STA(get_zeropage(PC), 3);
            PC += 2;
            break;
        case 0x95:
            STA(get_zeropage_x(PC), 4);
            PC += 2;
            break;
        case 0x8D:
            STA(get_absolute(PC), 4);
            PC += 3;
            break;
        case 0x9D:
            STA(get_absolute_x(PC), 5);
            PC += 3;
            break;
        case 0x99:
            STA(get_absolute_y(PC), 5);
            PC += 3;
            break;
        case 0x81:
            STA(get_indirect_x(PC), 6);
            PC += 2
            break;
        case 0x91:
            STA(get_indirect_y(PC), 6);
            PC += 2;
            break;

        case 0x86:
            STX(get_zeropage(PC), 3);
            PC += 2;
            break;
        case 0x96:
            STX(get_zeropage_x(PC), 4);
            PC += 2;
            break;
        case 0x8E:
            STX(get_absolute(PC), 4);
            PC += 3;
            break;

        case 0x84:
            STY(get_zeropage(PC), 3);
            PC += 2;
            break;
        case 0x94:
            STY(get_zeropage_x(PC), 4);
            PC += 2;
            break;
        case 0x8C:
            STY(get_absolute(PC), 4);
            PC += 3;
            break;

        case 0xAA:
            TAX(2);
            PC++;
            break;

        case 0xA8:
            TAY(2);
            PC++;
            break;

        case 0xBA:
            TSX(2);
            PC++;
            break;

        case 0x8A:
            TXA(2);
            PC++;
            break;

        case 0x9A:
            TXS(2);
            PC++;
            break;

        case 0x98:
            TYA(2);
            PC++;
            break;
        
        default:
            return false;
    }

    cout << hex << (unsigned int) (0xFF & MEM[PC]) << endl;

    return true;
}
