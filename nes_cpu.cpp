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
    value = (uint16_t) read_mem_from_address(addr + 1);
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

void nes_cpu::execute_implied_instruction(void (nes_cpu::*opcode_function)(uint8_t),
    uint8_t cycle_count)
{
    (this->*opcode_function)(cycle_count);
    PC++;
}

void nes_cpu::execute_relative_instruction(
    void (nes_cpu::*opcode_function)(uint16_t, uint8_t),
    uint8_t instruction_length,
    uint8_t cycle_count)
{
    (this->*opcode_function)(PC + 1, cycle_count);
    PC += instruction_length;
}

void nes_cpu::execute_instruction_with_bool(
    void (nes_cpu::*opcode_function)(uint16_t, uint8_t, bool),
    uint8_t instruction_length,
    uint8_t cycle_count,
    bool parameter_flag)
{
    (this->*opcode_function)(-1, cycle_count, parameter_flag);
    PC += instruction_length;
}

void nes_cpu::execute_instruction_with_bool_and_address(
    void (nes_cpu::*opcode_function)(uint16_t, uint8_t, bool),
    uint16_t (nes_cpu::*addressing_function)(uint16_t),
    uint8_t instruction_length,
    uint8_t cycle_count,
    bool parameter_flag)
{
    (this->*opcode_function)((this->*addressing_function)(PC + 1),
        cycle_count, parameter_flag);
    PC += instruction_length;
}

void nes_cpu::execute_instruction(
    void (nes_cpu::*opcode_function)(uint16_t, uint8_t),
    uint16_t (nes_cpu::*addressing_function)(uint16_t),
    uint8_t instruction_length,
    uint8_t cycle_count)
{
    (this->*opcode_function)((this->*addressing_function)(PC + 1), cycle_count);
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
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0x65:
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x75:
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0x6D:
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0x7D:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_absolute_x, 3, 4);
            break;
        case 0x79:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_absolute_y, 3, 4);
            break;
        case 0x61:
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_indirect_y, 2, 6);
            break;
        case 0x71:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::ADC, &nes_cpu::get_indirect_y, 2, 5);
            break;

        case 0x29:
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0x25:
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x35:
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0x2D:
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0x3D:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_absolute_x, 3, 4);
            break;
        case 0x39:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_absolute_y, 3, 4);
            break;
        case 0x21:
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_indirect_x, 2, 6);
            break;
        case 0x31:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::AND, &nes_cpu::get_indirect_y, 2, 5);
            break;

        case 0x0A:
            execute_instruction_with_bool(&nes_cpu::ASL, 1, 2, true);
            break;
        case 0x06:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ASL, &nes_cpu::get_zeropage, 2, 5, false);
            break;
        case 0x16:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ASL, &nes_cpu::get_zeropage_x, 2, 6, false);
            break;
        case 0x0E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ASL, &nes_cpu::get_absolute, 3, 6, false);
            break;
        case 0x1E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ASL, &nes_cpu::get_absolute_x, 3, 7, false);
            break;

        case 0x90:
            // todo: add another cycle on page boundry cross
            execute_relative_instruction(&nes_cpu::BCC, 2, 2);
            break;

        case 0xB0:
            // todo: add another cycle on page boundry cross
            execute_relative_instruction(&nes_cpu::BCS, 2, 2);
            break;

        case 0xF0:
            // todo: add another cycles on page boundry cross
            execute_relative_instruction(&nes_cpu::BEQ, 2, 2);
            break;

        case 0x24:
            execute_instruction(&nes_cpu::BIT, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x2C:
            execute_instruction(&nes_cpu::BIT, &nes_cpu::get_absolute, 3, 4);
            break;

        case 0x30:
            // todo: add another cycle on page boundry cross
            execute_relative_instruction(&nes_cpu::BMI, 2, 2);
            break;

        case 0xD0:
            // todo: add another cycle on page boundry cross
            execute_relative_instruction(&nes_cpu::BNE, 2, 2);
            break;

        case 0x10:
            // todo: add another cycle on page boundry cross
            execute_relative_instruction(&nes_cpu::BPL, 2, 2);
            break;

        case 0x00:
            execute_implied_instruction(&nes_cpu::BRK, 7);
            break;

        case 0x50:
            // todo: add another cycle on page boundry cross
            execute_relative_instruction(&nes_cpu::BVC, 2, 2);
            break;

        case 0x70:
            // todo: add another cycle on page boundry cross
            execute_relative_instruction(&nes_cpu::BVS, 2, 2);
            break;

        case 0x18:
            execute_implied_instruction(&nes_cpu::CLC, 2);
            break;

        case 0xD8:
            execute_implied_instruction(&nes_cpu::CLD, 2);
            break;

        case 0x58:
            execute_implied_instruction(&nes_cpu::CLI, 2);
            break;

        case 0xB8:
            execute_implied_instruction(&nes_cpu::CLV, 2);
            break;

        case 0xC9:
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0xC5:
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0xD5:
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0xCD:
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0xDD:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_absolute_x, 3, 4);
            break;
        case 0xD9:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_absolute_y, 3, 4);
            break;
        case 0xC1:
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_indirect_x, 2, 6);
            break;
        case 0xD1:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::CMP, &nes_cpu::get_indirect_y, 2, 5);
            break;

        case 0xE0:
            execute_instruction(&nes_cpu::CPX, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0xE4:
            execute_instruction(&nes_cpu::CPX, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0xEC:
            execute_instruction(&nes_cpu::CPX, &nes_cpu::get_absolute, 3, 4);
            break;

        case 0xC0:
            execute_instruction(&nes_cpu::CPY, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0xC4:
            execute_instruction(&nes_cpu::CPY, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0xCC:
            execute_instruction(&nes_cpu::CPY, &nes_cpu::get_absolute, 3, 4);
            break;

        case 0xC6:
            execute_instruction(&nes_cpu::DEC, &nes_cpu::get_zeropage, 2, 5);
            break;
        case 0xD6:
            execute_instruction(&nes_cpu::DEC, &nes_cpu::get_zeropage_x, 2, 6);
            break;
        case 0xCE:
            execute_instruction(&nes_cpu::DEC, &nes_cpu::get_absolute, 3, 6);
            break;
        case 0xDE:
            execute_instruction(&nes_cpu::DEC, &nes_cpu::get_absolute_x, 3, 7);
            break;

        case 0xCA:
            execute_implied_instruction(&nes_cpu::DEX, 2);
            break;

        case 0x88:
            execute_implied_instruction(&nes_cpu::DEY, 2);
            break;

        case 0x49:
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0x45:
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x55:
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0x4D:
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0x5D:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_absolute_x, 3, 4);
            break;
        case 0x59:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_absolute_y, 3, 4);
            break;
        case 0x41:
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_indirect_x, 2, 6);
            break;
        case 0x51:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::EOR, &nes_cpu::get_indirect_y, 2, 5);
            break;

        case 0xE6:
            execute_instruction(&nes_cpu::INC, &nes_cpu::get_zeropage, 2, 5);
            break;
        case 0xF6:
            execute_instruction(&nes_cpu::INC, &nes_cpu::get_zeropage_x, 2, 6);
            break;
        case 0xEE:
            execute_instruction(&nes_cpu::INC, &nes_cpu::get_absolute, 3, 6);
            break;
        case 0xFE:
            execute_instruction(&nes_cpu::INC, &nes_cpu::get_absolute_x, 3, 7);
            break;

        case 0xE8:
            execute_implied_instruction(&nes_cpu::INX, 2);
            break;

        case 0xC8:
            execute_implied_instruction(&nes_cpu::INY, 2);
            break;

        case 0x4C:
            execute_instruction_with_bool_and_address(
                &nes_cpu::JMP, &nes_cpu::get_absolute, 3, 3, false);
            break;
        case 0x6C:
            execute_instruction_with_bool_and_address(
                &nes_cpu::JMP, &nes_cpu::get_absolute, 3, 5, true);
            break;

        case 0x20:
            execute_instruction(&nes_cpu::JSR, &nes_cpu::get_absolute, 3, 6);
            break;

        case 0xA9:
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0xA5:
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0xB5:
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0xAD:
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0xBD:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_absolute_x, 3, 4);
            break;
        case 0xB9:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_absolute_y, 3, 4);
            break;
        case 0xA1:
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_indirect_x, 2, 6);
            break;
        case 0xB1:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::LDA, &nes_cpu::get_indirect_y, 2, 5);
            break;

        case 0xA2:
            execute_instruction(&nes_cpu::LDX, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0xA6:
            execute_instruction(&nes_cpu::LDX, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0xB6:
            execute_instruction(&nes_cpu::LDX, &nes_cpu::get_zeropage_y, 2, 4);
            break;
        case 0xAE:
            execute_instruction(&nes_cpu::LDX, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0xBE:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::LDX, &nes_cpu::get_absolute_y, 3, 4);
            break;

        case 0xA0:
            execute_instruction(&nes_cpu::LDY, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0xA4:
            execute_instruction(&nes_cpu::LDY, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0xB4:
            execute_instruction(&nes_cpu::LDY, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0xAC:
            execute_instruction(&nes_cpu::LDY, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0xBC:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::LDY, &nes_cpu::get_absolute_x, 3, 4);
            break;

        case 0x4A:
            execute_instruction_with_bool(&nes_cpu::LSR, 1, 2, true);
            break;
        case 0x46:
            execute_instruction_with_bool_and_address(
                &nes_cpu::LSR, &nes_cpu::get_zeropage, 2, 5, false);
            break;
        case 0x56:
            execute_instruction_with_bool_and_address(
                &nes_cpu::LSR, &nes_cpu::get_zeropage_x, 2, 6, false);
            break;
        case 0x4E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::LSR, &nes_cpu::get_absolute, 3, 6, false);
            break;
        case 0x5E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::LSR, &nes_cpu::get_absolute_x, 3, 7, false);
            break;

        case 0xEA:
            execute_implied_instruction(&nes_cpu::NOP, 2);
            break;

        case 0x09:
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0x05:
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x15:
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0x0D:
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0x1D:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_absolute_x, 3, 4);
            break;
        case 0x19:
            // todo: add another cycle on page boundry cross
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_absolute_y, 3, 4);
            break;
        case 0x01:
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_indirect_x, 2, 6);
            break;
        case 0x11:
            // todo: add another cycle on page boundry cross
            // detect_and_process_page_cross(PC + 1, get_indirect_y(PC + 1));
            execute_instruction(&nes_cpu::ORA, &nes_cpu::get_indirect_y, 2, 5);
            break;

        case 0x48:
            execute_implied_instruction(&nes_cpu::PHA, 3);
            break;

        case 0x08:
            execute_implied_instruction(&nes_cpu::PHP, 3);
            break;

        case 0x68:
            execute_implied_instruction(&nes_cpu::PLA, 4);
            break;

        case 0x28:
            execute_implied_instruction(&nes_cpu::PLP, 4);
            break;

        case 0x2A:
            execute_instruction_with_bool(&nes_cpu::ROL, 1, 2, true);
            break;
        case 0x26:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROL, &nes_cpu::get_zeropage, 2, 5, false);
            break;
        case 0x36:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROL, &nes_cpu::get_zeropage_x, 2, 6, false);
            break;
        case 0x2E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROL, &nes_cpu::get_absolute, 3, 6, false);
            break;
        case 0x3E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROL, &nes_cpu::get_absolute_x, 3, 7, false);
            break;

        case 0x6A:
            execute_instruction_with_bool(&nes_cpu::ROR, 1, 2, true);
            break;
        case 0x66:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROR, &nes_cpu::get_zeropage, 2, 5, false);
            break;
        case 0x76:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROR, &nes_cpu::get_zeropage_x, 2, 6, false);
            break;
        case 0x6E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROR, &nes_cpu::get_absolute, 3, 6, false);
            break;
        case 0x7E:
            execute_instruction_with_bool_and_address(
                &nes_cpu::ROR, &nes_cpu::get_absolute_x, 3, 7, false);
            break;

        case 0x40:
            execute_implied_instruction(&nes_cpu::RTI, 6);
            break;

        case 0x60:
            execute_implied_instruction(&nes_cpu::RTS, 6);
            break;

        case 0xE9:
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_immediate, 2, 2);
            break;
        case 0xE5:
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0xF5:
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0xED:
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0xFD:
            // todo: add another cycles on page boundry cross
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_absolute_x, 3, 4);
            break;
        case 0xF9:
            // todo: add another cycles on page boundry cross
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_absolute_y, 3, 4);
            break;
        case 0xE1:
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_indirect_x, 2, 6);
            break;
        case 0xF1:
            // todo: add another cycles on page boundry cross
            execute_instruction(&nes_cpu::SBC, &nes_cpu::get_indirect_y, 2, 5);
            break;

        case 0x38:
            execute_implied_instruction(&nes_cpu::SEC, 2);
            break;

        case 0xF8:
            execute_implied_instruction(&nes_cpu::SED, 2);
            break;

        case 0x78:
            execute_implied_instruction(&nes_cpu::SEI, 2);
            break;

        case 0x85:
            execute_instruction(&nes_cpu::STA, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x95:
            execute_instruction(&nes_cpu::STA, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0x8D:
            execute_instruction(&nes_cpu::STA, &nes_cpu::get_absolute, 3, 4);
            break;
        case 0x9D:
            execute_instruction(&nes_cpu::STA, &nes_cpu::get_absolute_x, 3, 5);
            break;
        case 0x99:
            execute_instruction(&nes_cpu::STA, &nes_cpu::get_absolute_y, 3, 5);
            break;
        case 0x81:
            execute_instruction(&nes_cpu::STA, &nes_cpu::get_indirect_x, 2, 6);
            break;
        case 0x91:
            execute_instruction(&nes_cpu::STA, &nes_cpu::get_indirect_y, 2, 6);
            break;

        case 0x86:
            execute_instruction(&nes_cpu::STX, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x96:
            execute_instruction(&nes_cpu::STX, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0x8E:
            execute_instruction(&nes_cpu::STX, &nes_cpu::get_absolute, 3, 4);
            break;

        case 0x84:
            execute_instruction(&nes_cpu::STY, &nes_cpu::get_zeropage, 2, 3);
            break;
        case 0x94:
            execute_instruction(&nes_cpu::STY, &nes_cpu::get_zeropage_x, 2, 4);
            break;
        case 0x8C:
            execute_instruction(&nes_cpu::STY, &nes_cpu::get_absolute, 3, 4);
            break;

        case 0xAA:
            execute_implied_instruction(&nes_cpu::TAX, 2);
            break;

        case 0xA8:
            execute_implied_instruction(&nes_cpu::TAY, 2);
            break;

        case 0xBA:
            execute_implied_instruction(&nes_cpu::TSX, 2);
            break;

        case 0x8A:
            execute_implied_instruction(&nes_cpu::TXA, 2);
            break;

        case 0x9A:
            execute_implied_instruction(&nes_cpu::TXS, 2);
            break;

        case 0x98:
            execute_implied_instruction(&nes_cpu::TYA, 2);
            break;
        
        default:
            return false;
    }

    cout << hex << (unsigned int) (0xFF & MEM[PC]) << endl;

    return true;
}
