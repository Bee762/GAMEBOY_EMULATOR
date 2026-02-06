#include <windows.h>
#include <string>
#include <iostream>
#include <cstdint>

struct memory {
memory () {
 mapping_status = map_rom_cartridge();
 if (!mapping_status) std::cerr << "rom mapping failed" << std::endl;
}

~memory () {
 UnmapViewOfFile (rom_adress);//unmapping rom adress to rom cartridge
  rom_adress = nullptr;
  //if handles exist close them.
 if (game_file) CloseHandle(game_file);
 if (game_rom) CloseHandle(game_rom);

 //order of destruction will be reverse of order of construction
}



bool map_rom_cartridge() {
//for more documentation read microsofts memory mapped io in windows

game_file = CreateFileA("Pokemon-Red.gb",GENERIC_READ,FILE_SHARE_READ,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_READONLY,NULL);
//if handle generation fails it returns invalid_handle_value.
if (game_file == INVALID_HANDLE_VALUE) {
    std::cerr << "HANDLE CREATION FOR GAMEFILE FAILED" << std::endl;
    //not closing gamefile handle here as if gamefile fails it returns invalid input value and
    //closehandle(invalid....) is undefined behaviour
    game_file = nullptr; //reset handle
    return false;
}

//createfilea() returns a handle to a specific file adress with added params

//we will pass that handle at createfilemapping() it will map the file and return a handle
//to the mapped file

game_rom = CreateFileMappingA (game_file,NULL,PAGE_READONLY,0,0,NULL);
//if this fails it returns null.
if (!game_rom) {
    std::cerr << "HANDLE CREATION FOR GAMEROM FAILED" << std::endl;
    CloseHandle(game_file);//close previous handle
    //current handle already returns null on faliure
    game_file = nullptr;
    game_rom = nullptr;
    return false;
}

//then passing this handle at mapviewoffile() lets us access mapped file

rom_adress = (uint8_t*)MapViewOfFile(game_rom,FILE_MAP_READ,0,0,0);
//mapviewof file returns a pointer but a void pointer,so to store its adress in c++
//i need to explicitely cast mapviewfile return type as int pointer
// thats why we use (int pntr) before this

if (!rom_adress) {
    std::cerr << "ADRESS MAPPING FAILED" << std::endl;
    CloseHandle(game_file);
    CloseHandle(game_rom);
    game_file = nullptr;
    game_rom = nullptr;
    return false;
}

return true;
}

uint8_t read_memory (uint16_t adress) const {  //read from cartridge rom
    if (adress < 0x8000)return rom_adress [adress];   //pointer arithmetic : rom_adress [i] = i elements after rom adress.
    else return ram [adress - 0x8000]; // ram indexing in my memory starts from 0 so convert.
    //if memory adress location is in rom return rom adress else return ram adress
    //rom memory adress in gameboy : 0x0000 to 0x7FFF
}

void write_memory(uint16_t adress, uint8_t value) {  //write into gameboy memory
    if (adress >= 0x8000) ram[adress - 0x8000] = value; //only write in ram
}

HANDLE game_file = nullptr; //storing handle outside the functon so they can be closed later.
HANDLE game_rom = nullptr;
uint8_t* rom_adress = nullptr; // the starting adress of mapped rom.

uint8_t ram [32768];   //gameboy memory .

bool mapping_status = false;

};

struct cpu {
//gameboy uses Sharp LR35902 cpu,it has 8 bit architecture

memory mem;

//cpu registers :
uint8_t IR;    //instruction register,stores the opcode from rom
uint8_t IE;    //interrupt enable ,tells cpu which interupts are allowed to trigger
uint8_t A;     //accumulator : stores arithmeti operations from alu
uint8_t F = 0b00000000;    //flags 
uint8_t B;
uint8_t C;
uint8_t D;//these 6 are general registers {b,c,d,e,h,l}
uint8_t E;
uint8_t H;
uint8_t L;
uint16_t PC; //program counter ,holds current adress of memory
uint16_t SP; //stack pointer : holds  adress of ram

//now lets make paired registers : 

uint16_t pair_registers (uint8_t& reg1 , uint8_t& reg2) {
  return ((static_cast <uint16_t> (reg1) << 8) | reg2);
  //converts first reg to 16 bit and leftshift by 8 bits and mask with lower bit
}

void split_registers (uint8_t& register1, uint8_t& register2, uint16_t& paired_register) {
    register1 = static_cast <uint8_t>(paired_register >> 8);
    register2 = static_cast <uint8_t> (paired_register);
}

//flags : gameboy cpu has fixed 4 flags(z,n,h,c)
//flag is nothing but a specific bit in F,its already specified in cpu
//each flag is set to 0 or 1 depending on the operation

void set_flag (char flag_name , bool data) {

if (flag_name == 'Z') {  //7th bit of F is flag zero
uint8_t flag_Z = 0x00 | (data << 7); //passing data into desired bit,adding a mask before to make sure no garbage value exists
F &= ~(1 << 7);  //clearing previous data from specified bit from the flag without clearing whole flag
F |= flag_Z;     //storing new data
}

else if (flag_name == 'N') { //6th bit of F is subrtact flag
uint8_t flag_N = 0x00 | (data << 6);
F &= ~(1 << 6);
F |= flag_N;
}

else if (flag_name == 'H') { //5th bit of F is halfcarry flag
uint8_t flag_H = 0x00 | (data << 5);
F &= ~(1 << 5);
F |= flag_H;
}

else if (flag_name == 'C') {  //4th bit of F is carry flag
uint8_t flag_C = 0x00 | (data << 4);
F &= ~(1 << 4);
F |= flag_C;
}
//we dont care about bit 0-3,ignore them
F &= 0xF0; //forcing lower nibble to zero
}//func end

bool get_flag (char flag_name) {
if (flag_name == 'Z') return F & (1 << 7);
else if (flag_name == 'N') return F & (1 << 6);
else if (flag_name == 'H') return F & (1 << 5);
else if (flag_name == 'C') return F & (1 << 4);
else return 0; //if there are no matches return 0;
//our i just want want specific value 0 or 1 ,my return gets the correct binary but returns 
//it as it is so it can be translated to 0 or the actual value of binary(128 for zero flag).we only want 0 and 1 hence return type
//is bool,anything > 0  is 1.
}

//now we need to emulate arithmetic logic unit aka alu.
//alu is a part of cpu that performs simple math operations and logical operations
//and alu stores it result to either A register or to memory (ram)

//Arithmetic Logic Unit : 

//arithmetic operations : can write back to memory adress or register depending on operation

void alu_add (uint8_t& register1 , uint8_t& register2) {

    uint16_t result = register1 + register2; //calculate result in 16 bit to check bit overflow in 8 bit result

    //update flags

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 0);

    if (((register1 & 0xF) + (register2 & 0xF)) > 0xF ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (result > 0xFF) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);  //store the result in first register
}

void alu_add_memory_adress (uint8_t& register1 , uint8_t& register2 ,uint8_t& register3) {

    uint16_t paired_register = pair_registers (register2,register3);

    uint8_t value = mem.read_memory(paired_register);

    uint16_t result = register1 + value;

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 0);

    if (((register1 & 0xF) + (value & 0xF)) > 0xF ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (result > 0xFF) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);

}

void alu_add_carry (uint8_t& register1 , uint8_t& register2) {

    bool carry_flag = get_flag('C');

    uint16_t result = register1 + register2 + carry_flag;

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 0);

    if (((register1 & 0xF) + (register2 & 0xF) + carry_flag) > 0xF ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (result > 0xFF) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);
}

void alu_add_memory_adress_carry (uint8_t& register1 , uint8_t& register2 ,uint8_t& register3) {

    uint16_t paired_register = pair_registers (register2,register3);

    uint8_t value = mem.read_memory(paired_register);

    bool carry_flag = get_flag('C');

    uint16_t result = register1 + value + carry_flag;

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 0);

    if (((register1 & 0xF) + (value & 0xF) + carry_flag) > 0xF ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (result > 0xFF) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);

}

void alu_add_paired_reg (uint8_t& register1, uint8_t& register2, uint8_t& register3, uint8_t& register4) {
    //for addition of 2 paired reg
    uint16_t paired_reg1 = pair_registers(register1,register2);

    uint16_t paired_reg2 = pair_registers(register3,register4);

    uint32_t result = paired_reg1 + paired_reg2;

    //no zero flag updates in 16 bit add

    set_flag ('N', 0);

    if (((paired_reg1 & 0x0FFF) + (paired_reg2 & 0x0FFF)) > 0x0FFF )set_flag ('H', 1); //checking if half overflow has happened
    //from lower byte + lower nibble of upperbyte to upper nibble of upper byte,
    else set_flag ('H', 0);

    if (result > 0xFFFF) set_flag ('C', 1);
    else set_flag ('C', 0);

    paired_reg1 = static_cast <uint16_t> (result);

    split_registers (register1,register2,paired_reg1);
}

void alu_add_paired_reg (uint8_t& register1, uint8_t& register2, uint16_t& register3) {
    //for addition of a paired reg with value in sp,(reg3 is stack pointer)
    uint16_t paired_reg1 = pair_registers(register1,register2);

    uint32_t result = paired_reg1 + register3;

    //no zero flag updates in 16 bit add

    set_flag ('N', 0);

    if (((paired_reg1 & 0x0FFF) + (register3 & 0x0FFF)) > 0x0FFF )set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (result > 0xFFFF) set_flag ('C', 1);
    else set_flag ('C', 0);

    paired_reg1 = static_cast <uint16_t> (result);

    split_registers (register1,register2,paired_reg1);
}

void alu_sub (uint8_t& register1 , uint8_t& register2) {

    uint16_t result = register1 - register2; 

    //update flags

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ( (register1 & 0xF) < (register2 & 0xF) ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (register1 < register2) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);  //store the result in first register
}

void alu_sub_memory_adress (uint8_t& register1 , uint8_t& register2 ,uint8_t& register3) {

    uint16_t paired_register = pair_registers (register2,register3);

    uint8_t value = mem.read_memory(paired_register);

    uint16_t result = register1 - value;

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ((register1 & 0xF) < (value & 0xF) ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (register1 < value) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);

}

void alu_sub_carry(uint8_t& register1 , uint8_t& register2) {

    bool carry_flag = get_flag('C');

    uint16_t result = register1 - register2 - carry_flag; 

    //update flags

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ( (register1 & 0xF) < ((register2 & 0xF) + carry_flag)) set_flag ('H', 1);
    else set_flag('H', 0);

    if (register1 < (register2 + carry_flag)) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);  //store the result in first register
}

void alu_sub_memory_adress_carry (uint8_t& register1 , uint8_t& register2 ,uint8_t& register3) {

    uint16_t paired_register = pair_registers (register2,register3);

    uint8_t value = mem.read_memory(paired_register);

    bool carry_flag = get_flag('C');

    uint16_t result = register1 - value - carry_flag;

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ((register1 & 0xF) < ( (value & 0xF) + carry_flag) ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (register1 < (value + carry_flag)) set_flag ('C', 1);
    else set_flag ('C', 0);

    register1 = static_cast<uint8_t> (result);

}

void alu_sub_paired_reg (uint8_t& register1, uint8_t& register2, uint8_t& register3, uint8_t& register4) {
    //for addition of 2 paired reg
    uint16_t paired_reg1 = pair_registers(register1,register2);

    uint16_t paired_reg2 = pair_registers(register3,register4);

    uint32_t result = paired_reg1 - paired_reg2;

    //no zero flag updates in 16 bit add

    set_flag ('N', 1);

    if ( (paired_reg1 & 0x0FFF) < (paired_reg2 & 0x0FFF) )set_flag ('H', 1); //checking if half overflow has happened
    //from lower byte + lower nibble of upperbyte to upper nibble of upper byte,
    else set_flag ('H', 0);

    if (paired_reg1 < paired_reg2) set_flag ('C', 1);
    else set_flag ('C', 0);

    paired_reg1 = static_cast <uint16_t> (result);

    split_registers (register1,register2,paired_reg1);
}

void alu_sub_paired_reg (uint8_t& register1, uint8_t& register2, uint16_t& register3) {
    //for addition of a paired reg with value in sp,(reg3 is stack pointer)
    uint16_t paired_reg1 = pair_registers(register1,register2);

    uint32_t result = paired_reg1 - register3;

    //no zero flag updates in 16 bit add

    set_flag ('N', 1);

    if ( (paired_reg1 & 0x0FFF) < (register3 & 0x0FFF) )set_flag ('H', 1); //checking if half overflow has happened
    //from lower byte + lower nibble of upperbyte to upper nibble of upper byte,
    else set_flag ('H', 0);

    if (paired_reg1 < register3) set_flag ('C', 1);
    else set_flag ('C', 0);


    paired_reg1 = static_cast <uint16_t> (result);

    split_registers (register1,register2,paired_reg1);
}

void alu_inc_reg (uint8_t& register1) {

    uint16_t result = register1 + 1;

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 0);

    if ((register1 & 0xF) == 0xF ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

  // carry flag stays unaffected in inc and dec in gameboy cpu

    register1 = static_cast<uint8_t> (result);

}

void alu_inc_memory (uint8_t& register1 , uint8_t& register2) {

    uint16_t paired_register = pair_registers (register1,register2);

    uint8_t value = mem.read_memory(paired_register);

    uint8_t old_value = value;//preserving old value for half carry check


    value++;

    mem.write_memory(paired_register,value);

    if (value == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 0);

    if ((old_value & 0xF) == 0xF ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);


    //no flags will be touched here

}

void alu_inc_paired_reg (uint8_t& register1, uint8_t& register2) {
 uint16_t paired_reg = pair_registers (register1,register2);
 paired_reg++;
 split_registers (register1,register2,paired_reg);
}

void alu_inc_paired_reg (uint16_t& register1) {
//for inc stack pointer
 register1++;
}

void alu_dec_reg (uint8_t& register1) {

    uint16_t result = register1 - 1;

    if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ((register1 & 0xF) == 0x0 ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

  // carry flag stays unaffected in inc and dec in gameboy cpu

    register1 = static_cast<uint8_t> (result);

}

void alu_dec_memory (uint8_t& register1 , uint8_t& register2) {

    uint16_t paired_register = pair_registers (register1,register2);

    uint8_t value = mem.read_memory(paired_register);

    uint8_t old_value = value;//preserving old value for half carry check

    value--;

    mem.write_memory(paired_register,value);

    if (value == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ((old_value & 0xF) == 0x0 ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

}

void alu_dec_paired_reg (uint8_t& register1, uint8_t& register2) {
 uint16_t paired_reg = pair_registers (register1,register2);
 paired_reg--;
 split_registers (register1,register2,paired_reg);
}

void alu_dec_paired_reg (uint16_t& register1) {
//for dec stack pointer
 register1--;
}

//comparision operations :

void alu_compare_reg (uint8_t& register1, uint8_t& register2) {
    uint16_t result = register1 - register2;

     if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ( (register1 & 0xF) < (register2 & 0xF) ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (register1 < register2) set_flag ('C', 1);
    else set_flag ('C', 0);

    //result  will be scrapped its only to update flags
}

void alu_compare_reg_memory_adress (uint8_t& register1, uint8_t& register2, uint8_t& register3) {

    uint16_t paired_register = pair_registers (register2,register3);

    uint8_t value = mem.read_memory(paired_register);
    uint16_t result = register1 - value;

     if (static_cast<uint8_t>(result) == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ( (register1 & 0xF) < (value & 0xF) ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    if (register1 < value) set_flag ('C', 1);
    else set_flag ('C', 0);

    //result  will be scrapped its only to update flags
}

// logical operations : always writes  back at register A
// i will use register as a param but alu will write back logical operators
//at A register

//logical alu operations have hardwired flag behaviour unlike arithmetic operations
//only need to check zero flag rest are hardwired

void alu_and(uint8_t& register1 , uint8_t value) {
    register1 &= value;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',1); //half carry is 1 in and operation(cpu design)

    set_flag ('C',0); //carry is 0 because  bit cant overflow in logical operations
}

void alu_and_memory_adress(uint8_t& register1 , uint8_t& register2, uint8_t& register3) {

    uint16_t adress = pair_registers(register2,register3);
    uint8_t value = mem.read_memory(adress);

    register1 = register1 & value;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',1); //half carry is 1 in and operation(cpu design)

    set_flag ('C',0); //carry is 0 because  bit cant overflow in logical operations
}

void alu_or(uint8_t& register1 , uint8_t value) {
    register1 |= value;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); //half carry is 0 in or operation(cpu design)

    set_flag ('C',0); //carry is 0 because  bit cant overflow in logical operations
}

void alu_or_memory_adress(uint8_t& register1 , uint8_t& register2, uint8_t& register3) {

    uint16_t adress = pair_registers(register2,register3);
    uint8_t value = mem.read_memory(adress);

    register1 = register1 | value;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); //half carry is 0 in or operation(cpu design)

    set_flag ('C',0); //carry is 0 because  bit cant overflow in logical operations
}

void alu_xor(uint8_t& register1 , uint8_t value) {
    register1 ^= value;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); //half carry is 0 in xor operation(cpu design)

    set_flag ('C',0); //carry is 0 because  bit cant overflow in logical operations
}

void alu_xor_memory_adress(uint8_t& register1 , uint8_t& register2, uint8_t& register3) {

    uint16_t adress = pair_registers(register2,register3);
    uint8_t value = mem.read_memory(adress);

    register1 = register1 ^ value;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); //half carry is 0 in xor operation(cpu design)

    set_flag ('C',0); //carry is 0 because  bit cant overflow in logical operations
}

//rotational operations : 

void alu_rotate_left_circular (uint8_t& register1) {
    //shift bits by 1 left, bit is wrapped around ,and felled of bits goes
    //into carry and rightmost bit

    bool fallen_bit = (register1 >> 7);  //so if i shift everyting right by 7 only last bit can have non 
    //zero value and storing it in a boolean.

    register1 = (register1 << 1) | fallen_bit;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit); 

}

void alu_rotate_left_circular_memory_adress(uint8_t& register1, uint8_t& register2) {
    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    bool fallen_bit = (value >> 7);

    value = (value << 1) | fallen_bit;

    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_rotate_right_circular (uint8_t& register1) {
    //shift bits by 1 right, bit is wrapped around ,and felled of bits goes
    //into carry and leftmost bit

    bool fallen_bit = (register1 & 0x01);  //so if i shift everyting right by 7 only last bit can have non 
    //zero value and storing it in a boolean.

    register1 = (register1 >> 1) | (fallen_bit << 7);

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0);

    set_flag ('C',fallen_bit); 

}

void alu_rotate_right_circular_memory_adress(uint8_t& register1, uint8_t& register2) {
    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    bool fallen_bit = (value & 0x01);

    value = (value >> 1) | (fallen_bit << 7);

    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_rotate_left_carry(uint8_t& register1) {
    bool old_carry = get_flag('C');
    bool fallen_bit = (register1 >> 7);

    register1 = (register1 << 1) | old_carry;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);

}

void alu_rotate_left_carry_memory_adress(uint8_t& register1,uint8_t& register2) {
    bool old_carry = get_flag('C');

    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    bool fallen_bit = (value >> 7);

    value = (value << 1) | old_carry;

    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);

}

void alu_rotate_right_carry(uint8_t& register1) {
    bool old_carry = get_flag('C');
    bool fallen_bit = (register1 & 0x01);

    register1 = (register1 >> 1) | (old_carry << 7);

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);

}

void alu_rotate_right_carry_memory_adress(uint8_t& register1,uint8_t& register2) {
    bool old_carry = get_flag('C');

    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    bool fallen_bit = (value & 0x01);

    value = (value >> 1) | (old_carry << 7);

    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);

}

//shift operations : 

void alu_shift_left (uint8_t& register1) {
    bool fallen_bit = (register1 >> 7);
    register1 <<= 1;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_shift_left_memory_adress (uint8_t& register1,uint8_t& register2) {
    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    bool fallen_bit = (value >> 7);
    value <<= 1;
    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_shift_right (uint8_t& register1) {
    //for signed ints,preserve leftmost bit to preserve sign
    bool fallen_bit = (register1 & 0x01);
    bool preserved_bit = (register1 >> 7);
    register1 = (register1 >> 1) | (static_cast<uint8_t>(preserved_bit) << 7);

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_shift_right_memory_adress (uint8_t& register1,uint8_t& register2) {
    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    bool fallen_bit = (value & 0x01);
    bool preserved_bit = (value >> 7);
    value = (value >> 1) | (static_cast<uint8_t>(preserved_bit) << 7);

    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_shift_right_logical (uint8_t& register1) {
    //same as shift right but not preserve rightmost bit
    bool fallen_bit = (register1 & 0x01);

    register1 >>= 1;

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_shift_right_logical_memory_adress (uint8_t& register1,uint8_t& register2) {
    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    bool fallen_bit = (value & 0x01);
    value >>= 1;
    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
}

void alu_swap (uint8_t& register1) {
    //swap upper nibble with lower nibble
    register1 = (register1 << 4) | (register1 >> 4);

    if (register1 == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',0);
}

void alu_swap_memory (uint8_t& register1 ,uint8_t& register2) {
    //swap upper nibble with lower nibble
    uint16_t adress = pair_registers(register1,register2);
    uint8_t value = mem.read_memory(adress);

    value = (value << 4) | (value >> 4);

    mem.write_memory(adress,value);

    if (value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',0);
}

//special operations : 

void alu_complement_reg (uint8_t& register1) {
    register1 = ~register1;//just flips bits in a register
    set_flag('N',1);//coz its like subtraction
    set_flag('H',1);
    //zero flag and carry flag remains unchanged
}

void alu_set_carry_flag() {
    set_flag('C',1);//just sets carry flag to 1
    //these flags are cleared
    set_flag('N',0);
    set_flag('H',0);
}

void alu_complement_carry_flag() {
    bool carry_flag = get_flag('C');
    set_flag ('C',!carry_flag);//just complements carry flag(inverse the true or false)
    //these flags are cleared
    set_flag('N',0);
    set_flag('H',0);
}

void alu_daa(uint8_t& register1) {
    uint8_t result = register1;
    bool carry_flag = get_flag('C');
    bool half_carry_flag = get_flag('H');
    bool subtract_flag = get_flag('N');

    //check condition with actual variable and modify copy

    if (!subtract_flag) {

        if (half_carry_flag || (register1 & 0x0F) > 0x09) result+= 0x06;
         if (carry_flag || register1 > 0x99) {
            result += 0x60;
            set_flag('C',1);
         }
         else set_flag ('C',0);
        register1 = result;
    }

    else {
         if (half_carry_flag) result-= 0x06;
           if (carry_flag) {
            result -= 0x60;
            }
        register1 = result;
        set_flag ('C',carry_flag); //carry flag will be preserved in subtraction
        }
    if (register1 == 0x00) set_flag('Z',1);
    else set_flag('Z',0);

    set_flag ('H',0);
}

//now lets write opcode fetching and execution.

uint8_t fetch_opcode () {
    IR = mem.read_memory(PC); //store opcode in instruction register
    PC++;//increment programm counter
    return IR; //return it
}


uint32_t execute_opcode () {

 uint8_t opcode = fetch_opcode();

 if (opcode == 0b01110110) {
    //halt :  0b01110110 is cpu a instruction called halt so we skip this
    //will add this later 
    return 0;//temporary return 
 }

//ld opcodes : 8 bit load operations :

 // load register opcodes (ld r r') means r = r',means add value of reg r' in r register
 //they have a fixed pattern they start with 0b01 next bit is first r and last 3 bit are 2nd r
 //depending on that we load a register

 //ld r r' : load register r with value of register r'
 //opcode : 0b01xxxyyyy (x,y = variable)

 else if ((opcode & 0b11000000) == 0b01000000) {  //masking first 2 digit and letting 01 in only
    //now lets get value of last register and store it in value (r')
    uint8_t value = 0b00000000;
    uint8_t last_register = opcode & 0b00000111; //only keep last 3 bbits

     if (last_register == 0b000) value = B; //here 000 after 0b represents last 3 bits from right(in lower nibble)
     else if (last_register == 0b001) value = C;
     else if (last_register == 0b010) value = D;
     else if (last_register == 0b011) value = E;
     else if (last_register == 0b100) value = H;
     else if (last_register == 0b101) value = L;
     else if (last_register == 0b110) value = mem.read_memory(pair_registers(H,L)); //value at memory adress pointed by (HL)
     else if (last_register == 0b111) value = A;

     //now lets assign value in designated register (ld r)

     uint8_t first_register = (opcode >> 3) & 0b00000111;
       //rightshifting by 3 removes last 3 bit and shifts bit 3-5 t0 6-8
      //and masking with 111 keeps last 3 bits only and rest are forced to zero
     //0b011 = 0b00000011 and != 0b01100000 remember this,also every other bit before 011 will be assigned to zero

     if (first_register == 0b000) B = value;
     else if (first_register == 0b001) C = value;
     else if (first_register == 0b010) D = value;
     else if (first_register == 0b011) E = value;
     else if (first_register == 0b100) H = value;
     else if (first_register == 0b101) L = value;
     else if (first_register == 0b110) mem.write_memory(pair_registers(H,L),value);
      //0b01110110 is already skipped as halt
      //also it translates to ld (hl) (hl) means write value stored by adress hl to adress hl,it does nothing anyways
      //thats why cpu uses this specific binary as halt
     else if (first_register == 0b111) A = value;

     //now we need to write machine cycles taken to execute this opcode as return statement

        if (first_register == 0b110) return 2;  //ld (hl) r takes 2 machine cycles
        else if (last_register == 0b110) return 2;  // ld r (hl) takes 2 machine cycles
        else return 1; // ld r r' takes 1 machine cycle


     //this takes care of all ld r r' instructions
 }


 //ld r , n : load register r with immediate value n from next opcode,so it will take 2 byte cycle of fetch decode execute
 //byte cycle means how many times i need to fetch opcode it doesnot means machine cycle
 //opcode range 0b00xxx110 (x = variable),

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00000111) == 0b00000110) {
    uint8_t reg = (opcode >> 3) & 0b00000111;//storing data of register 
    uint8_t n = fetch_opcode();//getting value of n

     if (reg == 0b000) B = n;
     else if (reg == 0b001) C = n;
     else if (reg == 0b010) D = n;
     else if (reg == 0b011) E = n;
     else if (reg == 0b100) H = n;
     else if (reg == 0b101) L = n;
     else if (reg == 0b110) mem.write_memory(pair_registers(H,L),n);
     else if (reg == 0b111) A = n;

     //return statements
       
     if (reg == 0b110) return 3; //ld (hl) n returns 3 machine cycle
     else return 2; //ld r n takes 2 machine cycle to execute

 }


 //ld a (bc) : load a from memory adress pointed by bc(1 byte cycle 2 machine cycle)
 //opcode : 0b00001010

 else if (opcode == 0b00001010) {
    A = mem.read_memory(pair_registers(B,C));
    return 2;
 }

 //ld a (de) : load a from memory adress pointed by de(1 byte cycle 2 machine cycle)
 //opcode : 0b00011010

 else if (opcode == 0b00011010) {
    A = mem.read_memory(pair_registers(D,E));
    return 2;
 }

 //ld (bc) a : load memory adress pointed by bc with value at a (1 byte cycle 2 machine cycle)
 //opcode : 0b00000010

 else if (opcode == 0b00000010) {
    mem.write_memory(pair_registers(B,C),A);
    return 2;
 }

 //ld (de) a : load memory adress pointed by de with value at a (1 byte cycle 2 machine cycle)
 //opcode : 0b00010010

 else if (opcode == 0b00010010) {
    mem.write_memory(pair_registers(D,E),A);
    return 2;
 }

 //ld a (nn) : load register a with value from adress specified by nn(3 byte cycle 4 machine cycle)
 //first n is actually low byte and second n is highbyte,accordin to cpu instruction
 //opcode : 0b11111010
 
 else if (opcode == 0b11111010) {
    uint8_t n1 = fetch_opcode();//low byte (first n)
    uint8_t n2 = fetch_opcode();//high byte (second n)
    //n1 and n2 are immediate value (nn) = (n2 n1)
    A = mem.read_memory(pair_registers(n2,n1)); //n1 and n2 are not registers but they will use same concept
    //but in pair register (a,b) a is used as high byte as we will use n2 as highbyte so n2 first
    return 4;
 }

 //ld (nn) a : load adress (nn) with data in a (3 byte cycle and 4 machine cycle)
 //opcode : 0b11101010

 else if (opcode == 0b11101010) {
    uint8_t n1 = fetch_opcode();
    uint8_t n2 = fetch_opcode();
    mem.write_memory(pair_registers(n2,n1),A);
    return 4;
 }

 //ldh a (c) : load A with high memory adress specified by c,
 //high memory means the high stream it ranges from 0xFF00 to 0xFFFF,it is used for I/O management in gameboy
 //opcode : 0b11110010               (2 machine cycle 1 byte cycle)

 else if (opcode == 0b11110010) {
    A = mem.read_memory(0xFF00 + C); //just making sure memory stays in range of 0xFF00 - 0xFFFF
    //as c is 8 bit int the result highbyte stays 0xFF
    return 2;
 }

 //ldh (c) a : load high memory adress c with a (1 byte cycle 2 machine cycle)
 //opcode : 0b11100010

 else if (opcode == 0b11100010) {
    mem.write_memory(C+0xFF00,A);
    return 2;
 }

 //ldh a (n) : load a with high memory (n) (2 byte cycle 3 machine cycle)
 //opcode = 0b11110000

 else if (opcode == 0b11110000) {
    uint8_t n = fetch_opcode();
    A = mem.read_memory(0xFF00+n);
    return 3;
 }

 //ldh (n) a : load high memory adress n with a (2 byte cycle 3 machine cycle)
 //opcode = 0b11100000

 else if (opcode == 0b11100000) {
    uint8_t n = fetch_opcode();
    mem.write_memory(0xFF00+n,A);
    return 3;
 }

 //ldh a (hl-) : load a with adress (hl),then decrement hl and store(1 byte cycle 2 machine cycle)
 //opcode : 0b00111010

 else if (opcode == 0b00111010) {
    uint16_t hl = pair_registers(H,L);
    A = mem.read_memory(hl);
    hl--;
    split_registers(H,L,hl);
    return 2;
 }

 //ld (hl-) a : load adress hl with a and then decrement hl and store (1 byte cycle 2 machine cycle)
 //opcode : 0b00110010

 else if (opcode == 0b00110010) {
    uint16_t hl = pair_registers(H,L);
    mem.write_memory(hl,A);
    hl--;
    split_registers(H,L,hl);
    return 2;
 }

 //ldh a (hl+) : load a with adress (hl),then increment hl and store(1 byte cycle 2 machine cycle)
 //opcode : 0b00101010

 else if (opcode == 0b00101010) {
    uint16_t hl = pair_registers(H,L);
    A = mem.read_memory(hl);
    hl++;
    split_registers(H,L,hl);
    return 2;
 }

 //ld (hl+) a : load adress hl with a and then increment hl and store (1 byte cycle 2 machine cycle)
 //opcode : 0b00100010

 else if (opcode == 0b00100010) {
    uint16_t hl = pair_registers(H,L);
    mem.write_memory(hl,A);
    hl++;
    split_registers(H,L,hl);
    return 2;
 }

 //ld opcodes : 16 bit load instructions :

 //ld rr nn : load 16 bit register rr' with 16 bit immediate data nn (3 byte cycle , 3 machine cycle)
 //opcode : 0b00xx0001

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00000001) {
    uint8_t reg = (opcode >> 4) & 0b00000011; //storing data of register r
    //now lets get nn
    uint8_t lowbyte = fetch_opcode();//first n aka next opcode is actually lowbyte (cpu design)
    uint8_t highbyte = fetch_opcode();
    uint16_t nn = pair_registers(highbyte,lowbyte); //storing nn 

    if (reg == 0b00) split_registers(B,C,nn);//split nn into b and c register
    else if (reg == 0b01) split_registers(D,E,nn);
    else if (reg == 0b10) split_registers(H,L,nn);
    else if (reg == 0b11) SP = nn;  //storing 16 bit value in stack pointer
    
    return 3;
 }

 //ld (nn) sp : load adress (nn) with value from stack pointer (3 byte cycle , 5 machine cycle)
 //opcode : 0b00001000

 else if (opcode == 0b00001000) {
    uint8_t lowbyte = fetch_opcode();//first n aka next opcode is actually lowbyte (cpu design)
    uint8_t highbyte = fetch_opcode();
    //we will split sp in two 8 bit int because memory cant store 16 bit data
    uint16_t adress = pair_registers(highbyte,lowbyte);
    mem.write_memory(adress,SP & 0x00FF);  //low byte in memory ,
    adress++;
    mem.write_memory(adress,((SP >> 8) & 0x00FF));  //high byte in memory ,

    return 5;
 }

 //ld sp hl : load stack pointer with data from hl register (1 byte cycle , 2 machine cycle)
 //opcode : 0b11111001

 else if (opcode == 0b11111001) {
    SP = pair_registers(H,L);
    return 2;
 }

 //push rr : push to stack -> push data from 16 bit registers to memory adress pointed by sp
 //opcode : 0b11xx0101 (1 byte cycle 4 machine cycle) 

 else if ((opcode & 0b11000000) == 0b11000000 && (opcode & 0b00001111) == 0b00000101) {
    SP--; //because we will write at memory ,sp had an adress from before so we move to next empty space
    //we are using decrement because stack starts at higher adress and grows lower (cpu designers choice).
    uint8_t rr = (opcode >> 4) & 0b00000011;

    if (rr == 0b00) { // BC register
    mem.write_memory(SP,B); //memory can store 1 byte at a adress as its 8 bit
    SP--; // so we decrement sp again to go to next adjacent slot
    mem.write_memory(SP,C); //and store 2nd register value there instead of pairing and storing
    }

    else if (rr == 0b01) { //DE register
    mem.write_memory(SP,D); 
    SP--; 
    mem.write_memory(SP,E); 
    }

    else if (rr == 0b10) { //HL register
    mem.write_memory(SP,H); 
    SP--; 
    mem.write_memory(SP,L); 
    }

    else if (rr == 0b11) { //AF register
    mem.write_memory(SP,A); 
    SP--; 
    mem.write_memory(SP,F & 0xF0);  // we only need first 4 bit of flag,lower bit is masked to zero
    }

    return 4;
 }

 //pop rr : pop from stack -> pop data from stack to 16 bit register (1 byte cycle 3 machine cycle)
 //opcode : 0b11xx0001

 else if ((opcode & 0b11000000) == 0b11000000 && (opcode & 0b00001111) == 0b00000001) {
    uint8_t lowbyte = mem.read_memory(SP);
    SP++;
    uint8_t highbyte = mem.read_memory(SP);
    SP++; //sp always points at next data to pop,so pop can be called again

    uint16_t stack_data = pair_registers(highbyte,lowbyte);//pairing high and lowbyte
    uint8_t rr = (opcode >> 4) & 0b00000011; //storing paired register data

    if (rr == 0b00) split_registers(B,C,stack_data);
    else if (rr == 0b01) split_registers(D,E,stack_data);
    else if (rr == 0b10) split_registers(H,L,stack_data);
    else if (rr == 0b11) {
    split_registers(A,F,stack_data);
    F &= 0xF0; //removing lower nibble as it needs to stay zero
    }

    return 3;
 }

 //ld hl,sp+e : load hl with stackpointer value + e where e is immediate signed  int (not unsigned)
 //opcode : 0b11111000; (2 byte cycle,3 machine cycle)

 else if (opcode == 0b11111000) {
    int8_t e = fetch_opcode();
    uint32_t HL = SP + e;
    //this operation sets flag 
    //clears z and n flag and sets carry and half carry flag
    set_flag('Z',0);
    set_flag('N',0);

    if ((SP & 0xF) + (e & 0xF) > 0xF) set_flag('H',1);
    else set_flag('H',0);

    if ((SP & 0xFF) + (e & 0xFF) > 0xFF) set_flag('C',1);
    else set_flag('C',0);

    uint16_t result = static_cast <uint16_t> (HL);

    split_registers(H,L,result);

    return 3;
 }

 // ld operations end

 // alu operations : 

 //additions : 

 //add r : add register r with a and store result back in a (1byte cycle 1 machine cycle)
 //opcode : 0b10000xxx

 else if ((opcode & 0b11111000) == 0b10000000) {
     uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_add(A,B);
     else if (r == 0b001) alu_add(A,C);
     else if (r == 0b010) alu_add(A,D);
     else if (r == 0b011) alu_add(A,E);
     else if (r == 0b100) alu_add(A,H);
     else if (r == 0b101) alu_add(A,L);
     else if (r == 0b110) alu_add_memory_adress(A,H,L);
     else if (r == 0b111) alu_add(A,A);

      //return 
       if (r == 0b110) return 2;
       else return 1;
 }

 //add n : add register a with immediate value n and store back in a(2 byte cycle 2 machine cycle)
 //opcode = 0b11000110

 else if (opcode == 0b11000110) {
    uint8_t n = fetch_opcode();
    alu_add (A,n);
    return 2;
 }

 //adc r : add a to register r with carry flag store back at a (1 byte cycle 1 machine cycle)
 //opcode = 0b10001xxx

 else if ((opcode & 0b11111000) == 0b10001000) {
    uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_add_carry(A,B);
     else if (r == 0b001) alu_add_carry(A,C);
     else if (r == 0b010) alu_add_carry(A,D);
     else if (r == 0b011) alu_add_carry(A,E);
     else if (r == 0b100) alu_add_carry(A,H);
     else if (r == 0b101) alu_add_carry(A,L);
     else if (r == 0b110) alu_add_memory_adress_carry(A,H,L);
     else if (r == 0b111) alu_add_carry(A,A);

      //return 
       if (r == 0b110) return 2;
       else return 1;
 }

 //adc n : add immediate value n to a with carry flag (2 byte cycle,2 machine cycle)
 //opcode = 0b11001110

 else if (opcode == 0b11001110) {
    uint8_t n = fetch_opcode();
    alu_add_carry(A,n);
    return 2;
 }

 //subtract 

 //sub r : subtract 8 bit register a with r and store result in a (1 byte cycle,1 machine cycle)
 //opcode : 0b10010xxx

 else if ((opcode & 0b11111000) == 0b10010000) {
     uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_sub(A,B);
     else if (r == 0b001) alu_sub(A,C);
     else if (r == 0b010) alu_sub(A,D);
     else if (r == 0b011) alu_sub(A,E);
     else if (r == 0b100) alu_sub(A,H);
     else if (r == 0b101) alu_sub(A,L);
     else if (r == 0b110) alu_sub_memory_adress(A,H,L);
     else if (r == 0b111) alu_sub(A,A);

      //return 
       if (r == 0b110) return 2;
       else return 1;

 }

 //sub n : sub register a with immediate value n and store back in a(2 byte cycle 2 machine cycle)
 //opcode = 0b11010110

 else if (opcode == 0b11010110) {
    uint8_t n = fetch_opcode();
    alu_sub (A,n);
    return 2;
 }

 //sbc r : sub a to register r with carry flag store back at a (1 byte cycle 1 machine cycle)
 //opcode = 0b10011xxx

 else if ((opcode & 0b11111000) == 0b10011000) {
    uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_sub_carry(A,B);
     else if (r == 0b001) alu_sub_carry(A,C);
     else if (r == 0b010) alu_sub_carry(A,D);
     else if (r == 0b011) alu_sub_carry(A,E);
     else if (r == 0b100) alu_sub_carry(A,H);
     else if (r == 0b101) alu_sub_carry(A,L);
     else if (r == 0b110) alu_sub_memory_adress_carry(A,H,L);
     else if (r == 0b111) alu_sub_carry(A,A);

       //return 
       if (r == 0b110) return 2;
       else return 1;
 }

 //sbc n : sub immediate value n to a with carry flag (2 byte cycle,2 machine cycle)
 //opcode = 0b11011110

 else if (opcode == 0b11011110) {
    uint8_t n = fetch_opcode();
    alu_sub_carry(A,n);
    return 2;
 }

 //comparisions : 

 // cp r : sub a with r and dont store results just update flags(1 byte cycle, 1 machine cycle)
 //opcode : 0b10111xxx

 else if ((opcode & 0b11111000) == 0b10111000) {
     uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_compare_reg(A,B);
     else if (r == 0b001) alu_compare_reg(A,C);
     else if (r == 0b010) alu_compare_reg(A,D);
     else if (r == 0b011) alu_compare_reg(A,E);
     else if (r == 0b100) alu_compare_reg(A,H);
     else if (r == 0b101) alu_compare_reg(A,L);
     else if (r == 0b110) alu_compare_reg_memory_adress(A,H,L);
     else if (r == 0b111) alu_compare_reg(A,A);

       //return 
       if (r == 0b110) return 2;
       else return 1;
 }

 //cp n : sub a immediate value n to a with carry flag dont update result change flags(2 byte cycle,2 machine cycle)
 //opcode = 0b11111110

 else if (opcode == 0b11111110) {
    uint8_t n = fetch_opcode();
    alu_compare_reg(A,n);
    return 2;
 }

 //increment : inc r -> increment register and store back (1 byte cycle 1 machine cycle)
 //opcode : 0b00xxx100

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00000111) == 0b00000100) {
    uint8_t r= (opcode >> 3) & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_inc_reg(B);
     else if (r == 0b001) alu_inc_reg(C);
     else if (r == 0b010) alu_inc_reg(D);
     else if (r == 0b011) alu_inc_reg(E);
     else if (r == 0b100) alu_inc_reg(H);
     else if (r == 0b101) alu_inc_reg(L);
     else if (r == 0b110) alu_inc_memory(H,L);
     else if (r == 0b111) alu_inc_reg(A);

      //return 
       if (r == 0b110) return 3;  //inc (hl) takes 3 machine cycle
       else return 1;
 }

  //decrement : dec r -> decrement register and store back (1 byte cycle 1 machine cycle)
 //opcode : 0b00xxx101

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00000111) == 0b00000101) {
    uint8_t r= (opcode >> 3) & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_dec_reg(B);
     else if (r == 0b001) alu_dec_reg(C);
     else if (r == 0b010) alu_dec_reg(D);
     else if (r == 0b011) alu_dec_reg(E);
     else if (r == 0b100) alu_dec_reg(H);
     else if (r == 0b101) alu_dec_reg(L);
     else if (r == 0b110) alu_dec_memory(H,L);
     else if (r == 0b111) alu_dec_reg(A);

      //return 
       if (r == 0b110) return 3;  //inc (hl) takes 3 machine cycle
       else return 1;
 }

 // logical operations :

 //and r : perform bitwise and on a with r and store in a (1 byte cycle,1 machine cycle)
 //opcode : 0b10100xxx

 else if ((opcode & 0b11111000) == 0b10100000 ) {
    uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_and(A,B);
     else if (r == 0b001) alu_and(A,C);
     else if (r == 0b010) alu_and(A,D);
     else if (r == 0b011) alu_and(A,E);
     else if (r == 0b100) alu_and(A,H);
     else if (r == 0b101) alu_and(A,L);
     else if (r == 0b110) alu_and_memory_adress(A,H,L);
     else if (r == 0b111) alu_and(A,A);

      //return 
       if (r == 0b110) return 2; 
       else return 1;
 }

 //and n : and a immediate value n to a (2 byte cycle,2 machine cycle)
 //opcode = 0b11100110

 else if (opcode == 0b11100110) {
    uint8_t n = fetch_opcode();
    alu_and(A,n);
    return 2;
 }

 //or r : perform bitwise or on a with r and store in a (1 byte cycle,1 machine cycle)
 //opcode : 0b10110xxx

 else if ((opcode & 0b11111000) == 0b10110000 ) {
    uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_or(A,B);
     else if (r == 0b001) alu_or(A,C);
     else if (r == 0b010) alu_or(A,D);
     else if (r == 0b011) alu_or(A,E);
     else if (r == 0b100) alu_or(A,H);
     else if (r == 0b101) alu_or(A,L);
     else if (r == 0b110) alu_or_memory_adress(A,H,L);
     else if (r == 0b111) alu_or(A,A);

      //return 
       if (r == 0b110) return 2; 
       else return 1;
 }

 //or n : or a immediate value n to a (2 byte cycle,2 machine cycle)
 //opcode = 0b11110110

 else if (opcode == 0b11110110) {
    uint8_t n = fetch_opcode();
    alu_or(A,n);
    return 2;
 }

 //xor r : perform bitwise xor on a with r and store in a (1 byte cycle,1 machine cycle)
 //opcode : 0b10101xxx

 else if ((opcode & 0b11111000) == 0b10101000 ) {
    uint8_t r= opcode & 0b00000111; //only keep last 3 bbits

     if (r == 0b000) alu_xor(A,B);
     else if (r == 0b001) alu_xor(A,C);
     else if (r == 0b010) alu_xor(A,D);
     else if (r == 0b011) alu_xor(A,E);
     else if (r == 0b100) alu_xor(A,H);
     else if (r == 0b101) alu_xor(A,L);
     else if (r == 0b110) alu_xor_memory_adress(A,H,L);
     else if (r == 0b111) alu_xor(A,A);

      //return 
       if (r == 0b110) return 2; 
       else return 1;
 }

 //xor n : xor a immediate value n to a (2 byte cycle,2 machine cycle)
 //opcode = 0b11101110

 else if (opcode == 0b11101110) {
    uint8_t n = fetch_opcode();
    alu_xor(A,n);
    return 2;
 }

 //special instructions :

 //ccf : flips the carry flag and clears h and n flag (1 byte cycle,1 machine cycle)
 //opcode : 0b00111111

 else if (opcode == 0b00111111) {
    alu_complement_carry_flag();
    return 1;
 }

 //scf : sets the carry flag and clears n and h flag (1 byte cycle 1 machine cycle)
 //opcode : 0b00110111

 else if (opcode == 0b00110111) {
    alu_set_carry_flag();
    return 1;
 }

 //daa : decimal adjust accumulator,used to fix a register for bcd arithmetic(1 byte cycle,1 machine cycle)
 //opcode : 0b00100111

 else if (opcode == 0b00100111) {
    alu_daa(A); // we will only apply daa for a register as it has a fixed opcode and it only applies on
    //a register where the result of a operation is stored mostly
    return 1;
 }

 //cpl : flips all bits in 8 bit register and sets n and h flags (1 byte cycle,1 machine cycle)
 //opcode : 0b00101111

 else if (opcode == 0b00101111) {
    alu_complement_reg(A);
    return 1;
 }

 //16 bit arithmetic operations : 

 // inc rr (1 byte cycle 2 machine cycle)
 //opcode = 0b00xx0011

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00000011) {
    uint8_t rr = (opcode >> 4) & 0b00000011;

     if (rr == 0b00) alu_inc_paired_reg(B,C);
     else if (rr == 0b01) alu_inc_paired_reg(D,E);
     else if (rr == 0b10) alu_inc_paired_reg(H,L);
     else if (rr == 0b11) SP++;

     return 2;
 }

 // dec rr (1 byte cycle 2 machine cycle)
 //opcode = 0b00xx1011

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00001011) {
    uint8_t rr = (opcode >> 4) & 0b00000011;

     if (rr == 0b00) alu_dec_paired_reg(B,C);
     else if (rr == 0b01) alu_dec_paired_reg(D,E);
     else if (rr == 0b10) alu_dec_paired_reg(H,L);
     else if (rr == 0b11) SP--;

     return 2;
 }

 // add hl,rr : add rr to hl and store result back into hl (1 byte cycle,2 machine cycle)
 //opcode : 0b00xx1001

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00001001) {
    uint8_t rr = (opcode >> 4) & 0b00000011;

     if (rr == 0b00) alu_add_paired_reg(H,L,B,C);
     else if (rr == 0b01) alu_add_paired_reg(H,L,D,E);
     else if (rr == 0b10) alu_add_paired_reg(H,L,H,L);
     else if (rr == 0b11) alu_add_paired_reg(H,L,SP);

     return 2;
 }

 // add sp,e : add sp to signed int e from next opcode (2 byte cycle,4 machine cycle)
 //opcode : 0b11101000

 else if (opcode == 0b11101000) {
    int8_t e = fetch_opcode();

    set_flag ('Z',0);
    set_flag ('N',0);
    if ((SP & 0x0F) + (e & 0x0F) > 0x0F) set_flag('H',1);
    else set_flag('H',0);
    if ((SP & 0xFF) + (e & 0xFF) > 0xFF) set_flag('C',1);
    else set_flag('C',0);

    SP += e;

    return 4;
 }

 //rotate,shift and swap operations : 

 // RLCA : rotate left circular (accumulator),same as rlc r but always set z flag to 0
 //opcode : 0b00000111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00000111) {
    alu_rotate_left_circular(A);
    set_flag('Z',0);
    return 1;
 }

  // RRCA : rotate left circular (accumulator),same as rRc r but always set z flag to 0
 //opcode : 0b00001111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00001111) {
    alu_rotate_right_circular(A);
    set_flag('Z',0);
    return 1;
 }

 // RLA : rotate left  (accumulator),same as rL r but always set z flag to 0
 //opcode : 0b00010111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00010111) {
    alu_rotate_left_carry(A);
    set_flag('Z',0);
    return 1;
 }

 // RRA : rotate right  (accumulator),same as rr r but always set z flag to 0
 //opcode : 0b00011111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00011111) {
    alu_rotate_right_carry(A);
    set_flag('Z',0);
    return 1;
 }


 //cb opcodes : if opcodes starts with 0xcb the next opcode actually have instruction

 else if (opcode == 0xCB) {

  uint8_t cb_opcode = fetch_opcode();

    //RLC r : rotates 8 bit register r value to left in circular manner (2 byte cycle,2 mqchine cycle) 
   //opcode : 0xCB >> 0b00000xxx 
  //this is a cb opcode means the current opcode will contaon 0XCB and the next opcode will have instruction

     if ((cb_opcode & 0b11111000) == 0b00000000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_rotate_left_circular(B);
     else if (r == 0b001) alu_rotate_left_circular(C);
     else if (r == 0b010) alu_rotate_left_circular(D);
     else if (r == 0b011) alu_rotate_left_circular(E);
     else if (r == 0b100) alu_rotate_left_circular(H);
     else if (r == 0b101) alu_rotate_left_circular(L);
     else if (r == 0b110) alu_rotate_left_circular_memory_adress(H,L);
     else if (r == 0b111) alu_rotate_left_circular(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

    //RRC r : rotates 8 bit register r value to right in circular manner (2 byte cycle,2 mqchine cycle) 
   //opcode : 0xCB >> 0b00001xxx 
  //this is a cb opcode means the current opcode will contain 0XCB and the next opcode will have instruction

     else if ((cb_opcode & 0b11111000) == 0b00001000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_rotate_right_circular(B);
     else if (r == 0b001) alu_rotate_right_circular(C);
     else if (r == 0b010) alu_rotate_right_circular(D);
     else if (r == 0b011) alu_rotate_right_circular(E);
     else if (r == 0b100) alu_rotate_right_circular(H);
     else if (r == 0b101) alu_rotate_right_circular(L);
     else if (r == 0b110) alu_rotate_right_circular_memory_adress(H,L);
     else if (r == 0b111) alu_rotate_right_circular(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

    //RL r : rotates 8 bit register r value to left with carry flag (2 byte cycle,2 mqchine cycle) 
   //opcode : 0xCB >> 0b00010xxx 
  //this is a cb opcode means the current opcode will contain 0XCB and the next opcode will have instruction

     else if ((cb_opcode & 0b11111000) == 0b00010000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_rotate_left_carry(B);
     else if (r == 0b001) alu_rotate_left_carry(C);
     else if (r == 0b010) alu_rotate_left_carry(D);
     else if (r == 0b011) alu_rotate_left_carry(E);
     else if (r == 0b100) alu_rotate_left_carry(H);
     else if (r == 0b101) alu_rotate_left_carry(L);
     else if (r == 0b110) alu_rotate_left_carry_memory_adress(H,L);
     else if (r == 0b111) alu_rotate_left_carry(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

    //RR r : rotates 8 bit register r value to right with carry flag (2 byte cycle,2 mqchine cycle) 
   //opcode : 0xCB >> 0b00011xxx 
  //this is a cb opcode means the current opcode will contain 0XCB and the next opcode will have instruction

     else if ((cb_opcode & 0b11111000) == 0b00011000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_rotate_right_carry(B);
     else if (r == 0b001) alu_rotate_right_carry(C);
     else if (r == 0b010) alu_rotate_right_carry(D);
     else if (r == 0b011) alu_rotate_right_carry(E);
     else if (r == 0b100) alu_rotate_right_carry(H);
     else if (r == 0b101) alu_rotate_right_carry(L);
     else if (r == 0b110) alu_rotate_right_carry_memory_adress(H,L);
     else if (r == 0b111) alu_rotate_right_carry(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

    //sla r : shifts the 8 bit register by 1 left (2 byte cycle,2 machine cycle)
   //opcode : 0xCB >> 0b00100xxx 
  //this is a cb opcode means the current opcode will contain 0XCB and the next opcode will have instruction

   else if ((cb_opcode & 0b11111000) == 0b00100000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_shift_left(B);
     else if (r == 0b001) alu_shift_left(C);
     else if (r == 0b010) alu_shift_left(D);
     else if (r == 0b011) alu_shift_left(E);
     else if (r == 0b100) alu_shift_left(H);
     else if (r == 0b101) alu_shift_left(L);
     else if (r == 0b110) alu_shift_left_memory_adress(H,L);
     else if (r == 0b111) alu_shift_left(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

    //sra r : shifts the 8 bit register by 1 right (2 byte cycle,2 machine cycle)
   //opcode : 0xCB >> 0b00101xxx 
  //this is a cb opcode means the current opcode will contain 0XCB and the next opcode will have instruction

   else if ((cb_opcode & 0b11111000) == 0b00101000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_shift_right(B);
     else if (r == 0b001) alu_shift_right(C);
     else if (r == 0b010) alu_shift_right(D);
     else if (r == 0b011) alu_shift_right(E);
     else if (r == 0b100) alu_shift_right(H);
     else if (r == 0b101) alu_shift_right(L);
     else if (r == 0b110) alu_shift_right_memory_adress(H,L);
     else if (r == 0b111) alu_shift_right(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

    //sla r : shifts the 8 bit register by 1 right logical (2 byte cycle,2 machine cycle)
   //opcode : 0xCB >> 0b00111xxx 
  //this is a cb opcode means the current opcode will contain 0XCB and the next opcode will have instruction

   else if ((cb_opcode & 0b11111000) == 0b00111000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_shift_right_logical(B);
     else if (r == 0b001) alu_shift_right_logical(C);
     else if (r == 0b010) alu_shift_right_logical(D);
     else if (r == 0b011) alu_shift_right_logical(E);
     else if (r == 0b100) alu_shift_right_logical(H);
     else if (r == 0b101) alu_shift_right_logical(L);
     else if (r == 0b110) alu_shift_right_logical_memory_adress(H,L);
     else if (r == 0b111) alu_shift_right_logical(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

    // swap r : swap the upper and lower nibble in register (2 byte cycle,2 machine cycle)
   //opcode : 0xCB >> 0b00110xxx 

   else if ((cb_opcode & 0b11111000) == 0b00110000) {
    uint8_t r= (cb_opcode & 0b00000111); //only keep last 3 bbits

     if (r == 0b000) alu_swap(B);
     else if (r == 0b001) alu_swap(C);
     else if (r == 0b010) alu_swap(D);
     else if (r == 0b011) alu_swap(E);
     else if (r == 0b100) alu_swap(H);
     else if (r == 0b101) alu_swap(L);
     else if (r == 0b110) alu_swap_memory(H,L);
     else if (r == 0b111) alu_swap(A);

      //return 
       if (r == 0b110) return 4; 
       else return 2;
    }

  return 0;
}

 return 0;
}//function end

};

void todo_list () {
//halt should execute before ld even starts
}

int main () {

}