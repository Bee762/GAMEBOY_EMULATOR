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

void compare_reg (uint8_t& register1, uint8_t& register2) {
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

void compare_reg_memory_adress (uint8_t& register1, uint8_t& register2, uint8_t& register3) {

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
    set_flag ('C',~carry_flag);//just complements carry flag
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

//ld opcodes :

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



}

};

void todo_list () {
//halt should execute before ld even starts
}
