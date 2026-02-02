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
//our i just want want specific value 0 or 1 ,my return gets the correct binary but returns 
//it as it is so it can be translated to 0 or the actual value of binary(128 for zero flag).we only want 0 and 1 hence return type
//is bool,anything > 0  is 1.
}

//now we need to emulate arithmetic logic unit aka alu.
//alu is a part of cpu that performs simple math operations and logical operations
//and alu stores it result to either A register or to memory (ram)

//Arithmetic Logic Unit : 

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

    uint16_t value = mem.read_memory(paired_register);

    value++;

    mem.write_memory(paired_register,value);

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

    uint16_t value = mem.read_memory(paired_register);

    value--;

    mem.write_memory(paired_register,value);

    //no flags will be touched here

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

};