#include <windows.h>
#include <string>
#include <iostream>
#include <cstdint>
#include <chrono>

#include <SFML/Graphics.hpp>

void render_frame(uint8_t framebuffer[144][160]);

struct memory {
memory () {
 mapping_status = map_rom_cartridge();
 if (!mapping_status) std::cerr << "rom mapping failed" << std::endl;

 //initialising variables : 

   div_counter = 0xAB00;
   DIV = 0XAB;
   TIMA = 0X00;
   TMA = 0X00;
   TAC = 0X00;

   IE = 0b00011111;
   IF = 0b00000000;

   LCDC = 0X91;
   STAT = 0X85;
   SCY = 0;
   SCX = 0;
   LY = 0;
   LYC = 0;
   BGP = 0XFC;
   OBP0 = 0XFF;
   OBP1 = 0XFF;
   WX = 0;
   WY = 0;

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

game_file = CreateFileA("cpu_instrs.gb",GENERIC_READ,FILE_SHARE_READ,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_READONLY,NULL);
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

     //v_ram
    else if (adress >= 0x8000 && adress < 0xA000) {
        return ram [adress - 0x8000];
    }
    // oam (sprite attribute table)
    else if (adress >= 0xFE00 && adress < 0xFEA0) {
        return ram [adress - 0x8000];
    }

    else if (adress == 0xFF0F) {
        return IF | 0XE0;
    }

    else  return ram [adress - 0x8000]; // ram indexing in my memory starts from 0 so convert.
    //if memory adress location is in rom return rom adress else return ram adress
    //rom memory adress in gameboy : 0x0000 to 0x7FFF
}

void write_memory(uint16_t address, uint8_t value) {

     //R0M write

    if ( address < 0x8000 ) {
        return; //rom address,should not write
    }
    //v_ram
    else if (address >= 0x8000 && address < 0xA000) {
         ram[address - 0x8000] = value;
    }
    //external_ram (cartridge ram)
    else if (address >= 0xA000 && address < 0xC000) {
        ram[address - 0x8000] = value;
    }
    //work ram bank 0
    else if (address >= 0xC000 && address < 0xD000) {
        ram[address - 0x8000] = value;
        ram[address - 0x8000 + 0x2000] = value;
    }
    //workram bank 1
    else if (address >= 0xD000 && address < 0xE000) {
        ram[address - 0x8000] = value;
        if ((address + 0x2000) <= 0xFDFF ) ram [address - 0x8000 + 0x2000] = value;
    }
    //echoram
    else if (address >= 0xE000 && address < 0xFE00) {
        ram[address - 0x8000] = value;
        ram[address - 0x8000 - 0x2000] = value;
    }
    // oam (sprite attribute table)
    else if (address >= 0xFE00 && address < 0xFEA0) {
        ram[address - 0x8000] = value;
    }
    //not useable region 
    else if (address >= 0xFEA0 && address < 0xFF00) {
        //do nothing
    }
    //I/O registers (FF00 - FF7F)
    else if (address >= 0xFF00 && address < 0xFF80) {
        //serial ports 
       /*
        //SB : holds the output that will be printed to user
      if (address == 0xFF01) {
        ram [address - 0x8000] = value;
       }
       //sc : decides when data in sb will be printed out
       else if (address == 0xFF02) {
        ram[address - 0x8000] = value;
        if ((value & 0b10000001) == 0b10000001) {
            char output = static_cast <char> (ram[0xFF01 - 0x8000]);
            std::cout << output << std::flush;
            ram[address - 0x8000] &= 0b01111111;//clear the 7th bit;
            //ram[0xFF0F - 0x8000] |= (1<<3);//request for serial  interrupt
        }
       }
        */
        
       // Serial output for Blargg test
    if (address == 0xFF01) {
        char output = static_cast<char>(value);
        std::cout << output;
        std::cout.flush(); // make sure it prints immediately
    }
        
      //div
      else if (address == 0xFF04) {

        uint16_t prev_div_counter  = div_counter;

        DIV = 0; //cpu writting to div resets it,this can also cause tima falling edge
        div_counter = 0;

        bool timer_enable = timer_enabled();

        if (timer_enable) {

        int bit = get_timer_bit();
        bool old_bit = (prev_div_counter >> bit) & 1;

        if (old_bit == 1 ) {
            TIMA++;
            if (TIMA == 0X00) {
                tima_overflow = true;
                tima_reload_delay = 4;
            }
         }
      }

    }
      //tima
      else if (address == 0xFF05) {
        TIMA = value; //cpu writes to tima during overflow stops tima reload
        tima_overflow = false;
      }
      //tma
      else if (address == 0xFF06) {
       TMA = value;
      }
      //tac
      else if (address == 0xFF07) {

        bool old_enable = timer_enabled();
        bool old_bit = get_timer_bit();
        bool old_state = (div_counter >> old_bit) & 1;

        TAC = (value & 0b00000111); //tac frequency change can also cause tima increment

        bool new_enable = timer_enabled();
        bool new_bit = get_timer_bit();
        bool new_state = (div_counter >> new_bit) & 1;

        if (old_enable && new_enable) {
 
         if (old_state == 1 && new_state == 0) { //here a edge case might be plausible,look later

            TIMA++; //if tac frequency changes tima can increment depending on selected bit
            if (TIMA == 0x00) {
                tima_overflow = true;
                tima_reload_delay = 4;
            }
         }

        }

      }

      else if (address == 0xFF41) {
        //stat register
        //ppu controls last 3 bits ,cpu cannot over right them
        uint8_t old_stat = ram [address - 0x8000];
        uint8_t stat_value = ((old_stat & 0b00000111) | (value & 0b11111000));
        ram [address - 0x8000] = (stat_value | 0b10000000);
        //in hardware bit 7 of state register is unused and always 1;
      }
      else if (address == 0xFF44) {
        //ly register
        //writes willbe ignored
        return;
      }

      else {
        ram [address - 0x8000] = value;
      }

    }
    //HIGH_ram
    else if (address >= 0xFF80 && address < 0xFFFF) {
        ram [address - 0x8000] = value;
    }
    //IE register
    else if (address == 0xFFFF) {
        ram[address - 0x8000] = value;
    }

}

int get_timer_bit () {

    switch (TAC & 0b11) {
        case 0 : return 9;
        case 1 : return 3;
        case 2 : return 5;
        case 3 : return 7;
        }

    return 9;
}

bool timer_enabled () {
    return (TAC & 0b100);
}


HANDLE game_file = nullptr; //storing handle outside the functon so they can be closed later.
HANDLE game_rom = nullptr;
uint8_t* rom_adress = nullptr; // the starting adress of mapped rom.

uint8_t ram [32768];   //gameboy memory.

//timer registers : 

uint8_t& DIV  =  ram [0xFF04 - 0x8000];
uint8_t& TIMA = ram [0xFF05 - 0x8000];
uint8_t& TMA  =  ram [0xFF06 - 0x8000];
uint8_t& TAC  =  ram [0xFF07 - 0x8000];

uint16_t div_counter = 0x0000;
uint8_t tima_reload_delay = 0;
bool tima_overflow = false;

//io registers :

 uint8_t& LCDC = ram[0XFF40 - 0X8000];
 uint8_t& STAT = ram[0XFF41 - 0x8000];
 uint8_t& SCY = ram[0XFF42 - 0X8000];
 uint8_t& SCX = ram[0xFF43 - 0X8000];
 uint8_t& LY = ram[0XFF44 - 0X8000];
 uint8_t& LYC = ram[0XFF45 - 0X8000];
 uint8_t& DMA = ram[0XFF46 - 0X8000];
 uint8_t& BGP = ram[0XFF47 - 0X8000];
 uint8_t& OBP0 = ram[0XFF48 - 0X8000];
 uint8_t& OBP1 = ram[0XFF49 - 0X8000];
 uint8_t& WY = ram[0XFF4A - 0X8000];
 uint8_t& WX = ram[0XFF4B - 0X8000];

//interupt related variables :
uint8_t& IE = ram[0xFFFF - 0x8000];    //interrupt enable ,tells cpu which interupts are allowed to trigger,it lives at the 0XFFFF
//last adress in ram,bit 0-5 of IE are the 5 interupts that can be allowed in gameboy cpu,bit 6-8 are always zero
uint8_t& IF = ram [0XFF0F - 0X8000];    //interrupt flag : tells cpu which interupts are currently being requested,
//bit 0-5 of IF are the 5 interupts that can be requested in gameboy cpu,bit 6-8 are always zero

bool mapping_status = false;
bool ppu_pixel_transfer = false; //during ppu pixel transfer writes to oam and vram are disabled and read returns 0xff
bool oam_search = false;

};

struct cpu {
//gameboy uses Sharp LR35902 cpu,it has 8 bit architecture

  memory* mem;//pointer to a memory obj so i can use memory from cpu

cpu (memory* memory) {   // taking a memory pointer in constructer parameter

    mem = memory; //and now mem is pointing to memory ,this is done to sync all other components later like ppu and stuff
    //to use same copy of memory and run in sync

 //constructor initialises the variables after bootram fr pokemon

   PC = 0X0100;
   SP = 0XFFFE;

   A = 0X01;
   F = 0XB0;
   B = 0X00;
   C = 0X13;
   D = 0X00;
   E = 0XD8;
   H = 0X01;
   L = 0X4D;

   IME = false;
   IME_scheduled = false;
   ime_pending = false;
   halted = false;
   interupt_pending = false;
   stopped = false;

}

//cpu registers :
uint8_t IR;    //instruction register,stores the opcode from rom
uint8_t A;     //accumulator : stores arithmetic operations from alu
uint8_t F;    //flags 
uint8_t B;
uint8_t C;
uint8_t D;//these 6 are general registers {b,c,d,e,h,l}
uint8_t E;
uint8_t H;
uint8_t L;
uint16_t PC; //program counter ,holds current adress of memory,skipping past bootrom
uint16_t SP; //stack pointer : holds  adress of ram

bool IME ; //master interupt enable flag,interupts can only happen when this is set to true
bool IME_scheduled;
bool ime_pending; // for delayed enable interupts
bool halted ; //set to true when cpu enters halted state aka cpu does nothing untill a interupt is fired
bool interupt_pending ;
bool stopped;
bool haltbug = false;
bool interrupt_serviced = false;

int ei_delay = 0;

//cycle accuracy related variables and flags

uint8_t opcode = 0x00;
uint8_t cb_opcode = 0x00;
uint8_t n = 0;
uint8_t n1 = 0;
uint8_t n2 = 0;
int8_t e = 0;
uint8_t highbyte = 0;
uint8_t lowbyte = 0;
uint8_t inc_dec_value = 0x00;// only used for inc hl and dec hl
uint8_t inc_dec_old_value = 0x00; //  only used for inc hl and dec hl
bool take_conditional_jump = false;

uint8_t cb_value = 0x00;
uint16_t cb_adress = 0x0000; //these value and address will be used in alu rrca and rlca for cb opcode alu operations

int current_machine_cycle = 0;

bool fetch_new_instruction = true;


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

void request_interrupt (char interrupt) {
    
 if (interrupt == 'V') {  // v blank interupt
 uint8_t v_blank = 0x00 | (1 << 0); //passing data into desired bit,adding a mask before to make sure no garbage value exists
 mem->IF |= v_blank;     //storing new data
 }

 else if (interrupt == 'L') {  // LCD stat interupt
 uint8_t lcd_strat = 0x00 | (1 << 1); //passing data into desired bit,adding a mask before to make sure no garbage value exists
 mem->IF |= lcd_strat;     //storing new data
 }

 else if (interrupt == 'T') {  // TIMER interupt
 uint8_t timer = 0x00 | (1 << 2); //passing data into desired bit,adding a mask before to make sure no garbage value exists
 mem->IF |= timer;     //storing new data
 }

 else if (interrupt == 'S') {  // serial interupt
 uint8_t serial = 0x00 | (1 << 3); //passing data into desired bit,adding a mask before to make sure no garbage value exists
 mem->IF |= serial;     //storing new data
 }

 else if (interrupt == 'J') {  // joypad interupt
 uint8_t joypad = 0x00 | (1 << 4); //passing data into desired bit,adding a mask before to make sure no garbage value exists
 mem->IF |= joypad;     //storing new data;
 }
 //we dont care about bit 5 - 8,ignore them
 mem->IF &= 0b00011111; //forcing upper 3 bits to zero
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

    uint8_t value = mem->read_memory(paired_register);

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

    uint8_t value = mem->read_memory(paired_register);

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

    uint8_t value = mem->read_memory(paired_register);

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

    uint8_t value = mem->read_memory(paired_register);

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

    if (current_machine_cycle == 1) {

     inc_dec_value = mem->read_memory(paired_register);

     inc_dec_old_value = inc_dec_value;//preserving old value for half carry check

    }

    else if (current_machine_cycle == 2) {

    inc_dec_value++;

    mem->write_memory(paired_register,inc_dec_value);

    if (inc_dec_value == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 0);

    if ((inc_dec_old_value & 0xF) == 0xF ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);


    //no flags will be touched here

    }

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

    if (current_machine_cycle == 1) {

     inc_dec_value = mem->read_memory(paired_register);

     inc_dec_old_value = inc_dec_value;//preserving old value for half carry check

    }

    else if (current_machine_cycle == 2) {

    inc_dec_value--;

    mem->write_memory(paired_register,inc_dec_value);

    if (inc_dec_value == 0) set_flag ('Z', 1);
    else set_flag ('Z', 0);

    set_flag ('N', 1);

    if ((inc_dec_old_value & 0xF) == 0x0 ) set_flag ('H', 1); //checking if half overflow has happened
    else set_flag ('H', 0);

    }

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

    uint8_t value = mem->read_memory(paired_register);
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
    uint8_t value = mem->read_memory(adress);

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
    uint8_t value = mem->read_memory(adress);

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
    uint8_t value = mem->read_memory(adress);

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

    if (current_machine_cycle == 2) {
     cb_adress = pair_registers(register1,register2);
     cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {
    bool fallen_bit = (cb_value >> 7);

    cb_value = (cb_value << 1) | fallen_bit;

    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
    }

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

    if (current_machine_cycle == 2) {
     cb_adress = pair_registers(register1,register2);
     cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {

    bool fallen_bit = (cb_value & 0x01);

    cb_value = (cb_value >> 1) | (fallen_bit << 7);

    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);

    }
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

    if (current_machine_cycle == 2) {
      cb_adress = pair_registers(register1,register2);
      cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {

    bool old_carry = get_flag('C');
    bool fallen_bit = (cb_value >> 7);

    cb_value = (cb_value << 1) | old_carry;

    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);

    }

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

    if (current_machine_cycle == 2) {
     cb_adress = pair_registers(register1,register2);
     cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {

    bool old_carry = get_flag('C');
    bool fallen_bit = (cb_value & 0x01);

    cb_value = (cb_value >> 1) | (old_carry << 7);

    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
    }

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

    if (current_machine_cycle == 2) {
     cb_adress = pair_registers(register1,register2);
     cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {
    bool fallen_bit = (cb_value >> 7);
    cb_value <<= 1;
    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
    }
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

    if (current_machine_cycle == 2) {
    cb_adress = pair_registers(register1,register2);
    cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {

    bool fallen_bit = (cb_value & 0x01);
    bool preserved_bit = (cb_value >> 7);
    cb_value = (cb_value >> 1) | (static_cast<uint8_t>(preserved_bit) << 7);

    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
    }
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

    if (current_machine_cycle == 2) {
    cb_adress = pair_registers(register1,register2);
    cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {
    bool fallen_bit = (cb_value & 0x01);
    cb_value >>= 1;
    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',fallen_bit);
    }
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

    if (current_machine_cycle == 2) {
    cb_adress = pair_registers(register1,register2);
    cb_value = mem->read_memory(cb_adress);
    }

    else if (current_machine_cycle == 3) {
    cb_value = (cb_value << 4) | (cb_value >> 4);

    mem->write_memory(cb_adress,cb_value);

    if (cb_value == 0x00) set_flag ('Z',1);
    else set_flag ('Z',0);

    set_flag ('N',0);

    set_flag ('H',0); 

    set_flag ('C',0);
    }
}

void alu_test_bit (uint8_t ref_reg,uint8_t bit_mask) { //bitmask will be used as 0b00100000 for 5th bit testing
    bool result = (ref_reg & bit_mask); //storing final data as 0 or 1 as thats whhat we need later
    if (result == 0) set_flag('Z',1);
    else set_flag('Z',0);

    set_flag('N',0);
    set_flag ('H',1);

}

void alu_reset_bit (uint8_t& ref_reg,uint8_t bit_mask) {
    //resets bit b to zero
    ref_reg &= (~bit_mask); // we will use bitmask as 0b01000000 to locate bit 6 so flip it and mask
    //no flags will be  touched
}

void alu_set_bit (uint8_t& ref_reg,uint8_t bit_mask) {
    //sets bit b to 1
    ref_reg |= bit_mask; // we will use bitmask as 0b01000000 to locate bit 6 so flip it and mask
    //no flags will be  touched
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
    IR = mem->read_memory(PC); //store opcode in instruction register
    if (haltbug) {
    haltbug = false; //introducing hALTBUG
    }
    else PC++;
    return IR; //return it
}

int8_t fetch_rawbyte () { //for getting signed int when needed
    IR = mem->read_memory(PC); //store opcode in instruction register
    PC++;
    return IR; //return it
}


uint32_t execute_opcode () {

 if (fetch_new_instruction) {

     opcode = fetch_opcode();
     fetch_new_instruction = false;
     current_machine_cycle = 0; //restarting current machine cycle count

 }

 if (opcode == 0b01110110) {
    //halt :  0b01110110 is cpu a instruction called halt 
    if (!IME && (mem->IF & mem->IE & 0x1F)) {
     haltbug = true;
    } else {
     halted = true;
    }
    fetch_new_instruction = true;
    return 1;//it returns 1 machine cycle
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
      else if (last_register == 0b111) value = A;
      else if (last_register == 0b110) {
        if (current_machine_cycle == 1) value = mem->read_memory(pair_registers(H,L)); //value at memory adress pointed by (HL)
        else { 
        current_machine_cycle++;
        return 1;//memory accessing takes 1 machine cycle,returning that so timer can catch up
        }
      }

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
     else if (first_register == 0b110) {
        if (current_machine_cycle == 1) mem->write_memory(pair_registers(H,L),value);
        else {
        current_machine_cycle++;
        return 1; //memory accessing takes 1 machine cycle
        }

        //general return statememnt 

        fetch_new_instruction = true;
        return 1;

     }
      //0b01110110 is already skipped as halt
      //also it translates to ld (hl) (hl) means write value stored by adress hl to adress hl,it does nothing anyways
      //thats why cpu uses this specific binary as halt
     else if (first_register == 0b111) A = value;

     //now we need to write machine cycles taken to execute this opcode as return statement

         fetch_new_instruction = true;
         return 1; // ld r r' takes 1 machine cycle


     //this takes care of all ld r r' instructions
 }


 //ld r , n : load register r with immediate value n from next opcode,so it will take 2 byte cycle of fetch decode execute
 //byte cycle means how many times i need to fetch opcode it doesnot means machine cycle
 //opcode range 0b00xxx110 (x = variable),

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00000111) == 0b00000110) {
    uint8_t reg = (opcode >> 3) & 0b00000111;//storing data of register 

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }
    
     else if (current_machine_cycle == 1) n = fetch_opcode();//getting value of n,condition is set so opcode fetchh
      //only happens once,coz i want to prevent program counter incrementing

     if (reg == 0b000) B = n;
     else if (reg == 0b001) C = n;
     else if (reg == 0b010) D = n;
     else if (reg == 0b011) E = n;
     else if (reg == 0b100) H = n;
     else if (reg == 0b101) L = n;
     else if (reg == 0b110) {
        if (current_machine_cycle == 2) mem->write_memory(pair_registers(H,L),n);
        else {
            current_machine_cycle++; //an extra condition is set in more than 1 opcode fetches to prevent 
            // program counter false updates
            return 1;
        }
      }

     else if (reg == 0b111) A = n;


     //return statements
     fetch_new_instruction = true;
     return 1;
     //ld (hl) n returns 3 machine cycle
     //ld r n takes 2 machine cycle to execute

 }


 //ld a (bc) : load a from memory adress pointed by bc(1 byte cycle 2 machine cycle)
 //opcode : 0b00001010

 else if (opcode == 0b00001010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1)  {
        A = mem->read_memory(pair_registers(B,C));
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ld a (de) : load a from memory adress pointed by de(1 byte cycle 2 machine cycle)
 //opcode : 0b00011010

 else if (opcode == 0b00011010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

   else if (current_machine_cycle == 1) {
        A = mem->read_memory(pair_registers(D,E));
        fetch_new_instruction = true;
        return 1;
   }

 }

 //ld (bc) a : load memory adress pointed by bc with value at a (1 byte cycle 2 machine cycle)
 //opcode : 0b00000010

 else if (opcode == 0b00000010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        mem->write_memory(pair_registers(B,C),A);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ld (de) a : load memory adress pointed by de with value at a (1 byte cycle 2 machine cycle)
 //opcode : 0b00010010

 else if (opcode == 0b00010010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        mem->write_memory(pair_registers(D,E),A);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ld a (nn) : load register a with value from adress specified by nn(3 byte cycle 4 machine cycle)
 //first n is actually low byte and second n is highbyte,accordin to cpu instruction
 //opcode : 0b11111010
 
 else if (opcode == 0b11111010) {

        if (current_machine_cycle == 0) {  //first m cycle
          current_machine_cycle++;
          return 1;
        }

        else if (current_machine_cycle == 1) {  //second machine cycle
          n1 = fetch_opcode();//low byte (first n)
          current_machine_cycle++;
          return 1;
        }

        else if (current_machine_cycle == 2) {   //third machine cycle
          n2 = fetch_opcode();//high byte (second n)
          current_machine_cycle++;
          return 1;
        }

        else if (current_machine_cycle == 3) {   //fourth machine cycle
          //n1 and n2 are immediate value (nn) = (n2 n1)
          A = mem->read_memory(pair_registers(n2,n1)); //n1 and n2 are not registers but they will use same concept
          //but in pair register (a,b) a is used as high byte as we will use n2 as highbyte so n2 first
          fetch_new_instruction = true;
          return 1;
        }
    

 }

 //ld (nn) a : load adress (nn) with data in a (3 byte cycle and 4 machine cycle)
 //opcode : 0b11101010

 else if (opcode == 0b11101010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n1 = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }
    
    else if (current_machine_cycle == 2) {
        n2 = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 3) {
        mem->write_memory(pair_registers(n2,n1),A);
        fetch_new_instruction = true;
        return 1;
    }
    
 }

 //ldh a (c) : load A with high memory adress specified by c,
 //high memory means the high stream it ranges from 0xFF00 to 0xFFFF,it is used for I/O management in gameboy
 //opcode : 0b11110010               (2 machine cycle 1 byte cycle)

 else if (opcode == 0b11110010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        A = mem->read_memory(0xFF00 + C); //just making sure memory stays in range of 0xFF00 - 0xFFFF
        //as c is 8 bit int the result highbyte stays 0xF
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ldh (c) a : load high memory adress c with a (1 byte cycle 2 machine cycle)
 //opcode : 0b11100010

 else if (opcode == 0b11100010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        mem->write_memory(C+0xFF00,A);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ldh a (n) : load a with high memory (n) (2 byte cycle 3 machine cycle)
 //opcode = 0b11110000

 else if (opcode == 0b11110000) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        A = mem->read_memory(0xFF00+n);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ldh (n) a : load high memory adress n with a (2 byte cycle 3 machine cycle)
 //opcode = 0b11100000

 else if (opcode == 0b11100000) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        mem->write_memory(0xFF00+n,A);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ldh a (hl-) : load a with adress (hl),then decrement hl and store(1 byte cycle 2 machine cycle)
 //opcode : 0b00111010

 else if (opcode == 0b00111010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        uint16_t hl = pair_registers(H,L);
        A = mem->read_memory(hl);
        hl--;
        split_registers(H,L,hl);

        fetch_new_instruction = true;
        return 1;
    }

 }

 //ld (hl-) a : load adress hl with a and then decrement hl and store (1 byte cycle 2 machine cycle)
 //opcode : 0b00110010

 else if (opcode == 0b00110010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        uint16_t hl = pair_registers(H,L);
        mem->write_memory(hl,A);
        hl--;
        split_registers(H,L,hl);

        fetch_new_instruction = true;
        return 1;
    }

 }

 //ldh a (hl+) : load a with adress (hl),then increment hl and store(1 byte cycle 2 machine cycle)
 //opcode : 0b00101010

 else if (opcode == 0b00101010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        uint16_t hl = pair_registers(H,L);
        A = mem->read_memory(hl);
        hl++;
        split_registers(H,L,hl);

        fetch_new_instruction = true;
        return 1;
    }

 }

 //ld (hl+) a : load adress hl with a and then increment hl and store (1 byte cycle 2 machine cycle)
 //opcode : 0b00100010

 else if (opcode == 0b00100010) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        uint16_t hl = pair_registers(H,L);
        mem->write_memory(hl,A);
        hl++;
        split_registers(H,L,hl);

        fetch_new_instruction = true;
        return 1;
    }

 }

 //ld opcodes : 16 bit load instructions :

 //ld rr nn : load 16 bit register rr' with 16 bit immediate data nn (3 byte cycle , 3 machine cycle)
 //opcode : 0b00xx0001

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00000001) {

    uint8_t reg = (opcode >> 4) & 0b00000011; //storing data of register r

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        //now lets get nn
        lowbyte = fetch_opcode();//first n aka next opcode is actually lowbyte (cpu design)
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = fetch_opcode();
        uint16_t nn = pair_registers(highbyte,lowbyte); //storing nn 

       if (reg == 0b00) split_registers(B,C,nn);//split nn into b and c register
       else if (reg == 0b01) split_registers(D,E,nn);
       else if (reg == 0b10) split_registers(H,L,nn);
       else if (reg == 0b11) SP = nn;  //storing 16 bit value in stack pointer

        fetch_new_instruction = true;
        return 1;
    }
    
 }

 //ld (nn) sp : load adress (nn) with value from stack pointer (3 byte cycle , 5 machine cycle)
 //opcode : 0b00001000

 else if (opcode == 0b00001000) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = fetch_opcode();//first n aka next opcode is actually lowbyte (cpu design)
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 3) {
        //we will split sp in two 8 bit int because memory cant store 16 bit data
        uint16_t adress = pair_registers(highbyte,lowbyte);
        mem->write_memory(adress,SP & 0x00FF);  //low byte in memory ,
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 4) {
         uint16_t adress = pair_registers(highbyte,lowbyte);
         adress++;
         mem->write_memory(adress,((SP >> 8) & 0x00FF));  //high byte in memory ,

        fetch_new_instruction = true;
        return 1;
    }
   
 }

 //ld sp hl : load stack pointer with data from hl register (1 byte cycle , 2 machine cycle)
 //opcode : 0b11111001

 else if (opcode == 0b11111001) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        SP = pair_registers(H,L);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //push rr : push to stack -> push data from 16 bit registers to memory adress pointed by sp
 //opcode : 0b11xx0101 (1 byte cycle 4 machine cycle) 

 else if ((opcode & 0b11000000) == 0b11000000 && (opcode & 0b00001111) == 0b00000101) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
         SP--; //because we will write at memory ,sp had an adress from before so we move to next empty space
        //we are using decrement because stack starts at higher adress and grows lower (cpu designers choice).
        uint8_t rr = (opcode >> 4) & 0b00000011;

       if (rr == 0b00) { // BC register
         mem->write_memory(SP,B); //memory can store 1 byte at a adress as its 8 bit
       }

       else if (rr == 0b01) { //DE register
         mem->write_memory(SP,D); 
       }

       else if (rr == 0b10) { //HL register
         mem->write_memory(SP,H); 
       }

       else if (rr == 0b11) { //AF register
         mem->write_memory(SP,A); 
       }

       current_machine_cycle++;
       return 1;
    }

    else if (current_machine_cycle == 2) {
        SP--; // so we decrement sp again to go to next adjacent slot
        uint8_t rr = (opcode >> 4) & 0b00000011;

        if (rr == 0b00) { // BC register
          mem->write_memory(SP,C); //and store 2nd register value there instead of pairing and storing
        }

        else if (rr == 0b01) { //DE register
          mem->write_memory(SP,E); 
        }

        else if (rr == 0b10) { //HL register
          mem->write_memory(SP,L); 
        }

        else if (rr == 0b11) { //AF register
          mem->write_memory(SP,F & 0xF0);  // we only need first 4 bit of flag,lower bit is masked to zero
        }

        current_machine_cycle++;
        return 1;
    }
    //idle cycle :  in push rr , cpu takes one idle cycle wrapping things up,(hardware requirement)
    else if (current_machine_cycle == 3) {
        fetch_new_instruction = true;
        return 1;
    }

 }

 //pop rr : pop from stack -> pop data from stack to 16 bit register (1 byte cycle 3 machine cycle)
 //opcode : 0b11xx0001

 else if ((opcode & 0b11000000) == 0b11000000 && (opcode & 0b00001111) == 0b00000001) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = mem->read_memory(SP);
        SP++;
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = mem->read_memory(SP);
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

        fetch_new_instruction = true;
        return 1;
    }
    
 }

 //ld hl,sp+e : load hl with stackpointer value + e where e is immediate signed  int (not unsigned)
 //opcode : 0b11111000; (2 byte cycle,3 machine cycle)

 else if (opcode == 0b11111000) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        e = fetch_rawbyte();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
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

        fetch_new_instruction = true;
        return 1;
    }
    
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
     else if (r == 0b110) {
       if (current_machine_cycle == 1) alu_add_memory_adress(A,H,L);
       else {
        current_machine_cycle ++;
        return 1;
       }
     }
     else if (r == 0b111) alu_add(A,A);

        fetch_new_instruction = true;
        return 1;
       
 }

 //add n : add register a with immediate value n and store back in a(2 byte cycle 2 machine cycle)
 //opcode = 0b11000110

 else if (opcode == 0b11000110) {

    if (current_machine_cycle == 0) {
     current_machine_cycle++;
     return 1;
    }

    else if (current_machine_cycle == 1) {
     n = fetch_opcode();
     alu_add (A,n);
     fetch_new_instruction = true;
     return 1;
    }

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
     else if (r == 0b110) {
       if (current_machine_cycle == 1) alu_add_memory_adress_carry(A,H,L);
       else {
        current_machine_cycle++;
        return 1;
       }
     }
     else if (r == 0b111) alu_add_carry(A,A);

        fetch_new_instruction = true;
        return 1;
       
 }

 //adc n : add immediate value n to a with carry flag (2 byte cycle,2 machine cycle)
 //opcode = 0b11001110

 else if (opcode == 0b11001110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        alu_add_carry(A,n);
        fetch_new_instruction = true;
        return 1;    
    }

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
     else if (r == 0b110) {
       if (current_machine_cycle == 1) alu_sub_memory_adress(A,H,L);
       else {
        current_machine_cycle++;
        return 1;
       }
     }
     else if (r == 0b111) alu_sub(A,A);

        fetch_new_instruction = true;
        return 1;

 }

 //sub n : sub register a with immediate value n and store back in a(2 byte cycle 2 machine cycle)
 //opcode = 0b11010110

 else if (opcode == 0b11010110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        alu_sub (A,n);
        fetch_new_instruction = true;
        return 1;   
    }

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
     else if (r == 0b110) {
      if (current_machine_cycle == 1) alu_sub_memory_adress_carry(A,H,L);
      else {
        current_machine_cycle++;
        return 1;
      }
     }
     else if (r == 0b111) alu_sub_carry(A,A);

        fetch_new_instruction = true;
        return 1;

 }

 //sbc n : sub immediate value n to a with carry flag (2 byte cycle,2 machine cycle)
 //opcode = 0b11011110

 else if (opcode == 0b11011110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        alu_sub_carry(A,n);
        fetch_new_instruction = true;
        return 1;
    }

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
     else if (r == 0b110) {
       if (current_machine_cycle == 1) alu_compare_reg_memory_adress(A,H,L);
       else {
        current_machine_cycle++;
        return 1;
       }
     }
     else if (r == 0b111) alu_compare_reg(A,A);

        fetch_new_instruction = true;
        return 1;

 }

 //cp n : sub a immediate value n to a with carry flag dont update result change flags(2 byte cycle,2 machine cycle)
 //opcode = 0b11111110

 else if (opcode == 0b11111110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        alu_compare_reg(A,n);
        fetch_new_instruction = true;
        return 1;
    }

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
     else if (r == 0b110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
      }

      else if (current_machine_cycle == 1) {
        alu_inc_memory(H,L);
        current_machine_cycle++;
        return 1;
       }

       else if (current_machine_cycle == 2) {
        
        alu_inc_memory(H,L);

       }

     }
     else if (r == 0b111) alu_inc_reg(A);

        fetch_new_instruction = true;
        return 1; //last cycle will be idle cycle as inc hl takes 3 cycle

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
     else if (r == 0b110)  {

      if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
      }

      else if (current_machine_cycle == 1) {
        alu_dec_memory(H,L);
        current_machine_cycle++;
        return 1;
       }

       else if (current_machine_cycle == 2) {
        
        alu_dec_memory(H,L);

       }

     }
     else if (r == 0b111) alu_dec_reg(A);
  
        fetch_new_instruction = true;
        return 1;

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
     else if (r == 0b110) {
       if (current_machine_cycle == 1) alu_and_memory_adress(A,H,L);
       else {
        current_machine_cycle++;
        return 1;
       }
     }
     else if (r == 0b111) alu_and(A,A);

        fetch_new_instruction = true;
        return 1;
       
 }

 //and n : and a immediate value n to a (2 byte cycle,2 machine cycle)
 //opcode = 0b11100110

 else if (opcode == 0b11100110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        alu_and(A,n);
        fetch_new_instruction = true;
        return 1;
    }

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
     else if (r == 0b110) {
      if (current_machine_cycle == 1) alu_or_memory_adress(A,H,L);
      else {
        current_machine_cycle++;
        return 1;
      }
     }
     else if (r == 0b111) alu_or(A,A);
   
        fetch_new_instruction = true;
        return 1;
       
 }

 //or n : or a immediate value n to a (2 byte cycle,2 machine cycle)
 //opcode = 0b11110110

 else if (opcode == 0b11110110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        alu_or(A,n);
        fetch_new_instruction = true;
        return 1;
    }

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
     else if (r == 0b110) {
       if (current_machine_cycle == 1) alu_xor_memory_adress(A,H,L);
       else {
        current_machine_cycle++;
        return 1;
       }
     }
     else if (r == 0b111) alu_xor(A,A);

        fetch_new_instruction = true;
        return 1;

 }

 //xor n : xor a immediate value n to a (2 byte cycle,2 machine cycle)
 //opcode = 0b11101110

 else if (opcode == 0b11101110) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        n = fetch_opcode();
        alu_xor(A,n);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //special instructions :

 //ccf : flips the carry flag and clears h and n flag (1 byte cycle,1 machine cycle)
 //opcode : 0b00111111

 else if (opcode == 0b00111111) {
    alu_complement_carry_flag();
    fetch_new_instruction = true;
    return 1;
 }

 //scf : sets the carry flag and clears n and h flag (1 byte cycle 1 machine cycle)
 //opcode : 0b00110111

 else if (opcode == 0b00110111) {
    alu_set_carry_flag();
    fetch_new_instruction = true;
    return 1;
 }

 //daa : decimal adjust accumulator,used to fix a register for bcd arithmetic(1 byte cycle,1 machine cycle)
 //opcode : 0b00100111

 else if (opcode == 0b00100111) {
    alu_daa(A); // we will only apply daa for a register as it has a fixed opcode and it only applies on
    //a register where the result of a operation is stored mostly
    fetch_new_instruction = true;
    return 1;
 }

 //cpl : flips all bits in 8 bit register and sets n and h flags (1 byte cycle,1 machine cycle)
 //opcode : 0b00101111

 else if (opcode == 0b00101111) {
    alu_complement_reg(A);
    fetch_new_instruction = true;
    return 1;
 }

 //16 bit arithmetic operations : 

 // inc rr (1 byte cycle 2 machine cycle)
 //opcode = 0b00xx0011

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00000011) {
    uint8_t rr = (opcode >> 4) & 0b00000011;

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    } 
    
    else if (current_machine_cycle == 1) {

     if (rr == 0b00) alu_inc_paired_reg(B,C);
     else if (rr == 0b01) alu_inc_paired_reg(D,E);
     else if (rr == 0b10) alu_inc_paired_reg(H,L);
     else if (rr == 0b11) SP++;

     fetch_new_instruction = true;
     return 1;
    }
 }

 // dec rr (1 byte cycle 2 machine cycle)
 //opcode = 0b00xx1011

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00001011) {
    uint8_t rr = (opcode >> 4) & 0b00000011;

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {

     if (rr == 0b00) alu_dec_paired_reg(B,C);
     else if (rr == 0b01) alu_dec_paired_reg(D,E);
     else if (rr == 0b10) alu_dec_paired_reg(H,L);
     else if (rr == 0b11) SP--;

     fetch_new_instruction = true;
     return 1;
    }

 }

 // add hl,rr : add rr to hl and store result back into hl (1 byte cycle,2 machine cycle)
 //opcode : 0b00xx1001

 else if ((opcode & 0b11000000) == 0b00000000 && (opcode & 0b00001111) == 0b00001001) {
    uint8_t rr = (opcode >> 4) & 0b00000011;

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {

     if (rr == 0b00) alu_add_paired_reg(H,L,B,C);
     else if (rr == 0b01) alu_add_paired_reg(H,L,D,E);
     else if (rr == 0b10) alu_add_paired_reg(H,L,H,L);
     else if (rr == 0b11) alu_add_paired_reg(H,L,SP);

     fetch_new_instruction = true;
     return 1;
    }

 }

 // add sp,e : add sp to signed int e from next opcode (2 byte cycle,4 machine cycle)
 //opcode : 0b11101000

 else if (opcode == 0b11101000) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        e = fetch_rawbyte();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {

        set_flag ('Z',0);
        set_flag ('N',0);
        if ((SP & 0x0F) + (e & 0x0F) > 0x0F) set_flag('H',1);
        else set_flag('H',0);
        if ((SP & 0xFF) + (e & 0xFF) > 0xFF) set_flag('C',1);
        else set_flag('C',0);

        current_machine_cycle++;
        return 1;

    }

    else if (current_machine_cycle == 3) {
        SP += e;
        fetch_new_instruction = true;
        return 1;
    }
    
 }

 //rotate,shift and swap operations : 

 // RLCA : rotate left circular (accumulator),same as rlc r but always set z flag to 0
 //opcode : 0b00000111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00000111) {
    alu_rotate_left_circular(A);
    set_flag('Z',0);
    fetch_new_instruction = true;
    return 1;
 }

  // RRCA : rotate left circular (accumulator),same as rRc r but always set z flag to 0
 //opcode : 0b00001111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00001111) {
    alu_rotate_right_circular(A);
    set_flag('Z',0);
    fetch_new_instruction = true;
    return 1;
 }

 // RLA : rotate left  (accumulator),same as rL r but always set z flag to 0
 //opcode : 0b00010111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00010111) {
    alu_rotate_left_carry(A);
    set_flag('Z',0);
    fetch_new_instruction = true;
    return 1;
 }

 // RRA : rotate right  (accumulator),same as rr r but always set z flag to 0
 //opcode : 0b00011111        (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00011111) {
    alu_rotate_right_carry(A);
    set_flag('Z',0);
    fetch_new_instruction = true;
    return 1;
 }


 //cb opcodes : if opcodes starts with 0xcb the next opcode actually have instruction

 else if (opcode == 0xCB) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
     cb_opcode = fetch_opcode();
    }

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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_rotate_left_circular_memory_adress(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_rotate_left_circular_memory_adress(H,L); //this calls for the calculation and write mem

     }
     else if (r == 0b111) alu_rotate_left_circular(A);

      fetch_new_instruction = true;
      return 1;

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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_rotate_right_circular_memory_adress(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_rotate_right_circular_memory_adress(H,L);

     }
     else if (r == 0b111) alu_rotate_right_circular(A);

     fetch_new_instruction = true;
     return 1;

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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_rotate_left_carry_memory_adress(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_rotate_left_carry_memory_adress(H,L);

     }
     else if (r == 0b111) alu_rotate_left_carry(A);

      fetch_new_instruction = true;
      return 1;
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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_rotate_right_carry_memory_adress(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_rotate_right_carry_memory_adress(H,L);

     } 
     else if (r == 0b111) alu_rotate_right_carry(A);

      fetch_new_instruction = true;
      return 1;
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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_shift_left_memory_adress(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_shift_left_memory_adress(H,L);

     }
     else if (r == 0b111) alu_shift_left(A);

     fetch_new_instruction = true;
     return 1;
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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_shift_right_memory_adress(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_shift_right_memory_adress(H,L);

     }
     else if (r == 0b111) alu_shift_right(A);

     fetch_new_instruction = true;
     return 1;
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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_shift_right_logical_memory_adress(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_shift_right_logical_memory_adress(H,L);

     } 
     else if (r == 0b111) alu_shift_right_logical(A);

      fetch_new_instruction = true;
      return 1;
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
     else if (r == 0b110) {

        if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
        }

        else if (current_machine_cycle == 2) {
        alu_swap_memory(H,L); //thhis only calls read mempory part
        current_machine_cycle++;
        return 1;
        }

        else if (current_machine_cycle == 3) alu_swap_memory(H,L);

     }
     else if (r == 0b111) alu_swap(A);

      fetch_new_instruction = true;
      return 1;
    }

    //BIT b,r : test bit register 
    //tests the bit b  of a register r , if b is 0 it sets the z flag to 1 else z flag is set to zero
    //n flag is set to 0 and h is set to 1 (2 byte cycle,2 machine cycle)
    //opcode = 0xcb >> 0b01xxxxxx 

    
    else if ((cb_opcode & 0b11000000) == 0b01000000) {
        uint8_t bit = (cb_opcode >> 3) & 0b00000111;
        uint8_t r = cb_opcode & 0b00000111;

        uint8_t* reg_pointer = nullptr;

    if (r==0b110) {

         if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
         }

         else if (current_machine_cycle == 2) {
          n = mem->read_memory(pair_registers(H,L));

          reg_pointer = &n;//n is a 8 bit int that we get by reading mem earlier

             if (bit == 0b000) alu_test_bit(*reg_pointer,0b00000001);
             else if (bit == 0b001) alu_test_bit(*reg_pointer,0b00000010);
             else if (bit == 0b010) alu_test_bit(*reg_pointer,0b00000100);
             else if (bit == 0b011) alu_test_bit(*reg_pointer,0b00001000);
             else if (bit == 0b100) alu_test_bit(*reg_pointer,0b00010000);
             else if (bit == 0b101) alu_test_bit(*reg_pointer,0b00100000);
             else if (bit == 0b110) alu_test_bit(*reg_pointer,0b01000000);
             else if (bit == 0b111) alu_test_bit(*reg_pointer,0b10000000);

             fetch_new_instruction = true;
             return 1;
          
          }

          
    }

    else {

      if (r == 0b000) reg_pointer = &B;
      else if (r == 0b001) reg_pointer = &C;
      else if (r == 0b010) reg_pointer = &D;
      else if (r == 0b011) reg_pointer = &E;
      else if (r == 0b100) reg_pointer = &H;
      else if (r == 0b101) reg_pointer = &L;
      // r == 110 has been taken care of above
      else if (r == 0b111) reg_pointer = &A;

      if (bit == 0b000) alu_test_bit(*reg_pointer,0b00000001);
      else if (bit == 0b001) alu_test_bit(*reg_pointer,0b00000010);
      else if (bit == 0b010) alu_test_bit(*reg_pointer,0b00000100);
      else if (bit == 0b011) alu_test_bit(*reg_pointer,0b00001000);
      else if (bit == 0b100) alu_test_bit(*reg_pointer,0b00010000);
      else if (bit == 0b101) alu_test_bit(*reg_pointer,0b00100000);
      else if (bit == 0b110) alu_test_bit(*reg_pointer,0b01000000);
      else if (bit == 0b111) alu_test_bit(*reg_pointer,0b10000000);

      fetch_new_instruction = true;
      return 1;
    }

    }


    //res b,r : reset bit register 
    //resets the bit b  of a register r , (2 byte cycle,2 machine cycle)
    //opcode = 0xcb >> 0b10xxxxxx 

    else if ((cb_opcode & 0b11000000) == 0b10000000) {
        uint8_t bit = (cb_opcode >> 3) & 0b00000111;
        uint8_t r = cb_opcode & 0b00000111;

        uint8_t* reg_pointer = nullptr; // reg pointer

    if (r==0b110) {

         if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
         }

         else if (current_machine_cycle == 2) {
          n = mem->read_memory(pair_registers(H,L));
          current_machine_cycle++;
          return 1;
          }

          else if (current_machine_cycle == 3) {
            reg_pointer = &n;

             if (bit == 0b000) alu_reset_bit(*reg_pointer,0b00000001);
             else if (bit == 0b001) alu_reset_bit(*reg_pointer,0b00000010);
             else if (bit == 0b010) alu_reset_bit(*reg_pointer,0b00000100);
             else if (bit == 0b011) alu_reset_bit(*reg_pointer,0b00001000);
             else if (bit == 0b100) alu_reset_bit(*reg_pointer,0b00010000);
             else if (bit == 0b101) alu_reset_bit(*reg_pointer,0b00100000);
             else if (bit == 0b110) alu_reset_bit(*reg_pointer,0b01000000);
             else if (bit == 0b111) alu_reset_bit(*reg_pointer,0b10000000);

             mem->write_memory(pair_registers(H,L),n); //for memory operations write back

             fetch_new_instruction = true;
             return 1;
          }

    }

    else {

      if (r == 0b000) reg_pointer = &B;
      else if (r == 0b001) reg_pointer = &C;
      else if (r == 0b010) reg_pointer = &D;
      else if (r == 0b011) reg_pointer = &E;
      else if (r == 0b100) reg_pointer = &H;
      else if (r == 0b101) reg_pointer = &L;
      // r == 110 has been taken care of above
      else if (r == 0b111) reg_pointer = &A;

      if (bit == 0b000) alu_reset_bit(*reg_pointer,0b00000001);
      else if (bit == 0b001) alu_reset_bit(*reg_pointer,0b00000010);
      else if (bit == 0b010) alu_reset_bit(*reg_pointer,0b00000100);
      else if (bit == 0b011) alu_reset_bit(*reg_pointer,0b00001000);
      else if (bit == 0b100) alu_reset_bit(*reg_pointer,0b00010000);
      else if (bit == 0b101) alu_reset_bit(*reg_pointer,0b00100000);
      else if (bit == 0b110) alu_reset_bit(*reg_pointer,0b01000000);
      else if (bit == 0b111) alu_reset_bit(*reg_pointer,0b10000000);

      fetch_new_instruction = true;
      return 1;
    }

    }

    //set b,r : sets bit register 
    //sets the bit b  of a register r to 1 , (2 byte cycle,2 machine cycle)
    //opcode = 0xcb >> 0b11xxxxxx 

    else if ((cb_opcode & 0b11000000) == 0b11000000) {
        uint8_t bit = (cb_opcode >> 3) & 0b00000111;
        uint8_t r = cb_opcode & 0b00000111;

        uint8_t* reg_pointer = nullptr; // reg pointer

        if (r==0b110) {

         if (current_machine_cycle == 1) {
            current_machine_cycle++;
            return 1;
         }

         else if (current_machine_cycle == 2) {
          n = mem->read_memory(pair_registers(H,L));
          current_machine_cycle++;
          return 1;
          }

          else if (current_machine_cycle == 3) {
            reg_pointer = &n;

             if (bit == 0b000) alu_set_bit(*reg_pointer,0b00000001);
             else if (bit == 0b001) alu_set_bit(*reg_pointer,0b00000010);
             else if (bit == 0b010) alu_set_bit(*reg_pointer,0b00000100);
             else if (bit == 0b011) alu_set_bit(*reg_pointer,0b00001000);
             else if (bit == 0b100) alu_set_bit(*reg_pointer,0b00010000);
             else if (bit == 0b101) alu_set_bit(*reg_pointer,0b00100000);
             else if (bit == 0b110) alu_set_bit(*reg_pointer,0b01000000);
             else if (bit == 0b111) alu_set_bit(*reg_pointer,0b10000000);

             mem->write_memory(pair_registers(H,L),n); //for memory operations write back

             fetch_new_instruction = true;
             return 1;
          }

    }

    else {

      if (r == 0b000) reg_pointer = &B;
      else if (r == 0b001) reg_pointer = &C;
      else if (r == 0b010) reg_pointer = &D;
      else if (r == 0b011) reg_pointer = &E;
      else if (r == 0b100) reg_pointer = &H;
      else if (r == 0b101) reg_pointer = &L;
      // r == 110 has been taken care of above
      else if (r == 0b111) reg_pointer = &A;

      if (bit == 0b000) alu_set_bit(*reg_pointer,0b00000001);
      else if (bit == 0b001) alu_set_bit(*reg_pointer,0b00000010);
      else if (bit == 0b010) alu_set_bit(*reg_pointer,0b00000100);
      else if (bit == 0b011) alu_set_bit(*reg_pointer,0b00001000);
      else if (bit == 0b100) alu_set_bit(*reg_pointer,0b00010000);
      else if (bit == 0b101) alu_set_bit(*reg_pointer,0b00100000);
      else if (bit == 0b110) alu_set_bit(*reg_pointer,0b01000000);
      else if (bit == 0b111) alu_set_bit(*reg_pointer,0b10000000);

      fetch_new_instruction = true;
      return 1;
    }
     
     
    }

  return 0;

 }//cb opcodes end

 // regular opcodes continue here : 

 //control flow instructions : 

 //jp nn : jump -> unconditional jump to absolute adress specified by absolute adress specified by immediate value nn
 //opcode : 0b11000011    (3 byte cycle,4 machine cycle)

 else if (opcode == 0b11000011) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 3) {
        PC = pair_registers(highbyte,lowbyte);
        fetch_new_instruction = true;
        return 1;
    }
    
 }

 //jp HL : unconditional jump to adress specified by HL (1 byte cycle,1 machine cycle)
 //opcode : 0b11101001

 else if (opcode == 0b11101001) {
    PC = pair_registers(H,L);
    fetch_new_instruction = true;
    return 1;
 }

 //jp cc,nn : conditional jump to the absolute adress specified by nn depending on condition cc
 //operand is read even when condition is false 
 //opcode  = 0b110xx010              (3 byte cycle and (4 machine cycle if cc is true or 3 byte cycle if cc is false))
 //gameboy cpu is 4 conditiions and only zero and carry flags are used (zero,not zero,carry,not carry)

 else if ((opcode & 0b11100000) == 0b11000000 && (opcode & 0b00000111) == 0b00000010) {
    bool z_flag = get_flag('Z');
    bool C_flag = get_flag('C');

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = fetch_opcode();

        uint8_t cc = (opcode >> 3) & 0b00000011;

    if (cc == 0b00) { //NZ = not zero

        if (!z_flag) current_machine_cycle++;
        else fetch_new_instruction = true;

    }

    else if (cc == 0b01) { //Z = zero

        if (z_flag)  current_machine_cycle++;
        else fetch_new_instruction = true;

    }

    else if (cc == 0b10) { //NC = not carry

        if (!C_flag) current_machine_cycle++;
        else fetch_new_instruction = true;
    }

    else if (cc == 0b11) { //C = carry
        if (C_flag) current_machine_cycle++;
        else fetch_new_instruction = true;
    }

    return 1;
       
    }

    else if (current_machine_cycle == 3) {

         PC = pair_registers(highbyte,lowbyte);
         fetch_new_instruction = true;
         return 1;
    }


 }

 //JR e : jump to the relative adress specified by 8 bit signed operand e (2 byte cycle,3 machine cycle)
 //opcode = 0b00011000

 else if (opcode == 0b00011000) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        e = fetch_rawbyte ();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        PC += e;
        fetch_new_instruction = true;
        return 1;
    }
    
 }

 //jr cc e : jump to the relativve adress specified by 8 bit operand e depending on cc
 //opcode = 0b001xx000                    (2 byte cycle,(3 machine cycle if cc is true,2 if cc is false))

 else if ((opcode & 0b11100000) == 0b00100000 && (opcode & 0b00000111) == 0b00000000) {
    bool z_flag = get_flag('Z');
    bool C_flag = get_flag('C');

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        e = fetch_rawbyte();
        uint8_t cc = (opcode >> 3) & 0b00000011;

        if (cc == 0b00) { //NZ = not zero
         if (!z_flag) current_machine_cycle++;
         else fetch_new_instruction = true;;
        }

        else if (cc == 0b01) { //Z = zero
         if (z_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
        }

        else if (cc == 0b10) { //NC = not carry
         if (!C_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
        } 

        else if (cc == 0b11) { //C = carry
         if (C_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
        }

        return 1;
     }

    else if (current_machine_cycle == 2) {

        PC += e;
        fetch_new_instruction = true;
        return 1;
    }

    
    }


 //call nn : call function -> unconditional function call to the absolute adress specified by nn
 //opcode : 0b11001101           (3 byte cycle,6 machine cycle)

 else if (opcode == 0b11001101) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 3) {
        //cpu does internal operations here and prepares stack
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 4) {
        SP--; //move to next empty space in stack pointer
        mem->write_memory(SP,(PC >> 8) & 0xFF); //writting high byte
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 5) {
        SP--; //stack grows downward
        mem->write_memory (SP,PC & 0xFF);

        PC = pair_registers(highbyte,lowbyte);

        fetch_new_instruction = true;
        return 1;
    }

 }

 //call cc,nn : conditional call (3 byte cycle,(6 machine cycle if cc true or 3 machine cycle if cc false))
 //opcode : 0b110xx100

 else if ((opcode & 0b11100000) == 0b11000000 && (opcode & 0b00000111) == 0b00000100) {
    bool z_flag = get_flag('Z');
    bool C_flag = get_flag('C');

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = fetch_opcode();
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = fetch_opcode();
        uint8_t cc = (opcode >> 3) & 0b00000011;

        if (cc == 0b00) { //NZ = not zero
         if (!z_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }

    else if (cc == 0b01) { //Z = zero
        if (z_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }
    
    else if (cc == 0b10) { //NC = not carry
        if (!C_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }


    else if (cc == 0b11) { //C = carry
        if (C_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }

    return 1;

    }

    else if (current_machine_cycle == 3) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 4) {
        SP--; //move to next empty space in stack pointer
        mem->write_memory(SP,(PC >> 8) & 0xFF); //writting high byte 
        current_machine_cycle++;
        return 1;  
    }

    else if (current_machine_cycle == 5) {
         SP--; //stack grows downward
        mem->write_memory (SP,PC & 0xFF);
        PC = pair_registers(highbyte,lowbyte);

        fetch_new_instruction = true;
        return 1;
    }

    }

 // ret : unconditional return from a function (1 byte cycle,4 machine cycle) 
 //opcode : 0b11001001

 else if (opcode == 0b11001001) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = mem->read_memory(SP);
        SP++;
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = mem->read_memory(SP);
        SP++;
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 3) {
        PC = pair_registers (highbyte,lowbyte);
        fetch_new_instruction = true;
        return 1;
    }

 }

 //ret cc : return from function conditional (1 byte cycle (5 machine cycle if cc true or 2 machine cycle if cc false))
 //opcode : 0b110xx000

 else if ((opcode & 0b11100000) == 0b11000000 && (opcode & 0b00000111) == 0b00000000) {
    bool z_flag = get_flag('Z');
    bool C_flag = get_flag('C');

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        uint8_t cc = (opcode >> 3) & 0b00000011;

    if (cc == 0b00) { //NZ = not zero
         if (!z_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }

    else if (cc == 0b01) { //Z = zero
        if (z_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }
    

    else if (cc == 0b10) { //NC = not carry
        if (!C_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }


    else if (cc == 0b11) { //C = carry
        if (C_flag) current_machine_cycle++;
         else fetch_new_instruction = true;
    }

    return 1;

    }

    else if (current_machine_cycle == 2) {
         lowbyte = mem->read_memory(SP);
         SP++;
         current_machine_cycle++;
         return 1;
    }

    else if (current_machine_cycle == 3) {
         highbyte = mem->read_memory(SP);
         SP++;
         current_machine_cycle++;
         return 1;
    }

    else if (current_machine_cycle == 4) {
         PC = pair_registers (highbyte,lowbyte);
         fetch_new_instruction = true;
         return 1;
    }

    
 }

 //reti : return from interupt handler -> unconditional return from a function also sets ime to 1 
 // enabling interupts (1 byte cycle,4 machine cycle)
 //opcode = 0b11011001

 else if (opcode == 0b11011001) {

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        lowbyte = mem->read_memory(SP);
        SP++;
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        highbyte = mem->read_memory(SP);
        SP++;
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 3) {
        PC = pair_registers (highbyte,lowbyte);
        IME = true;
        fetch_new_instruction = true;
        return 1;
    }
   
 }

 //RST n : restart / call function (implied)   (1 byte cycle,4 machine cycle)
 // unconditional functional call to to the absolute adress defined by opcode
 // similar to call functions but fast as we jump to 8 absolute specified adresses
 //opcode = 0b11xxx111
 else if ((opcode & 0b11000000) == 0b11000000 && (opcode & 0b00000111) == 0b00000111) {
    uint8_t adress = (opcode >> 3) & 0b00000111;

    if (current_machine_cycle == 0) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 1) {
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 2) {
        SP--; //move to next empty space in stack pointer
        mem->write_memory(SP,(PC >> 8) & 0xFF); //writting high byte
        current_machine_cycle++;
        return 1;
    }

    else if (current_machine_cycle == 3) {
         SP--; //stack grows downward
        mem->write_memory (SP,PC & 0xFF);

     if (adress == 0b000) PC = 0x0000; // RST 00
     else if (adress == 0b001) PC = 0x0008; // RST 08
     else if (adress == 0b010) PC = 0x0010; // RST 10
     else if (adress == 0b011) PC = 0x0018; // RST 18
     else if (adress == 0b100) PC = 0x0020; // RST 20
     else if (adress == 0b101) PC = 0x0028; // RST 28
     else if (adress == 0b110) PC = 0x0030; // RST 30
     else if (adress == 0b111) PC = 0x0038; // RST 38

     fetch_new_instruction = true;
     return 1;
    }
       
 }

 // miscellaneous instructions : 

 // halt : implemented at the start

  // disable interupts : disables interupts by setting ime to 0
 //opcode = 0b11110011 (1 byte cycle,i machine cycle)

 else if (opcode == 0b11110011) {
    IME = false;
    IME_scheduled = false; // also disables EI schedule
    fetch_new_instruction = true;
    return 1;
 }

  //enable interupts : enables interupts by setting ime to 0 after the next machine_cycle
 //opcode = 0b11111011 (1 byte cycle,i machine cycle)

 else if (opcode == 0b11111011) {
    IME_scheduled = true;
    fetch_new_instruction = true;
    return 1;
 }

 //nop : no operation : eat fivestar and do nothing,pc will be auto incremented,can be used to add delay of 1 machine cycle
 // opcode = 0b00000000 (1 byte cycle,1 machine cycle)

 else if (opcode == 0b00000000){
    //nothing
    fetch_new_instruction = true;
    return 1;
 }

 else if (opcode == 0x10) {
    //stop
    //fetch_new_instruction = true;
    return 0;
 }

std::cout << "unimplemented opcode : " << std::hex << +opcode << std::endl;
fetch_new_instruction = true;
 return 1;
}//function end

uint8_t check_interupt () {
    if ((mem->IF & mem->IE) & 0X1F) {
        interupt_pending = true;
        uint8_t interrupt = (mem->IF & mem->IE) & 0X1F;
        if (interrupt & 0b00000001) return 0b00000001;//returns type of the interrupt
        else if (interrupt & 0b00000010) return 0b00000010;
        else if (interrupt & 0b00000100) return 0b00000100;
        else if (interrupt & 0b00001000) return 0b00001000;
        else if (interrupt & 0b00010000) return 0b00010000;
    }
     return 0;
}

uint16_t get_interupt_jump_adress (uint8_t Interupt) const {
    // priority wise
    if (Interupt == 0b00000001) return 0x0040;
    else if (Interupt == 0b00000010) return 0x0048;
    else if (Interupt == 0b00000100) return 0x0050;
    else if (Interupt == 0b00001000) return 0x0058;
    else if (Interupt == 0b00010000) return 0x0060;
    else return 0;
}

void handle_interupt() {
    uint8_t interupt_present = check_interupt();

    if (IME && interupt_present) {

        IME = false;

        halted = false;

        SP--; //move to next empty space in stack pointer
        mem->write_memory(SP,(PC >> 8) & 0xFF); //writting high byte
        SP--; //stack grows downward
        mem->write_memory (SP,PC & 0xFF);
        
        PC = get_interupt_jump_adress(interupt_present);

        mem->IF &= ~(interupt_present); //clearing specific bit

        interrupt_serviced = true;
    }
}

int cpu_cycle() {


 if (halted) {

   if (check_interupt ()) {

    halted = false; // if interupt is present wake cpu up even if ime is 0

   }

  else return 1; //else keep halted

 }

   if (stopped) return 0;

    int machine_cycle = execute_opcode();

    if (IME_scheduled) {
        ei_delay = 4;
        IME_scheduled = false;
    }

    handle_interupt(); // check if we can service an interupt

    if (interrupt_serviced) {
        machine_cycle += 5;  //servicing an interrupt takes 5 machine cycle
        interrupt_serviced = false;
    }

 return machine_cycle*4; //return machine cycle in tick cycle

}


void timer_tick() {

    uint16_t prev_div_counter = mem->div_counter; //get current div
    mem->div_counter++; //increment
    mem->DIV = ((mem->div_counter >> 8) & 0xFF); //update

    bool timer_update = false;
    bool timer_enabled = mem->timer_enabled();

    if (timer_enabled) {
     int bit = mem->get_timer_bit();

     bool old_bit = (prev_div_counter >> bit) & 1;
     bool new_bit = (mem->div_counter >> bit) & 1;

     if (old_bit == 1 && new_bit == 0) {
        timer_update = true;
     }
    } 

   if (timer_update && timer_enabled) {
        
        if (!mem->tima_overflow) {

             mem->TIMA++; //if timer is enabled and tima can update,increment tima

            if (mem->TIMA == 0x00) { //if tima overflows
            mem->tima_overflow = true;
            mem->tima_reload_delay = 4;
            }

        }

    }

    if (mem->tima_overflow) {

            if (mem->tima_reload_delay == 0) {
                mem->TIMA = mem->TMA;
                mem->tima_overflow = false;
                request_interrupt('T');
            }
            else mem->tima_reload_delay--;

    }

}

};

struct ppu {

 memory* mem;

 ppu (memory* memory) {
    mem = memory;
   internal_ppu_ticks = 0;
 }

 int internal_ppu_ticks; //ppu clock

 //these are the data of sprite stored in oam,a sprite is group of 4 bytes in oam
 uint8_t sprite_y;          //1st byte
 uint8_t sprite_x;          //2nd byte
 uint8_t sprite_tile;       //3rd byte
 uint8_t sprite_attribute;  //4th byte

 //oam search : ppu searches the oam,oam has 40 sprites ,each 4 byte,in oam search
 //ppu searches the oam sprites to check which sprites meet height criteria for 
 //current scanline ,height of sprite can be 8 pixel or 16 pixel ,it depends on bit 2 
 //of lcdc register

 int sprite_height () { 
    if (mem->LCDC & 0b00000100) return 16;
    else return 8;
 }

 uint8_t loadable_sprite_table [40]; // oam search can store maximum 10 sprite per scanline out of 40 in gameboy classic
 // we will store all loadable sprites in this table

 uint8_t pixel_buffer [160]; //the viewing screen of gameboy is 160x144 pixels,each scanline is
 // 1 row of pixel aka 160 pixel,we will pixel data for 1 scanline in this buffer to draw it

uint8_t frame_buffer [144] [160]; //this is the entire visible screen 144 scanline and 160 pixel each


 //now lets make functions to create frame buffer and drawing data

 void draw_background_pixels () {

    if (!(mem->LCDC & 0b00000001)) return; //if lcdc bit 0 is 0 both bg and window are disabled

    int scanline = mem->LY; //current scanline information
    int scroll_y = mem->SCY; // scy register decides which 144 out of 256 background pixel will
    // be rendered on visible screen,meaning it decides the starting position 
    int scroll_x = mem->SCX; //scx register decides which 160 out of 256 background pixel will
    // be rendered on visible screen,meaning it decides the starting position 
    uint16_t verticle_pixel = (scanline + scroll_y) & 0XFF;//which verticle pixel is about to be drawn,
    //masked to 0xff so it ranges from 0-255,and 256 gets rounded off to 0 again

    uint8_t tile_row = verticle_pixel / 8; //we are getting which row of background tile map
    // to use ,as scroll_y is defined in pixel we divide the whole by 8 to get tile row;
    uint8_t row_inside_tile = verticle_pixel % 8;//this tells which pixel row inside the tile we will load

    //now lets fill the buffer 

    for (int screen_x = 0; screen_x < 160; screen_x++) {
        uint16_t horizontal_pixel = (scroll_x + screen_x) & 0XFF; //tells which horizontal pixel we are looking at the screen
        uint8_t tile_column = horizontal_pixel / 8; //tells the column of tile
        uint8_t pixel_inside_tile = horizontal_pixel % 8;  //tells how many pixels deep in a tile we are

        uint16_t index_start_pos;
        bool start = (mem->LCDC & 0b00001000); //controlled by lcdc bit 3;
        if (start == 0) index_start_pos = 0x9800 - 0x8000;
        else index_start_pos = 0x9C00 - 0x8000;

        uint8_t tile_index = mem->ram [index_start_pos + tile_row*32 + tile_column]; //this fetches which tile will go here
        //now each tile is 16 byte ,now we will use this index to access tile data storing region of vram

        //lcdc bit 4 actually decides if the data stored in vrm ox8000 to 0x97ff is gonna be interpreted as 
        //signed int or unsigned int

        uint16_t tile_data_start;
        uint16_t tile_data_address;

        if (mem->LCDC & 0b00010000) {
            tile_data_start = 0x8000 - 0x8000;
            tile_data_address = tile_data_start + (tile_index * 16);}
        else {
            int8_t signed_tile_index = (int8_t) tile_index; //for signed data, we cast index as signed and start from 9000
            tile_data_start = 0x9000 - 0x8000;
            tile_data_address = tile_data_start + (signed_tile_index * 16);
        }

        uint16_t row_adress = tile_data_address + (row_inside_tile * 2); //this gives the address of the pixel row inside tile we want
        //we multiply it by 2 because a gameboy pixel is 2 bit each row has 8 pixel hence 2 byte
        //so now we have exact pixel address which to load

        //now lets fetch 2 byte data for the pixel row,low byte first the high

        uint8_t lowbyte = mem->ram[row_adress]; //bit 76543210 represents low bit of pixel 76543210;
        uint8_t highbyte = mem->ram[row_adress+1];//bit 76543210 represents high bit of pixel 76543210;

        //now we have data for the whole pixel row,but data for each pixel colour data is stored differently
        //each pixel is 2 bit,bit 0 is from low byte and bit 1 is from high byte

        //lets fetch colour for  a pixel
        //first locate whivh pixel

        uint8_t pixel = 7 - pixel_inside_tile;  //7th bit is the highest bit,bit is stored in memory as 7654---0,so 
        //we are reversing the order by minusing pixel inside tile from msb;becauuse high and lowbyte is also stored as
        //76543210;

        uint8_t colour = (((highbyte >> pixel) & 1) << 1) | ((lowbyte >> pixel) & 1); //now low bit and high bit is set
        //its masked with one so we only get o and 1;
        uint8_t shade = (mem->BGP >> (colour * 2)) & 0X03; //choosing final shade from pallete register
        //now we have the colour of the pixel ,lets load it in pixel buffer
        pixel_buffer[screen_x] = shade;
        //so when this loop ends we will have colour of entire scan line background
    }
 }

 void draw_window_pixel () {
    //5th bit of lcdc register decides if window is enabled or not
    if (!(mem->LCDC & 0b00100000)) return;
    if (!(mem->LCDC & 0b00000001)) return; //if lcdc bit 0 is 0 both bg and window are disabled
    //if window is enabled we will start drawing window,the scanline window will be drawn is
    //decided by wy and pixel position is decided by wx

    bool draw_window = false;
    if ((mem->LY >= mem->WY) && (mem->WX <= 166)) draw_window = true; //we are using 166 because 
    // real windowx = wx -7;so its actually 159,,hardwire quirk!!!
    else return; //if we cannot draw a window in current scanline,return

    //now lets get information regarding pixel and tiles,
    //window is nonscr0llable ,so scx and scy wont come in play here

    uint8_t scanline = mem->LY;
    uint8_t veritcle_pixel = scanline - mem->WY; //if its 0 means its first window pixel row
    uint8_t tile_row = veritcle_pixel / 8; //gives the tile row
    uint8_t row_in_tile = veritcle_pixel % 8; //gives which pixel we are starting in selected tile

    for (int screen_x = 0; screen_x < 160 ; screen_x++) {
        int window_start = mem->WX - 7;
        if (window_start < 0) window_start = 0; //prevents underflow bug
        if (screen_x < window_start) continue; //window wont be drawn in current pixel if wx - 7 < screenx;

        uint8_t window_x = screen_x - (mem->WX - 7);
        uint8_t tile_column = window_x / 8;
        uint8_t pixel_in_tile = window_x % 8;

        uint16_t index_start_pos = (mem->LCDC & 0b01000000); //controlled by lcdc bit 6;
        if (index_start_pos == 0) index_start_pos = 0x9800 - 0x8000;
        else index_start_pos = 0x9C00 - 0x8000;

        uint8_t tile_index = mem->ram [index_start_pos + tile_row*32 + tile_column]; //this fetches which tile will go here

        uint16_t tile_data_start;
        uint16_t tile_data_address;

        if (mem->LCDC & 0b00010000) {
            tile_data_start = 0x8000 - 0x8000;
            tile_data_address = tile_data_start + (tile_index * 16);}
        else {
            int8_t signed_tile_index = (int8_t) tile_index; //for signed data, we cast index as signed and start from 9000
            tile_data_start = 0x9000 - 0x8000;
            tile_data_address = tile_data_start + (signed_tile_index * 16);
        }

        uint16_t tile_row_address = tile_data_address + (row_in_tile * 2);

        uint8_t lowbyte = mem->ram [tile_row_address];
        uint8_t highbyte = mem->ram [tile_row_address+1];

        uint8_t pixel = 7 - pixel_in_tile;
        uint8_t colour = (((highbyte >> pixel) & 1) << 1) | ((lowbyte >> pixel) & 1);
        uint8_t shade = (mem->BGP >> (colour * 2)) & 0X03; //choosing final shade from pallete register
        pixel_buffer[screen_x] = shade;
    }
 }

 void draw_sprite_pixel () {
    if (!(mem->LCDC & 0b00000010)) return; //lcdc bit 1 decides if drawing sprites is enabled or not
    uint8_t height = sprite_height();
    for (int sprite_count = 0; sprite_count < 40 ;sprite_count++) {

        sprite_y = loadable_sprite_table[sprite_count];
        sprite_count++;
        sprite_x = loadable_sprite_table[sprite_count];
        sprite_count++;
        sprite_tile = loadable_sprite_table[sprite_count];
        sprite_count++;
        sprite_attribute = loadable_sprite_table[sprite_count];

        int screen_x =  sprite_x - 8; //oam stores positions with offsets,so convert them
        //to screen pixel
        int screen_y = sprite_y - 16;

        if ( mem->LY < screen_y || mem-> LY >= screen_y + height) continue; //if sprite is not in current scanline vertically,continue
        //now figure out which row of sprite pixels to draw in curreny scanline

        uint16_t row = mem->LY - screen_y;

        //now handle flip : sprite attribuute bit 6 decides if sprite rows will be flipped or not
        //meaning 012345678 can be 876543210 if bit 6 = 1;

        if (sprite_attribute & 0b01000000) row = (height - row) - 1; //to flip we are gonna minus heigh of the
        //sprite with row ,but as row index starts from 0 we are gonna subtract a 1 from it;

        //handle 8X16 sprites : oam stores only 1 tile index but 8 X 16 sprites are 2 tiles stacked vertically
        //hardware forces the sprite index to be even in 8X16 sprites so only 1 pair of top and bottom half existt
        //so by a index we will decide if its top or bottom half;

        if (height == 16) {

            sprite_tile &= 0xFE ; //sprite tile is forced to be even,hardwire does this

            if (row >= 8) { //bottom half
                sprite_tile += 1; //so we move to next tile,but because we forced index to be even
                //this tile cannot be a top tile for the next tile
                row -= 8;  //because a tile only has 8 rows of pixel
            }
        }

            //now lets fetch tile data,data is stored in vram 8000 to 8fff
            uint16_t tile_data_start = 0x8000 - 0x8000; //it starts from vram 0,lol
            uint16_t tile_address = tile_data_start + sprite_tile*16;
            uint16_t tile_row_address = tile_address + row*2; //decides which row of pixel from selected tile to draw,
            //each row is of 2 byte

            //tile data

            uint8_t lowbyte = mem->ram [tile_row_address];
            uint8_t highbyte = mem->ram [tile_row_address+1];

            //now get the pixel data


            for (int column = 0 ; column < 8 ; column++) { //now we need individual  pixel data and load it in pixel buffer
                //in background and window pixel we draw every horizpntal pixel in a loop for visible screen,but during
                //sprite we are not looping every visible screen pixel,so we need to calculate screenx with column of the row
                // and get to individual pixels 

                int bit;
                if (sprite_attribute & 0b00100000) bit = column; //x flip , decided by bit 5 of attribute
                else bit = 7 - column; //7th bit is the highest bit,bit is stored in memory as 7654---0,so 
             //we are reversing the order by minusing pixel inside tile from msb;becauuse high and lowbyte is also stored as
             //76543210;so during x flip we just dont flip
                uint8_t colour = (((highbyte >> bit) & 1) << 1) | ((lowbyte >> bit) & 1);
                if (colour == 0) continue; //if colour of sprite pixel is 0,background/window should remain visible
                int pixel_x = screen_x + column; //position where pixel will be drawn
                if (pixel_x < 0 || pixel_x >= 160) continue; //if its outside visible screen continue

                //now we need to check if sprite is behind background,sprite attribute bit 7 decides that

                if ((sprite_attribute & 0b10000000) && (pixel_buffer [pixel_x] != 0)) continue; 
                //dont draw sprite pixel as its behind background,but if background colour is 0,we draw sprite pixel

                //now lets choose sprite colour pallete
                //it can be choosen from 2 registers,obp0 and obp1,attribute bit 4 decides that
                uint8_t pallete = 0;
                if (sprite_attribute & 0b00010000) pallete = mem->OBP1;
                else pallete = mem->OBP0;

                //now inside pallete colours are stored in 2 bit groups ,we will extract the colour

                uint8_t shade = (pallete >> (colour*2)) & 3; //final shade 
                pixel_buffer [pixel_x] = shade;

            }
        }
    }

 //ppu has 4 modes : lets write them here

 void oam_search () {  //mode 2
    //uint8_t value = (mem->STAT & 0b11111100);
    //mem->STAT = (value | 0b10);

    if (mem->STAT & 0b00100000) mem->IF |=  (1 << 1); //fire lcd stat interrupt

     mem->oam_search = true;
    int marker = 0; //marks the begining of the loadable sprite table array;set to 0 at the start of the search
    // so it looks at the start of the table

    for (int i = 0; i < 40 ; i++) {
        loadable_sprite_table [i] = 0; //clear prev table
    }

    for (int sprite_count = 0 ; sprite_count < 40 ; sprite_count++) {
        for (int sprite_data_table = 0; sprite_data_table < 4 ; sprite_data_table++) {

            uint8_t sprite_data = mem->ram [(0XFE00 - 0X8000) + (sprite_count * 4 + sprite_data_table)];

            switch (sprite_data_table) {
                case 0 : sprite_y = sprite_data; break;
                case 1 : sprite_x = sprite_data; break;
                case 2 : sprite_tile = sprite_data; break;
                case 3 : sprite_attribute = sprite_data; break;
            }

        }// for j

        int sprite_top = sprite_y - 16; //it is subtracted with 16 as in gameboy sprite can be 16 pixel above
        // the visible screen and 8 pixel below visible screen ,so we check from very top
        int height = sprite_height(); //height of sprite depends on lcdc register
        int sprite_bottom = sprite_top + height;

        //LY register holds the information of current scanline,so now we have to check if our sprite
        //is in current scanline or not

        if (mem->LY >= sprite_top && mem->LY < sprite_bottom) {

            //if it meets we store the sprite in sprite table

            loadable_sprite_table[marker] = sprite_y;
            marker++;
            loadable_sprite_table[marker] = sprite_x;
            marker++;
            loadable_sprite_table[marker] = sprite_tile;
            marker++;
            loadable_sprite_table[marker] = sprite_attribute;
            marker++;

            if (marker >= 40) break; //cant store more than 10 sprite per scanline

        }

    }//for  i
 }

 void pixel_transfer () { //mode 3 
    

    mem->ppu_pixel_transfer = true;

    if (mem->LY >= 144) return; //pixel transfer will only happemn for  visible scanline
    //create pixel buffer
    draw_background_pixels();
    draw_window_pixel();
    draw_sprite_pixel();
    //load onto frame buffer for current scanline
    for (int i = 0; i < 160 ; i++) {
        frame_buffer [mem->LY] [i] = pixel_buffer[i];
    }
 }

 void h_blank() { //mode 0,resting phase after finshing 160 pixels for a  scanline
    uint8_t value = (mem->STAT & 0b11111100);
    mem->STAT = (value | 0b00);

    if (mem->STAT & 0b00001000) mem->IF |=  (1 << 1); //fire lcd stat interrupt

    mem->ppu_pixel_transfer = false;
    mem->oam_search = false;
    //eat 5 star and do nothing
 }

 void v_blank() { //mode 1
    uint8_t value = (mem->STAT & 0b11111100);
    mem->STAT = (value | 0b01);

    if (mem->STAT & 0b00010000) mem->IF |=  (1 << 1); //fire lcd stat interrupt

    if (mem->LY == 144) mem->IF |= (1 << 0); //fire vblank interrupt

 }

 void ppu_tick() {

    bool LYC_interrupt_enable = (mem->STAT & 0b01000000);

    if (mem->LY == mem->LYC) {

        mem->STAT |= 0b00000100;

        if (LYC_interrupt_enable) mem->IF |=  (1 << 1); //fire lcd stat interrupt

    }

    else mem->STAT &= 0b11111011; //else force thhat bit to 0

    if (internal_ppu_ticks == 456) {
        internal_ppu_ticks = 0;
        mem->LY++;

         if (mem->LY == 144) {
         v_blank();
         render_frame(frame_buffer);
         }

         else if (mem->LY == 154) mem->LY = 0;

    }

    if (internal_ppu_ticks == 0) oam_search();

    else if (internal_ppu_ticks == 80) pixel_transfer();

    else if (internal_ppu_ticks == 252) h_blank();

   internal_ppu_ticks++;

 }
 
};

struct game_clock { 
//this struct controls the gameclock and syncs memory cpu ppu and other components
 memory mem;
 cpu CPU;
 ppu PPU;

 game_clock() : CPU(&mem) , PPU (&mem) {

 }

//game_clock related variables :
const double main_clock = 4194304.0 ;  //gameboy main clock oscilates at 4.194304 million hertz per second
// so 1 clock cycle is 1/4.19mill seconds long

int ticks_remaining = 0; // Number of remaining T-cycles the CPU provided.it is Set to (M-cycles * 4) when an instruction starts.
// While > 0, the CPU cannot fetch a new opcode,but the global clock (PPU, timers, DMA) keeps running.
//this emulates the  sense of time that the actual cpu needs time to execute hardware tasks

 void execute_tick_cycle() {
   CPU.timer_tick();
   PPU.ppu_tick();
}

void clock_tick () {

       if (!ticks_remaining) {
        ticks_remaining = CPU.cpu_cycle();
       }

       while (ticks_remaining) {

        if (CPU.IME_scheduled) {
            if (!CPU.ei_delay) {
                CPU.IME = true;
            }
            else CPU.ei_delay--;
        }
          execute_tick_cycle();
          ticks_remaining--;

        }

}

};

void todo_list () {
// add halt, should execute before ld checks even starts : done
// add stop clock insruction later : done
// check for tac very specific edge case later (new enabled) : not done
// add conditions for ending stop : not done
}

    //renderer part with sfml 3.0

 sf::RenderWindow window (sf::VideoMode ( sf::Vector2u (160,144)),"gameboy_classic");
 sf::Texture texture;
 sf::Sprite sprite(texture);

 void init_renderer () {
    if(!texture.resize({160,144}))
        std::cout << "Texture failed\n";
        sprite.setTexture(texture,true);
 }
void render_frame(uint8_t framebuffer[144][160]) {
    static uint8_t pixels[160 * 144 * 4];

    for (int y = 0; y < 144; y++)
    {
        for (int x = 0; x < 160; x++)
        {
            uint8_t shade = framebuffer[y][x];

            uint8_t color;

            switch (shade)
            {
                case 0: color = 255; break;
                case 1: color = 170; break;
                case 2: color = 85; break;
                case 3: color = 0; break;
            }

            int i = (y * 160 + x) * 4;

            pixels[i+0] = color;
            pixels[i+1] = color;
            pixels[i+2] = color;
            pixels[i+3] = 255;
        }
    }

    texture.update(pixels);

    window.clear();
    window.draw(sprite);
    window.display();
}

int main()
{
    init_renderer();

    game_clock gameboy;

    while (window.isOpen())
    {
        while (auto event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        gameboy.clock_tick();
    }
}
