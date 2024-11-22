#![allow(dead_code)]
use std::{fs::OpenOptions, io::{stdin, stdout, Read, Write}, mem::MaybeUninit, ops::{Index, IndexMut}, os::fd::RawFd, sync::Arc};

use libc::{fd_set, select, timeval, FD_SET, FD_ZERO};
use termios::{tcgetattr, tcsetattr, Termios, ECHO, ICANON, TCSANOW};

const REG_COUNT: usize = 10;
const FL_POS: u16 = 1u16 << 0;
const FL_ZRO: u16 = 1u16 << 1;
const FL_NEG: u16 = 1u16 << 2;

fn get_char() -> char {
    let mut buf = [0u8; 1];                    
    stdin().lock().read_exact(&mut buf).expect("Could not read byte from stdin");
    buf[0] as char
}

pub struct Memory {
    inner: [u16; u16::MAX as usize]
}

impl Memory {
    pub fn new() -> Self {
        Self {
            inner: [0;u16::MAX as usize]
        }
   }

    fn read(&mut self, offset: u16) -> u16 {
        if offset == MR::KBSR as u16 {
            if check_key() > 0 {
                self[MR::KBSR as u16] = 1 << 15;
                self[MR::KBDR as u16] = get_char() as u16;
            } else {
                self[MR::KBSR as u16] = 0;
            }
        }  
        self[offset]
    }
}

impl Index<R> for Memory {
    type Output = u16;

    fn index(&self, index: R) -> &Self::Output {
        self.inner.get(index as usize).unwrap()
    }
}
impl IndexMut<R> for Memory {
    fn index_mut(&mut self, index: R) -> &mut Self::Output {
        self.inner.get_mut(index as usize).unwrap()
    }
}

impl Index<u16> for Memory {
    type Output = u16;

    fn index(&self, index: u16) -> &Self::Output {
        self.inner.get(index as usize).unwrap()
    }
}
impl IndexMut<u16> for Memory {
    fn index_mut(&mut self, index: u16) -> &mut Self::Output {
        self.inner.get_mut(index as usize).unwrap()
    }
}

#[derive(Debug)]
#[repr(u16)]
enum R {
    R0 = 0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    PC,
    COND,
}


#[repr(u16)]
enum MR {
    KBSR = 0xFE00,
    KBDR = 0xFE02
}


#[derive(Debug)]
#[repr(u16)]
enum OpCodes {
    BR = 0, // branch,
    ADD,    // add 
    LD,     // load
    ST,     // store
    JSR,    // jump register
    AND,    // bitwise and
    LDR,    // load register
    STR,    // store register
    RTI,    // unused
    NOT,    // bitwise not
    LDI,    // load indirect
    STI,    // store indirect
    JMP,    // jump
    RES,    // reserved (unused)
    LEA,    // load effective address
    TRAP    // execute trap
}


impl TryFrom<u16> for OpCodes {
    type Error = String;
    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(OpCodes::BR), 
            1 => Ok(OpCodes::ADD), 
            2 => Ok(OpCodes::LD), 
            3 => Ok(OpCodes::ST), 
            4 => Ok(OpCodes::JSR),
            5 => Ok(OpCodes::AND),
            6 => Ok(OpCodes::LDR),
            7 => Ok(OpCodes::STR),
            8 => Ok(OpCodes::RTI),
            9 => Ok(OpCodes::NOT),
            10 => Ok(OpCodes::LDI),
            11 => Ok(OpCodes::STI),
            12 => Ok(OpCodes::JMP),
            13 => Ok(OpCodes::RES),
            14 => Ok(OpCodes::LEA),
            15 => Ok(OpCodes::TRAP),
            _ => Err("Bad OpCode".to_string())
        }
    }
}

#[derive(Debug)]
#[repr(u16)]
enum T {
    GETC = 0x20, //get character from keyboard, not echoed onto the terminal
    OUT = 0x21, //output character
    PUTS = 0x22, //output a word string
    IN = 0x23, //get character from keyboard, echoed onto the terminal
    PUTSP = 0x24, //output a byte string
    HALT = 0x25, //halt the program
}

impl TryFrom<u16> for T {
    type Error = String;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0x20 => Ok(T::GETC),
            0x21 => Ok(T::OUT),
            0x22 => Ok(T::PUTS),
            0x23 => Ok(T::IN),
            0x24 => Ok(T::PUTSP),
            0x25 => Ok(T::HALT),
            _ => Err("Bad Trap code".to_string())
        }
    }
}


pub struct Reg {
    inner: [u16; REG_COUNT]
}

impl Reg {
    pub fn new() -> Self {
        Self {
            inner: [0;REG_COUNT]
        }
        
    }
}
impl Index<R> for Reg {
    type Output = u16;

    fn index(&self, index: R) -> &Self::Output {
        self.inner.get(index as usize).unwrap()
    }
}
impl IndexMut<R> for Reg {
    fn index_mut(&mut self, index: R) -> &mut Self::Output {
        self.inner.get_mut(index as usize).unwrap()
    }
}

impl Index<u16> for Reg {
    type Output = u16;

    fn index(&self, index: u16) -> &Self::Output {
        self.inner.get(index as usize).unwrap()
    }
}
impl IndexMut<u16> for Reg {
    fn index_mut(&mut self, index: u16) -> &mut Self::Output {
        self.inner.get_mut(index as usize).unwrap()
    }
}

fn read_file(file: &str, mem: &mut Memory) -> Result<(),std::io::Error> {
    println!("Reading file {file}");
    let mut file = OpenOptions::new().read(true).open(file)?;
    
    let mut bytes: [u8; 2] = [0; 2];
    
    file.read_exact(&mut bytes)?;
    let mut origin = u16::from_be_bytes(bytes);
    loop  {
        match file.read_exact(&mut bytes) {
            Ok(_) => {
                mem[origin] = u16::from_be_bytes(bytes);
                origin += 1;
            },
            Err(_) => {
                break;
            },
        }
    }
    Ok(())
}

fn sign_extend(mut x: u16, bits: usize) -> u16 {

    if ((x >> (bits - 1)) & 1) > 0 {
        x |= 0xFFFF << bits;
    }
    x
}

fn update_flags(reg: &mut Reg, r: u16) {
    if reg[r] == 0 {
        reg[R::COND] = FL_ZRO;
    } else if reg[r] >> 15 > 0 {
        reg[R::COND] = FL_NEG;
    } else {
        reg[R::COND] = FL_POS;
    }
    
}


fn add(reg: &mut Reg, instr: u16) {
    //destination register DR
    let r0: u16 = (instr >> 9) & 0x7; 
    //first operand SR1
    let r1: u16 = (instr >> 6) & 0x7;
    //check mode
    let imm_flag = (instr >> 5) & 0x1;

    if imm_flag > 0 {
        let imm5 = sign_extend(instr & 0x1F, 5);
        let (val, _) = reg[r1].overflowing_add(imm5); 
        reg[r0] = val;
    } else {
        let r2 = instr & 0x7;
        let (val, _) = reg[r1].overflowing_add(reg[r2]); 
        reg[r0] = val;
    }
    update_flags(reg, r0);
}

fn and(reg: &mut Reg, instr: u16) {
    let r0: u16 = (instr >> 9) & 0x7; 
    let r1: u16 = (instr >> 6) & 0x7;
    let imm_flag = (instr >> 5) & 0x1;

    if imm_flag > 0 {
        let imm5 = sign_extend(instr & 0x1F, 5);
        reg[r0] = reg[r1] & imm5;
    } else {
        let r2 = instr & 0x7;
        reg[r0] = (reg[r1] & reg[r2]);
    }
    update_flags(reg, r0);
}

fn ld(reg: &mut Reg, mem: &mut Memory, instr: u16) {
    let r0: u16 = (instr >> 9) & 0x7;
    let pc_offset = sign_extend(instr & 0x1FF, 9);
    reg[r0] = mem.read(reg[R::PC].overflowing_add(pc_offset).0);
    update_flags(reg, r0);
}

fn ldi(reg: &mut Reg, mem: &mut Memory, instr: u16) {
    //destination register DR
    let r0: u16 = (instr >> 9) & 0x7;
    let pc_offset = sign_extend(instr & 0x1FF, 9);
    let idx = {
        mem.read(reg[R::PC].overflowing_add(pc_offset).0)
    };
    reg[r0] = mem.read(idx);
    update_flags(reg, r0);
}

fn br(reg: &mut Reg, instr: u16) {
    let pc_offset = sign_extend(instr & 0x1FF, 9);
    let cond_flag = (instr >> 9) & 0x7;
    if cond_flag & reg[R::COND] > 0 {
        reg[R::PC] = reg[R::PC].overflowing_add(pc_offset).0;
    }
}

fn st(reg: &mut Reg, mem: &mut Memory, instr: u16) {
    let r0 = (instr >> 9) & 0x7;
    let pc_offset = sign_extend(instr & 0x1FF, 9);
    mem[reg[R::PC].overflowing_add(pc_offset).0] = reg[r0];
}

fn jsr(reg: &mut Reg, instr: u16) {
    let cond_flag = (instr >> 11) & 1;
    reg[R::R7] = reg[R::PC];
    if cond_flag > 0 {
        let pc_offset = sign_extend(instr & 0x7FF, 11);
        reg[R::PC] = reg[R::PC].overflowing_add(pc_offset).0;
    } else {
        let r1 = (instr >> 6) & 0x7;
        reg[R::PC] = reg[r1];
    }
}

fn ldr(reg: &mut Reg, mem: &mut Memory, instr: u16) {

    let r0 = (instr >> 9) & 0x7;
    let r1 = (instr >> 6) & 0x7;
    let pc_offset = sign_extend(instr & 0x3F, 6);

    reg[r0] = mem.read(reg[r1].overflowing_add(pc_offset).0);
    update_flags(reg, r0);
}

fn str_(reg: &Reg, mem: &mut Memory, instr: u16) {
    let r0 = (instr >> 9) & 0x7;
    let base_r = (instr >> 6) & 0x7;
    let offset = sign_extend(instr & 0x3F, 6);
    let (idx, _) = reg[base_r].overflowing_add(offset);
    mem[idx] = reg[r0];
}

fn not(reg: &mut Reg, instr: u16) {
    let r0 = (instr >> 9) & 0x7;
    let r1 = (instr >> 6) & 0x7;
    reg[r0] = !reg[r1];
    update_flags(reg, r0);
}

fn sti(reg: &Reg, mem: &mut Memory, instr: u16) {
    let r0 = (instr >> 9) & 0x7;
    let pc_offset = sign_extend(instr & 0x1FF, 9);
    let mem_loc = {
        mem.read(reg[R::PC] + pc_offset)
    };
    mem[mem_loc] = reg[r0];
}

fn jmp(reg: &mut Reg, instr: u16) {
    let r1 = (instr >> 6) & 0x7;
    reg[R::PC] = reg[r1];
}

fn lea(reg: &mut Reg, instr: u16) {
    let r0 = (instr >> 9) & 0x7;
    let pc_offset = sign_extend(instr & 0x1FF, 9);
    reg[r0] = reg[R::PC] + pc_offset;
    update_flags(reg, r0);
}

fn disable_input_buffer(orig_tio: &mut Termios) -> Result<(), std::io::Error> {
    tcgetattr(RawFd::from(libc::STDIN_FILENO), orig_tio)?;
    let new_tio = orig_tio;
    new_tio.c_lflag &= !ICANON & !ECHO;
    tcsetattr(RawFd::from(libc::STDIN_FILENO), TCSANOW, new_tio)?;
    Ok(())
}

fn restore_input_buffering(orig_tio: &Termios) -> Result<(), std::io::Error> {
    tcsetattr(libc::STDIN_FILENO, TCSANOW, orig_tio)?;
    Ok(())
}

fn check_key() -> u16 {
    unsafe {
        let readfds: *mut fd_set = MaybeUninit::zeroed().as_mut_ptr();
        FD_ZERO(readfds);
        FD_SET(libc::STDIN_FILENO, readfds);
        let timeout: *mut timeval = MaybeUninit::zeroed().as_mut_ptr();
        (*timeout).tv_sec = 0;
        (*timeout).tv_usec = 0;
        select(1, readfds, std::ptr::null_mut(), std::ptr::null_mut(), timeout) as u16
    }
}

fn main() {
    let mut orig_term = Termios::from_fd(RawFd::from(libc::STDIN_FILENO)).expect("Error setting up termios");
    
    disable_input_buffer(&mut orig_term).expect("Error disabling input buffer");
    let orig_tio = Arc::new(orig_term);

    let ot = Arc::clone(&orig_tio);
   ctrlc::set_handler(move || {
       println!("Shutting down...");
        restore_input_buffering(&*ot).expect("Error restoring input buffer");

    std::process::exit(-1)
   }).expect("Error setting SIGINT handler");
    

    let mut args = std::env::args();
    let program = args.next().unwrap();
    let file_path = if args.len() == 1 {
        args.next().unwrap()
    } else {
        println!("Usage: {program} [image file]");
        std::process::exit(1)
    };

    let mut mem = Memory::new();
    read_file(&file_path, &mut mem).expect("Reading program");
    let mut reg = Reg::new();
    reg[R::COND] = FL_ZRO;
    let pc_start = 0x3000;
    reg[R::PC] = pc_start;

   loop {
       let pc = reg[R::PC];
       reg[R::PC] += 1;
       let instr = mem[pc]; 
       let op = OpCodes::try_from(instr >> 12);

       match op {
           Ok(OpCodes::BR)   => {
               br(&mut reg, instr);
           },
           Ok(OpCodes::ADD)  => {
               add(&mut reg, instr);
           },
           Ok(OpCodes::LD)   => {
               ld(&mut reg, &mut mem, instr);
           },
           Ok(OpCodes::ST)   => {
               st(&mut reg, &mut mem, instr);
           },
           Ok(OpCodes::JSR)  => {
               jsr(&mut reg, instr);
           },
           Ok(OpCodes::AND)  => {
               and(&mut reg, instr);
           },
           Ok(OpCodes::LDR)  => {
               ldr(&mut reg, &mut mem, instr);
           },
           Ok(OpCodes::STR)  => {
               str_(&reg, &mut mem, instr);
           },
           Ok(OpCodes::NOT)  => {
               not(&mut reg, instr);
           },
           Ok(OpCodes::LDI)  => {
               ldi(&mut reg, &mut mem, instr);
           },
           Ok(OpCodes::STI)  => {
               sti(&reg, &mut mem, instr);
           },
           Ok(OpCodes::JMP)  => {
               jmp(&mut reg, instr);
           },
           Ok(OpCodes::LEA)  => {
               lea(&mut reg, instr);
           },
           Ok(OpCodes::TRAP) => {
               reg[R::R7] = reg[R::PC];
               let trap = T::try_from(instr & 0xFF);
               match trap {
                   Ok(T::GETC) => {
                       let ch = get_char();
                       reg[R::R0] = ch as u16;
                       update_flags(&mut reg, R::R0 as u16);
                   },
                   Ok(T::OUT) => {
                       let mut stdout = stdout().lock();
                       print!("{}", reg[R::R0] as u8 as char);
                       stdout.flush().unwrap();
                   },
                   Ok(T::PUTS) => {
                       unsafe {
                           let mut ptr = mem.inner.as_ptr().offset(reg[R::R0] as isize);
                           while ptr.read() != 0 {
                               print!("{}", (*ptr) as u8 as char);
                               ptr = ptr.add(1);
                           }
                           stdout().flush().unwrap();
                       }
                   },
                   Ok(T::IN) => {
                       print!("Enter a character: ");
                       let ch = get_char();
                       print!("{}", ch);
                       reg[R::R0] = ch as u8 as u16;
                       update_flags(&mut reg, R::R0 as u16);
                   },
                   Ok(T::PUTSP) => {

                      unsafe {
                          let mut ptr = mem.inner.as_ptr().offset(reg[R::R0] as isize);
                          while ptr.read() != 0 {
                               
                              let mut ch = ((*ptr) & 0xFF) as u8 as char;
                              ch = ((*ptr) >> 8) as u8 as char;
                              if ch != '\0' {
                                  println!("{ch}");
                              }
                              ptr = ptr.add(1);
                          }
                      }
                   },
                   Ok(T::HALT) => {
                       print!("HALT");
                       break;
                   },
                   Err(_) => unreachable!("Unknown TRAP code")
               }
           },
           Ok(OpCodes::RES)  => {},
           Ok(OpCodes::RTI)  => {},
           Err(err) => {
               panic!("{err}");
           }
       }
 //      std::thread::sleep(Duration::from_millis(500));
   }
}

