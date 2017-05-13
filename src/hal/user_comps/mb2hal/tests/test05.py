#!/usr/bin/env python
# MB2HAL Validation tool for HAL_MAP_PIN feature

import sys
import subprocess
import os
import signal
import atexit
import commands
import re
import ctypes

import time
import logging
import threading

import modbus_tk
import modbus_tk.defines as cst
import modbus_tk.modbus as modbus
import modbus_tk.modbus_tcp as modbus_tcp

def read_hal_pin(name):
    rc = commands.getoutput("halcmd show pin %s | tail -2" % (name))
    v = re.split('[ ]+', rc)[4];
    if v == "TRUE": return 1;
    if v == "FALSE": return 0;
    if (len(v) < 2 or v[1] != 'x'):
        return float(v)
    else:
        return int(v, 0)

def write_hal_pin(name, value):
    os.system("halcmd setp %s %s" % (name, value))
        
def check_hal_pin(name, awaited):
    v = read_hal_pin(name)
    if v: rc = abs((v - awaited)/v) > 1e-3
    else: rc = abs(v - awaited) > 1.0e-6
    if rc: status = "Error"
    else: status = "OK"
    print status, "pin", name,  v, awaited
    return rc

def check_reg(name, v, awaited):
    rc = (v != awaited)
    if rc: status = "Error"
    else: status = "OK"
    print status, "pin", name, v, awaited
    return rc

def map16(v, scale, offset):
    v = v - offset
    v = v/scale
    return (ctypes.c_uint16(int(v)).value, )

def map32(v, scale, offset):
    v = v - offset
    v = v/scale
    v = ctypes.c_uint32(int(v)).value
    return ( v>>16, v & 0xFFFF )
           
logger = modbus_tk.utils.create_logger(name="console", record_format="%(message)s")

server = modbus_tcp.TcpServer(port=8502)
server.start()

slave = server.add_slave(1)
slave.add_block('in', cst.DISCRETE_INPUTS, 100, 32)
slave.add_block('coils', cst.COILS, 0, 32)
slave.add_block('analog', cst.ANALOG_INPUTS, 0, 32)
slave.add_block('hreg', cst.HOLDING_REGISTERS, 0, 32)
slave.add_block('single-coil', cst.COILS, 64, 1)

slave.set_values('in', 100, ( 1, 0) )

if len(sys.argv) > 1:   # Server mode
    while True:
        time.sleep(1)
        print "analog ", slave.get_values('analog', 0, 16)
        print "hreg   ", slave.get_values('hreg', 0, 32)
    exit(0)
    
halrun = subprocess.Popen(['halrun', '-If', 'test05.hal'], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
time.sleep(1)

err = False;

# Check MODBUS read input registers

scale = 0.1
offset = 50
f16 = -723
s16 = -723
u16 = 6500
slave.set_values('analog', 0, map16(f16, scale, offset) );
slave.set_values('analog', 1, map16(s16, scale, offset) );
slave.set_values('analog', 2, map16(u16, scale, offset) );

f32 = -1e7
s32 = int(-1e7)
u32 = int(4e8)
slave.set_values('analog',  4, map32(f32, scale, offset) );
slave.set_values('analog',  6, map32(s32, scale, offset) );
slave.set_values('analog',  8, map32(u32, scale, offset) );
slave.set_values('analog', 10, map32(300, 0.059, 0) );

slave.set_values('analog', 16, 4242 );

time.sleep(0.2)

err |= check_hal_pin("mb2hal.analog.pin-0", f16)
err |= check_hal_pin("mb2hal.analog.pin-1", s16)
err |= check_hal_pin("mb2hal.analog.pin-2", u16)
err |= check_hal_pin("mb2hal.analog.pin-4", f32)
err |= check_hal_pin("mb2hal.analog.pin-6", s32)
err |= check_hal_pin("mb2hal.analog.pin-8", u32)
err |= check_hal_pin("mb2hal.analog.pin-10", 300)
err |= check_hal_pin("mb2hal.analog2.00", 4242)

# Check MODBUS write holding registers

write_hal_pin("mb2hal.hreg.pin-0", f16)
write_hal_pin("mb2hal.hreg.pin-1", s16)
write_hal_pin("mb2hal.hreg.pin-2", u16)
write_hal_pin("mb2hal.hreg.pin-4", f32)
write_hal_pin("mb2hal.hreg.pin-6", s32)
write_hal_pin("mb2hal.hreg.pin-8", u32)
write_hal_pin("mb2hal.hreg2.00",   1234)
time.sleep(0.2)

err |= check_reg("mb2hal.hreg.pin-0", slave.get_values('hreg', 0, 1), map16(f16, 1/scale, -offset/scale))
err |= check_reg("mb2hal.hreg.pin-1", slave.get_values('hreg', 1, 1), map16(s16, 1/scale, -offset/scale))
err |= check_reg("mb2hal.hreg.pin-2", slave.get_values('hreg', 2, 1), map16(u16, 1/scale, -offset/scale))
err |= check_reg("mb2hal.hreg.pin-4", slave.get_values('hreg', 4, 2), map32(f32, 1/scale, -offset/scale))
err |= check_reg("mb2hal.hreg.pin-6", slave.get_values('hreg', 6, 2), map32(s32, 1/scale, -offset/scale))
err |= check_reg("mb2hal.hreg.pin-8", slave.get_values('hreg', 8, 2), map32(u32, 1/scale, -offset/scale))
err |= check_reg("mb2hal.hreg2.00",   slave.get_values('hreg', 16, 1), ( 1234, ))

# Check MODBUS read discrete inputs

slave.set_values('in', 100, ( 0, 1) )
slave.set_values('in', 116, ( 0, 1) )
time.sleep(0.2)
err |= check_hal_pin("mb2hal.in.pin-0", 0)
err |= check_hal_pin("mb2hal.in.pin-1", 1)
err |= check_hal_pin("mb2hal.in2.00", 0)
err |= check_hal_pin("mb2hal.in2.01", 1)

slave.set_values('in', 100, ( 1, 0) )
slave.set_values('in', 116, ( 1, 0) )
time.sleep(0.2)
err |= check_hal_pin("mb2hal.in.pin-0", 1)
err |= check_hal_pin("mb2hal.in.pin-1", 0)
err |= check_hal_pin("mb2hal.in2.00", 1)
err |= check_hal_pin("mb2hal.in2.01", 0)

# Check MODBUS write multiple coils

write_hal_pin("mb2hal.coils.pin-0", 0)
write_hal_pin("mb2hal.coils.pin-1", 1)
write_hal_pin("mb2hal.coils.pin-2", 0)
write_hal_pin("mb2hal.coils.pin-3", 0)
write_hal_pin("mb2hal.coils2.00", 0)
write_hal_pin("mb2hal.coils2.01", 1)
time.sleep(0.2)
err |= check_reg("mb2hal.coils.pin-0", slave.get_values('coils', 0, 4), ( 0, 1, 0, 0 ))
err |= check_reg("mb2hal.coils.coisl2.00", slave.get_values('coils', 16, 2), ( 0, 1))

write_hal_pin("mb2hal.coils.pin-0", 1)
write_hal_pin("mb2hal.coils.pin-1", 0)
write_hal_pin("mb2hal.coils.pin-2", 0)
write_hal_pin("mb2hal.coils.pin-3", 0)
write_hal_pin("mb2hal.coils2.00", 1)
write_hal_pin("mb2hal.coils2.01", 0)
time.sleep(0.2)
err |= check_reg("mb2hal.coils.pin-0", slave.get_values('coils', 0, 4), ( 1, 0, 0, 0 ))
err |= check_reg("mb2hal.coils.coisl2.00", slave.get_values('coils', 16, 2), ( 1, 0))

write_hal_pin("mb2hal.coils.pin-0", 0)
write_hal_pin("mb2hal.coils.pin-1", 0)
write_hal_pin("mb2hal.coils.pin-2", 1)
write_hal_pin("mb2hal.coils.pin-3", 0)
time.sleep(0.2)
err |= check_reg("mb2hal.coils.pin-0", slave.get_values('coils', 0, 4), ( 0, 0, 1, 0 ))

write_hal_pin("mb2hal.coils.pin-0", 0)
write_hal_pin("mb2hal.coils.pin-1", 0)
write_hal_pin("mb2hal.coils.pin-2", 0)
write_hal_pin("mb2hal.coils.pin-3", 1)
time.sleep(0.2)
err |= check_reg("mb2hal.coils.pin-0", slave.get_values('coils', 0, 4), ( 0, 0, 0, 1 ))

# Check MODBUS write single coil

write_hal_pin("mb2hal.single-coil.pin-0", 0)
time.sleep(0.2)
err |= check_reg("mb2hal.single-coil.pin-0", slave.get_values('single-coil', 64, 1), ( 0, ))

write_hal_pin("mb2hal.single-coil.pin-0", 1)
time.sleep(0.2)
err |= check_reg("mb2hal.single-coil.pin-0", slave.get_values('single-coil', 64, 1), ( 1, ))

# End of test

os.system("halcmd show pin")
halrun.stdin.write("quit\n")
halrun.stdin.close()
time.sleep(1)
server.stop()

if err: 
    print "TEST FAIL"
    exit(1)
else: 
    print "TEST PASS"
    exit(0)

