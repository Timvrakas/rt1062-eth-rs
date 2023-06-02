cargo objcopy --release -- -O ihex hello-world.hex

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 hello-world.hex -w

python3 -m serial.tools.miniterm /dev/ttyACM0

https://github.com/PaulStoffregen/teensy41_ethernet/blob/master/teensy41_ethernet.ino

MIMXRT1062DVJ6B

https://www.nxp.com/webapp/Download?colCode=IMXRT1060RM

2079-2250

Caveats:
- No IP CRC acceleration
- Only 100BASE-TX
- No Magic Packet handling
- No PTP (for now)
- Basic Descriptor (for now)