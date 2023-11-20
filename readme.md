cargo objcopy --release -- -O ihex out.hex
cargo objcopy --example teensy_ethernet --release -- -O ihex out.hex

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 out.hex -w

python3 -m serial.tools.miniterm /dev/ttyACM0

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 out.hex -w && sleep 1 && python3 -m serial.tools.miniterm /dev/ttyACM0

teensy_loader_cli --mcu TEENSY41 out.hex -w && sleep 1 && python3 -m serial.tools.miniterm /dev/tty.usbmodem2101

https://github.com/PaulStoffregen/teensy41_ethernet/blob/master/teensy41_ethernet.ino

MIMXRT1062DVJ6B

https://www.nxp.com/webapp/Download?colCode=IMXRT1060RM

2079-2250

Caveats:
- No IP CRC acceleration
- Only 100BASE-TX
- No Magic Packet handling
- No PTP (for now)
- No VLANs




## Lessons Learned
- smoltcp will only ever create one each TXToken and RXToken. Device implementations are meant to enforce this by having the token borrow something from the Device, so that multiple calls to transmit() or receive() will be prevented by the borrow checker.
- The ENET uDMA engine maintains pointers/counters to the last/next used TX and RX descriptor table entries. When TDAR is asserted, only the "next" descriptor is checked for readiness (and any subsequent until exahusted). Thus, you can't populate any free descriptor, you need to maintain your own pointer so that you populate the next entry. For RX, you don't need to scan the entire descriptor table for non-empty entries, just the "next" one.
- There may be data caching going on. I don't have a complete understanding, but out of an abundance of caution, I added atomic fences. More work needed.
- There's only 16k allocated for the stack in the teensy-rs runtime, and there's nothing to warn you if you break that limit. You should keep large data structures statically allocated.
