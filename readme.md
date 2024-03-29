cargo objcopy --release -- -O ihex out.hex
cargo objcopy --example teensy_ethernet --release -- -O ihex out.hex

cargo b --example teensy_demo --features="teensy4"
cargo b --example rt1176_evk_demo --features="imxrt1170evk"

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 out.hex -w

python3 -m serial.tools.miniterm /dev/ttyACM0

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 out.hex -w && sleep 1 && python3 -m serial.tools.miniterm /dev/ttyACM0

cargo flash --example rt1176_evk_demo --features="imxrt1170evk" --release --target=thumbv7em-none-eabihf --chip=mimxrt1170 && miniterm /dev/ttyACM0 115200

cargo build --example=rt1176_evk_demo --features=imxrt1170evk --target=thumbv7em-none-eabihf



teensy_loader_cli --mcu TEENSY41 out.hex -w && sleep 1 && python3 -m serial.tools.miniterm /dev/tty.usbmodem2101

### EVK
export IMXRT_LOG_BUFFER_SIZE=8192
cargo build --example=rt1176_evk_demo --features=imxrt1170evk --target=thumbv7em-none-eabihf
pyocd load --target=mimxrt1170_cm7 --format=elf target/thumbv7em-none-eabihf/debug/examples/rt1176_evk_demo
pyocd reset --target=mimxrt1170_cm7 -m hw
pyocd gdbserver --target=mimxrt1170_cm7
miniterm /dev/ttyACM0 115200

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


cargo objcopy --features=teensy4 --target=thumbv7em-none-eabihf --example=teensy_demo --release -- -O ihex out.hex