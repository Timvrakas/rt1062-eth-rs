cargo objcopy --release -- -O ihex out.hex

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 out.hex -w

python3 -m serial.tools.miniterm /dev/ttyACM0

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 out.hex -w && sleep 1 && python3 -m serial.tools.miniterm /dev/ttyACM0

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
- No VLANs


Open questions
- How do I deal with unused or out-of-order descriptor table usage?
    - Does the TX/RX engine scan in order? does it cycle through the whole table?
        - it looks like it cycles the whole ring before giving up.
        - presumably it fills up the RXDT sequentially
            - but we could search exahasutively??
        - One solution here is to build a proper allocator that keeps track of table entries and clears them when they go out of scope
        - Other solution is to not gaurentee a table entry until the token is consumed
            - This feels wrong, but easy...
    - So for reference, the [stm32 implementation](https://github.com/stm32-rs/stm32-eth/blob/master/src/dma/smoltcp_phy.rs) does not actually allocate a descriptor, and is vulnerable to the caller requesting many tokens, and then being unable to redeem them all in a short time.
        - I suspect the smoltcp makes the implicit promise that it won't do this. But that's a bit weaksauce.

I think what I'll do here is implement similar to STM32, and then go ask how it's supposed to be done, and decide if I want to actually track allocation.


Note: The TX hardware does not search the TxDT exaustively when TDAR is asserted. If you write a packet into descriptor zero, and assert TDAR, it will send. If you write annother packet into descriptor 0, and assert TDAR again, it will not send, and descriptor zero will never be ready. It appears the TX hardware maintains a buffer that is only incremented after success.


New strategy:
Tokens do not reserve _specific_ buffer descriptors, we just increment a counter of tokens that are outstanding. Need a way to deal with unused descriptors.