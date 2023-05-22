cargo objcopy --release -- -O ihex hello-world.hex

/home/timv/teensy_loader_cli/teensy_loader_cli --mcu TEENSY41 hello-world.hex -w

python3 -m serial.tools.miniterm /dev/ttyACM0