.PHONY: run

run: .pio/build/uno/firmware.elf
	avr-gdb -b 115200 -ex "tar rem /dev/ttyACM0" -ex "mo dwc" -ex "lo" $<

.pio/build/uno/firmware.elf: src/*
	pio run
