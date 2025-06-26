# Raspberry KMBox

This is a lil kmbox implementation on the raspberry pi pico 1 and 2. It uses dual cores, 1 dedicated to host stack and one dedicated to the device stack. The rp2350 build also has a full  hardware accelerator using PIO.

## Devices supported

I really only test this on an adafruit feather rp2040 usb host an adafruit rp2350 hstx. It should run on any pico though.

### RP2350

Faster cores and pio hardware acceleration. But doesn't have control over the usb port power and doesn't support hot plugging device. (This goes for most picos). There is a waveshare usb host which I think can toggle the power pin, but I havent tested.

### RP2040

Generally slower, but the adafruit rp2040 usb host can toggle the usb port power on and off and supports hot plugging devices.

## Quickstart

### Software

Install the pico sdk, then pull down the repo and run ./build and youre set. Runs on any os.

### Hardware

The rp2040 doesn't need any changes.
The rp2350 needs a usb host breakout connected with power connected to the usb pin, ground to ground, and then dm and dp connected to pins 10 and 11.
Serial on both boards through uart on the default pins, the board will run with or without it.

## Is iT PaSsThRouGh

PID and VID are mirrored properly, some string descriptors are. It's as passthrough as any other non-fpga dual usb stack device.

(Still a WIP)
