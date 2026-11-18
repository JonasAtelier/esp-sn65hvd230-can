# Classic CAN on ESP32 with an SN65HVD230

Send and receive Classic CAN frames using the ESP32’s built-in TWAI controller and a 3.3 V SN65HVD230 transceiver. Start with the receive example to see identifiers and data from another node on your bench bus. Hardware validation is pending.

## Receive your first frame

1. Use an ESP-IDF 5.x application targeting an ESP32 with TWAI. Copy this checkout into `components/hvd230_can/`. Use the [receive example](examples/basic_receive.c) as your main component’s `app_main` source.
2. For the example, connect transceiver `D` (TXD) to GPIO 5 and `R` (RXD) to GPIO 4. Connect VCC to 3.3 V, GND to ground and RS to ground for normal high-speed operation. Check your breakout’s labels. [Transceiver datasheet](doc/sn65hvd230.pdf)
3. Connect CANH to CANH and CANL to CANL on a second node with a shared ground reference. Use one 120 Ω terminator at each physical end of the bus, counting any resistors already installed. Configure both nodes for 500 kbit/s, the receive example’s default.
4. From the application root, run `idf.py build flash monitor` with your board connected. When the other node transmits, the monitor prints the frame ID and payload. A quiet bus produces no frame lines. A second ESP32 can run the [transmit example](examples/basic_transmit.c).

The examples are source files to integrate into an application. For more than one CAN controller, use ESP-IDF 5.2+ within the 5.x series, a target with multiple controllers and a transceiver per bus.

[Wiring, filtering and bus-off recovery](CAN.md) · [CAN API](include/hvd230can.h) · [Monitoring and callbacks](examples/callbacks_health.c) · [MIT license](LICENSE)
