# IrRemote

> Currently WIP, created the library based from ESP-IDf RMT example. this library is created based on the ESP-IDF RMT example. It is part of the GoGo Board 7 firmware.

An infrared remote library designed for use with ESP32S3 using Arduino Core version 3. This library utilizes the RMT driver based on ESP-IDF 5.1.



## Behaviour notes

- **RX** re-arms the RMT channel from the done interrupt into the second of two small frame buffers
  (`IR_RX_BUFFER_COUNT` x `IR_RX_SYMBOL_BUF_SIZE` symbols), so the next NEC repeat code is captured
  even when the caller polls `getIrCode()` slowly. A frame that completes while the caller still
  holds the previous one is dropped (same as before).
- **TX** keeps the payload of every queued frame in a small ring (`IR_TX_PAYLOAD_SLOTS`); the RMT driver
  only stores a pointer and the encoder reads it later from the ISR. `sendNEC()` is still blocking
  once `IR_TX_QUEUE_DEPTH` frames are pending.
