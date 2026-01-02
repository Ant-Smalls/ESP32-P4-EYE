# ESP32-P4 EYE Prototype (esp_p4_eye_proto)

This project is a prototype for interacting with the hardware on the ESP32-P4 EYE board. It demonstrates camera initialization, frame capture, ISP (Image Signal Processor) control, PDM audio recording, and serial communication for command-based image acquisition.

## Features

- **Camera Support:**
  - Initialization of OV2710 sensor via MIPI CSI.
  - Integration with the `esp_video` component for V4L2-like frame management.
  - Manual sensor detection and configuration for RAW10 output.
  - Frame capture and memory mapping using PSRAM.
- **ISP (Image Signal Processor) Control:**
  - Real-time adjustment of contrast, saturation, brightness, and hue via `ioctl`.
- **Audio Support:**
  - PDM microphone initialization using I2S.
  - 5-second audio recording capability stored in SPIRAM.
  - WAV header generation for PCM data.
- **Serial Communication:**
  - Custom command protocol over `USB_SERIAL_JTAG`.
  - Commands: `TAKE_PHOTO` (captures and sends a frame) and `SHUTDOWN`.
  - Reliable image data transmission with synchronization markers and headers.
- **Memory Management:**
  - Extensive use of SPIRAM for large buffers (frame buffers and audio).
  - DMA-capable allocation helpers in `psram_utils`.

## Hardware Requirements

- **ESP32-P4 EYE Board** (or similar ESP32-P4 based hardware with MIPI CSI camera and PDM microphone).
- **Camera Sensor:** OV2710 (connected via MIPI CSI).
- **Microphone:** PDM microphone (connected via I2S).
- **External RAM:** SPIRAM (PSRAM) is required for frame and audio buffering.

## Project Structure

- `main/main.c`: Main application loop and command handler.
- `main/camera_init.c`: Hardware-level camera initialization (XCLK, GPIO, I2C) and `esp_video` setup.
- `main/camera_sensor.c`: OV2710 specific detection and register configuration.
- `main/camera_capture.c`: High-level frame capture logic.
- `main/esp32_p4_eye_audio.c`: PDM audio recording and WAV formatting.
- `main/serial_utils.c`: Serial communication driver for commands and data transfer.
- `main/psram_utils.c`: Memory allocation utilities for SPIRAM.
- `main/camera_init.h`: Configuration and hardware definitions (pins, resolution, etc.).

## How to Use

### 1. Build and Flash

Standard ESP-IDF build process:

```bash
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

### 2. Sending Commands

The application listens for commands over the USB Serial/JTAG interface. Commands must follow a specific format: `[FRAME_START]CMD:[COMMAND_NAME][FRAME_END]`.

- **To capture a photo:**
  Send `TAKE_PHOTO` command. The ESP32-P4 will capture a frame and stream it back over serial.
- **To shutdown:**
  Send `SHUTDOWN` command to gracefully de-initialize hardware.

### 3. Image Data

When a photo is captured, it is sent over serial in the following format:
1. Sync Marker: `0xFF 0xFF`
2. Header: `[FRAME_START][Width LSB][Width MSB][Height LSB][Height MSB][FRAME_END]`
3. Raw Data: Frame data chunks.

## Technical Details

### Camera Pipeline
The project utilizes the `esp_video` component which provides a Linux-like V4L2 interface. The pipeline involves:
1. Initializing I2C and XCLK for the sensor.
2. Powering on and resetting the sensor.
3. Detecting the OV2710 via SCCB (I2C).
4. Initializing the `esp_video` CSI driver.
5. Opening `/dev/video0` and requesting buffers in MMAP mode.
6. Starting the stream and dequeuing/enqueuing buffers for capture.

### Audio Pipeline
Audio is captured via the PDM RX mode of the I2S peripheral. It uses the `esp_codec_dev` and `esp_audio_codec` components to manage the microphone interface.

