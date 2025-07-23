# MultiMotor: C++ Library for motor control

[![Top Language](https://img.shields.io/github/languages/top/t413/multimotor?style=flat-square)](https://github.com/t413/multimotor)
[![Lines of Code](https://tokei.rs/b1/github/t413/multimotor?style=flat-square)](https://github.com/t413/multimotor/graphs/code-frequency)
[![GitHub Repo stars](https://img.shields.io/github/stars/t413/multimotor?style=flat-square)](https://github.com/t413/multimotor/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/t413/multimotor?style=flat-square)](https://github.com/t413/multimotor/network/members)
[![GitHub issues](https://img.shields.io/github/issues/t413/multimotor?style=flat-square)](https://github.com/t413/multimotor/issues)
[![Last commit](https://img.shields.io/github/last-commit/t413/multimotor?style=flat-square)](https://github.com/t413/multimotor/commits/main)
[![Users Total](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fdiscord.com%2Fapi%2Finvites%2FDqJNftD7Hw%3Fwith_counts%3Dtrue&query=%24.approximate_member_count&logo=discord&logoColor=white&label=Users&color=5865F2&style=flat-square)](https://3d.t413.com/go/discord?ref=gh-omni)
[![Users Online](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fdiscord.com%2Fapi%2Finvites%2FDqJNftD7Hw%3Fwith_counts%3Dtrue&query=%24.approximate_presence_count&label=Online&color=5865F2&style=flat-square)](https://3d.t413.com/go/discord?ref=gh-omni)
[![Motors: Multi](https://img.shields.io/badge/Motors-Multi-yellow?style=flat-square)](#)

_Used by my rover controller [OmniCtrl](https://t413.com/go/omnictrl?ref=multimotor) and [Dug the robot](https://t413.com/go/dug-mw?ref=multimotor)_

## Features:

- Supports different CanBus motors _on the same CAN bus_
  * ODrive and odrive-compatible motors
  * Xiaomi CyberGear motors
  * Robstride motors: in development
- **Environment independent** CAN support
  * Arduino, PlatformIO, ESP-IDF, linux, etc.
  * Includes support for ESP32's native TWAI CanBus library


_Join my [3D Design Discord](https://3d.t413.com/go/discord?ref=gh-omni) and say hi and talk shop!_

[![Users Total](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fdiscord.com%2Fapi%2Finvites%2FDqJNftD7Hw%3Fwith_counts%3Dtrue&query=%24.approximate_member_count&logo=discord&logoColor=white&label=Users&color=5865F2&style=flat-square)](https://3d.t413.com/go/discord?ref=gh-omni)
[![Users Online](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fdiscord.com%2Fapi%2Finvites%2FDqJNftD7Hw%3Fwith_counts%3Dtrue&query=%24.approximate_presence_count&label=Online&color=5865F2&style=flat-square)](https://3d.t413.com/go/discord?ref=gh-omni)


## Usage
- Include the library in your project (see below)
- Create a `CanInterface` object, for example the `CanEsp32Twai` class that's included:
  ```cpp
  CanEsp32Twai interface;
  interface.setup(PIN_CAN_RX, PIN_CAN_TX, 250000, &Serial);
  ```
- Create a driver object (on stack, heap, etc)
  ```cpp
  ODriveDriver omotor(ID1, &interface);
  CyberGearDriver cmotor(ID2, &interface);
  ```
- Handle incoming messages in your loop:
  ```cpp
  void loop() {
    // (other loop code)

    while (interface.available()) {
        CanMessage message = interface.readOne();
        bool handled = false;
        for (int i = 0; i < NUM_DRIVES; i++) {
            if (drives_[i]->handleIncoming(message.id, message.data, message.len, now))
                handled = true;
        }
    }
  }
  ```

## Installation

- If you use PlatformIO, add the following to your `platformio.ini`:
  ```ini
  lib_deps = t413/multimotor
  ```
- Or use via submodule:
  ```bash
  git submodule add https://github.com/t413/multimotor.git
  ```
