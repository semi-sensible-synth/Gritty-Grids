// Copyright 2012 Emilie Gillet.
//
// Author: Emilie Gillet (emilie.o.gillet@gmail.com)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef GRIDS_HARDWARE_CONFIG_H_
#define GRIDS_HARDWARE_CONFIG_H_

#include "avrlib/base.h"
#include "avrlib/gpio.h"
#include "avrlib/parallel_io.h"
#include "avrlib/serial.h"
#include "avrlib/spi.h"

namespace grids {

enum AdcChannel {
  ADC_CHANNEL_X_CV,
  ADC_CHANNEL_Y_CV,
  ADC_CHANNEL_RANDOMNESS_CV,
  ADC_CHANNEL_BD_DENSITY_CV,
  ADC_CHANNEL_SD_DENSITY_CV,
  ADC_CHANNEL_HH_DENSITY_CV,
  ADC_CHANNEL_TEMPO,
  ADC_CHANNEL_LAST
};

enum LedBits {
  LED_CLOCK = 1,
  LED_BD = 8,
  LED_SD = 4,
  LED_HH = 2,
  LED_ALL = LED_CLOCK | LED_BD | LED_SD | LED_HH
};

enum InputBits {
  INPUT_CLOCK = 2,
  INPUT_RESET = 4,
  INPUT_SW_RESET = 8
};

using avrlib::Gpio;
using avrlib::DigitalInput;
using avrlib::ParallelPort;
using avrlib::PortB;
using avrlib::PortD;
using avrlib::Serial;
using avrlib::SerialPort0;
using avrlib::SpiMaster;
//using avrlib::BufferedSoftwareSerialOutput;

typedef ParallelPort<PortD, avrlib::PARALLEL_NIBBLE_HIGH> Leds;   // D4, D5, D6, D7

// Replaced by GPIOs below, since D1 in now used for MIDI OUT (on SerialPort0/TX/D1)
//typedef ParallelPort<PortD, avrlib::PARALLEL_NIBBLE_LOW> Inputs;  // (D0), D1, D2, D3

typedef Gpio<PortD, 2> _resetPin;  // D2 (PD2)
typedef Gpio<PortD, 3> _buttonPin; // D3 (PD3)
typedef Gpio<PortB, 0> _clockPin;  // D8 (PB0) (moved from PD1)

typedef DigitalInput<_resetPin> ResetInput;  // D2 (PD2)
typedef DigitalInput<_buttonPin> ButtonInput; // D3 (PD3)
typedef DigitalInput<_clockPin> ClockInput;  // D8 (PB0) (moved from PD1)

typedef SpiMaster<Gpio<PortB, 2>, avrlib::MSB_FIRST, 2> ShiftRegister;

// typedef Serial<SerialPort0, 31250, avrlib::POLLED, avrlib::DISABLED> MidiInput;
// MIDI serial is now in and out (PD0/PD1)
typedef Serial<SerialPort0, 31250, avrlib::POLLED, avrlib::POLLED> MidiIO;

// Alternative - use buffered software serial as MIDI out ?
// (if this works well enough, we also get some flexibility with hardware pins)
// BufferedSoftwareSerialOutput<Gpio<PortD, 1>, 31250, 4800, 16> MidiOut;
// MIDI In (with Rx enabled, Tx disabled)
//typedef Serial<SerialPort0, 31250, avrlib::POLLED, avrlib::DISABLED> MidiIO;

static uint8_t midi_channel = 9;  // channel 10

}  // namespace grids

#endif  // GRIDS_HARDWARE_CONFIG_H_
