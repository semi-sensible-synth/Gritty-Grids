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


// Modified by Sonic Insurgence, June 2021
//
// IMPROVED MIDI IMPLEMENTATION
//
// Set the tempo knop to its minimum position. After receiving a 
// MIDI Start or MIDI Continue message Grids switches into a 
// "clocked_by_midi" mode. In this mode Grids advances with every 
// MIDI clock message, whereas a rising edge at its clock input
// no longer has an effect. The MIDI Start message also resets 
// the Grids engine.
// As an external reset in "clocked_by_midi" mode would break 
// the synchronization with the external MIDI master device the 
// function of the reset input as well as the reset button changes. 
// They now assume a "mute" functionality that suppresses all drum 
// and accent outputs. When muted all three output leds will light 
// up permanently.
// To leave the "clocked_by_midi" mode turn the tempo knob to the 
// right to activate internal clocking.
//
// I also disabled the retriggering of the outputs when a 
// reset signal has been received.


#include <avr/eeprom.h>

#include "avrlib/adc.h"
#include "avrlib/boot.h"
#include "avrlib/op.h"
#include "avrlib/watchdog_timer.h"

#include "grids/clock.h"
#include "grids/hardware_config.h"
#include "grids/pattern_generator.h"
#include "grids/midi.h"
//#include "avrlib/software_serial.h"

using namespace avrlib;
using namespace grids;

Leds leds;
// Inputs inputs;
ResetInput reset_input;
ButtonInput button_input;
ClockInput clock_input;

AdcInputScanner adc;
ShiftRegister shift_register;
//MidiInput midi;
MidiIO midi;

enum Parameter {
  PARAMETER_NONE,
  PARAMETER_WAITING,
  PARAMETER_CLOCK_RESOLUTION,
  PARAMETER_TAP_TEMPO,
  PARAMETER_SWING,
  PARAMETER_GATE_MODE,
  PARAMETER_OUTPUT_MODE,
  PARAMETER_CLOCK_OUTPUT
};

uint32_t tap_duration = 0;
uint8_t led_pattern ;
uint8_t led_off_timer;

int8_t swing_amount;

volatile Parameter parameter = PARAMETER_NONE;
volatile bool long_press_detected = false;
const uint8_t kUpdatePeriod = F_CPU / 32 / 8000;

uint8_t clocked_by_midi = 0;  // 1 = MIDI Clock advances Grids, 2 = MIDI Stop received
uint8_t mute = 0;             // 1 = drum and accent outputs are muted
uint8_t external_clock = 0;   // 1 = Grids is in external clock mode (clock knob = min,
                              // accept pulses on clock input or by midi clock messages)


inline void UpdateLeds() {
  uint8_t pattern;
  if (parameter == PARAMETER_NONE) {
    if (led_off_timer) {
      --led_off_timer;
      if (!led_off_timer) {
        led_pattern = 0;
      }
    }
    if (mute) {
                            // indicate muted outputs by turning
      led_pattern ^= LED_BD | LED_SD | LED_HH;  // all 3 leds on (at 50 % duty cycle)
    }
    pattern = led_pattern;
    if (pattern_generator.tap_tempo()) {
      if (pattern_generator.on_beat()) {
        pattern |= LED_CLOCK;
      }
    } else {
      if (pattern_generator.on_first_beat()) {
        pattern |= LED_CLOCK;
      }
    }
  } else {
    pattern = LED_CLOCK;
    switch (parameter) {
      case PARAMETER_CLOCK_RESOLUTION:
        pattern |= LED_BD >> pattern_generator.clock_resolution();
        break;
        
      case PARAMETER_CLOCK_OUTPUT:
        if (pattern_generator.output_clock()) {
          pattern |= LED_ALL;
        }
        break;
      
      case PARAMETER_SWING:
        if (pattern_generator.swing()) {
          pattern |= LED_ALL;
        }
        break;

      case PARAMETER_OUTPUT_MODE:
        if (pattern_generator.output_mode() == OUTPUT_MODE_DRUMS) {
          pattern |= LED_ALL;
        }
        break;
      
      case PARAMETER_TAP_TEMPO:
        if (pattern_generator.tap_tempo()) {
          pattern |= LED_ALL;
        }
        break;
        
      case PARAMETER_GATE_MODE:
        if (pattern_generator.gate_mode()) {
          pattern |= LED_ALL;
        }
    }
  }
  leds.Write(pattern);
}

inline void UpdateShiftRegister() {
  static uint8_t previous_state = 0;
  uint8_t state = pattern_generator.state();
  if (mute) {
    state &= ~(0x07);						// clear drum bits (BD, SD, HH)
    if (pattern_generator.output_mode() == OUTPUT_MODE_DRUMS) {
      if (pattern_generator.output_clock()) {
        state &= ~(OUTPUT_BIT_COMMON);		// clear common accent bit
      } else {
        state &= ~(0x07 << 3);				// clear all 3 accent bits
      }
    }
  }
  if (state != previous_state) {
    previous_state = state;
    shift_register.Write(state);
    
    /*
    WIP: Hardware MIDI out works (tested with an Arduino sketch)
         When sending MIDI out here, we get (reproducible) incorrect bits
         Buffering MIDI in the ISR and sending outside the ISR doesn't help.

    // TODO: Set velocity (or open HH vs close HH) based on accents 
    if (state & 0x01) { // BD
      // TODO: 3-byte MIDI messages are garbled, but consistent. Seems like the first byte is often close, but the
      //       second two are completely wrong, and often followed by two 0x00, 0x00 bytes I didn't send ...
      //       Bytes are as expected if we use a simple Arduino example instead.
      //       Tried:
      //         - smaller serial i/o buffer size (kSerialOutputBufferSize = 8)
      //         - changing Serial config to <avrlib::POLLED, avrlib::BUFFERED> - works, but unsure if buffered output
      //           is actually implemented in avrlib
      //         - using midi_->NonBlockingWrite(byte) instead of midi_->Write(byte) - no apparent difference
      //         - sending simple bytes (0xFF works, but others like 0xF8 don't send at all ?)
      //         - MIDI In clock doesn't seem to be working ? Should trace routes with multimeter as sanity check.

      // This is supposed to be what we want (note on, channel 10, C2?) - it's not !
      MidiDevice::BufferNote(midi_channel, BD_NOTE, 0x7f);
      
      // as a test, this appears to work (we get 11111111 [0xFF])
      // MidiDevice::SendMidiNow(0xFF);
    
      // 0x00 is sent as 0x88, 0x00, 0x00 ==:
      // 10001000 00000000 00000000
      // The first byte is the same as when we attempt to send BD_NOTE (0x99, 0x24, 0x7f) and get:
      // 10001000 00110001 00001000 11111111 11111111  (0x88, 0x31, 0x08, 0x00, 0x00) instead of:
      // 10011001 00100100 01111111
      // MidiDevice::SendMidiNow(0x00);

      // no bytes received over MIDI - (should be 11111000) ?
      // MidiDevice::SendMidiNow(0xF8);  // clock (supposed to be 24 pqn)
    }
    if (state & 0x02) { // SD
      MidiDevice::BufferNote(midi_channel, SD_NOTE, 0x7f);
    }
    if (state & 0x04) { // HH
      // Accent bitmasks are 0x08, 0x10, 0x20
      if (state & 0x20) {
        MidiDevice::BufferNote(midi_channel, OH_NOTE, 0x7f);
      } else {
        MidiDevice::BufferNote(midi_channel, HH_NOTE, 0x7f);
      }
    }
    */

    if (!state) {
      // Switch off the LEDs, but not now.
      led_off_timer = 200;
    } else {
      // Switch on the LEDs with a new pattern.
      led_pattern = pattern_generator.led_pattern();
      led_off_timer = 0;
    }
  }
}

uint8_t ticks_granularity[] = { 6, 3, 1 };

inline void HandleClockResetInputs() {
  //static uint8_t previous_inputs;
  static bool previous_clock_value;
  static bool previous_reset_value;
  
  //uint8_t inputs_value = ~inputs.Read();
  bool clock_value = ~clock_input.Read();
  bool reset_value = reset_input.Read();
  uint8_t num_ticks = 0;
  uint8_t increment = ticks_granularity[pattern_generator.clock_resolution()];
  
  // CLOCK
  if (clock.bpm() < 40 && !clock.locked()) {
    if (!external_clock) {
      external_clock = 1;
      mute = 1;			// activate mute when entering external clock mode
    }
    if ((clock_value) && !(previous_clock_value)) {
      if (!clocked_by_midi) {
        num_ticks = increment;
      }
    }
    if (!(clock_value) && (previous_clock_value)) {
      pattern_generator.ClockFallingEdge();
    }
    if (midi.readable()) {
      uint8_t byte = midi.ImmediateRead();
      if (byte == 0xf8) {			// MIDI Clock message
        if (clocked_by_midi == 1) {
          num_ticks = 1;
        }
      }
      else if (byte == 0xfa) {		// MIDI Start message
        pattern_generator.Reset();
        clocked_by_midi = 1;
      }
      else if (byte == 0xfb) {		// MIDI Continue message
        clocked_by_midi = 1;
      }
      else if (byte == 0xfc) {		// MIDI Stop message
        clocked_by_midi = 2;
        // AllNotesOff();
      }
    }
  } else {
    if (external_clock) {
      external_clock = 0;
      mute = 0;			// deactivate mute when leaving external clock mode
      led_pattern = 0;
    }
    if (clocked_by_midi) {
      clocked_by_midi = 0;
      mute = 0;
      pattern_generator.Reset();
    }
    clock.Tick();
    clock.Wrap(swing_amount);
    if (clock.raising_edge()) {
      num_ticks = increment;
    }
    if (clock.past_falling_edge()) {
      pattern_generator.ClockFallingEdge();
    }
  }

  // RESET
  if (clocked_by_midi) {
    if ((reset_value) && !(previous_reset_value)) {
      mute = 1;			// activate mute on rising edge
    }
    if (!(reset_value) && (previous_reset_value)) {
      mute = 0;			// deactivate mute on falling edge
      led_pattern = 0;
    }
  } else {
    if ((reset_value) && !(previous_reset_value)) {
      pattern_generator.Reset();
      // AllNotesOff();

      // !! HACK AHEAD !!
      // 
      // Earlier versions of the firmware retriggered the outputs whenever a
      // RESET signal was received. This allowed for nice drill'n'bass effects,
      // but made synchronization with another sequencer a bit glitchy (risk of
      // double notes at the beginning of a pattern). It was later decided
      // to remove this behaviour and make the RESET transparent (just set the 
      // step index without producing any trigger) - similar to the MIDI START
      // message. However, the factory testing script relies on the old behaviour.
      // To solve this problem, we reproduce this behaviour the first 5 times the
      // module is powered. After the 5th power-on (or settings change) cycle,
      // this odd behaviour disappears.
      /* ### As we don't do factory testing we don't need the hack. ###
      if (pattern_generator.factory_testing() ||
        clock.bpm() >= 40 ||
        clock.locked()) {
      */
      if (clock.bpm() >= 40 || clock.locked()) {
        // I don't like the retriggering. So I comment it out.
        // pattern_generator.Retrigger();
        clock.Reset();
      }
    }
  }
  //previous_inputs = inputs_value;
  previous_clock_value = clock_value;
  previous_reset_value = reset_value;
  
  if (num_ticks) {
    swing_amount = pattern_generator.swing_amount();
    pattern_generator.TickClock(num_ticks);
  }
}

enum SwitchState {
  SWITCH_STATE_JUST_PRESSED = 0xfe,
  SWITCH_STATE_PRESSED = 0x00,
  SWITCH_STATE_JUST_RELEASED = 0x01,
  SWITCH_STATE_RELEASED = 0xff
};

inline void HandleTapButton() {
  static uint8_t switch_state = 0xff;
  static uint16_t switch_hold_time = 0;
  
  switch_state = switch_state << 1;
  if (button_input.Read()) {
    switch_state |= 1;
  }
  
  if (switch_state == SWITCH_STATE_JUST_PRESSED) {
    if (parameter == PARAMETER_NONE) {
      if (clocked_by_midi) {		// in clocked_by_midi mode, button toggles mute state
        if (mute) {
          mute = 0;
          led_pattern = 0;
        } else {
          mute = 1;
        }
      } else {
        if (external_clock == 1) {	// in external clock mode
          mute = 0;					// the first button press unmutes the outputs
          led_pattern = 0;
          external_clock = 2;
        }
        if (!pattern_generator.tap_tempo()) {
          pattern_generator.Reset();
          /*  no need for the hack (see above)
          if (pattern_generator.factory_testing() ||
              clock.bpm() >= 40 ||
              clock.locked()) {
*/
          if (clock.bpm() >= 40 || clock.locked()) {
            clock.Reset();
          }
        } else {
          uint32_t new_bpm = (F_CPU * 60L) / (32L * kUpdatePeriod * tap_duration);
          if (new_bpm >= 30 && new_bpm <= 480) {
            clock.Update(new_bpm, pattern_generator.clock_resolution());
            clock.Reset();
            clock.Lock();
          } else {
            clock.Unlock();
          }
          tap_duration = 0;
        }
      }
    }
    switch_hold_time = 0;
  } else if (switch_state == SWITCH_STATE_PRESSED) {
    ++switch_hold_time;
    if (switch_hold_time == 500) {
      long_press_detected = true;
    }
  }
}

ISR(TIMER2_COMPA_vect, ISR_NOBLOCK) {
  static uint8_t switch_debounce_prescaler;
  
  ++tap_duration;
  ++switch_debounce_prescaler;
  if (switch_debounce_prescaler >= 10) {
    // Debounce RESET/TAP switch and perform switch action.
    HandleTapButton();
    switch_debounce_prescaler = 0;
  }
  
  HandleClockResetInputs();

  adc.Scan();

  pattern_generator.IncrementPulseCounter();
  UpdateShiftRegister();
  UpdateLeds();
}

static int16_t pot_values[8];

void ScanPots() {
  if (long_press_detected) {
    if (parameter == PARAMETER_NONE) {
      // Freeze pot values
      for (uint8_t i = 0; i < 8; ++i) {
        pot_values[i] = adc.Read8(i);
      }
      parameter = PARAMETER_WAITING;
    } else {
      parameter = PARAMETER_NONE;
      pattern_generator.SaveSettings();
    }
    long_press_detected = false;
  }
  
  if (parameter == PARAMETER_NONE) {
    uint8_t bpm = adc.Read8(ADC_CHANNEL_TEMPO);
    bpm = U8U8MulShift8(bpm, 220) + 20;
    if (bpm != clock.bpm() && !clock.locked()) {
      clock.Update(bpm, pattern_generator.clock_resolution());
    }
    PatternGeneratorSettings* settings = pattern_generator.mutable_settings();
    settings->options.drums.x = ~adc.Read8(ADC_CHANNEL_X_CV);
    settings->options.drums.y = ~adc.Read8(ADC_CHANNEL_Y_CV);
    settings->options.drums.randomness = ~adc.Read8(ADC_CHANNEL_RANDOMNESS_CV);
    settings->density[0] = ~adc.Read8(ADC_CHANNEL_BD_DENSITY_CV);
    settings->density[1] = ~adc.Read8(ADC_CHANNEL_SD_DENSITY_CV);
    settings->density[2] = ~adc.Read8(ADC_CHANNEL_HH_DENSITY_CV);
  } else {
    for (uint8_t i = 0; i < 8; ++i) {
      int16_t value = adc.Read8(i);
      int16_t delta = value - pot_values[i];
      if (delta < 0) {
        delta = -delta;
      }
      if (delta > 32) {
        pot_values[i] = value;
        switch (i) {
          case ADC_CHANNEL_BD_DENSITY_CV:
            parameter = PARAMETER_CLOCK_RESOLUTION;
            pattern_generator.set_clock_resolution((255 - value) >> 6);
            clock.Update(clock.bpm(), pattern_generator.clock_resolution());
            pattern_generator.Reset();
            break;
            
          case ADC_CHANNEL_SD_DENSITY_CV:
            parameter = PARAMETER_TAP_TEMPO;
            pattern_generator.set_tap_tempo(!(value & 0x80));
            if (!pattern_generator.tap_tempo()) {
              clock.Unlock();
            }
            break;

          case ADC_CHANNEL_HH_DENSITY_CV:
            parameter = PARAMETER_SWING;
            pattern_generator.set_swing(!(value & 0x80));
            break;

          case ADC_CHANNEL_X_CV:
            parameter = PARAMETER_OUTPUT_MODE;
            pattern_generator.set_output_mode(!(value & 0x80) ? 1 : 0);
            break;

          case ADC_CHANNEL_Y_CV:
            parameter = PARAMETER_GATE_MODE;
            pattern_generator.set_gate_mode(!(value & 0x80));
            break;

          case ADC_CHANNEL_RANDOMNESS_CV:
            parameter = PARAMETER_CLOCK_OUTPUT;
            pattern_generator.set_output_clock(!(value & 0x80));
            break;
        }
      }
    }
  }
}

void Init() {
  sei();
  UCSR0B = 0;
  
  leds.set_mode(DIGITAL_OUTPUT);
  //inputs.set_mode(DIGITAL_INPUT);
  //inputs.EnablePullUpResistors();
  reset_input.EnablePullUpResistor();
  button_input.EnablePullUpResistor();
  clock_input.EnablePullUpResistor();
  
  clock.Init();
  adc.Init();
  adc.set_num_inputs(ADC_CHANNEL_LAST);
  Adc::set_reference(ADC_DEFAULT);
  Adc::set_alignment(ADC_LEFT_ALIGNED);
  pattern_generator.Init();
  shift_register.Init();
  midi.Init();
  MidiDevice::Init(midi);
  
  TCCR2A = _BV(WGM21);
  TCCR2B = 3;
  OCR2A = kUpdatePeriod - 1;
  TIMSK2 |= _BV(1);
}

int main(void) {
  ResetWatchdog();
  Init();
  clock.Update(120, pattern_generator.clock_resolution());
  
  //MidiDevice::SendMidi3(midi_channel, BD_NOTE, 0x7f);

  while (1) {
    // Use any spare cycles to read the CVs and update the potentiometers
    ScanPots();

    // Disable interupts when sending MIDI buffer so it can get updated underneath us
    if (output_buffer_index > 0) {
      cli();
      MidiDevice::SendBuffer();
      sei();
    }
  }
}
