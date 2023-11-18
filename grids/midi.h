#ifndef GRIDS_MIDI_H_
#define GRIDS_MIDI_H_

#include "avrlib/base.h"
#include "grids/hardware_config.h"

#define MIDI_BUFFER_SIZE 16

namespace grids {

static uint8_t BD_NOTE = 0x24;
static uint8_t SD_NOTE = 0x26;
static uint8_t HH_NOTE = 0x2a;
static uint8_t OH_NOTE = 0x2e;

static volatile uint8_t output_buffer[MIDI_BUFFER_SIZE];
static volatile uint8_t output_buffer_index = 0;  

class MidiDevice {
public:
  // Initialize the static MidiIO instance
  static inline void Init(MidiIO& midi_instance) {
    midi_ = &midi_instance;
  }

  static inline void Buffer(uint8_t byte) {
    if (output_buffer_index < MIDI_BUFFER_SIZE) {
      output_buffer[output_buffer_index++] = byte;
    }
  }

  static inline void SendBuffer() {
    if (output_buffer_index > 0) {
      for (uint8_t i = 0; i < output_buffer_index; ++i) {
        midi_->Write(output_buffer[i]);
      }
      output_buffer_index = 0;
    }
  }

  static inline void SendMidiNow(uint8_t byte) {
    if (midi_) {
      midi_->NonBlockingWrite(byte);
    }
  }

  static inline void SendMidi(uint8_t byte) {
    if (midi_) {
      midi_->Write(byte);
    }
  }

  // Send a 3-byte MIDI message
  static inline void SendMidi3(uint8_t a, uint8_t b, uint8_t c) {
    if (midi_) {
      midi_->Write(a);
      midi_->Write(b);
      midi_->Write(c);
      // midi_->NonBlockingWrite(a);
      // midi_->NonBlockingWrite(b);
      // midi_->NonBlockingWrite(c);
    }
  }

  static inline void BufferNote(uint8_t channel, uint8_t note, uint8_t velocity) {
    Buffer(0x90 | channel);
    Buffer(note);
    Buffer(velocity);
  }

  // Trigger a MIDI note
  static inline void TriggerNote(uint8_t channel, uint8_t note, uint8_t velocity) {
    SendMidi3(0x90 | channel, note, velocity);
  }

  // Turn off all MIDI notes
  static inline void AllNotesOff(uint8_t channel) {
    SendMidi3(0x80 | channel, 0x24, 0);
    SendMidi3(0x80 | channel, 0x26, 0);
    SendMidi3(0x80 | channel, 0x2a, 0);
    SendMidi3(0x80 | channel, 0x2e, 0);
  }

private:
  // Static member variable to hold the MidiIO instance
  static MidiIO* midi_;
};

}  // namespace grids

#endif  // GRIDS_MIDI_H_
