#include <SPI.h>

const int RESET_SETTLE_TIME_MS = 2;

typedef struct spot_gen {
// Write pins
  const int x_vcc_pin;
  const int x_gnd_pin;
  const int y_vcc_pin;
  const int y_gnd_pin;
  const int eng_vcc_pin;
  const int eng_gnd_pin;
  const int digi_pot_xy_ss_pin;
  const uint8_t digi_pot_x_addr;
  const uint8_t digi_pot_y_addr;
  const int digi_pot_eng_ss_pin;
  const uint8_t digi_pot_eng_addr;
  const int reset_write_pin;
  const int crowbar_reset_pin;

// Read pins
  const int x_read;
  const int y_read;

// State flags
  bool writing; // Whether or not the player is being written or read
                // Gates somes functions to prevent unwanted pinMode changes
                // Used by audio effects to know which lines to check for being reset
                // to avoid playing sound effects when they should not be
  uint8_t english_pos;
} spot_gen;

spot_gen p1_spot = {
                     .x_vcc_pin = 49,
                     .x_gnd_pin = 48,
                     .y_vcc_pin = 47,
                     .y_gnd_pin = 46,
                     .eng_vcc_pin = 41,
                     .eng_gnd_pin = 40,
                     .digi_pot_xy_ss_pin = 37,
                     .digi_pot_x_addr = 0x00,
                     .digi_pot_y_addr = 0x10,
                     .digi_pot_eng_ss_pin = 35,
                     .digi_pot_eng_addr = 0x00,
                     .reset_write_pin = 33,
                     .crowbar_reset_pin = A14,
                     .x_read = A0,
                     .y_read = A1,
                     .writing = false,
                     .english_pos = 0,
};


void init_player_as_writeable(spot_gen *player) {
    pinMode(player->x_vcc_pin, OUTPUT);
    pinMode(player->x_gnd_pin, OUTPUT);
    pinMode(player->y_vcc_pin, OUTPUT);
    pinMode(player->y_gnd_pin, OUTPUT);
    pinMode(player->eng_vcc_pin, INPUT);
    pinMode(player->eng_gnd_pin, INPUT);

    // Set digital pins to power digital potentiometer
    digitalWrite(player->x_vcc_pin, HIGH);
    digitalWrite(player->x_gnd_pin, LOW);
    digitalWrite(player->y_vcc_pin, HIGH);
    digitalWrite(player->y_gnd_pin, LOW);

    pinMode(player->digi_pot_xy_ss_pin, OUTPUT);
    digitalWrite(player->digi_pot_xy_ss_pin, HIGH);
    pinMode(player->digi_pot_eng_ss_pin, OUTPUT);
    digitalWrite(player->digi_pot_eng_ss_pin, HIGH);

    pinMode(player->reset_write_pin, INPUT);
    pinMode(player->crowbar_reset_pin, OUTPUT);
    digitalWrite(player->crowbar_reset_pin, LOW);

    player->writing = true;
}