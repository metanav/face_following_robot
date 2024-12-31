#include <Wire.h>

#undef degrees
#undef radians

#include "src/pico_motor_shim.hpp"
#include "src/person_sensor.h"

#define LED_PIN LED_BUILTIN

using namespace motor;

const int32_t SAMPLE_DELAY_MS = 200;

// The scaling to apply to each motor's speed to match its real-world speed
constexpr float SPEED_SCALE = 5.4f;

// The speed to drive the wheels at, from 0.0 to SPEED_SCALE
constexpr float DRIVING_SPEED = SPEED_SCALE;

constexpr int TURN_THRESHOLD = 25;
constexpr int MIN_BOX_WIDTH = 40;
constexpr int MAX_BOX_WIDTH = 60;

Motor left(pico_motor_shim::MOTOR_1, NORMAL_DIR, SPEED_SCALE);
Motor right(pico_motor_shim::MOTOR_2, NORMAL_DIR, SPEED_SCALE);

void forward(float speed = DRIVING_SPEED) {
  left.speed(speed);
  right.speed(speed);
}

void backward(float speed = DRIVING_SPEED) {
  left.speed(-speed);
  right.speed(-speed);
}

void stop() {
  left.stop();
  right.stop();
}

void coast() {
  left.coast();
  right.coast();
}

void turn_left(float speed = DRIVING_SPEED) {
  left.speed(-speed);
  right.speed(speed);
}

void turn_right(float speed = DRIVING_SPEED) {
  left.speed(speed);
  right.speed(-speed);
}


void setup()
{
  pinMode(LED_PIN, OUTPUT);
  Wire.begin();
  Serial.begin(115200);

  while (!left.init() || !right.init()) {
    Serial.println("Cannot initialise the motors. Check the provided parameters.");
    sleep_ms(1000);
  }
}

void loop()
{
  person_sensor_results_t results = {};

  if (!person_sensor_read(&results)) {
    Serial.println("No person sensor results found on the i2c bus");
    return;
  }

  if (results.num_faces > 0 ) {
    const person_sensor_face_t* face = &results.faces[0];
    int centroid_x = (face->box_left + face->box_right) / 2;
    int centroid_y = (face->box_top  + face->box_bottom) / 2;
    int box_width = face->box_right - face->box_left;
    int turn_direction = (centroid_x - 128);



    Serial.printf("%d,%d,%d,%d,%d,%d\n",
                  face->box_left,
                  face->box_top,
                  face->box_right,
                  face->box_bottom,
                  centroid_x,
                  centroid_y);

    Serial.printf("%d,%d,%d\n", face->box_confidence, box_width, turn_direction);

    if (face->box_confidence < 80) {
      return;
    }

    if (turn_direction < -TURN_THRESHOLD) {
      turn_right(0.35 * DRIVING_SPEED);
      sleep_ms(500);
      stop();
    } else if (turn_direction > TURN_THRESHOLD) {
      turn_left(0.35  * DRIVING_SPEED);
      sleep_ms(500);
      stop();
    } else if (box_width < MIN_BOX_WIDTH) {
      forward(0.35 * DRIVING_SPEED);
      sleep_ms(500);
      stop();
    } else if (box_width > MAX_BOX_WIDTH) {
      backward(0.35 * DRIVING_SPEED);
      sleep_ms(500);
      stop();
    }
  }
  
  if (results.num_faces == 0 ) {
    sleep_ms(SAMPLE_DELAY_MS);
  }
}
