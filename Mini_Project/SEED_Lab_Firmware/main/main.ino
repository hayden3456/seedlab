#include "defs.h"
#include "localization_mod.h"

localization_mod loc_mod();

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(millis() % 100 == 0)
  {
    Serial.print(loc_mod.get_wheel_pos(LEFT_ID));
    Serial.print(" ")
    Serial.println(loc_mod.get_wheel_pos(RIGHT_ID));
  }
}
