#include <SPI.h>
#include <Pixy.h>
Pixy pixy;    //Setup

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  getBlocks();  //gets array of objects and properties (More info: http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino))
                //Make sure you copy the parenthesis in the link (clicking it won't work)
}
