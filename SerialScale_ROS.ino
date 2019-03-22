/* sample for digital weight scale of hx711, display with a HD44780 liquid crtstal monitor
 *
 * hardware design: syyyd
 * available at http://syyyd.taobao.com
 *
 * library design: Weihong Guan (@aguegu)
 * http://aguegu.net
 *
 * library host on
 * https://github.com/aguegu/Arduino
 */

// Hx711.DOUT - pin #A1
// Hx711.SCK - pin #A0

#include "hx711.h"

#include <ros.h>
#include <std_msgs/Float32.h>

Hx711 scale(A1, A0);

ros::NodeHandle  nh;

std_msgs::Float32 weight_msg;
ros::Publisher weight("weight", &weight_msg);

void setup() {

  Serial.begin(57600);
  
  nh.initNode();
  nh.advertise(weight);

}

void loop() {

  //Serial.print(scale.getGram(), 1);
  //Serial.println(" g");

  weight_msg.data = scale.getGram();
  weight.publish( &weight_msg );
  nh.spinOnce();  

  delay(200);
}
