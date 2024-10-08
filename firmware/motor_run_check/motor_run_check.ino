
#include <Encoder.h>

Encoder encoder_bright(43, 42);//BR
Encoder encoder_bleft(45, 44); //BL
Encoder encoder_fright(47, 46);//FR
Encoder encoder_fleft(49, 48);//FL 

int pwm=100;

class BTSMotor{
  protected:
    int pwmf,pwmr;
  public:
    BTSMotor(int p1,int p2){
      pwmf=p1;pwmr=p2;
      pinMode(pwmf,OUTPUT);
      pinMode(pwmr,OUTPUT);
    }
    void setspeed(int val){
      int spd;
      if(val<0) {
        spd=val*-1;
      }
      else {
        spd=val;
      }
      if(val>=255){
        spd=255;
      }
      else if(val<=-255){
        spd=255;
      }
      
      if(val>=0){
        analogWrite(pwmf,spd);
        analogWrite(pwmr,0);
      }
      else if(val<0){
        analogWrite(pwmr,spd);
        analogWrite(pwmf,0);
      }
    }
};


BTSMotor motor1(10,11); // left front 
BTSMotor motor2(4,5); // right front
BTSMotor motor3(12,13); // left back 
BTSMotor motor4(2,3); // right back

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  motor1.setspeed(pwm);
  motor2.setspeed(pwm);
  motor3.setspeed(pwm);
  motor4.setspeed(pwm);
  //analogWrite(front_right_f,255);
  //analogWrite(front_right_r,0);
  Serial.print("fl: ");
  Serial.print(encoder_fleft.read());
  Serial.print(" fr: ");
  Serial.print(encoder_fright.read());
  Serial.print(" bl: ");
  Serial.print(encoder_bleft.read());
  Serial.print(" br: ");
  Serial.print(encoder_bright.read());

  Serial.println("");
}
