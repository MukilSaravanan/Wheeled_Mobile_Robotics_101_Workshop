// Motor A connections
int in1 =14; //d5
int in2 =5; //d1
int enA =4; //d2 ~left motor

// Motor B connections
int in3=13; //d7
int in4=16; //d0
int enB=12; //d6 ~right motor

//IR connections
int irL=2; //d4
int irR=0; //d3

float Kp =1; //related to the proportional control term; 
              //change the value by trial-and-error (ex: 0.07).
float Ki = 0; //related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;


int lastError =0;

int maxspeeda = 500;
int maxspeedb = 500;
int basespeeda = 100;
int basespeedb = 100;


void setup() {
	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
  Serial.begin(9600);
  analogWrite(enA,basespeeda);
  analogWrite(enB,basespeedb);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);

}
void motor_driver(int mtr1,int mtr2) {
	// For PWM maximum possible values are 0 to 1023
	analogWrite(enA, mtr1);
	analogWrite(enB,mtr2);
  

	 
}
void PID_CONTROL(int irl,int irr){
  irl=1-irl;
  irr=1-irr;
  Serial.print("irl:");
  Serial.println(irl);
  Serial.print("irr2");
  Serial.println(irr);
  if ((irl==LOW) && (irr==HIGH)){
    motor_driver(50,100);
    return;
  }

  int position=(irl*120)+(irr*50);
  float error = 170 - position; //400 is the ideal position (the centre)

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                   //needed to be applied to the speed
  Serial.println("general_speed");
  Serial.println(motorspeed);
  
  Serial.print("error");
  Serial.println(error);


  Serial.println(motorspeed);

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  motor_driver(motorspeeda,motorspeedb);

  Serial.print("speed_of_a:");
  Serial.println(motorspeeda);

  Serial.print("speed_of_b:");

  Serial.println(motorspeedb);

  }


void loop() {
 
  PID_CONTROL(digitalRead(irL),digitalRead(irR));
	delay(1000);
}


 