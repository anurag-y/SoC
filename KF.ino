
#include<stdio.h>

//variable declaration 
#define in A1

double A=1, B=-0.0001, C=0.7, D=-0.01,sig_w = 0.00001,sig_v=0.1, R_0=0.01;
double x_es=1, sig_x=0;
double y,L,cur_pre=1,vol,vk,cur;


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  vol = analogRead(in);                                           //here, we directly measured voltage from cell using arduino pin
  vk = (vol/1024)*5000;                                           //we can also use voltage sensor for that if cell voltage is more than 5V
  cur = 0;                                                        //we take average of current measurement
  for(int i = 0; i < 1000; i++)                                   //this current sensor reading only will work for ACS712
  {                                                               //with 0-5A range
    cur = cur + (.0264 * analogRead(A0) -13.51) / 1000;             
    delay(1);
  }
  predict(cur_pre,cur);
  correction(vk);
  
  Serial.print("voltage : ");
  Serial.println(vk);
  Serial.print("current : ");
  Serial.println(cur);
  cur_pre = cur;
}


void predict(double u_pre,double u) {
    x_es = A*x_es + B*u_pre ;
    sig_x = A*sig_x*A + sig_w ;
    y = C*x_es + D*u ;
    Serial.println(x_es,9);
    Serial.println(sig_x,9);
    Serial.println(y,9);
}

void correction(double y_meas) {
    double inv = C*sig_x*C + sig_v ;
    L = sig_x*C/inv ;
    x_es = x_es + L*(y_meas - y);
    sig_x = (1-L*C)*sig_x;
    Serial.println(L,9);
    Serial.println(x_es,9);
    Serial.println(sig_x,9);
}
