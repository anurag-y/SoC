
#include <BasicLinearAlgebra.h>
using namespace BLA;
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define in A1
double cur=0,adcvol=0;
float vol=0;
int offsetvol=2500;
int sens=185;

//variables 
double tau=0.45,r1=0.08,m0=0.6,m1=0.1,r0=0.01,q=2.78,xes,xpre,yes,ymea,ga=0.47,deltat=2,I=1,res,ik,vk,sigmaV,sigmaW,Ah;  //q in Ah deltat in sec, I is the previous current
BLA::Matrix<3, 3> Acap;
BLA::Matrix<3, 1> Bcap;
BLA::Matrix<3, 2> B;
BLA::Matrix<1, 3> Ccap;
BLA::Matrix<1, 1> Dcap;
BLA::Matrix<3, 1> Xhatm; //xhat minus
BLA::Matrix<3, 1> Xhatp; //xhat plus
BLA::Matrix<2, 1> b;
BLA::Matrix<3, 3>sigmaXp;
BLA::Matrix<3, 3>sigmaXm;
BLA::Matrix<1, 1> sigmaY;
BLA::Matrix<3, 1> L;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initialization();
  lcd.init();         // initialize the lcd
  lcd.backlight();
  pinMode(in,INPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  //measure voltage and current 
  vol = analogRead(in);                                           //here, we directly measured voltage from cell using arduino pin
  vk = (vol/1024)*5;                                              //we can also use voltage sensor for that if cell voltage is more than 5V
  cur = 0;                                                        //we take average of current measurement
  for(int i = 0; i < 1000; i++)                                   //this current sensor reading only will work for ACS712
  {                                                               //with 0-5A range
    cur = cur + (.0264 * analogRead(A0) -13.51) / 1000;             
    delay(1);
  }
  iteration(cur,vk);
  Serial.print("voltage : ");
  Serial.println(vk,5);
  Serial.print("current : ");
  Serial.println(cur,5);
}

//OCV function
double OCV(double zk){
//  double ocv = 3.5 + 0.7*zk;
  double ocv = 2.85 + 0.89*zk;
  return ocv;
}

//initialization--> all values and intial condition are intialized here
void initialization(){
  sigmaXp(0,0)=0.06;
  sigmaXp(1,1)=0.001;
  sigmaXp(2,2)=0.01;
  sigmaV = 0.01; 
  sigmaW = 0.01;
  Xhatp(0,0)=0.5;
  Xhatp(1,0)=0;
  Xhatp(2,0)=0.99;
  b(0,0)=I;
  b(1,0)=1;
}


//iteration(update)

void iteration(double ik, double vk){

 //1a
  Acap(2,2)=1;
  Bcap(2,0)=-deltat/(3600*q);
  Acap(0,0)=tau;
  Bcap(0,0)=1-tau;
  double buff1 = I*ga*deltat/(3600*q) ; 
  Ah=exp(-abs(buff1));
  Acap(1,1)=Ah;
  double buff2 = ga*deltat/(3600*q)*Ah*(1+Xhatp(1));
  Bcap(1,0)=-abs(buff2);

  B(2,0)=-deltat/(3600*q);
  B(1,1)=Ah-1;
  B(0,0)=1-tau;

  Xhatm = Acap*Xhatp + B*b;

  //1b

  sigmaXm = Acap*sigmaXp*(~Acap) + Bcap*sigmaW*(~Bcap);
//  Serial.println(Xhatm(2,0));


  //1c

  yes = OCV(Xhatm(2,0)) + m0 + m1*Xhatm(1,0) - r1*Xhatm(0,0) - r0*ik;

  //2a---> calculate kalman gain matrix
  Ccap(0,2)=0.89;
  Ccap(0,1)=m1;
  Ccap(0,0)=-r1;
  Dcap=1;
  sigmaY = Ccap*sigmaXm*(~Ccap) + sigmaV ;
  L = sigmaXm*(~Ccap)/(Invert(sigmaY));
  Serial.print("Kalman Gain : ");
  Serial.print(L(0,0),5);
  Serial.print(" ");
  Serial.print(L(1,0),5);
  Serial.print(" ");
  Serial.println(L(2,0),5);


  //2b-->predict next state

  res = vk - yes;
  Xhatp = Xhatm + L*res;

  //2c--> error covariance calculate

  sigmaXp = sigmaXm - L*sigmaY*(~L);

  //conclusion
  I = ik ;
  Serial.print("SoC : ");
  Serial.println(Xhatp(2,0),5);
  Serial.print("Error : ");
  Serial.println(sqrt(sigmaXm(2,2)),5);
  lcd.setCursor(1, 0);
  lcd.print(Xhatp(2,0),5);
  lcd.setCursor(1, 1);
  lcd.print(sqrt(sigmaXm(2,2)),5);
  delay(3000);

  
}
