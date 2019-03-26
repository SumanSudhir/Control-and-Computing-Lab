int ctrl_a=A6;
int ctrl_b=A5;
//int potval = A15;
float set=0;
float error=0;
float tot_err=0;
float integrate=0;
float preverror=0;
float potpos=0;
int fin_val;
float init_val;
float new_val;
int fin_val_n;            //motor no. 11 

unsigned long start;
char startval;
float p = 20;
float i=0.3;
float d = 7;



void setup() {
  Serial.begin(9600);
  pinMode(ctrl_a,OUTPUT);
  pinMode(ctrl_b,OUTPUT);
  init_val = 0;
  init_val = float(analogRead(A15));
  init_val = (init_val*360.0)/1024;
  init_val = -0.916*init_val + 390.24;

  Serial.print("initial point ");
  Serial.println(init_val, DEC);
  
  fin_val = int(180+init_val);
  fin_val=(fin_val)%360;
  Serial.print("final point ");
  Serial.println(fin_val, DEC);   
}

void loop() {

 new_val = float(analogRead(A15));
 new_val = (new_val*360.0)/1024;
 new_val = -0.916*new_val+390.25;
 Serial.print(millis());
 Serial.print('\t');
 Serial.print(new_val, DEC);
 
 
 error = float(fin_val-new_val); 
 integrate = integrate+error;
 tot_err = p*error + i*integrate + d*(error-preverror);
 preverror=error;

 
 if(error<-6){
 analogWrite(ctrl_b,(min(abs(tot_err),255)));
 analogWrite(ctrl_a,0); 
 }
 
if(error>6){
 analogWrite(ctrl_b,0);
 analogWrite(ctrl_a,(min(abs(tot_err),255)));    
}

if(error<6 && error>-6){
 analogWrite(ctrl_a,0);
 analogWrite(ctrl_b,0); }   


Serial.print('\n');
}
