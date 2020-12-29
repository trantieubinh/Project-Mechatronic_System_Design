#fuses HS, NOWDT, MCLR, PUT, NOLVP, NOPROTECT, NOBROWNOUT
#use delay(clock = 20M)
#use rs232(baud=9600,parity =N,xmit=PIN_C6,rcv=PIN_C7)
#define  Wire1 PIN_D4 // define motor IN pin
#define  Wire2 PIN_D5
#define  Wire3 PIN_D6
#define  Wire4 PIN_D7
#include <math.h>


float  vpwml , vpwmr ;
int16 pwmleft, pwmright;

// robot parameter
float R  = 0.04; //wheel radius m
float width  = 0.197; // robot width m
float d  = 0.08; //distance from the center wheel axis to the middle sensor m

int16 in_max[8], in_min[8];
unsigned int8 b[8], s;

float e1 =  d; 
float e2 =  0; // tracking error
float e3 =  0;


float e2_prev =0;


// int out_min= 0; //minimum value we wanted
 //int out_max=50 ;  //maximum value we wanted
 
//int sen_space=17;//distance between 2 sensors

int i, state=0, flag=0,t =0;
int16 Pulse1 = 0, Pulse2 = 0;
float rpm1, rpm2, wl, wr;
int16 y[8];
//int c=0;


//Lyapunov parameter
float k1 = 1;
float k2 = 500;
float k3 = 0;

float v, w, v_current;
float v_ref = 0.3; //rpm
float w_ref = 0;

float y1_calib, y2_calib, y3_calib, y4_calib, y5_calib, y6_calib, y7_calib;
int digi1, digi2, digi3, digi4, digi5, digi6, digi7;

      

#separate void calib() {
   for(i=0;i<7;i++) {
      set_ADC_channel(i);
      delay_us(50);  
      y[i+1]=read_adc();   
   }
y1_calib = 100*(((float)y[1]-in_min[1])/(in_max[1]-in_min[1])); //calibrate sensor (based on the formula)
   if(y1_calib >100) y1_calib = 100; //limiting the value
   if(y1_calib < 0) y1_calib = 0; //limiting the value
 
     
y2_calib = 100*(((float)y[2]-in_min[2])/(in_max[2]-in_min[2])); //calibrate sensor (based on the formula)
   if(y2_calib >100) y2_calib = 100; //limiting the value
   if(y2_calib < 0) y2_calib = 0; //limiting the value
  

y3_calib = 100*(((float)y[3]-in_min[3])/(in_max[3]-in_min[3])); //calibrate sensor (based on the formula)
   if(y3_calib >100) y3_calib = 100; //limiting the value
   if(y3_calib < 0) y3_calib = 0; //limiting the value
     
y4_calib = 100*(((float)y[4]-in_min[4])/(in_max[4]-in_min[4])); //calibrate sensor (based on the formula)
   if(y4_calib >100) y4_calib = 100; //limiting the value
   if(y4_calib < 0) y4_calib = 0; //limiting the value
         
y5_calib = 100*(((float)y[5]-in_min[5])/(in_max[5]-in_min[5])); //calibrate sensor (based on the formula)
   if(y5_calib >100) y5_calib = 100; //limiting the value
   if(y5_calib < 0) y5_calib = 0; //limiting the value
           
   
y6_calib = 100*(((float)y[6]-in_min[6])/(in_max[6]-in_min[6])); //calibrate sensor (based on the formula)
   if(y6_calib >100) y6_calib = 100; //limiting the value
   if(y6_calib < 0) y6_calib = 0; //limiting the value
         
   
y7_calib = 100*(((float)y[7]-in_min[7])/(in_max[7]-in_min[7])); //calibrate sensor (based on the formula)
   if(y7_calib >100) y7_calib = 100; //limiting the value
   if(y7_calib < 0) y7_calib = 0; //limiting the value
        
}

// getting initial value on white background
void white() {
   for(i=0;i<7;i++) {
      set_ADC_channel(i);
      delay_us(100);  
      in_min[i+1]=read_adc();
      delay_ms(250);
   }
printf("%lu   %lu  %lu  %lu  %lu   %lu   %lu;  \n ",in_min[1],in_min[2],in_min[3],in_min[4],in_min[5],in_min[6],in_min[7]);
}
// getting initial value on black background
void black() {
   for(i=0;i<7;i++) {
      set_ADC_channel(i);
      delay_us(50);  
      in_max[i+1]=read_adc(); 
      delay_ms(250);
   }
printf("%lu   %lu  %lu  %lu  %lu   %lu   %lu;  \n ",in_max[1],in_max[2],in_max[3],in_max[4],in_max[5],in_max[6],in_max[7]);

}


// reading encoder
#INT_EXT
void encoder_ISR()
{
Pulse1++;
}

#INT_EXT1
void encoder_ISR1()
{
Pulse2++;
}

//reading motor rpm
#INT_TIMER1
void Timer1_isr(void){
  int counter, counter1;
  counter++;
   counter1++;
 
 if(counter==100)//0.01 second = 1 counter, 100 counter = 1 second
 {
rpm1 = Pulse1*0.15;  // calculate RPM for Motor 1 pulse/(400 per revolution)*60
rpm2 = Pulse2*0.15;  // calculate RPM for Motor 2 
Pulse1=0;   //reset pulse1
Pulse2=0;   //reset pulse2

counter=0; // reset counter
 }
 if(counter1==2)
 {
 counter1=0;
 }
 // something here
 
 
 
  set_timer1(59286);                                // Timer0 preload value
  clear_interrupt(INT_TIMER1);           
}




// Calculate e2 and find V and W
void vomegaref() {  
 e2_prev=e2;
   e2=((3*(y7_calib-y1_calib)+2*(y6_calib-y2_calib)+(y5_calib-y3_calib))/(y1_calib+y2_calib+y3_calib+y4_calib+y5_calib+y6_calib+y7_calib))*17;
   if (e2 >-1 && e2 <1) e2=0;
  // calculate e3
   //v_current = (r/2)*(rpm1+rpm2);
   //e3=atan((e2-e2_prev)/(v_current*0.02));
   

    //path tracking lyapunov-based control
        //  v = v_ref*cos(e3) + k1*e1;
        // w = k2*e2*v_ref*0.001 + k3*sin(e3) + w_ref;

        v = v_ref*cos(e3) + k1*e1; 
        w = k2*e2*v_ref*0.001 + k3*sin(e3) + w_ref;
         

//printf("%f;  \n ",e2);
}



double e_speed1 = 0; //error of speed = set_speed - pv_speed
double e_speed_pre1 = 0;  //last error of speed
double e_speed_sum1 = 0;  //sum error of speed
int pwm_pulse1 = 75;  

//motor 1 PID parameter
double kp1 = 0.0021314;
double ki1 = 0.1247;
double kd1 = 0;
double set_speed1=100;

void motor_PID_1()
{
    e_speed1 = set_speed1 - rpm1; // the error
    pwm_pulse1 = e_speed1*kp1 + e_speed_sum1*ki1 + (e_speed1 - e_speed_pre1)*kd1; //PID
    e_speed_pre1 = e_speed1;  //save last (previous) error
    e_speed_sum1 += e_speed1; //sum of error
    if (e_speed_sum1 >4000) e_speed_sum1 = 4000;
    if (e_speed_sum1 <-4000) e_speed_sum1 = -4000;
}

void motor_PID_2()
{
}




void main()
{
//setting up motor
setup_ccp1(CCP_PWM);                   // Configure CCP1 as a PWM pin 17 (18f4550)
setup_ccp2(CCP_PWM);                   // Configure CCP2 as a PWM pin 16
setup_timer_2(T2_DIV_BY_16, 624, 1);   // Set PWM frequency 
//PWM period = [(PR2) + 1] * 4 * Tosc * (TMR2 Prescale Value)
//Where PWM frequency is defined as 1/[PWM period].
//PR2 is Timer2 preload value,
//Tosc = 1/(MCU_frequency)
//TMR2 Prescale Value can be 1, 4 or 16.

output_high(Wire2);           
output_low (Wire1);            // Set the motor to direction
output_high(Wire4);           
output_low (Wire3);            // Set the motor to direction
 set_pwm1_duty(0); 
 set_pwm2_duty(0); 
delay_ms(100);                         // Wait 100ms

//setup ADC
setup_ADC(ADC_CLOCK_DIV_32); 
setup_ADC_ports(AN0_TO_AN6 );
 
 delay_ms(100);

 //setup timer1
 clear_interrupt(INT_TIMER1);
 enable_interrupts(INT_Timer1);
 enable_interrupts(GlOBAL);
 setup_timer_1 ( T1_INTERNAL | T1_DIV_BY_8);//0-255 tran 51.2us
 set_timer1(59286); //sampling time 0.01ms

//enable external interrupt for reading the encoder
enable_interrupts(GlOBAL);
enable_interrupts(INT_EXT);
clear_interrupt(INT_EXT);

enable_interrupts(INT_EXT1);
clear_interrupt(INT_EXT1);

    ext_int_edge(L_TO_H);
    







   
   while(1)
   { 
   

   //printf("rpm1 = %f ,rpm2= %f  ;  \n ",   rpm1,rpm2);
   
   
   
//motor_PID_1();
//set_pwm1_duty(pwm_pulse1);
//printf("rpm1 = %f ,RPM2= %f  ;  \n ",   rpm1,rpm2);
//set_pwm2_duty(85);


    //set_pwm1_duty(pwm_pulse); 
   //printf("Pulse1 = %lu ,Pulse2= %lu  ;  \n ",Pulse1,Pulse2);
   
   // initial sensor value
   if (INPUT(PIN_D0)==0 && state == 0) //getting initial value
    {
       black();
       delay_ms(1250); //delay for 5 second  
       white();
       delay_ms(2500);  
       state = 1;
    }
     
     if(state==1){
     calib(); //read and calibrate the sensor
     vomegaref();
     //printf("%lu   %lu  %lu  %lu  %lu   %lu   %lu;  \n ",y[1],y[2],y[3],y[4],y[5],y[6],y[7]);
     //printf("Pulse1 = %lu ,Pulse2= %lu  ;  \n ",Pulse1,Pulse2);
     
     
     
     
     
   
     
     // calculate each wheel angular velocity
     //wl=(2*v - 1.5*width*w)*16.8/(2*R);
     //wr=(2*v + 1.5*width*w)*16.8/(2*R);
     
     wl=(2*v - width*w)*17.5/(2*R);
     wr=(2*v + width*w)*17.5/(2*R);
     
     if (wl > 137)   wl=137; 
     if (wr > 137)     wr=137;
     
     if (wl< 45) pwmleft=5;
      else  pwmleft=  (wl+  3.0444)  /  0.4216;
     if (wr< 45) pwmright=5;
      else  pwmright= (wr -10.255)   /  0.4191; 

    
    set_pwm1_duty(pwmright);
    set_pwm2_duty(pwmleft);
     
    
     
     //limiting
     //if (wl>150) wl=150;
     //if (wl<70) pwmleft=0;

     //if (wr>150) wr=150;
      //if (wr<70) pwmright = 0;
     
     // int8
     //pwmleft=  (wl   -  31.58)/1.4014;
     //pwmright= (wr   -  28.554)/1.5033;
     
     //int16
     //pwmleft=  (wl+  3.0444)  /  0.4216;
     //pwmright= (wr -10.255)   /  0.4191; 

     //set_pwm1_duty(pwmright);
     //set_pwm2_duty(pwmleft);
     
     printf("rpm1 = %f ,rpm2= %f , wl=%f , wr=%f ;  \n ",   rpm1,rpm2, wl, wr);
     
     
     //if  (pwmleft < 75) pwmleft=70;
     //if  (pwmleft>100)  pwmleft=100;
     
     //printf("e2= %f, e3=%f W = %f ,V= %f, wl=%f , wr=%f  ;  \n "e2,e3 ,w,v, wl, wr);
     
     
     //set_pwm1_duty();
     //set_pwm2_duty();
     
     
     
     
     
      //
      //wl=(2*v - width*w)/(2*R);
     //if (wl < 31) wl=31;
         //pwmleft=(wl   -  31.58)/1.4014;
         //if (pwmleft>100)  pwmleft=100;
        
         
         
         
         
         //if(vpwml < 75) pwmleft=70;
         //else if(vpwml>200) pwmleft=100;
         //else pwmleft=90;
        // set_pwm2_duty(pwmleft);

     
     
     //wr=(2*v + width*w)*17/(2*R);
     //wr=(2*v + width*w)/(2*R);
     //if (wr <  28) wr = 28; 
        // pwmright=(wr   -  28.554)/1.5033;
         //if (pwmright>100) pwmright =100; 
         
          
         
         //if(vpwmr < 75) pwmright=70;
         //else if(vpwmr>200) pwmright=100;
         //else pwmright=90;
          //set_pwm1_duty(pwmright);

     
     
     //vomegaref(); //calculating the error e2 and find V and W
     
     //printf("e2= %f, e3=%f W = %f ,V= %f, wl=%f , wr=%f  ;  \n "e2,e3 ,w,v, wl, wr);
     
     //printf("rpm1 = %f ,RPM2= %f  ;  \n ",   rpm1,rpm2);
     
     //printf("left = %d ,wl = %f,right= %d   ,wr= %f  ;  \n ",pwmleft, wl, pwmright, wr);
     
     //printf("left = %d ,right= %d  ;  \n ",pwmleft, pwmright);
//printf("right = %lu ,left= %lu  ;  \n ",wr,wl);
//printf("%f   %f  %f  %f  %f   %f   %f;  \n ",y1_calib,y2_calib,y3_calib,y4_calib,y5_calib,y6_calib,y7_calib);
     
     
     
  
     }     
     
     
    }
   }
   