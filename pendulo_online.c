
#include <18f4550.h>

#build(reset=0x02000,interrupt=0x02008)
#org 0x0000,0x1FFF {}

#fuses HSPLL, NOWDT, NOPROTECT, NOLVP, NODEBUG, USBDIV, PLL5, CPUDIV1, VREGEN
#use delay(clock = 48000000)  
// -------------------------------------------------------------------------------------


// ---------------------------------  MODULO LCD I2C  ----------------------------------
#define MPU_SDA PIN_B0                             
#define MPU_SCL PIN_B1        
#use I2C(master, sda=MPU_SDA, scl=MPU_SCL, Fast) 

#define PCF8574A        // Comentar para módulos PCF8574
#define A2_A1_A0 0B111  // Estado pines (A2 A1 A0) del PCF8574A/PCF8574.
#include "lcd_i2c_pcf8574_es.h"
// ---------------------------------------------------------------------------------------

#use TIMER(TIMER = 1, TICK = 500us, BITS = 16, NOISR)  

#define PWM_MIN 520
#define PWM_MAX 1023

#include <math.h>
#include "MPU6050.c"
// 180/PI
#define A_DEG 57.2957
#define A_RAD 0.0174532
#define REBOTE 250

#define MINIMA_MOTOR 530
#define PWM_MS_TEST 5

#define FILTRO_C1 0.9066
#define FILTRO_C2 0.09337

// mgl sen 0
//#define MASA 0.857
//#define LONGITUD 0.115
//#define GRAVEDAD 9.81
#define MGL 0.9668

float angX, angY;
float no_filtrada_X = 0, no_filtrada_Y = 0;

unsigned int8 accX_H, accY_H, accZ_H, accX_L, accY_L, accZ_L;
signed int16 accX, accY, accZ;

float velo = 0;

int16 velocidad = 500;
int8 giro = 0;
char sentido = 0;
float Y_old = 0, X_old = 0;

float ki = 0, kd = 0;
float kp = 0;

float mape(float x, float in_min, float in_max, float out_min, float out_max){
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void angulos(void){
   
   int8 menu = 0;
   while( input(PIN_C2) == 1 ){
   
         accX_H = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_H);  
         accY_H = Mpu6050_Read(MPU6050_RA_ACCEL_YOUT_H);
         accZ_H = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_H);
         
         accX_L = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_L);  
         accY_L = Mpu6050_Read(MPU6050_RA_ACCEL_YOUT_L);
         accZ_L = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
         
         accX = make16(accX_H, accX_L);
         accY = make16(accY_H, accY_L);
         accZ = make16(accZ_H, accZ_L);
         
         angX = (atan2(accX, accZ) + PI) * A_DEG;
         no_filtrada_X = angX - 180;
         
         angY = (atan2(accY, accZ) + PI) * A_DEG;
         no_filtrada_Y = angY - 180;
         
         angX = ( FILTRO_C1 * X_old) + ( FILTRO_C2 * no_filtrada_X);
         //angX = no_filtrada_X;
         X_old = angX;
         
         angY = ( FILTRO_C1 * Y_old) + ( FILTRO_C2 * no_filtrada_Y);
         //angY = no_filtrada_X;
         Y_old = angY;
         
         
         if(angX == 0){
            velocidad = 0;
            giro = 0;
            sentido = ' ';
         }
         else if(angX > 0){
            velo = mape(angX, 0.01, 100, PWM_MIN, PWM_MAX);
            velocidad = (int16)velo;
   
            giro = 1;
            sentido = 'R';
         }
         else if(angX < 0){
            velo = mape(angX, -0.01, -100, PWM_MIN, PWM_MAX);
            velocidad = (int16)velo;
            
            giro = 2;
            sentido = 'L';
         }
         
         set_pwm2_duty(velocidad);
         
         if(giro == 1){
            output_low(PIN_B5);
            output_high(PIN_B6);
         }
         else if(giro == 2){
            output_low(PIN_B6);
            output_high(PIN_B5);
         }
         else{
            output_low(PIN_B6);
            output_low(PIN_B5);
         }
   
   if( input(PIN_A0) == 1 && menu >=0 ){
      menu++;
      delay_ms(REBOTE);
   }
  
   if( input(PIN_A1) == 1 && menu <=3 ){
      menu--;
      delay_ms(REBOTE);
   }
   
   if(menu >= 3){
      menu = 3;
   }
   if(menu <= 0){
      menu = 0;
   }
      

   
   switch(menu){
      case 0:
         lcd_i2c_go_xy(1, 1);
         printf(escribir_lcd_i2c,"    Filtrada    ");
         
         lcd_i2c_go_xy(1, 2);
         printf(escribir_lcd_i2c,"  %3.2fX %3.2fY  ", angX, angY);
         break;

               
      case 1:
         lcd_i2c_go_xy(1, 1);
         printf(escribir_lcd_i2c,"  No  Filtrada  ");
         
         lcd_i2c_go_xy(1, 2);
         printf(escribir_lcd_i2c,"  %3.2fX %3.2fY  ", no_filtrada_x, no_filtrada_y);
         break;
               
      default:
         //printf(escribir_lcd_i2c,"CASO DEFAULT %d", menu);
         //delay_ms(20);
         menu = 0;
         break;
         
      }
   }

} // funcion

int8 gir = 1;
char sent = 'R';

void pMotores(void){
   
   int16 i;
   int8 aux = 1, band = 0;
   
   while( input(PIN_C2) == 1 ){
   
   aux = input(PIN_C2);
   lcd_i2c_go_xy(1, 1);
   printf(escribir_lcd_i2c,"     Testing    ");
   
   for(i = MINIMA_MOTOR; i <= 1023; i++){
      set_pwm2_duty(i);
      delay_ms(PWM_MS_TEST);
      
      lcd_i2c_go_xy(1, 2);
      printf(escribir_lcd_i2c,"   PWMU  %ld%c    ", i, sent);
      
      aux = input(PIN_C2);
      if(aux == 0){
         band = 1;
         break;
      }
   }
   
   delay_ms(500);
   
   for(i = 1023; i >= MINIMA_MOTOR; i--){
      set_pwm2_duty(i);
      delay_ms(PWM_MS_TEST);
      
      lcd_i2c_go_xy(1, 2);
      printf(escribir_lcd_i2c,"   PWMD  %ld%c   ", i, sent);
      
      aux = input(PIN_C2);
      if(aux == 0){
         band = 1;
         break;
      }
   }
   
   if(band){
      break;
   }
   
   set_pwm2_duty(0);
   delay_ms(500);
   
   if(gir == 1){
      
      output_low(PIN_B5);
      output_high(PIN_B6);
      sent = 'R';
   }
   if(gir == 2){
      
      output_low(PIN_B6);
      output_high(PIN_B5);
      sent = 'L';
   }
   gir++;
   if(gir > 2){
      gir = 1;
   }

   } // while

} // funcion


int vector = 0;
void interaccion(char pid,int direccion){

   int8 CEN_PS = 0, DEC_PS = 0, UNI_PS = 0, vector = 0, TMR_PS = 0;
   
   lcd_i2c_go_xy(1, 1);
   printf(escribir_lcd_i2c,"       k%c       ", pid);
   
   float maybe;
   if(direccion == 3){
      maybe = kp;
   }
   if(direccion == 4){
      maybe = ki;
   }
   if(direccion == 5){
      maybe = kd;
   }
   
   lcd_i2c_go_xy(1, 2);
   printf(escribir_lcd_i2c,"        %2.1f         ", maybe);
   
   delay_ms(REBOTE);
   
   while( input(PIN_C2) == 1 ){
      if( input(PIN_A0) == 1 && vector == 0 && CEN_PS < 9 ){
         CEN_PS++;
         
         lcd_i2c_go_xy(8, 2);
         printf(escribir_lcd_i2c,"%d", CEN_PS);
         delay_ms(REBOTE);
      }
    
      if( input(PIN_A1) == 1 && vector == 0 && CEN_PS > 0 ){
         CEN_PS--;
         lcd_i2c_go_xy(8, 2);
         printf(escribir_lcd_i2c,"%d", CEN_PS);
         delay_ms(REBOTE);
      }
 
   } // while
   
   vector = 1;
   delay_ms(REBOTE);
   
   while( input(PIN_C2) == 1 ){
      if( input(PIN_A0) == 1 && vector == 1 && DEC_PS <20 ){
         DEC_PS++;
         lcd_i2c_go_xy(8, 2);
         printf(escribir_lcd_i2c,"%d", DEC_PS);
         delay_ms(REBOTE);
      }
    
      if( input(PIN_A1) == 1 && vector == 1 && DEC_PS > 0 ){
         DEC_PS--;
         lcd_i2c_go_xy(8, 2);
         printf(escribir_lcd_i2c,"%d", DEC_PS);
         delay_ms(REBOTE);
      
      }
   } // while
  
   vector = 2;
   delay_ms(REBOTE);
  
   CEN_PS *= 10;
   TMR_PS = CEN_PS + DEC_PS + UNI_PS;

   int kpl, kil, kdl;
   if(direccion == 3){
      kpl = TMR_PS;
      write_eeprom(3, kpl);
      //kp = kpl / 10;
      kp = kpl;
   }
   if(direccion == 4){
      kil = TMR_PS;
      write_eeprom(4, kil);
      ki = kil / 100;
      //ki = kil;
   }
   if(direccion == 5){
      kdl = TMR_PS;
      write_eeprom(5, kdl);
      kd = kdl / 100;
      //kd = kdl;
   }

} // funcion Interaccion


void ganancias(void){
   
   int8 menu = 0;
   while( input(PIN_C2) == 1 ){
            
      if( input(PIN_A0) == 1 && menu >=0 ){
         menu++;
         delay_ms(REBOTE);
      }
     
      if( input(PIN_A1) == 1 && menu <=3 ){
         menu--;
         delay_ms(REBOTE);
      }
      
      if(menu >= 4){
         menu = 4;
      }
      if(menu <= 0){
         menu = 0;
      }
      

      lcd_i2c_go_xy(1, 2);
      printf(escribir_lcd_i2c,"                ");
      switch(menu){
         case 0:
            lcd_i2c_go_xy(1, 1);
            printf(escribir_lcd_i2c,"       kp       ");
            
            if(input(PIN_C2) == 0){
               lcd_i2c_go_xy(1, 2);
               interaccion('p', 3);
               
               delay_ms(250);
               break;
            }
            break;

               
         case 1:
            lcd_i2c_go_xy(1, 1);
            printf(escribir_lcd_i2c,"       ki       ");
            
            if(input(PIN_C2) == 0){
               lcd_i2c_go_xy(1, 2);
               interaccion('i', 4);
               
               delay_ms(250);
               break;
            }
            break;
            
         case 2:
            lcd_i2c_go_xy(1, 1);
            printf(escribir_lcd_i2c,"       kd       ");
            
            if(input(PIN_C2) == 0){
               lcd_i2c_go_xy(1, 2);
               interaccion('d', 5);
               
               delay_ms(250);
               break;
            }
            break;
                 
         default:
            menu = 0;
            break;
            
      } // switch
   } // while

} // funcion


void main(void){

   set_tris_a(0xFF);
   //set_tris_a(0b00000001);
   set_tris_b(0b11101110);         //Configura puerto B  1= entrada  0= salida
   //set_tris_c(0b10011111);       //Configura puerto C  1= entrada  0= salida
   set_tris_c(0xBF);                      // Configuramos puerto c (PINES DE COMUNICACION SERIAL Y PIN DE ENTRADA BOTON EN RC2)
   
   setup_timer_2(T2_DIV_BY_1, 224, 1);
   setup_ccp2(CCP_PWM);   
 
   inicializa_lcd_i2c();
   
   Mpu6050_Init();
   delay_ms(500);
   
   int8 x;
   x = Mpu6050_Read(MPU6050_RA_WHO_AM_I);
                                                       
   if(x != 0x68){
      //printf ("\nConnection ERR!!!");
      return;
   } // capacitor ceramico de 0.1uF, puente H, a 12V capacitor de almenos 470uF
   

   int8 c_menu = 0;
   float error = 0;
   
   float c_gravedad = 0, seno = 0, ang_aux = 0;
   
   kp = read_eeprom(3);
   //kp = kp / 10;
   ki = read_eeprom(4);
   ki = ki / 10;
   
   kd = read_eeprom(5);
   kd = kd / 10;
   
   unsigned int16 timer = 0;
   float error_new = 0, error_old = 0, derivada = 0, integral = 0;
   
   do{
   
      set_ticks(0);   
      
      accX_H = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_H);  
      accY_H = Mpu6050_Read(MPU6050_RA_ACCEL_YOUT_H);
      accZ_H = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_H);
      
      accX_L = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_L);  
      accY_L = Mpu6050_Read(MPU6050_RA_ACCEL_YOUT_L);
      accZ_L = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
      
      accX = make16(accX_H, accX_L);
      accY = make16(accY_H, accY_L);
      accZ = make16(accZ_H, accZ_L);
      
      
      angX = (atan2(accX, accZ) + PI) * A_DEG;
      no_filtrada_X = angX - 180;
      
      angY = (atan2(accY, accZ) + PI) * A_DEG;
      no_filtrada_Y = angY - 180;
      
      angX = ( FILTRO_C1 * X_old) + ( FILTRO_C2 * no_filtrada_X);
      //angX = no_filtrada_X;
      X_old = angX;
      
      angY = ( FILTRO_C1 * Y_old) + ( FILTRO_C2 * no_filtrada_Y);
      //angY = no_filtrada_X;
      Y_old = angY;
      
      
      if(angX == 0){
         giro = 0;
         sentido = 'S';
      }
      
      else if(angX >= 0){
         giro = 1;
         sentido = 'R';
      }
      else if(angX < 0){
         giro = 2;
         sentido = 'L';
      }
      
      if( input(PIN_A0) == 1 && c_menu >=0 ){
         c_menu++;
         delay_ms(REBOTE);
      }
  
      if( input(PIN_A1) == 1 && c_menu <=3 ){
         c_menu--;
         delay_ms(REBOTE);
      }
   
      if(c_menu >= 4){
         c_menu = 0;
      }
      if(c_menu <= 0){
         c_menu = 0;
      }
      
      lcd_i2c_go_xy(1, 1);
      switch(c_menu){
         case 0:
            printf(escribir_lcd_i2c,"     Inicio      ");
            
            // mgl sen 0
            ang_aux = angX * A_RAD;
            seno = sin(ang_aux);
            c_gravedad = MGL * seno;
            // setpoint es 0, por eso no aparece en el calculo de error
            error = angX;
            
            error_new = error;
            derivada = (error_new - error_old) / timer;
            integral = integral + error_new;
            
            
            if(error < 0){
               error = error * -1;
            }
            
            //velo = (kp * error) + MINIMA_MOTOR + (derivada * kd) + (integral * ki);
            velo = (kp * error) + (derivada * kd) + (integral * ki) + c_gravedad;
            
            if(velo < 0){
               velo = velo * -1;
            }
            
            if(giro == 1){
               output_low(PIN_B5);
               output_high(PIN_B6);
            }
            else if(giro == 2){
               output_low(PIN_B6);
               output_high(PIN_B5);
            }
            else{
               output_low(PIN_B6);
               output_low(PIN_B5);
            }
            
            velocidad = (int16)velo;
            
            if(velocidad >= PWM_MAX){
               velocidad = PWM_MAX;
            }
            set_pwm2_duty(velocidad);
            
            lcd_i2c_go_xy(1, 2);
            printf(escribir_lcd_i2c,"   %ld%c  %ldms    ", velocidad, sentido, timer);
            //delay_ms(25);
            break;
               
         case 1:
            printf(escribir_lcd_i2c,"      MPU       ");
            lcd_i2c_go_xy(1, 2);
            printf(escribir_lcd_i2c,"                ");
            if(input(PIN_C2) == 0){
               delay_ms(REBOTE);
               angulos();
               c_menu = 0;
               break;
            }
            break;
               
         case 2:
            printf(escribir_lcd_i2c,"    Pruebas     ");
            lcd_i2c_go_xy(1, 2);
            printf(escribir_lcd_i2c,"    Motores     ");
            
            if(input(PIN_C2) == 0){
               delay_ms(REBOTE);
               pMotores();
               c_menu = 0;
               break;
               }
            break;
            
          case 3:
            printf(escribir_lcd_i2c, "   Ganancias    ");
            lcd_i2c_go_xy(1, 2);
            printf(escribir_lcd_i2c, " %2.0fKp %1.1fKi %1.1fKd    ", kp, ki, kd);
            
            if(input(PIN_C2) == 0){
               delay_ms(REBOTE);
               ganancias();
               c_menu = 0;
               break;
            }
            break;
               
         default:
            printf(escribir_lcd_i2c,"CASO DEFAULT %d", c_menu);
            c_menu = 0;
            break;
      } // switch
      
      //set_ticks(0);
      //delay_ms(1);
      
      error_old = error_new;
      timer = (get_ticks() ) / 2;  
      
      }while(TRUE); // bucle infinito.

} // main


