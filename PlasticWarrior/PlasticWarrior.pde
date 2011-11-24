/*
Copyright 2011 Jonathan Castro

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Author: Jonathan Castro.
  Co-Author and Shield PCB disigner: Engineer Joksan Alvarado.
  Project Name: PlasticWarrior
  Description: A Robotic arm developed in Arduino
*/

#include <Servo.h>

Servo MotorWarrior[7];

int valx = 0;
int valy = 0;
int Motor[7] = {7, 8, 9, 10, 11, 12, 13};
//int posAngular[7]={426, 472, 472,341, 568, 425, 570};
int posAngular[7] = {75, 75, 72, 64, 121, 75, 0 }; //posicion Inici

int velDiv=0;

enum Estados {base,  muneca,  codo, secuencia, descansando};
enum Estados estado_actual = base;

//definicion de estados para secuencia1

enum estados_secuencia {subirIzquierda, desplazarDerecha, bajarDerecha, subirDerecha, desplazarIzquierda, bajarIzquierda, paro, inicial};
enum estados_secuencia estado_secuencia1 = inicial;

boolean botones[5]={HIGH, HIGH, HIGH, HIGH, HIGH};
boolean botonAnterior[5]={HIGH, HIGH, HIGH, HIGH, HIGH};
boolean botonPresionado[5];




void setup(){
  
  for(int PinCounter = 2; PinCounter<7; PinCounter++){
    pinMode(PinCounter, INPUT);
    digitalWrite(PinCounter, HIGH);
  }
  Serial.begin(9600);
  
//valores iniciales
  
  for(int i = 0; i<7; i++){
    MotorWarrior[i].attach(Motor[i]);
    MotorWarrior[i].write(posAngular[i]);
  }
  
}

void loop(){
  inData();
  MEF1();
  outData();
}

void inData(){
//    valx = (map(analogRead(A0),0,1023,-512,511) + 10)/32;
//    valy = (map(analogRead(A1),0,1023,-512,511) + 5)/32;
   
   valx = map(analogRead(A0),0,1023,-45,45)/8;
   valy = map(analogRead(A1),0,1023,-45,45)/8;
   
    for(int i = 0; i<5; i++){
      botonAnterior[i]=botones[i];
      botones[i]=digitalRead(i+2);
      botonPresionado[i]=(!(botonAnterior[i]) && botones[i]);
    }
    
}

void MEF1(){
  switch(estado_actual){
    
    case base:
      posAngular[0] = posAngular[0] - valx;   //integrador para Motor0
      posAngular[1] = posAngular[1] + valy;   //integrador para Motor1

      if(botonPresionado[0]) estado_actual=descansando;      
      if(botonPresionado[1]) estado_actual=codo;
      if(botonPresionado[2]) estado_actual=muneca;
      if(botonPresionado[4]){ estado_actual=secuencia;
                              estado_secuencia1=inicial;}
      
    break;
    
    case codo:
      posAngular[3]=posAngular[3] - valy;
      posAngular[6]=posAngular[6] + valx;

      if(botonPresionado[0]) estado_actual=descansando;
      if(botonPresionado[2]) estado_actual=muneca;
      if(botonPresionado[3]) estado_actual=base;
      if(botonPresionado[4]){ estado_actual=secuencia;
                              estado_secuencia1=inicial;}          
    break;
    
    case muneca:
      posAngular[5]=posAngular[5] + valx;
      posAngular[4]=posAngular[4] + valy;
      
      if(botonPresionado[0]) estado_actual=descansando;      
      if(botonPresionado[1]) estado_actual=codo;
      if(botonPresionado[3]) estado_actual=base;
      if(botonPresionado[4]){ estado_actual=secuencia;
                              estado_secuencia1=inicial;}
    
    break;
   
    case descansando:
         posInicial();
  
         if(botonPresionado[1]) estado_actual=codo;
         if(botonPresionado[2]) estado_actual=muneca;
         if(botonPresionado[3]) estado_actual=base;
         if(botonPresionado[4]){ estado_actual=secuencia;
                              estado_secuencia1=inicial;}
    break;
    
    case secuencia:
      switch(estado_secuencia1){
        case inicial:
          posInicial();
         
         if((posAngular[0]==75) && (posAngular[1]==75) && (posAngular[2]==72) && (posAngular[3]==64) && (posAngular[4]==121) && (posAngular[5]==75) && (posAngular[6]==12))
           estado_secuencia1 = desplazarIzquierda;
        break;
        
        case desplazarIzquierda:
           if(posAngular[0]>170) posAngular[0]--;
           else if(posAngular[0]<170) posAngular[0]++;
           else if(posAngular[0] == 170) estado_secuencia1 = subirIzquierda;
          
        break;

        
        case subirIzquierda:
          if(posAngular[4]>20) posAngular[4]--;
          else if(posAngular[4]<20) posAngular[4]++;
          if(posAngular[4]==20){
            if(posAngular[6]>50) posAngular[6]--;
            else if(posAngular[6]<50) posAngular[6]++;
            else estado_secuencia1=bajarIzquierda;
          }
        
        break;

        case subirDerecha:
          if(posAngular[4]>20) posAngular[4]--;
          else if(posAngular[4]<20) posAngular[4]++;
          if(posAngular[4]==20){
            if(posAngular[6]>50) posAngular[6]--;
            else if(posAngular[6]<50) posAngular[6]++;
            else estado_secuencia1=bajarDerecha;
          }          
        
        break;
        
        case bajarIzquierda:
          if(posAngular[6]>12) posAngular[6]--;
          else if(posAngular[6]<12) posAngular[6]++;
          if(posAngular[6]==12){
            if(posAngular[4]>121) posAngular[4]--;
            else if(posAngular[4]<121) posAngular[4]++;
            else estado_secuencia1=desplazarDerecha;
          }
        
        break;

        case bajarDerecha:
          if(posAngular[6]>12) posAngular[6]--;
          else if(posAngular[6]<12) posAngular[6]++;
          if(posAngular[6]==12){
            if(posAngular[4]>121) posAngular[4]--;
            else if(posAngular[4]<121) posAngular[4]++;
            else estado_secuencia1=desplazarIzquierda;
          }
        
        break;

        case desplazarDerecha:
          if(posAngular[0]>75) posAngular[0]--;
           else if(posAngular[0]<75) posAngular[0]++;
           else if(posAngular[0] == 75) estado_secuencia1 = subirDerecha;
        
        break;
        



        case paro:
          
        
        break;
        
      }
      
    
      if(botonPresionado[0]) estado_actual=descansando;
      if(botonPresionado[1]) estado_actual=codo;
      if(botonPresionado[2]) estado_actual=muneca;
      if(botonPresionado[3]) estado_actual=base;
      
    break;

  }

}

void outData(){
    int desfase = -3;
    

    for(int i=0; i<7; i++){
      
      if(posAngular[i]>180) posAngular[i]=180;
      else if(posAngular[i]<=3) posAngular[i]=3;
      
      posAngular[2]= posAngular[1] + desfase;
      
      if(posAngular[6]>90) posAngular[6]=90;
      else  if(posAngular[6] < 12) posAngular[6]=12;
      
      MotorWarrior[i].write(posAngular[i]);
   // Serial.print(posAngular[i],DEC);
   // Serial.print(" ");
    }
   // Serial.print(valx,DEC);
   // Serial.print(" ");
   
   Serial.print(posAngular[4],DEC);
   Serial.print("\n");
    delay(50);
}

void posInicial(){


         
         if(posAngular[3] > 64) posAngular[3]--;
         else if(posAngular[3] < 64) posAngular[3]++;
         
         if(posAngular[6] > 12) posAngular[6]--;

     
     if(posAngular[0] > 75) posAngular[0]=posAngular[0]-1;
     else if(posAngular[0] < 75) posAngular[0]=posAngular[0]+1;
          
     if(posAngular[1] > 75) posAngular[1]=posAngular[1]-1;
     else if(posAngular[1] < 75) posAngular[1]=posAngular[1]+1;
               
     if(posAngular[2] > 72) posAngular[2]=posAngular[2]-1;
     else if(posAngular[2] < 72) posAngular[2]=posAngular[2]+1;
               
               
     if(posAngular[4] > 121) posAngular[4]=posAngular[4]-1;
     else if(posAngular[4] < 121) posAngular[4]=posAngular[4]+1;
               
     if(posAngular[5] > 75) posAngular[5]=posAngular[5]-1;
     else if(posAngular[5] < 75) posAngular[5]=posAngular[5]+1;
           
  }


