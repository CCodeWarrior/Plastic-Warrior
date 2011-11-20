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
*/

#include <Servo.h>

Servo MotorWarrior[7];

int valx = 0;
int valy = 0;
int Motor[7] = {7, 8, 9, 10, 11, 12, 13};
int Acumulador[7]={512, 512, 512, 512, 512, 512, 512};
char posAngular[7] = {};

enum Estados {base,  muneca,  codo, secuencia};
enum Estados estado_actual = base;

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
    posAngular[i]=45;
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
    valx = (map(analogRead(A0),0,1023,-512,511) + 10)/16;
    valy = (map(analogRead(A1),0,1023,-512,511) + 5)/16;
    for(int i = 0; i<5; i++){
      botonAnterior[i]=botones[i];
      botones[i]=digitalRead(i+2);
      botonPresionado[i]=(!(botonAnterior[i]) && botones[i]);
    }
    
}

void MEF1(){
  switch(estado_actual){
    
    case base:
      Acumulador[0] = Acumulador[0] + valx;   //integrador para Motor0
      Acumulador[1] = Acumulador[1] + valy;   //integrador para Motor1
      
      if(botonPresionado[1]) estado_actual=codo;
      if(botonPresionado[2]) estado_actual=muneca;
      if(botonPresionado[4]) estado_actual=secuencia;      
    break;
    
    case codo:
      Acumulador[3]=Acumulador[3] + valy;
      Acumulador[6]=Acumulador[6] + valx;
      
      if(botonPresionado[2]) estado_actual=muneca;
      if(botonPresionado[3]) estado_actual=base;
      if(botonPresionado[4]) estado_actual=secuencia;            
    break;
    
    case muneca:
      Acumulador[5]=Acumulador[5] + valx;
      Acumulador[4]=Acumulador[4] + valy;
      
      if(botonPresionado[1]) estado_actual=codo;
      if(botonPresionado[3]) estado_actual=base;
      if(botonPresionado[4]) estado_actual=secuencia;    
    
    break;
   
    
    case secuencia:
    
      if(botonPresionado[1]) estado_actual=codo;
      if(botonPresionado[2]) estado_actual=muneca;
      if(botonPresionado[3]) estado_actual=base;
    break;

  }

}

void outData(){
    for(int i=0; i<7; i++){
      if(Acumulador[i]>1023) Acumulador[i]=1023;
      else if(Acumulador[i]<0) Acumulador[i]=0;
  
      posAngular[i]=map(Acumulador[i],0,1023,0,90);
      MotorWarrior[i].write(posAngular[i]);
      //visualizacion de variables
      Serial.print(posAngular[i],DEC); 
      Serial.print(" ");
    }
    Serial.print(valx, DEC);
    Serial.print(" ");
    Serial.print(valy, DEC);
    Serial.print(" ");
    Serial.print(estado_actual, DEC);
    Serial.println(" ");    

    delay(100);
}


