/*Bibliotecas */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define FACTOR_USV 0.0057 // datasheet do tubo Conversão para Fator Sieverts 
#define GM_TUBE_PIN 3
#define INTERRUPT_PIN 2
#define LED_PIN 18 
#define I2C_SCL 8
#define I2C_SDA 9

/*Protitipos de funções */
uint16_t update_cps(void);
void dataProcess(void);
void configMCU(void);
void dmpDataReady(void);
void get_teapot(void);
String get_accel_world(void);
String get_accel(void);
String get_quartenion(void);

/*Varaveis Globais */
uint16_t count = 0, CPS = 0, value[5];
unsigned long timeCurrent, time_Count;
bool is_update = false;
bool status_I2C = false,blinkState =false;

bool dmpReady = false;  // verifica se inicialização foi correta
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;   
uint16_t fifoCount;     // contabiliza o numero de bytes atual na FIFO
uint8_t fifoBuffer[64]; 

Quaternion q;         
VectorInt16 aa;         
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;   // vetor gravidade em [x,y,z]
float euler[3];         
float ypr[3]; 

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;  


/*Criando objeto*/
MPU6050 mpu; /*Por padrão o endereço I2C do MPU é 0x68*/

void setup() {
  configMCU();
}

void loop() {
  timeCurrent = millis();
  if (!dmpReady){Serial.println("Perda de comunicação com MPU"); return;}
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
        get_teapot();
        blinkState = !blinkState;// Indicar que está em operação
        digitalWrite(LED_PIN, blinkState);
    } 
  eventTube();
  dataProcess();
  update_cps();
}

void dataProcess(void){// Processo de aquisão e tratamento dos dados
 uint8_t INDEX;
  if (timeCurrent - time_Count > 100){ 
   value[INDEX] = count;
 
   // Integra os valor de contagem realizado a cada 100ms (x5)
   // Após (500) milisegundo os valor estarão armazenado em CPS
   CPS = value[0] + value[1] + value[2] + value[3] + value[4];

   if (INDEX == 4) {
       is_update = true;
       // Reseta as variaveis de contagem para iniciar um novo processo    
       value[0] = 0;
       value[1] = 0; 
       value[2] = 0; 
       value[3] = 0; 
       value[4] = 0; 
       
       INDEX = 0;      // Reseta o processo de integração     
    }
   else {INDEX ++;} 
  
  } 

 time_Count = timeCurrent;

}

uint16_t update_cps(void){
  uint16_t newvalueCPS; 
  if(is_update){
      
      newvalueCPS = (CPS/5);
      // Serial.println(newvalueCPS);
      // Serial.write(newvalueCPS,16);
      is_update = false;
  }
return newvalueCPS;
}

void eventTube(void){ // Realiza a soma dos pulsos "enviados" pelo tubo
 if (!digitalRead(GM_TUBE_PIN)){count++;}
}

void configMCU(void){

    status_I2C = Wire.begin(I2C_SCL,I2C_SDA); // Inicializa o barramento I2C
    Serial.begin(115200);       // Inicializa a comunicação Serial 
    if(status_I2C){Serial.print("Erro I2C \n");}
    while(!Serial);// Enquanto o canal de comunicação serial não for aberto... nãos segue para a inicialização do dispositivo
    
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(GM_TUBE_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    
    if(mpu.testConnection()){Serial.print("MPU6050 funcionando\n");}
    else{Serial.print("MPU6050 erro\n");}                                    
   
    devStatus = mpu.dmpInitialize();
    /* Os valores de offset para Giroscopio deve ser verificado antes de 
     *  preencher os parametros das quatro funções abaixo
    */
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) { // Conferindo se a inicialização ocorreu nos conformes
       
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        digitalPinToInterrupt(INTERRUPT_PIN);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize(); // Pega o tamanho do último pacote de dados
    } else {
        Serial.println("Erro devStatus -> ");
        Serial.println(devStatus);
    }
    
}

void get_teapot(void){
  
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; 
}
String get_accel_world(void){
  String accel;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  accel = "areal\t"
         + String(aaWorld.x)
         + "\t" 
         + String(aaWorld.y)
         + "\t" 
         + String(aaWorld.z);  
  return accel;
}
String get_accel(void){
  String accel;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  accel = "areal\t"
         + String(aaReal.x)
         + "\t" 
         + String(aaReal.y)
         + "\t" 
         + String(aaReal.z);        
  return accel;
} 
String get_YamPitchRoll(void){
  String ypr_string;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  ypr_string =  "ypr\t"
                + String(ypr[0] * 180/M_PI)
                + "\t" 
                + String(ypr[1] * 180/M_PI)
                + "\t" 
                + String(ypr[2] * 180/M_PI);
  
  return ypr_string;
}
String get_quartenion(void){
   String quartenion ; 
   mpu.dmpGetQuaternion(&q, fifoBuffer);  
   quartenion = "quat\t"+String(q.w) 
                + "\t" + String(q.x) 
                + "\t" + String(q.y)
                + "\t" + String(q.z);
     return quartenion;
}
void dmpDataReady(void){ // Chamada da função sempre que o pino de interrupção do MPU for nível lógico alto
  mpuInterrupt = true;
}
