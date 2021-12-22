#define FACTOR_USV 0.0057 // datasheet do tubo Conversão para Fator Sieverts 
#define GM_TUBE 3

/*Protitipos de funções */
uint32_t update_cps(void);
void dataProcess(void);
void configMCU(void);

/*Varaveis Globais */
uint32_t count = 0, CPS = 0, value[5];
unsigned long timeCurrent, time_Count;
bool is_update = false;

void setup() {
  configMCU();
}

void loop() {
  timeCurrent = millis();

}

void dataProcess(void){// Processo de aquisão e tratamento dos dados
 uint8_t INDEX;
  if (timeCurrent - time_Count > 100){ 
   value[INDEX] = count;
 
   // Integra os valor de contagem realizado a cada 10ms (x5)
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

uint32_t update_cps(void){
  uint32_t newvalueCPS; 
  if(is_update){
      
      newvalueCPS = (CPS/5)* (FACTOR_USV);
      is_update = false;
  }
return newvalueCPS;
}

static void eventTube(void){ // Realiza a soma dos pulsos "enviados" pelo tubo
 if (!digitalRead(GM_TUBE)){count++;}
}

void configMCU(void){
    pinMode(GM_TUBE, INPUT);
}
