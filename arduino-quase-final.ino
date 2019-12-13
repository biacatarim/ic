/*
 * Projeto: Expansao de portas analogicas 8 canais do Arduino Nano V3.0 e envio das leituras por serial.
 * Autor: FilipeFlop e Pedro Bertoleti
 * Data: Janeiro/2018
 */
 
//Defines
#define TAMANHO_STRING_SERIAL 30     //((4*6) + 7 + 1)  //6 canais (de 4 bytes de informação cada, em ASCII) e 7 separadores (;) e 1 terminador de string (\0)
#define ldrpin 0 
//Variáveis globais
int LeiturasADC[6]; //armazenará as leituras dos canais de ADC
 
//protótipops
void FazLeituraCanaisADC(void);
void TransmiteLeiturasADC(void);

//Porta ligada ao pino IN1 do modulo
int porta_rele1 = 14; //lampada
//Porta ligada ao pino IN2 do modulo
int porta_rele2 = 15; //ventilador
//Porta ligada ao botao 1
int porta_botao1 = 17; 
//Porta ligada ao botao 2
int porta_botao2 = 18;
 
//Armazena o estado do rele - 0 (LOW) ou 1 (HIGH)
int estadorele1 = 1;
int estadorele2 = 1;
//Armazena o valor lido dos botoes
int leitura1 = 0;
int leitura2 = 0;
 
/*
 * Implementações
 */
 
//Função: faz a leitura dos canais de ADC
//Parâmetros: nenhum
//Retorno: nenhum
void FazLeituraCanaisADC(void)
{
  char i;
 
  for(i=0; i<8; i++)
    LeiturasADC[i] = analogRead(i);
}
 
//Função: Transmite via serial, na forma textual/string, as leituras de ADC obtidas
//Parâmetros: nenhum
//Retorno: nenhum
void TransmiteLeiturasADC(void)
{
   char InfoCanaisADC[TAMANHO_STRING_SERIAL];
 
   //limpa string
   memset(InfoCanaisADC,0,TAMANHO_STRING_SERIAL);
 
   //coloca as leituras numa string
   sprintf(InfoCanaisADC,"%04d;%04d;%04d;%04d;%04d;%04d;%04d;%04d", LeiturasADC[0], 
                                                                    LeiturasADC[1], 
                                                                    LeiturasADC[2], 
                                                                    LeiturasADC[3], 
                                                                    LeiturasADC[4], 
                                                                    LeiturasADC[5]);
                                                                  
 
   //Transmite a string pela serial
   Serial.write(InfoCanaisADC, TAMANHO_STRING_SERIAL);                                                                     
}
  
void setup() 
{
  memset(LeiturasADC,0,sizeof(LeiturasADC));
   
  //configura o baudrate da comunicação serial em 19200
  Serial.begin(19200); 
  //Define pinos para o rele como saida
  pinMode(porta_rele1, OUTPUT); 
  pinMode(porta_rele2, OUTPUT);
  //Define pinos dos botoes como entrada
  pinMode(porta_botao1, INPUT); 
  pinMode(porta_botao2, INPUT);
  //Estado inicial dos reles - desligados
  digitalWrite(porta_rele1, HIGH);
  digitalWrite(porta_rele2, HIGH);
    
}
 
void loop() {

 int readLdr = analogRead(ldrpin);
 readLdr = LeiturasADC[0];
delay(500);
  FazLeituraCanaisADC();  
  TransmiteLeiturasADC();

    //Verifica o acionamento do botao 1
  leitura1 = digitalRead(porta_botao1);
  if (leitura1 != 0)
  {
    while(digitalRead(porta_botao1) != 0)
    {
      delay(100);
    }
    //Inverte o estado da porta
    estadorele1 = !estadorele1;
    //Comandos para o rele 1
    digitalWrite(porta_rele1, estadorele1);  
  }   
   
  //Verifica o acionamento do botao 2
  leitura2 = digitalRead(porta_botao2);
  if (leitura2 != 0)
  {
    while(digitalRead(porta_botao2) != 0)
    {
      delay(100);
    }
    //Inverte o estado da porta
    estadorele2 = !estadorele2;
    //Comandos para o rele 2
    digitalWrite(porta_rele2, estadorele2);  

   if (readLdr < 7) {
     digitalWrite(porta_rele1, LOW);
   }
  
  } 
}
