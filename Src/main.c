/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* USER CODE BEGIN Includes */
//Dados do autômato (Não pode ser declarado dentro da função main por ser const)
#define NTRANS 114	//Número de Transições
#define NESTADOS 29	//Número de Estados
#define BUFFER 10  	//Máximo Número de Eventos no Buffer
const unsigned int event[NTRANS]={20,18,2,6,10,12,14,16,1,19,16,18,4,6,16,11,18,2,8,16,3,18,16,13,18,16,5,18,16,9,17,18,19,18,2,6,10,12,14,16,15,2,6,10,12,14,19,18,4,8,16,
		21,18,4,6,16,18,2,8,16,19,7,2,6,10,12,14,18,16,13,18,16,5,18,16,9,17,15,2,6,10,12,14,19,16,21,18,19,21,18,4,8,16,7,2,6,10,12,14,2,6,10,
		12,14,1,19,21,19,21,2,6,10,12,14};
const unsigned int in_state[NTRANS]={1,2,3,4,5,6,7,8,9,10,11,2,1,12,8,13,2,12,1,8,14,2,8,1,2,8,1,2,8,1,1,15,16,2,3,4,17,18,19,8,1,3,4,5,6,7,20,21,4,3,22,
		23,2,9,12,8,2,12,9,8,24,1,3,4,5,6,7,2,8,9,2,8,9,2,8,9,9,25,3,4,5,6,7,10,26,2,27,16,8,2,4,3,8,25,3,4,5,6,7,3,4,5,
		6,7,28,20,11,24,15,3,4,17,18,19};
const unsigned int rfirst[NESTADOS] = {1,9,11,16,21,24,27,31,33,40,46,47,52,56,60,61,67,70,73,77,83,86,89,93,99,105,107,109,114};
const unsigned int rnext[NTRANS] = {0,0,2,3,4,5,6,7,8,0,10,0,12,13,14,15,0,17,18,19,20,0,22,23,0,25,26,0,28,29,30,0,32,0,34,35,36,37,38,39,0,41,42,43,44,45,0,0,48,49,50,
		51,0,53,54,55,0,57,58,59,0,0,62,63,64,65,66,0,68,69,0,71,72,0,74,75,76,0,78,79,80,81,82,0,84,85,0,87,88,0,90,91,92,0,94,95,96,97,98,0,100,101,
		102,103,104,0,106,0,108,0,110,111,112,113};

#define TAM 8
//mapeamento de eventos não controláveis como entradas
#define Bl 20	//Entrada 0
#define p1a 2	//Entrada 1
#define p1p 4	//Entrada 2
#define p2a 6	//Entrada 3
#define p2p 8	//Entrada 4
#define p3a 10	//Entrada 5
#define p4a 12	//Entrada 6
#define p5a 14	//Entrada 7
#define s1 16	//Entrada 8
#define s2 18	//Entrada 9


#define VELO 0.3
//definição dos eventos de saída
//Timer 2 corresponde ao PWM gerado para acionamento dos motores
//Timer 3 corresponde ao PWM gerado para acionamento dos servo motores para ajuste da angulação das bandeiras

#define EF_50  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,200*VELO);	//Saida 0 PWM 50%
#define EF_100 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,200);	//Saida 0 PWM 100%
#define EF_0 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,0);	//Saida 0 PWM 0%
#define ER_50   __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,200*VELO);	//Saida 1 PWM 50%
#define ER_100 __HAL_TIM_SetCompare(&htim2	,TIM_CHANNEL_1,200);	//Saida 1 PWM 100%
#define ER_0  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,0);	//Saida 1 PWM 0%
#define DF_50  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,200*VELO);	//Saida 2 PWM 50%
#define DF_100 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,200);	//Saida 2 PWM 100%
#define DF_0 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,0);	//Saida 2 PWM 0%
#define DR_50   __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,200*VELO);	//Saida 3 PWM 50%
#define DR_100 __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,200);	//Saida 3 PWM 100%
#define DR_0  __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,0);	//Saida 3 PWM 0%
#define SM1_0 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,10);	//Saida 4 PWM 40%
#define SM1_180 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,20);	//Saida 4 PWM 60%
#define SM2_0 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,5);	//Saida 4 PWM 40%
#define SM2_180 __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,17);	//Saida 4 PWM 60%

// Macros para definir sinal lógico do LED
#define LD_ON  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);	//Saida 4 ON
#define LD_OFF HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);	//Saida 4 OFF
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/



/* USER CODE BEGIN PV */
char *bufftr;
uint8_t buffrc[2];
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* Private variables ---------------------------------------------------------*/
unsigned char buffer[BUFFER];		//Buffer para armazenar a fila de enventos externos
unsigned char n_buffer=0;		//Número de eventos no Buffer
unsigned int i;
unsigned int tempo=0;
char buf[32];
unsigned int vec[20];
unsigned int n_event=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void GPIO_INIT(void);
void Temporizador_ms(unsigned int TEMPO);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void freia(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Configuração do microcontrolador*/

  /* Reseta os periféricos e reinicializa a flash */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  	SystemInit();
    SystemCoreClockUpdate();
  /* USER CODE END SysInit */

  /* Inicializa os periféricos configurados */
  MX_GPIO_Init();
 // MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();


  /* USER CODE BEGIN 2 */
 // __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
 // __HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);

  //ajusta a angulação inicial dos servo motores
   //SM1_0;
   //SM2_180;


  GPIO_INIT();

  	unsigned int k;
	int occur_event=-1;			//Evento ocorrido
	unsigned int current_state = 0;	//Estado atual inicializado com estado inicial
	int g=0; 			//Flag para gerador aleatório de eventos
	int gerar_evento=1;			//Flag para habilitar a temporização de eventos controláveis
	int mealy_output = 0;		//Inicializa saída periférica

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  if(n_buffer == 0)//se não existir evento no buffer então gerar um evento interno(evento controlável)
	  		{
	  			if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)== SET)	//Se o timer estourar, habilita a geração de eventos
	  			{
					__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
					HAL_TIM_Base_Stop_IT(&htim1);
					gerar_evento=1;
	  			}
	  			if(gerar_evento==1)
	  			{
	  				switch(g)	//Aqui é implementado um gerador automático de eventos controláveis
	  				{
	  				case(0):
	  					occur_event=1;
	  					g++;
	  					break;
	  				case(1):
	  					occur_event=3;
	  					g++;
	  					break;
	  				case(2):
	  					occur_event=5;
	  					g++;
	  					break;
	  				case(3):
	  					occur_event=7;
	  					g++;
	  					break;
	  				case(4):
	  					occur_event=9;
	  					g++;
	  					break;
	  				case(5):
	  					occur_event=11;
	  					g++;
	  					break;
	  				case(6):
	  					occur_event=13;
	  					g++;
	  					break;
	  				case(7):
	  					occur_event=15;
	  					g++;
	  					break;
	  				case(8):
	  					occur_event=17;
	  					g++;
	  					break;
	  				case(9):
	  					occur_event=19;
	  					g++;
	  					break;
	  				case(10):
	  					occur_event =21;
	  					g=0;
	  					break;
	  				}
	  			}
	  		}
	  		else 	//se existir evento não controlável pegar do buffer
	  		{
	  			occur_event = buffer[0];
	  			n_buffer--;
	  			k = 0;
	  			while(k<n_buffer)
	  			{
	  				buffer[k] = buffer[k+1];
	  				k++;
	  			}
	  		}

	  		//Jogador de autômato
	  		k = rfirst[current_state];
	  		if(k==0)
	  		{
	  			return 0;     //Dead Lock!!!
	  		}
	  		else
	  		{
	  			while(k>0)
	  			{
	  				k--;
	  				if(event[k] == occur_event)
	  				{
	  					current_state = in_state[k];
	  					mealy_output = 1;
	  					break;
	  				}
	  				k = rnext[k];
	  			}
	  		}

	  		if(mealy_output) //Se o evento ocorrido for válido, então imprimir saída física
	  		{
	  			switch(occur_event)
	  			{
	  				case(20):	//Adicionar Ação para o Evento 20 -- Bl;
	  					GPIO_INIT();
	  					snprintf(buf,TAM,"\n\rInicializando\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
	  					for(i=0;i<5;i++){
	  						LD_ON;
	  						Temporizador_ms(500);
	  						while(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)== RESET);
	  						__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	  						HAL_TIM_Base_Stop_IT(&htim1);
	  						LD_OFF;
	  						Temporizador_ms(500);
	  						while(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)== RESET);
	  						__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	  						HAL_TIM_Base_Stop_IT(&htim1);
	  						strcpy(buf,"");
	  					}
	  				  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	  				  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

	  					//aciona as saídas e altera a angulação dos servo-motores
	  					 EF_50;
	  					 DF_50;
	  					 DR_0;
	  					 ER_0;
	  					 SM1_0;
	  					 SM2_180;
	  					Temporizador_ms(250);
	  					gerar_evento=0;
	  					break;
	  				case(1):	//Adicionar Ação para o Evento 1 -- f;
						snprintf(buf,TAM,"\n\rf\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_50;
	  					ER_0;
	  					DF_50;
	  					DR_0;
	  					gerar_evento=0;
	  					break;

	  				case(2):	//Adicionar Ação para o Evento 2 -- p1a;
						snprintf(buf,TAM,"\n\rp1a\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento = 1;
			  			break;
	  				case(4):	//Adicionar Ação para o Evento 4 -- p1p;
						snprintf(buf,TAM,"\n\rp1p\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(3):	//Adicionar Ação para o Evento 3 -- gd1;
						snprintf(buf,TAM,"\n\rgd1\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_50;
	  					ER_0;
	  					DF_0;
	  					DR_0;
	  					gerar_evento=0;
	  					break;
	  				case(5):	//Adicionar Ação para o Evento 5 -- gd2;
						snprintf(buf,TAM,"\n\rgd2\n\r");
	  					HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_50;
	  					ER_0;
	  					DF_0;
	  					DR_50;
	  					Temporizador_ms(250);
	  					gerar_evento=0;
	  					break;
	  				case(6):	//Adicionar Ação para o Evento 6 -- p2a;
						snprintf(buf,TAM,"\n\rp2a\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(7):	//Adicionar Ação para o Evento 7 -- gd3;
						snprintf(buf,TAM,"\n\rgd3\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
			  			GPIO_INIT();
	  					EF_50;
	  					ER_0;
	  					DF_0;
	  					DR_50;
	  					Temporizador_ms(350);
	  					gerar_evento=0;
	  					break;
	  				case(8):	//Adicionar Ação para o Evento 8 -- p2p;
						snprintf(buf,TAM,"\n\rp2p\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(9):	//Adicionar Ação para o Evento 9 -- gd4;
						snprintf(buf,TAM,"\n\rgd4\n\r");
			  			HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
			  			strcpy(buf,"");GPIO_INIT();
	  					EF_50;
	  					ER_0;
	  					DF_0;
	  					DR_50;
	  					Temporizador_ms(450);
	  					gerar_evento=0;
	  					break;
	  				case(10):	//Adicionar Ação para o Evento 10 -- p3a;
						snprintf(buf,TAM,"\n\rp3a\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(11):	//Adicionar Ação para o Evento 11 -- ge1;
						snprintf(buf,TAM,"\n\rge1\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_0;
	  					ER_0;
	  					DF_50;
	  					DR_0;
	  					gerar_evento=0;
	  					break;
	  				case(12):	//Adicionar Ação para o Evento 12 -- p4a;
						snprintf(buf,TAM,"\n\rp4a\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(13):	//Adicionar Ação para o Evento 13 -- ge2;
						snprintf(buf,TAM,"\n\rge2\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_0;
	  					ER_50;
	  					DF_50;
	  					DR_0;
	  					Temporizador_ms(250);
	  					gerar_evento=0;
	  					break;
	  				case(14):	//Adicionar Ação para o Evento 14 -- p5a;
						snprintf(buf,TAM,"\n\rp5a\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(15):	//Adicionar Ação para o Evento 15 -- ge3;
						snprintf(buf,TAM,"\n\rge3\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_0;
	  					ER_50;
	  					DF_50;
	  					DR_0;
	  					Temporizador_ms(350);
	  					gerar_evento=0;
	  					break;
	  				case(16):	//Adicionar Ação para o Evento 16 -- s1;
						snprintf(buf,TAM,"\n\rs1\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(17):	//Adicionar Ação para o Evento 17 -- ge4;
						snprintf(buf,TAM,"\n\rge4\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_0;
	  					ER_50;
	  					DF_50;
	  					DR_0;
	  					Temporizador_ms(450);
	  					gerar_evento=0;
	  					break;
	  				case(18):	//Adicionar Ação para o Evento 18 -- s2;
						snprintf(buf,TAM,"\n\rs2\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						gerar_evento=1;
	  					break;
	  				case(19):	//Adicionar Ação para o Evento 19 -- r;
						snprintf(buf,TAM,"\n\r r\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
					    freia();
						Temporizador_ms(100);
						while(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)== RESET);
						__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
						HAL_TIM_Base_Stop_IT(&htim1);
	  					EF_0;
	  					ER_50;
	  					DF_0;
	  					DR_50;
	  					Temporizador_ms(1000);
	  					gerar_evento=0;
	  					break;
	  				case(21): //Adicionar acao para o evento 21 -- ft
						snprintf(buf,TAM,"\n\rft\n\r");
						HAL_UART_Transmit_IT(&huart1, buf ,strlen(buf));
						strcpy(buf,"");
						GPIO_INIT();
	  					EF_100;
						ER_0;
						DF_100;
						DR_0;
						gerar_evento=0;
						break;
	  			}//fim switch
	  			mealy_output = 0;
	  			occur_event = -1;
	  		}//fim if(mealy_output)
	  	}//fim while(1)

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}
  /* USER CODE END 3 */


/** Função de configuração do clock do sistema
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


//Função que inicializa a contagem para que sejam gerados eventos controláveis
void Temporizador_ms(unsigned int TEMPO)
{
	//HAL_TIM_Base_DeInit(&htim1);
	 __HAL_TIM_SET_AUTORELOAD(&htim1, TEMPO-1);
	 //HAL_TIM_Base_Init(&htim1);
	 HAL_TIM_Base_Start_IT(&htim1);
}


//Função que inicializa as saídas
void GPIO_INIT(void){
	EF_0;
	ER_0;
	DF_0;
	DR_0;
	LD_OFF;
}



/* Função de inicialização Timer 1 */
static void MX_TIM1_Init(void)
{
	//Configura um Timer de Captura com período de 1 ms
	__HAL_RCC_TIM1_CLK_ENABLE();

	 htim1.Instance = TIM1;
	 htim1.Init.Period = 0;
	 htim1.Init.Prescaler = 16000-1;
	 htim1.Init.ClockDivision = 0;
	 htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	 htim1.Init.RepetitionCounter = 0;
	 HAL_TIM_Base_Init(&htim1);

}

/* Função de inicialização Timer 2 */
static void MX_TIM2_Init(void)
{
	//Configura um PWM com frequência de 3.3 kHz
	__HAL_RCC_TIM2_CLK_ENABLE();

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200;
  htim2.Init.ClockDivision = 0;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* Função de inicialização Timer 3 */
static void MX_TIM3_Init(void)
{
	//Configura um PWM com frequência de 50Hz

	__HAL_RCC_TIM3_CLK_ENABLE();

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler =1600;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 201-1;
  htim3.Init.ClockDivision = 0;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* Função de inicialização da UART1 */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
 * Função de configuração dos pinos de entrada e saída
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Habilita clock para as portas A, B, D, E e H */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();



  // Configura PB9 como saída para acionamento do LED

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA,&GPIO_InitStruct);

  // Configura todos os pinos da porta D com nível lógico 0
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                            |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                            |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  // Pinos de 0 a 7 como entradas que entram em interrupção com borda de subida e descida

  	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                                       |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                                       |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                                       |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
   GPIO_InitStruct.Pin = GPIO_PIN_12;
	   GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	   GPIO_InitStruct.Pull = GPIO_NOPULL;
	   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Coloca prioridade de tratamento de interrupção para cada pino
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
      HAL_NVIC_EnableIRQ(EXTI0_IRQn);
      HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 1);
      HAL_NVIC_EnableIRQ(EXTI1_IRQn);
      HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 2);
      HAL_NVIC_EnableIRQ(EXTI2_IRQn);
      HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 3);
      HAL_NVIC_EnableIRQ(EXTI3_IRQn);
      HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 4);
      HAL_NVIC_EnableIRQ(EXTI4_IRQn);
      HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 5);
      HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
      HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 5);
      HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
      HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 6);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);




}

/* USER CODE BEGIN 4 */
// Essas funções fazem com que o barramento de interrupção chame a função que trata as interrupções
// passando como parâmetro o pino correspondente àquela interrupção
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}
void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);

}
void EXTI2_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);

}
void EXTI3_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);

}
void EXTI4_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);

}
void EXTI9_5_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
		{
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
		}
	else if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
		{
			HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
		}
	else if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
			{
				HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
			}
}

void EXTI15_10_IRQHandler(void)
{
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12)!= RESET)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Tratamento de borda de subida e descida para o sensor P1
	if(GPIO_Pin & GPIO_PIN_0)
	{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0) == GPIO_PIN_RESET)
		{
			buffer[n_buffer]=p1p;	//Atribuir evento à borda de descida de GPIOD_Pin_2
			n_buffer++;
		}
		else
		{
			buffer[n_buffer]=p1a;	//Atribuir evento à borda de subida de GPIOD_Pin_2
			n_buffer++;
		}
	}
	//Tratamento de borda de subida e descida para o sensor P2
	else if(GPIO_Pin & GPIO_PIN_1)
		{
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1) == GPIO_PIN_RESET)
				{
					buffer[n_buffer]=p2p;	//Atribuir evento à borda de descida de GPIOD_Pin_1
					n_buffer++;
				}
				else
				{
					buffer[n_buffer]=p2a;	//Atribuir evento à borda de subida de GPIOD_Pin_1
					n_buffer++;
				}
		}
	// Tratamento de borda de subida para o sensor P3
	else if(GPIO_Pin & GPIO_PIN_2)
		{
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2) == GPIO_PIN_SET){
				buffer[n_buffer]=p3a;	//Atribuir evento à borda de subida de GPIOD_Pin_3
				n_buffer++;
			}
		}
	// Tratamento de borda de subida para o sensor P4
	else if(GPIO_Pin & GPIO_PIN_4)
		{
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4) == GPIO_PIN_SET){
				buffer[n_buffer]=p4a;	//Atribuir evento à borda de subida de GPIOD_Pin_4
				n_buffer++;
			}
		}
	// Tratamento de borda de subida para o sensor  P5
	else if(GPIO_Pin & GPIO_PIN_12)
			{
				if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12) == GPIO_PIN_SET){
					buffer[n_buffer]=Bl;	//Atribuir evento à borda de subida de GPIOC_Pin_12
					n_buffer++;
				}
			}
	// Tratamento de borda de subida para o sensor S1
	else if(GPIO_Pin & GPIO_PIN_5)
		{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5) == GPIO_PIN_SET){
			buffer[n_buffer]=s1;	//Atribuir evento à borda de subida de GPIOD_Pin_5
			n_buffer++;
			}
		}
	// Tratamento de borda de subida para o sensor S2
	else if(GPIO_Pin & GPIO_PIN_6)
		{
			if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6) == GPIO_PIN_SET){
				buffer[n_buffer]=s2;	//Atribuir evento à borda de subida de GPIOD_Pin_6
				n_buffer++;
			}
		}
}

static void freia(void){
	EF_50;
	ER_50;
	DF_50;
	DR_50;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
