#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);

char content[3200];
uint16_t DMA_pos, current_pos;
char LSB, MSB;

#define freq_source 84000000 //internal clock source freq

uint16_t prescaler = 40; //prescaler used by timer
uint32_t freq_counter; //the freq of the timer calculated from freq_source and prescaler
uint16_t init_speed = 25000; //this sets the acceleration by setting the timing of first step, the smaller the number the faster the acceleration
uint16_t SPR = 3200; //steps per revolution of the stepper motor
float tick_freq[3]; //the freq that the steps need to be calculated from frq_counter RPM and SPR
float speed[3]; //the current speed measured by timer ticks in ARR value to count up to
float target_speed[3]; //the target speed that speed is accelerating towards

int32_t n[3];
int8_t curret_dir[3], target_dir[3], RPM_zero[3];

float alpha1, alpha2, alpha3; //the angles of force from motors
float a, b, c, d, e, f, g, h, i;//the input matrix
float det, a2, b2, c2, d2, e2,f2, g2, h2, i2;// the inverse matrix
float x_speed, y_speed, w_speed; //desired speeds in 3 DOF


void stepper_setup(void){

	freq_counter = freq_source / (prescaler + 1); //calculate the timer freq

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN; //enable port A and port D clock

	//setup timer pins
	GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE6_1; //setup pin A0 and pin A6 to AF
	GPIOD->MODER |= GPIO_MODER_MODER12_1; //setup pin D12 to AF mode
	GPIOA->AFR[0] = (GPIOD->AFR[1] &  ~(0b1111 | (0b1111<<(6*4)))) | 0b0010 | (0b0010<<(6*4)); //set pin A0 and pin A6 to AF timer mode       ;
	GPIOD->AFR[1] = (GPIOD->AFR[1] & ~(0b1111<<(4*(12-8)))) | 0b0010<<(4*(12-8)); //set pin D12 to AF timer mode

	//setup direction and enable pins
	GPIOA->MODER |= GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 |  GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;
	GPIOD->MODER |= GPIO_MODER_MODE10_0 | GPIO_MODER_MODE11_0;

	//set all 3 enable pins to low
	GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD4);
	GPIOD->ODR &= ~GPIO_ODR_OD10;

	//set all 3 dir pins
	GPIOA->ODR &= ~(GPIO_ODR_OD2 | GPIO_ODR_OD5);
	GPIOD->ODR &= ~GPIO_ODR_OD11;

	//setup all 3 timers

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enable the timer4 clock
	TIM3->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM3->PSC = prescaler;   //set prescale
	TIM3->CCMR1 = (TIM3->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM3->CCR1 = 10; //set to min rise time
	TIM3->ARR = init_speed; //set to timing
	TIM3->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM3->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM3->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM3_IRQn); // Enable interrupt(NVIC level)

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; //enable the timer4 clock
	TIM4->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM4->PSC = prescaler;   //set prescale
	TIM4->CCMR1 = (TIM4->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM4->CCR1 = 10; //set to min rise time
	TIM4->ARR = init_speed; //set to timing
	TIM4->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM4->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM4->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM4_IRQn); // Enable interrupt(NVIC level)


	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; //enable the timer4 clock
	TIM5->CR1 &= ~TIM_CR1_CEN; //disable channel 1.
	TIM5->PSC = prescaler;   //set prescale
	TIM5->CCMR1 = (TIM4->CCMR1 & ~(0b111<<4)) | (0b110<<4); //set PWM mode 110
	TIM5->CCR1 = 10; //set to min rise time
	TIM5->ARR = init_speed; //set to timing
	TIM5->CCER |= TIM_CCER_CC1E; //enable output to pin.
	TIM5->CR1 |= TIM_CR1_ARPE; //buffer ARR
	TIM5->DIER |= TIM_DIER_UIE; //enable interupt
	NVIC_EnableIRQ(TIM5_IRQn); // Enable interrupt(NVIC level)

	//initialize variables
	speed[0] = init_speed;
	speed[1] = init_speed;
	speed[2] = init_speed;
	RPM_zero[0] = 1;
	RPM_zero[1] = 1;
	RPM_zero[2] = 1;
	curret_dir[0] = 1;
	curret_dir[1] = 1;
	curret_dir[2] = 1;
	target_dir[0] =1;
	target_dir[1] =1;
	target_dir[2] =1;
}

void disable_steppers(void){
	//set all 3 enable pins to low
	GPIOA->ODR &= ~(GPIO_ODR_OD1 | GPIO_ODR_OD4);
	GPIOD->ODR &= ~GPIO_ODR_OD10;
}

void enable_steppers(void){
	//set all 3 enable pins to high
	GPIOA->ODR |= GPIO_ODR_OD1 | GPIO_ODR_OD4;
	GPIOD->ODR |= GPIO_ODR_OD10;
}

void set_speed(uint8_t motor_num, float RPM){

	//if((RPM<2)&&(RPM>-2))RPM=0;


	RPM = RPM * -1;
	if(RPM==0){
		RPM_zero[motor_num] = 1;
		target_speed[motor_num] = init_speed;
	}else{
		printf("RPM = %d   ", (int32_t)RPM);
		RPM_zero[motor_num] = 0;
		if(RPM>0)target_dir[motor_num] = 1;
		else{
			target_dir[motor_num] = 0;
			RPM = RPM *-1;
		}
		tick_freq[motor_num] = SPR * RPM / 60;
		target_speed[motor_num] = freq_counter / tick_freq[motor_num];

		//printf("Target speed = %d\r\n", (uint32_t)target_speed[motor_num]);
		//if(target_speed[motor_num]>65335)target_speed[motor_num]=65335;
		if(motor_num==0)TIM3->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 3.
		if(motor_num==1)TIM4->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 4.
		if(motor_num==2)TIM5->CR1 |= TIM_CR1_CEN; //enable channel 1 of timer 5.

	}

}


void motion_setup(){
	  //calucate force direction from motors in radians
	   alpha1 = 240 * M_PI/180;
	   alpha2 = 120 * M_PI/180;
	   alpha3 = 0 * M_PI/180;

	   //fill input matrix
	   a = cos(alpha1);
	   b = cos(alpha2);
	   c = cos(alpha3);
	   d = sin(alpha1);
	   e = sin(alpha2);
	   f = sin(alpha3);
	   g = 1;
	   h = 1;
	   i = 1;

	   //caluate the determint
	   det = a * e * i + b * f * g + c * d * h - c * e * g - a * f * h - b * d * i;

	   //calulate the inverse
	   a2 = (e * i - f * h) / det;
	   b2 = (h * c - i * b) / det;
	   c2 = (b * f - c * e) / det;
	   d2 = (g * f - d * i) / det;
	   e2 = (a * i - g * c) / det;
	   f2 = (d * c - a * f) / det;
	   g2 = (d * h - g * e) / det;
	   h2 = (g * b - a * h) / det;
	   i2 = (a * e - d * b) / det;
}

void move_robot (float x, float y, float w){
	  set_speed(0, a2 * x + b2 * y + c2 * w);
	  set_speed(1, d2 * x + e2 * y + f2 * w);
	  set_speed(2, g2 * x + h2 * y + i2 * w);
}


void DMA_Init(void){
	  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	  //enable DMA on UART2 receive
	  USART3->CR3 |= USART_CR3_DMAR;
	  //enable interrupt on receive
	  USART3->CR1 |= USART_CR1_RXNEIE;

	  //reset DMA1 stream5
	  DMA1_Stream1->CR &= ~DMA_SxCR_EN;
	  while (DMA1_Stream1->CR & DMA_SxCR_EN){
	  }  //wait for reset to complete

	  //set the UART2_RX register
	  DMA1_Stream1->PAR = (uint32_t)&(USART3->DR);
	  //set memory buffer to write to
	  DMA1_Stream1->M0AR = &content;
	  //set number of bytes to transfer
	  DMA1_Stream1->NDTR = 3200;
	  //set the channel to CH4 (UART2_RX), circlular buffer and incremant
	  DMA1_Stream1->CR =  DMA_SxCR_CHSEL_2 | DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_TCIE;

	  //enable the DMA
	  DMA1_Stream1->CR |= DMA_SxCR_EN;
}

void USART3_IRQHandler(void){
  DMA_pos++;
  if (DMA_pos == 3200){
	  DMA_pos = 0;
  }
}


int main(void){

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();

  //printf("hardware reset complete\r\n");
  NVIC_EnableIRQ(USART3_IRQn);
  DMA_Init();

  stepper_setup();
  motion_setup();

  int x_stick, y_stick, throttle_stick, yaw_stick;


  printf("Start up\r\n");

  enable_steppers();






  while (1){


	  current_pos = DMA_pos;

	  //wait for first start byte
	  while (content[current_pos] != 0x20){
		  HAL_Delay(1);
		  if (current_pos != DMA_pos){
			  current_pos++;
			  if (current_pos == 3200){
				  current_pos = 0;
			  }
		  }
	  }

	  //wait for next byte to be avaible
	  while(current_pos==DMA_pos){
		  HAL_Delay(1);
	  }
	  current_pos++;
	  if (current_pos == 3200){
		  current_pos = 0;
	  }

	  //if it is seconf start byte the get needed data
	  if (content[current_pos] == 0x40){

		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get LSB
		  LSB = content[current_pos];

		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get MSB
		  MSB = content[current_pos];

		  //combine bytes then translate and translate
		  x_stick = (LSB | (MSB << 8));
		  x_stick -= 1500;


		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get LSB
		  LSB = content[current_pos];

		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get MSB
		  MSB = content[current_pos];

		  //combine bytes then translate and translate
		  y_stick = (LSB | (MSB << 8));
		  y_stick -= 1500;


		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get LSB
		  LSB = content[current_pos];

		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get MSB
		  MSB = content[current_pos];

		  //combine bytes then translate and translate
		  throttle_stick = (LSB | (MSB << 8))-1000;


		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get LSB
		  LSB = content[current_pos];

		  //wait for next byte to be avaiable
		  while(current_pos==DMA_pos){
			  HAL_Delay(1);
		  }
		  current_pos++;
		  if (current_pos == 3200){
			  current_pos = 0;
		  }

		  //get MSB
		  MSB = content[current_pos];

		  //combine bytes then translate and dilate
		  yaw_stick = (LSB | (MSB << 8));
		  yaw_stick -= 1500;


		  //printf("%d\r\n", y_stick);
		   move_robot(x_stick * 0.7, y_stick * 0.7, yaw_stick * -0.7 );
		  //set_speed(1,y_stick);


	  }





  }
}



void TIM3_IRQHandler(void){

	TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[0] && (speed[0] >= init_speed)){
		TIM3->CR1 &= ~TIM_CR1_CEN;
		speed[0] = init_speed;
		n[0]=0;
	}

	//if the current direction is same as target direction
	if(target_dir[0] == curret_dir[0]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[0]>=init_speed){
			speed[0] = init_speed;
			n[0]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[0] >= init_speed) && (speed[0] >= init_speed)){
			speed[0] = target_speed[0];
			n[0]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[0]>target_speed[0]){
					n[0]++;
					speed[0] = speed[0] - ( (2 * speed[0]) / (4 * n[0] + 1) );
		  	  }else if(n[0]>0){
		  		  speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
		  		  n[0]--;
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[0] > init_speed - 100){
			if(target_dir[0])GPIOA->ODR &= ~GPIO_ODR_OD5; //set direction pin
			else GPIOA->ODR |= GPIO_ODR_OD5; //set direction pin
			curret_dir[0] = target_dir[0];
			speed[0] = init_speed;
			n[0] = 0;

		//else slow down
		}else if(n[0]>0){
			speed[0] = (speed[0] * (4 * n[0] + 1) / (4 * n[0] - 1));
			n[0]--;
		}
	}

	if(speed[0]<11)speed[0]=11;
	if(speed[0]>65535){
		TIM5->CR1 &= ~TIM_CR1_CEN;
		speed[0] = init_speed;
		n[0]=0;
	}

	TIM3->ARR = (uint32_t)speed[0];//update ARR
}



void TIM4_IRQHandler(void){

	TIM4->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[1] && (speed[1] >= init_speed - 1000)){
		TIM4->CR1 &= ~TIM_CR1_CEN;
		speed[1] = init_speed;
		n[1]=0;
	}

	//if the current direction is same as target direction
	if(target_dir[1] == curret_dir[1]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[1]>=init_speed){
			speed[1] = init_speed;
			n[1]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[1] >= init_speed) && (speed[1] >= init_speed)){
			speed[1] = target_speed[1];
			n[1]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[1]>target_speed[1]){
					n[1]++;
					speed[1] = speed[1] - ( (2 * speed[1]) / (4 * n[1] + 1) );
		  	  }else if(n[1]>0){
		  		  	  speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
		  		  	  n[1]--;
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[1] > init_speed - 100){
			if(target_dir[1])GPIOD->ODR &= ~GPIO_ODR_OD11; //set direction pin
			else GPIOD->ODR |= GPIO_ODR_OD11; //set direction pin
			curret_dir[1] = target_dir[1];
			speed[1] = init_speed;
			n[1] = 0;

		//else slow down
		}else if(n[1]>0){
			speed[1] = (speed[1] * (4 * n[1] + 1) / (4 * n[1] - 1));
			n[1]--;
		}
	}

	if(speed[1]<12)speed[1]=12;
	if(speed[1]>65535){
		TIM4->CR1 &= ~TIM_CR1_CEN;
		speed[1] = init_speed;
		n[1]=0;
	}

	TIM4->ARR = (uint32_t)speed[1];//update ARR

}


void TIM5_IRQHandler(void){

	TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

	//if the target speed is zero & current speed is slower init speed the disable the channel
	if (RPM_zero[2] && (speed[2] >= init_speed)){
		TIM5->CR1 &= ~TIM_CR1_CEN;
		speed[2] = init_speed;
		n[2]=0;
	}


	//if the current direction is same as target direction
	if(target_dir[2] == curret_dir[2]){

		//if target current speed is slower than init speed then set to init and reset n
		if (speed[2]>=init_speed){
			speed[2] = init_speed;
			n[2]=0;
		}

		//if target speed is slower than init and current speed is slower than init speed then set current to target and reset n
		if((target_speed[2] >= init_speed) && (speed[2] >= init_speed)){
			speed[2] = target_speed[2];
			n[2]=0;

			//if current speed is slower than target then speed up else slow down
		}else if(speed[2]>target_speed[2]){
					n[2]++;
					speed[2] = speed[2] - ( (2 * speed[2]) / (4 * n[2] + 1) );
		  	  }else if(n[2]>0){
		  		  speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
		  		  n[2]--;
		  	  }

	//else the current direction is not same as target direction
	}else{

		//if the current speed is slower than init speed then flip the direction pin and reset
		if(speed[2] > init_speed-100){
			if(target_dir[2])GPIOA->ODR &= ~GPIO_ODR_OD2; //set direction pin
			else GPIOA->ODR |= GPIO_ODR_OD2; //set direction pin
			curret_dir[2] = target_dir[2];
			speed[2] = init_speed;
			n[2] = 0;

		//else slow down
		}else if(n[2]>0){
			speed[2] = (speed[2] * (4 * n[2] + 1) / (4 * n[2] - 1));
			n[2]--;
		}
	}

	if(speed[2]<11)speed[2]=11;
	if(speed[2]>65535){
		TIM5->CR1 &= ~TIM_CR1_CEN;
		speed[2] = init_speed;
		n[2]=0;
	}
	TIM5->ARR = (uint32_t)speed[2];//update ARR
}




/** System Clock Configuration
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 256000;
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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