/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "math.h"
#include "string.h"
#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAITING 2 // Secondi da attendere tra un cambio di riferimento e l'altro. Tale tempo coincide anche con il numero di secondi tra un invio USART e l'altro.
#define REF_DIM 5 // Dimensione del buffer dei riferimenti.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* BEGIN CIRCULAR DATA BUFFER */
typedef struct circular_buffer {
	void *buffer;     // Buffer dei dati.
	void *buffer_end; // Fine del buffer dei dati.
	size_t capacity;  // Massimo numero di elementi nel buffer.
	size_t count;     // Numero di elementi nel buffer.
	size_t sz;        // Dimensione di ogni elemento del buffer.
	void *head;       // Puntatore alla testa.
	void *tail;       // Puntatore alla coda.
	bool writing;  	// Segnale che indica se si sta scrivendo o meno nel buffer.
} circular_buffer;

void circularBufferInit(circular_buffer *cb, size_t capacity, size_t sz) {
	cb->buffer = calloc(capacity, sz);
	if (cb->buffer == NULL)
		printf("ALLOCATED NULL\n\r");
	
	cb->buffer_end = (char*) cb->buffer + capacity * sz;
	cb->capacity = capacity;
	cb->count = 0;
	cb->sz = sz;
	cb->head = cb->buffer;
	cb->tail = cb->buffer;
	cb->writing = false;

}

void circularBufferFree(circular_buffer *cb) {
	free(cb->buffer);
}

void circularBufferPushBack(circular_buffer *cb, const void *item) {
	if (cb->count == cb->capacity) {
		printf("ERROR PUSH BACK \n\r");
	}
	cb->writing = true;
	memmove(cb->head, item, cb->sz);
	cb->head = (char*) cb->head + cb->sz;
	if (cb->head == cb->buffer_end)
		cb->head = cb->buffer;
	cb->count++;
	cb->writing = false;
}

void circularBufferPopFront(circular_buffer *cb, void *item) {
	if (cb->count == 0) {
		printf("ERROR PUSH BACK \n\r");
	}
	memmove(item, cb->tail, cb->sz);
	cb->tail = (char*) cb->tail + cb->sz;
	if (cb->tail == cb->buffer_end)
		cb->tail = cb->buffer;
	while ((cb->writing))
		;
	cb->count--;
}

circular_buffer buffer;

/* BEGIN RECORD TYPEDEF*/
typedef struct record {
	double current_u; // Valore dell'input di controllo attuale.
	double current_y; // Valore dell'uscita del motore attuale.
	double current_r; // Valore del riferimento attuale.
	uint32_t cycle_core_duration; // Tempo necessario per leggere, computare e attuare.
	uint32_t cycle_begin_delay; // Differenza tra il tempo attuale e il tempo atteso di inizio del ciclo.
	uint32_t current_timestamp; // Timestamp corrente in millisecondi.
} record;

/* BEGIN USART WRITE FUNCTION (used by printf)*/
int _write(int file, char *data, int len) {
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
		errno = EBADF;
		return -1;
	}

	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*) data, len, 1000);

	return (status == HAL_OK ? len : 0);
}

void setPulseFromDutyValue(double dutyVal) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

	uint16_t channelToModulate;
	uint16_t channelToStop;

	if (dutyVal > 0) {
		channelToModulate = TIM_CHANNEL_1;
		channelToStop = TIM_CHANNEL_2;
	} else {
		channelToModulate = TIM_CHANNEL_2;
		channelToStop = TIM_CHANNEL_1;
	}

	__HAL_TIM_SET_COMPARE(&htim3, channelToStop, 0);
	__HAL_TIM_SET_COMPARE(&htim3, channelToModulate,
			(abs(dutyVal) * ((double )htim3.Init.Period)) / 100); 
}

/**
 * @brief Implementa un contatore modulo 89796.
 *
 * @param lastTicks_star Ultimo valore di conteggio.
 * @param ticksDelta Variazione dei ticks dell'encoder.
 *
 * @return Il valore attuale di conteggio.
 **/
double modCounter(double lastTicks_star, double ticksDelta) {
	int sum = (int)(lastTicks_star + ticksDelta);
	return (double)(sum%89796);
}

/**
 * @brief Calcola la posizione del motore a partire dal valore di conteggio del contatore.
 *
 * @param ticksStar Valore di conteggio.
 *
 * @return La posizione relativa attuale del motore.
 **/
double getPosition(double ticksStar) {
	return 2*M_PI*ticksStar/3591.84;
}

/**
 * @brief Calcola la variazione dei ticks generati dall'encoder nell'ultimo intervallo di campionamento.
 *
 * @param current_ticks Conteggio attuale di ticks.
 * @param last_ticks Conteggio passato di ticks.
 * @param Ts Tempo di campionamento.
 *
 * @return Variazione dei ticks.
 **/
double getTicksDelta(double current_ticks, double last_ticks, double Ts) {
	double delta;

	if (abs(current_ticks - last_ticks) <= ceil(8400 * Ts))
		delta = current_ticks - last_ticks;
	else {
		if (last_ticks > current_ticks)
			delta = current_ticks + pow(2, 16) - 1 - last_ticks;
		else
			delta = current_ticks - pow(2, 16) + 1 - last_ticks;
	}
	return delta;
}


double last_ticks = 0;
double current_ticks = 0;
double ticks_star = 0;
double last_ticks_star = 0;
uint32_t tic_control_step;
uint32_t toc_control_step;
uint32_t control_computation_duration;

uint32_t controller_k = -1;
int sampling_prescaler = 2;
int sampling_prescaler_counter = 0;

double Ts = 0.005;

double reference_array[REF_DIM] = {0, -M_PI, M_PI, 2*M_PI, 0};
double reference;

double u = 0;


double **Ad;
double **Bd;
double **Cd;
double **state_kp1;
double **state_k;
double **y_k_expected;
double **L;
double **u_matrix;
double **sum_center;
double **sub_y;
double **sum_top;
double **sum_bottom;

int Ad_rows = 2;
int Ad_columns = 2;
int Bd_rows = 2;
int Bd_columns = 1;
int Cd_rows = 1;
int Cd_columns = 2;
int L_rows = 2;
int L_columns = 1;
int state_rows = 2;
int state_columns = 1;
int y_k_expected_rows = 1;
int y_k_expected_columns = 1;
int u_rows = 1;
int u_columns = 1;

double kp_1 = 29.4478;
double kp_2 = 0.6136;
double ki = 0.5884;
double z = 0;
double z_last = 0;
double error_last = 0;

/**
 * @brief Crea e restituisce una matrice di dimensione nXm.
 *
 * @param n Numero di righe della matrice.
 * @param m Numero di colonne della matrice.
 *
 * @return La matrice creata.
 **/
double** createMatrix(int n, int m) {
	double *values = (double*) calloc(m * n, sizeof(double));
	double **rows = (double**) malloc(n * sizeof(double*));
	for (int i = 0; i < n; ++i) {
		rows[i] = values + i * m;
	}
	return rows;
}

/**
 * @brief Inizializza le matrici del modello e le matrici degli stati attuali e futuri. 
 **/
void initMatricies() {

	Ad = createMatrix(Ad_rows, Ad_columns);
	Ad[0][0] = 1.0;
	Ad[0][1] = 0.0046;
	Ad[1][0] = 0.0;
	Ad[1][1] = 0.8259;

	Bd = createMatrix(Bd_rows, Bd_columns);
	Bd[0][0] = 0.0005;
	Bd[1][0] = 0.2049;

	Cd = createMatrix(Cd_rows, Cd_columns);
	Cd[0][0] = 1.0;
	Cd[0][1] = 0.0;

	L = createMatrix(L_rows, L_columns);
	L[0][0] = 0.9902;
	L[1][0] = 0.0001;

	state_kp1 = createMatrix(state_rows, state_columns);
	state_k = createMatrix(state_rows, state_columns);
	y_k_expected = createMatrix(y_k_expected_rows, y_k_expected_columns);

	u_matrix = createMatrix(u_rows, u_columns);
	sum_center = createMatrix(Bd_rows, u_columns);
	sub_y = createMatrix(y_k_expected_rows, y_k_expected_columns);
	sum_top = createMatrix(L_rows, y_k_expected_columns);
	sum_bottom = createMatrix(Ad_rows, state_columns);
}

/**
 * @brief Effettua la moltiplicazione righe per colonne tra due matrici.
 *
 * @param m1 La prima matrice.
 * @param m2 La seconda matrice.
 * @param m1_rows Numero di righe della prima matrice.
 * @param m1_columns Numero di righe della prima matrice.
 * @param m2_rows Numero di righe della seconda matrice.
 * @param m2_columns Numero di righe della seconda matrice.
 * @param m3 La matrice che conterra' il prodotto.
 **/
void multiplyMatricies(double **m1, double **m2, int m1_rows, int m1_columns,
		int m2_rows, int m2_columns, double **m3) {

	for (int i = 0; i < m1_rows; i++) {
		for (int j = 0; j < m2_columns; j++) {
			m3[i][j] = 0;
			for (int k = 0; k < m2_rows; k++) {
				m3[i][j] += m1[i][k] * m2[k][j];
			}
		}
	}
}

/**
 * @brief Setta tutti gli elementi di una matrice a zero.
 *
 * @param arr Matrice da azzerare.
 * @param n Numero di righe della matrice.
 * @param m Numero di colonne della matrice.
 *
 **/
void resetArray(double **arr, int n, int m) {

	for (int i = 0; i < n; i++) {
	  for( int j = 0; j < m; j++){
		  arr[i][j] = 0;
	  }
	}
}

/**
 * @brief Effettua la somma tra due matrici. Il risultato sara' contenuto nella prima.
 *
 * @param m1 La prima matrice.
 * @param m2 La seconda matrice.
 * @param rows Numero di righe delle matrici.
 * @param columns Numero di righe delle matrici.
 **/
void sumMatricies(double **m1, double **m2, double rows, double columns) {

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < columns; j++) {
			m1[i][j] = m1[i][j] + m2[i][j];
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */
	/* MCU Configuration--------------------------------------------------------*/

	initMatricies();
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	size_t buffer_size = (size_t) ceil(2 * WAITING / (Ts * sampling_prescaler));
	circularBufferInit(&buffer, buffer_size, sizeof(record));

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim4);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	int reference_index = 0;
	reference = reference_array[reference_index];

	printf("INIT\n\r"); 

	while (1) {
		size_t n_entries_to_send = buffer.count; // Numero di campioni non ancora letti
		record retrieved;

		for (size_t count = 0; count < n_entries_to_send; count++) {
			circularBufferPopFront(&buffer, &retrieved); // Prende le entry dal buffer
			printf("%lu, %f, %f, %f, %lu\n\r", retrieved.current_timestamp,
					retrieved.current_u, retrieved.current_y, retrieved.current_r,
					retrieved.cycle_core_duration);
		}

		// Cambio del riferimento
		reference = reference_array[reference_index];
		reference_index = (reference_index + 1)%REF_DIM;
		HAL_Delay(WAITING * 1000);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 84 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 500 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 84 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 5000 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
 * @brief Implementa l'osservatore di Luenberger.
 *
 * @param u_last Valore dell'ingresso di controllo attuale.
 * @param y Valore attuale dell'uscita del sistema.
 **/
void luenbergerObserver(double u_last, double y) {

	// Assegna lo stato futuro computato precedentemente allo stato attuale e azzera il futuro.
	for(int i=0; i < state_rows; i++){
		for(int j =0; j < state_columns; j++){
			state_k[i][j] = state_kp1[i][j];
			state_kp1[i][j] = 0;
		}
	}

	u_matrix[0][0] = u_last;

	// Moltiplicazione tra la matrice B e l'ingresso di controllo.
	multiplyMatricies(Bd, u_matrix, Bd_rows, Bd_columns, 1, 1, sum_center);

	// Moltiplicazione tra lo stato stimato e la matrice C.
	multiplyMatricies(Cd, state_k, Cd_rows, Cd_columns, state_rows, state_columns, y_k_expected);

	// Calcolo errore tra uscita stimata e uscita effettiva.
	sub_y[0][0] = y - y_k_expected[0][0];

	// Moltiplicazione dell'errore tra uscita stimata e uscita effettiva e la matrice L.
	multiplyMatricies(L, sub_y, L_rows, L_columns, y_k_expected_rows, y_k_expected_columns, sum_top);

	// Moltiplicazione tra stato stimato e matrice A.
	multiplyMatricies(Ad, state_k, Ad_rows, Ad_columns, state_rows, state_columns, sum_bottom);

	// Somma di tutti i contributi.
	sumMatricies(state_kp1, sum_top, L_rows, y_k_expected_columns);
	sumMatricies(state_kp1, sum_center, L_rows, y_k_expected_columns);
	sumMatricies(state_kp1, sum_bottom, L_rows, y_k_expected_columns);

	resetArray(sub_y, y_k_expected_rows, y_k_expected_columns);
	resetArray(sum_top, L_rows, y_k_expected_columns);
	resetArray(sum_center, L_rows, y_k_expected_columns);
	resetArray(sum_bottom, L_rows, y_k_expected_columns);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == &htim4) {
		controller_k = controller_k + 1;

		if (controller_k == 0) {
			tic_control_step = HAL_GetTick();
		}

		toc_control_step = HAL_GetTick();

		// Prende il valore attuale dei tick contando i fronti dell'encoder.
		current_ticks = (double) __HAL_TIM_GET_COUNTER(&htim1);

		// Calcola il valore attuale di conteggio.
		ticks_star = modCounter(last_ticks_star, getTicksDelta(current_ticks, last_ticks, Ts));

		// Calcola la posizione attuale del motore
		double position = getPosition(ticks_star);

		//Stima dello stato con l'osservatore di Luenberger. Dopo tale chiamata lo stato viene aggiornato con la nuova stima.
		luenbergerObserver(u, position);

		//* Retroazione di stato *//
		double error = position - reference;
		z = z_last - ki*error_last;

		double kx = kp_1*state_k[0][0] + kp_2*state_k[1][0];

		double u = z - kx;

		// Attuazione
		setPulseFromDutyValue(u * 100 / 12);

		//* ---------------------- *//

		error_last = error;
		z_last = z;

		control_computation_duration = HAL_GetTick() - toc_control_step;
		last_ticks = current_ticks;
		last_ticks_star = ticks_star;

		record r;
		r.current_u = u;
		r.current_y = position;
		r.current_r = reference;
		r.cycle_core_duration = control_computation_duration;
		r.cycle_begin_delay = toc_control_step - tic_control_step - (controller_k * Ts * 1000);
		r.current_timestamp = HAL_GetTick();

		if (sampling_prescaler_counter == (sampling_prescaler - 1)) {
			circularBufferPushBack(&buffer, &r);
			sampling_prescaler_counter = -1;
		}
		sampling_prescaler_counter++;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
