#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "arm_math.h" // CMSIS DSP library for LMS

#define BLOCK_SIZE 1
#define NUM_TAPS 50
#define SAMPLING_RATE 8000 // Sampling rate in Hz
#define DURATION_SECONDS 50 // Identification duration in seconds
#define GAIN 0.05f          // Gain for the noise signal (adjustable)

float32_t beta = 1E-5; // Normalized LMS step size
float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
float32_t firCoeffs32[NUM_TAPS] = {0.0f};
arm_lms_norm_instance_f32 S;
float32_t refnoise, signal, yout, error;

volatile int intr_flag = 0;  // Volatile because it will be modified inside an interrupt handler
volatile uint32_t block_count = 0; // Block counter for tracking progress
extern int16_t tx_sample_L; // Transmitted signal (to loudspeaker)
extern int16_t tx_sample_R; // Transmitted signal (to loudspeaker)
extern int16_t rx_sample_L; // Recorded signal (from microphone)

UART_HandleTypeDef huart1;

// UART Initialization
void UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10; // TX (PA9) and RX (PA10)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

// Function to plot coefficients on the LCD display
void PlotCoefficients(float32_t *coeffs, uint32_t num_taps) {
    uint32_t i;
    float32_t max_coeff = 0.0f;

    // Find the maximum coefficient for scaling
    for (i = 0; i < num_taps; i++) {
        if (fabsf(coeffs[i]) > max_coeff) {
            max_coeff = fabsf(coeffs[i]);
        }
    }

    // Clear the display
    BSP_LCD_Clear(LCD_COLOR_WHITE);

    // Draw axes
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawHLine(10, 200, 300); // X-axis
    BSP_LCD_DrawVLine(10, 50, 150); // Y-axis

    // Plot coefficients as bars
    int x_offset = 20;
    int bar_width = 2;
    for (i = 0; i < num_taps; i++) {
        int bar_height = (int)((coeffs[i] / max_coeff) * 100); // Scale to 100 px
        BSP_LCD_SetTextColor((coeffs[i] >= 0) ? LCD_COLOR_GREEN : LCD_COLOR_RED);
        if (coeffs[i] >= 0) {
            BSP_LCD_FillRect(x_offset, 200 - bar_height, bar_width, bar_height);
        } else {
            BSP_LCD_FillRect(x_offset, 200, bar_width, -bar_height);
        }
        x_offset += (bar_width + 1); // Spacing between bars
    }
}


void TransmitFIRCoeffs(UART_HandleTypeDef *huart, float32_t *coeffs, uint32_t num_taps) {
    uint32_t i;
    char buffer[256];  // Buffer to hold the CSV line
    
    // Start the CSV format transmission
    for (i = 0; i < num_taps; i++) {
        // Convert the coefficient to a string
        int len = snprintf(buffer, sizeof(buffer), "%f", coeffs[i]);
        
        // If this is not the last coefficient, append a comma
        if (i < num_taps - 1) {
            snprintf(buffer + len, sizeof(buffer) - len, ",");
            len += 1;  // Update the length with the added comma
        }
        
        // Transmit the current coefficient or the whole line over UART
        HAL_UART_Transmit(huart, (uint8_t *)buffer, len, HAL_MAX_DELAY);
    }
    
    // After transmitting all coefficients, send a newline to finish the line
    snprintf(buffer, sizeof(buffer), "\n");  // Add a newline at the end
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}


// Audio Interrupt Callback
void BSP_AUDIO_SAI_Interrupt_CallBack() {
    if (block_count >= (DURATION_SECONDS * SAMPLING_RATE / BLOCK_SIZE)) {
        // Stop the noise generation after the specified duration
        tx_sample_L = 0; // Silence the loudspeaker
        return;
    }

    // Generate white noise as input (refnoise)
    refnoise = GAIN * ((float32_t)(rand() % 200) - 100) / 100.0f; // Random noise scaled by gain

    // Send refnoise to the loudspeaker
    tx_sample_L = (int16_t)(refnoise * 32767); // Convert to 16-bit audio range
		tx_sample_R = tx_sample_L;

    // Record the response from the microphone
    signal = (float32_t)(rx_sample_L);

    // Apply LMS to adapt filter coefficients
    arm_lms_norm_f32(&S, &refnoise, &signal, &yout, &error, BLOCK_SIZE);

    // Set interrupt flag to signal that processing is complete
    intr_flag = 1;

    // Increment the block counter
    block_count++;
}

int main(void) {
    uint32_t num_blocks = DURATION_SECONDS * SAMPLING_RATE / BLOCK_SIZE;

    // Initialize HAL library
    HAL_Init();

    // Initialize LMS filter
    arm_lms_norm_init_f32(&S, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], beta, BLOCK_SIZE);

    // Initialize audio codec
    stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                        IO_METHOD_INTR,
                        INPUT_DEVICE_INPUT_LINE_1,
                        OUTPUT_DEVICE_HEADPHONE,
                        WM8994_HP_OUT_ANALOG_GAIN_0DB,
                        WM8994_LINE_IN_GAIN_0DB,
                        WM8994_DMIC_GAIN_0DB,
                        "stm32f7_system_iden.c",
                        GRAPH);

    // Initialize LCD
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

    // Main loop to process LMS and update the display
    while (block_count < num_blocks) {
        // Wait for the interrupt callback to process each block
        while (!intr_flag);
        intr_flag = 0; // Reset interrupt flag

        // Plot coefficients on LCD
        PlotCoefficients(firCoeffs32, NUM_TAPS);
    }
		
		UART_Init();

    // Send "Hello, World!" message
    // char *message = "Hello, World!\n";
    // HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
		TransmitFIRCoeffs(&huart1, firCoeffs32, NUM_TAPS);

    while (1) {
        // Infinite loop
    }

    return 0;
}

