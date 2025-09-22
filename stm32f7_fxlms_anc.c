#include "stm32f7_wm8994_init.h"
#include "stm32f7_display.h"
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "arm_math.h" // CMSIS DSP library for LMS

#define BLOCK_SIZE 1
#define NUM_TAPS 50
#define SAMPLING_RATE 8000 // Sampling rate in Hz
#define GAIN 0.05f          // Gain for the noise signal (adjustable)
#define REFERENCE_FREQ 500  // Reference frequency for synthesized sinusoid in Hz
#define ERROR_BUFFER_SIZE 50  // Adjust as needed

float32_t error_signal_buffer[2][ERROR_BUFFER_SIZE]; // Buffer to store error signal
uint8_t active_buffer = 0; // Indicates which buffer is being used
static uint32_t error_buffer_index = 0;

float32_t beta = 1E-6; // Normalized LMS step size
float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
float32_t firCoeffs32[NUM_TAPS] = {0.0f};
float32_t secondaryPathCoeffs[NUM_TAPS] = {0.0f}; // Coefficients for the secondary path
arm_fir_instance_f32 SecondaryPathFilter;

float32_t refnoise, signal, yout, error, filtered_ref;
volatile int intr_flag = 0;  // Volatile because it will be modified inside an interrupt handler
extern int16_t tx_sample_L; // Transmitted signal (to loudspeaker)
extern int16_t tx_sample_R; // Transmitted signal (to loudspeaker)
extern int16_t rx_sample_L; // Recorded signal (from microphone)
UART_HandleTypeDef huart1;


// UART Initialization
void UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable GPIOA clock
    __HAL_RCC_GPIOB_CLK_ENABLE(); // Enable GPIOB clock

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9; // TX (PA10)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7; // RX (PB7)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

void ReceiveFIRCoeffs(UART_HandleTypeDef *huart, float32_t *coeffs, uint32_t num_taps) {
    char buffer[512]; // Buffer for received data
    HAL_UART_Receive(huart, (uint8_t *)buffer, sizeof(buffer), HAL_MAX_DELAY);

    char *token = strtok(buffer, ",");
    uint32_t parsed_count = 0;

    // Parse received coefficients
    for (uint32_t i = 0; i < num_taps && token != NULL; i++) {
        coeffs[i] = atof(token); // Convert each token to float
        token = strtok(NULL, ",");
        parsed_count++;
    }

    // Zero out any remaining coefficients if fewer were received
    for (uint32_t j = parsed_count; j < num_taps; j++) {
        coeffs[j] = 0.0f;
    }
		
		// Send acknowledgment
    const char *ack_message = "Coefficients received and echoed back successfully.\n";
    HAL_UART_Transmit(huart, (uint8_t *)ack_message, strlen(ack_message), HAL_MAX_DELAY);

    // Echo back parsed coefficients
    char echo_buffer[512] = "Parsed Coefficients: ";
    for (uint32_t k = 0; k < num_taps; k++) {
        char temp[32];
        snprintf(temp, sizeof(temp), (k == num_taps - 1) ? "%.6f\n" : "%.6f,", coeffs[k]);
        strcat(echo_buffer, temp);
    }
    HAL_UART_Transmit(huart, (uint8_t *)echo_buffer, strlen(echo_buffer), HAL_MAX_DELAY);

}

void PlotErrorSignal(float32_t *error_signal, uint32_t signal_length) {
    uint32_t i;
    float32_t max_error = 0.0f;

    // Find the maximum absolute error for scaling
    for (i = 0; i < signal_length; i++) {
        if (fabsf(error_signal[i]) > max_error) {
            max_error = fabsf(error_signal[i]);
        }
    }

    // Avoid division by zero in scaling
    if (max_error == 0.0f) {
        max_error = 1.0f;
    }

    // Clear the display
    BSP_LCD_Clear(LCD_COLOR_WHITE);

    // Draw axes
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawHLine(10, 120, 300); // X-axis centered at y = 120
    BSP_LCD_DrawVLine(10, 50, 140); // Y-axis from y = 50 to y = 190

    // Calculate x-axis scaling
    uint32_t x_offset = 20;              // Start drawing at x = 20
    uint32_t x_step = (280 / signal_length); // Scale x-axis spacing based on signal length

    // Prevent x_step from being 0 for small signal lengths
    if (x_step == 0) {
        x_step = 1;
    }

    int last_y = 120; // Start y-coordinate at the center of the y-axis

    // Plot error signal as a line graph
    for (i = 0; i < signal_length; i++) {
        // Map the error signal value to the y-axis range (-100 to +100 pixels)
        int y = 120 - (int)((error_signal[i] / max_error) * 100);

        // Ensure y stays within valid LCD bounds
        y = fmax(50, fmin(190, y)); // Clip y to the range [50, 190]

        // Draw a line connecting the previous point to the current point
        BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
        if (x_offset + x_step < 320) { // Ensure x stays within LCD width
            BSP_LCD_DrawLine(x_offset, last_y, x_offset + x_step, y);
        }

        // Update x_offset and last_y for the next point
        x_offset += x_step;
        last_y = y;

        // If x_offset exceeds LCD width, reset or wrap (optional)
        if (x_offset >= 320) {
            break; // Stop drawing if we run out of space
        }
    }
}




// FxLMS filter structure
typedef struct {
    float32_t coeffs[NUM_TAPS];        // Adaptive filter coefficients
    float32_t state[NUM_TAPS];         // Filter state buffer
    float32_t filteredState[NUM_TAPS]; // Filtered reference state buffer
    float32_t secondaryPath[NUM_TAPS]; // Secondary path model coefficients
    float32_t stepSize;                // Step size for the adaptation
    uint32_t numTaps;                  // Number of filter taps
} FxLMSFilter;

// Declare the filter globally
FxLMSFilter filter;

// Initialize the FxLMS filter
void FxLMS_Init(FxLMSFilter *filter, float32_t stepSize, float32_t *secondaryPathCoeffs, uint32_t numTaps) {
    memset(filter->coeffs, 0, sizeof(float32_t) * numTaps);       // Initialize adaptive coefficients to 0
    memset(filter->state, 0, sizeof(float32_t) * numTaps);        // Clear filter state
    memset(filter->filteredState, 0, sizeof(float32_t) * numTaps); // Clear filtered reference state
    memcpy(filter->secondaryPath, secondaryPathCoeffs, sizeof(float32_t) * numTaps); // Copy secondary path model
    filter->stepSize = stepSize;
    filter->numTaps = numTaps;
}

// Perform Normalized FxLMS filtering and coefficient adaptation
float32_t FxLMS_Process(FxLMSFilter *filter, float32_t refSignal, float32_t errorSignal) {
    float32_t filteredRef = 0.0f;
    float32_t yOut = 0.0f;
    float32_t power = 0.0f;

    // Shift state buffer to make room for the new sample
    memmove(&filter->state[1], &filter->state[0], (filter->numTaps - 1) * sizeof(float32_t));
    filter->state[0] = refSignal;

    // Compute the adaptive filter output (anti-noise signal)
    for (uint32_t m = 0; m < filter->numTaps; m++) {
        yOut += filter->state[m] * filter->coeffs[m];
    }

    // Filter the reference signal through the secondary path model
    for (uint32_t l = 0; l < filter->numTaps; l++) {
        filteredRef += filter->state[l] * filter->secondaryPath[l];
    }

    // Shift filtered reference state buffer
    memmove(&filter->filteredState[1], &filter->filteredState[0], (filter->numTaps - 1) * sizeof(float32_t));
    filter->filteredState[0] = filteredRef;

    // Compute the power of the filtered reference signal
    for (uint32_t p = 0; p < filter->numTaps; p++) {
        power += filter->filteredState[p] * filter->filteredState[p];
    }

    // Avoid division by zero in normalization
    if (power < 1e-12f) {
        power = 1e-12f;
    }

    // Update adaptive filter coefficients using the Normalized FxLMS formula
    for (uint32_t n = 0; n < filter->numTaps; n++) {
        filter->coeffs[n] -= filter->stepSize * errorSignal * filter->filteredState[n] / power;
    }

    return yOut; // Output the anti-noise signal
}

float32_t GenerateReference(float32_t frequency, float32_t sampleRate, uint32_t sampleIndex) {
    return sinf(2 * PI * frequency * sampleIndex / sampleRate);
}

// Audio Interrupt Callback
void BSP_AUDIO_SAI_Interrupt_CallBack(void) {
	
    static uint32_t sampleIndex = 0;
    float32_t refSignal = GenerateReference(REFERENCE_FREQ, SAMPLING_RATE, sampleIndex++);
    float32_t errorSignal = (float32_t)rx_sample_L;
    float32_t yOut = FxLMS_Process(&filter, refSignal, errorSignal);
    tx_sample_L = (int16_t)(yOut * 32767.0f);
		tx_sample_R = tx_sample_L ;
	
    // Store the error signal in the buffer
    error_signal_buffer[active_buffer][error_buffer_index++] = errorSignal;

    // Wrap the buffer index when full
    if (error_buffer_index >= ERROR_BUFFER_SIZE) {
        error_buffer_index = 0;
        active_buffer = !active_buffer; // Switch buffers
        intr_flag = 1;
    }
}

int main(void) {
    // Initialize HAL library
    HAL_Init();
	
		uint32_t last_update_time = HAL_GetTick();

    // Initialize UART for receiving coefficients
    UART_Init();

    // Receive secondary path coefficients via UART
    ReceiveFIRCoeffs(&huart1, secondaryPathCoeffs, NUM_TAPS);

    // Initialize FIR filter for secondary path
    arm_fir_init_f32(&SecondaryPathFilter, NUM_TAPS, secondaryPathCoeffs, firStateF32, BLOCK_SIZE);

    FxLMS_Init(&filter, 1e-4, secondaryPathCoeffs, NUM_TAPS); // Initialize the FxLMS filter

    stm32f7_wm8994_init(AUDIO_FREQUENCY_8K,
                        IO_METHOD_INTR,
                        INPUT_DEVICE_INPUT_LINE_1,
                        OUTPUT_DEVICE_HEADPHONE,
                        WM8994_HP_OUT_ANALOG_GAIN_0DB,
                        WM8994_LINE_IN_GAIN_0DB,
                        WM8994_DMIC_GAIN_0DB,
                        "stm32f7_fxlms_anc.c",
                        GRAPH);
		
		// Initialize LCD
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
    BSP_LCD_SelectLayer(0);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

		while (1) {
			// Update display at regular intervals (e.g., 100 ms)
			if (intr_flag == 1 && (HAL_GetTick() - last_update_time >= 1000)) {
					PlotErrorSignal(error_signal_buffer[active_buffer], ERROR_BUFFER_SIZE);
					intr_flag = 0; // Reset the flag
					last_update_time = HAL_GetTick(); // Update the time
			}
		}
}

