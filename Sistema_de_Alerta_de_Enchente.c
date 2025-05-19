// Sistema de Alerta de Enchente com Simulação por Joystick
// Desenvolvido por José Vinicius 

#include "pico/stdlib.h"            // funções padrão do Pico SDK 
#include "hardware/gpio.h"          // controle de pinos GPIO 
#include "hardware/adc.h"           // conversor adc para leitura do joystick (eixo X e Y)
#include "hardware/i2c.h"           // comunicação I2C para o display OLED SSD1306
#include "hardware/pwm.h"           // controle PWM para o buzzer 
#include "lib/ssd1306.h"            // biblioteca para controle do display OLED SSD1306 
#include "lib/font.h"               // fonte de texto para exibição no display OLED
#include "generated/ws2812.pio.h"   // programa PIO gerado para comunicação com a matriz WS2812
#include "FreeRTOS.h"               // núcleo do FreeRTOS 
#include "task.h"                   // gerenciamento de tarefas no FreeRTOS
#include "queue.h"                  // filas para comunicação entre tarefas 
#include <stdio.h>                  // funções de entrada/saída padrão 
#include <string.h>                 // manipulação de strings 
#include "pico/bootrom.h"           // função reset_usb_boot para entrar no modo BOOTSEL

// definições de pinos
#define BOTAO_A 5                   // GPIO para Botão A (para alternar modos)
#define BOTAO_B 6                   // GPIO para Botão B (para modo BOOTSEL)
#define WS2812_PIN 7                // GPIO para matriz de LEDs WS2812
#define BUZZER 10                   // GPIO para buzzer
#define LED_G 11                    // GPIO do LED RGB verde
#define LED_B 12                    // GPIO do LED RGB azul
#define LED_R 13                    // GPIO do LED RGB vermelho
#define I2C_SDA 14                  // GPIO para pino SDA do I2C (OLED)
#define I2C_SCL 15                  // GPIO para pino SCL do I2C (OLED)
#define I2C_PORT i2c1               // porta I2C usada para o display OLED
#define OLED_ADDRESS 0x3C           // endereço I2C do display OLED SSD1306
#define ADC_JOYSTICK_Y 27           // pino do joystick (eixo Y) para nível de água
#define ADC_JOYSTICK_X 26           // pino do joystick (eixo X) para volume de chuva
#define WIDTH 128                   // largura do display OLED 
#define HEIGHT 64                   // altura do display OLED

// estados de alerta com base no nível de água ou volume de chuva
typedef enum { SEGURO, ALERTA, EMERGENCIA } EstadoAlerta;

// modos de operação ("Nível de Água" ou "Volume de Chuva")
typedef enum { NIVEL_AGUA, VOLUME_CHUVA } ModoOperacao;

// estrutura para armazenar os dados do joystick ("Nível de Água" ou "Volume de Chuva")
typedef struct {
    uint16_t y_pos; // valor do eixo Y (nível de água, 0 a 4095)
    uint16_t x_pos; // valor do eixo X (volume de chuva, 0 a 4095)
} DadosJoystick;

// fila para comunicação entre tarefas (envio e recebimento de dados do joystick)
QueueHandle_t xFilaDadosJoystick;

// variável global para controlar o modo de operação ("Nível de Água" ou "Volume de Chuva")
volatile ModoOperacao modo_atual = NIVEL_AGUA; // inicia no modo "Nível de Água"

// função auxiliar para enviar um pixel à matriz WS2812 (controla LEDs)
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u); // envia o valor RGB ao PIO
}

// função auxiliar para converter valores RGB em formato de 32 bits para a matriz WS2812
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b); // combina R, G e B em um único valor
}

// padrões da matriz de LEDs
void set_matriz_nivel(uint16_t valor, EstadoAlerta estado) {
    int pixel_map[5][5] = {              // mapeamento de índices da matriz WS2812
        {24, 23, 22, 21, 20},            // Linha 1: índices dos LEDs
        {15, 16, 17, 18, 19},            // Linha 2: índices dos LEDs
        {14, 13, 12, 11, 10},            // Linha 3: índices dos LEDs
        {5,  6,  7,  8,  9},             // Linha 4: índices dos LEDs
        {4,  3,  2,  1,  0}              // Linha 5: índices dos LEDs
    };
    uint32_t pixels[25] = {0}; // array para armazenar os valores RGB dos LEDs

    // representação do nível/volume com LEDs azuis para todos os estados
    int linhas_acesas = (valor * 5) / 4095; // escala o valor de 0 a 4095 para 0 a 4
    for (int i = 4; i >= (4 - linhas_acesas); i--) { // preenche de baixo para cima
        for (int j = 0; j < 5; j++) {
            pixels[pixel_map[i][j]] = urgb_u32(0, 0, 8); // cor azul para representar água/chuva
        }
    }

    // envia os valores RGB para a matriz de LEDs
    for (int i = 0; i < 25; i++) put_pixel(pixels[i]);
}

// inicializa o PWM para o buzzer com uma frequência específica
uint pwm_init_buzzer(uint gpio, uint freq) {
    gpio_set_function(gpio, GPIO_FUNC_PWM); // configura o pino para PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio); // obtém o slice PWM associado ao pino
    uint clk_div = clock_get_hz(clk_sys) / (freq * 4096); // calcula o divisor para a frequência
    pwm_set_clkdiv(slice_num, clk_div); // define o divisor de clock
    pwm_set_wrap(slice_num, 4095); // define o valor de wrap (resolução do PWM)
    pwm_set_enabled(slice_num, true); // ativa o PWM
    return slice_num; // retorna o slice para controle posterior
}

// tarefa para ler o joystick e enviar os valores para a fila
void vTaskJoystick(void *params) {
    adc_gpio_init(ADC_JOYSTICK_Y); // configura o pino do joystick (eixo Y)
    adc_gpio_init(ADC_JOYSTICK_X); // configura o pino do joystick (eixo X)
    adc_init(); // inicializa o conversor adc

    DadosJoystick data; // estrutura para armazenar os valores lidos
    while (true) {
        adc_select_input(1); // seleciona o canal ADC1 (GPIO 27, eixo Y)
        data.y_pos = adc_read(); // lê o valor do eixo Y (nível de água)
        adc_select_input(0); // seleciona o canal ADC0 (GPIO 26, eixo X)
        data.x_pos = adc_read(); // lê o valor do eixo X (volume de chuva)
        xQueueSend(xFilaDadosJoystick, &data, 0); // envia os valores para a fila
        vTaskDelay(pdMS_TO_TICKS(100)); // delay de 100ms (frequência de 10 Hz)
    }
}

// tarefa para atualizar o display OLED com o nível de água ou volume de chuva
void vTaskDisplay(void *params) {
    // inicializa a comunicação I2C para o display OLED
    i2c_init(I2C_PORT, 400 * 1000); // configura I2C a 400kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // define SDA como função I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // define SCL como função I2C
    gpio_pull_up(I2C_SDA); // ativa pull-up no SDA
    gpio_pull_up(I2C_SCL); // ativa pull-up no SCL

    // configura o display OLED
    ssd1306_t disp; // estrutura para o display
    ssd1306_init(&disp, WIDTH, HEIGHT, false, OLED_ADDRESS, I2C_PORT); // inicializa o display
    ssd1306_config(&disp); // configura o display
    ssd1306_fill(&disp, 0); // limpa o display
    ssd1306_send_data(&disp); // envia os dados iniciais para o display

    DadosJoystick data; // estrutura para receber os valores da fila
    char valor_str[16]; // string para formatar o valor (nível ou volume)
    while (true) {
        // recebe os valores da fila
        if (xQueueReceive(xFilaDadosJoystick, &data, portMAX_DELAY) == pdTRUE) {
            ssd1306_fill(&disp, 0); // limpa o display

            // desenha uma borda retangular ao redor do conteúdo
            ssd1306_rect(&disp, 0, 0, WIDTH, HEIGHT, true, false); // borda nas extremidades (128x64)

            // determina o valor a ser exibido com base no modo atual
            uint16_t valor = (modo_atual == NIVEL_AGUA) ? data.y_pos : data.x_pos;
            const char* titulo = (modo_atual == NIVEL_AGUA) ? "Nivel Agua" : "Volume Chuva";
            uint8_t percentual = (valor * 100) / 4095;     // converte o valor de 0-4095 para porcentagem
            sprintf(valor_str, "Valor: %u%%", percentual); // formata o valor como porcentagem

            // título centralizado 
            ssd1306_draw_string(&disp, titulo, (WIDTH - strlen(titulo) * 8) / 2, 8);
            // exibe o valor 
            ssd1306_draw_string(&disp, valor_str, (WIDTH - strlen(valor_str) * 8) / 2, 24);

            // determina e exibe o estado de alerta 
            if (modo_atual == NIVEL_AGUA) {
                if (valor <= 2457) { // 0 a 60%
                    ssd1306_draw_string(&disp, "Seguro", (WIDTH - 6 * 8) / 2, 40);
                } else if (valor <= 2867) { // 60 a 70%
                    ssd1306_draw_string(&disp, "Alerta", (WIDTH - 6 * 8) / 2, 40);
                } else { // ≥ 70%
                    ssd1306_draw_string(&disp, "Emergencia", (WIDTH - 10 * 8) / 2, 40);
                }
            } else { // volume de Chuva
                if (valor <= 2867) { // 0 a 70%
                    ssd1306_draw_string(&disp, "Seguro", (WIDTH - 6 * 8) / 2, 40);
                } else if (valor <= 3276) { // 70 a 80%
                    ssd1306_draw_string(&disp, "Alerta", (WIDTH - 6 * 8) / 2, 40);
                } else { // ≥ 80%
                    ssd1306_draw_string(&disp, "Emergencia", (WIDTH - 10 * 8) / 2, 40);
                }
            }
            ssd1306_send_data(&disp); // atualiza o display com as novas informações
        }
    }
}

// tarefa para controlar a matriz de LEDs com base no nível de água ou volume de chuva
void vTaskMatriz(void *params) {
    DadosJoystick data; // estrutura para receber os valores da fila
    while (true) {
        // recebe os valores da fila
        if (xQueueReceive(xFilaDadosJoystick, &data, portMAX_DELAY) == pdTRUE) {
            // determina o valor e o estado com base no modo atual
            uint16_t valor = (modo_atual == NIVEL_AGUA) ? data.y_pos : data.x_pos;
            EstadoAlerta estado;
            if (modo_atual == NIVEL_AGUA) {
                estado = (valor <= 2457) ? SEGURO : (valor <= 2867) ? ALERTA : EMERGENCIA;
            } else {
                estado = (valor <= 2867) ? SEGURO : (valor <= 3276) ? ALERTA : EMERGENCIA;
            }
            set_matriz_nivel(valor, estado); // atualiza a matriz de LEDs
        }
    }
}

// tarefa para controlar o LED RGB com base no estado de alerta
void vTaskLedRGB(void *params) {
    // inicializa os pinos do LED RGB como saída
    gpio_init(LED_R); gpio_set_dir(LED_R, GPIO_OUT);
    gpio_init(LED_G); gpio_set_dir(LED_G, GPIO_OUT);
    gpio_init(LED_B); gpio_set_dir(LED_B, GPIO_OUT);

    DadosJoystick data; // estrutura para receber os valores da fila
    while (true) {
        // recebe os valores da fila
        if (xQueueReceive(xFilaDadosJoystick, &data, portMAX_DELAY) == pdTRUE) {
            // determina o valor com base no modo atual
            uint16_t valor = (modo_atual == NIVEL_AGUA) ? data.y_pos : data.x_pos;
            // ajusta a cor do LED RGB com base no nível/volume
            if (modo_atual == NIVEL_AGUA) {
                if (valor <= 2457) { // 0 a 60%: seguro (verde)
                    gpio_put(LED_G, 1); gpio_put(LED_R, 0); gpio_put(LED_B, 0);
                } else if (valor <= 2867) { // 60 a 70%: alerta (amarelo)
                    gpio_put(LED_G, 1); gpio_put(LED_R, 1); gpio_put(LED_B, 0);
                } else { // ≥ 70%: emergência (vermelho)
                    gpio_put(LED_R, 1); gpio_put(LED_G, 0); gpio_put(LED_B, 0);
                }
            } else { // volume de Chuva
                if (valor <= 2867) { // 0 a 70%: seguro (verde)
                    gpio_put(LED_G, 1); gpio_put(LED_R, 0); gpio_put(LED_B, 0);
                } else if (valor <= 3276) { // 70 a 80%: alerta (amarelo)
                    gpio_put(LED_G, 1); gpio_put(LED_R, 1); gpio_put(LED_B, 0);
                } else { // ≥ 80%: emergência (vermelho)
                    gpio_put(LED_R, 1); gpio_put(LED_G, 0); gpio_put(LED_B, 0);
                }
            }
        }
    }
}

// tarefa para controlar o buzzer com base no estado de alerta
void vTaskBuzzer(void *params) {
    uint slice_num = pwm_init_buzzer(BUZZER, 1000); // inicializa o buzzer com PWM a 1000Hz

    DadosJoystick data; // estrutura para receber os valores da fila
    while (true) {
        // recebe os valores da fila
        if (xQueueReceive(xFilaDadosJoystick, &data, portMAX_DELAY) == pdTRUE) {
            // determina o valor com base no modo atual
            uint16_t valor = (modo_atual == NIVEL_AGUA) ? data.y_pos : data.x_pos;
            EstadoAlerta estado;
            if (modo_atual == NIVEL_AGUA) {
                estado = (valor <= 2457) ? SEGURO : (valor <= 2867) ? ALERTA : EMERGENCIA;
            } else {
                estado = (valor <= 2867) ? SEGURO : (valor <= 3276) ? ALERTA : EMERGENCIA;
            }

            // ajusta o comportamento do buzzer com base no estado
            if (estado == SEGURO) { // seguro: sem som (Modo Normal)
                pwm_set_gpio_level(BUZZER, 0); // desliga o buzzer
            } else if (estado == ALERTA) { // alerta: beeps intermitentes (200ms ligado, 800ms desligado)
                pwm_set_gpio_level(BUZZER, 256); // % duty cycle 
                vTaskDelay(pdMS_TO_TICKS(200)); // mantém ligado por 200ms
                pwm_set_gpio_level(BUZZER, 0); // desliga o buzzer
                vTaskDelay(pdMS_TO_TICKS(800)); // mantém desligado por 800ms
            } else { // emergência: som contínuo
                pwm_set_gpio_level(BUZZER, 256); // % duty cycle
            }
        }
    }
}

// função de interrupção para alternar o modo com o botão A
void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_press = 0; // para debounce
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_press < 200) return; // debounce de 200ms
    last_press = current_time;

    if (gpio == BOTAO_A && events == GPIO_IRQ_EDGE_FALL) {
        modo_atual = (modo_atual == NIVEL_AGUA) ? VOLUME_CHUVA : NIVEL_AGUA; // alterna o modo
    } else if (gpio == BOTAO_B && events == GPIO_IRQ_EDGE_FALL) {
        reset_usb_boot(0, 0); // ativa o modo BOOTSEL para atualização via USB
    }
}

// função principal: inicializa os periféricos e cria as tarefas
int main() {
    stdio_init_all(); // inicializa a comunicação USB para depuração
    sleep_ms(2000); // aguarda 2s para estabilizar a inicialização

    // inicialização da matriz de LEDs WS2812
    PIO pio = pio0; // usa o PIO0 para comunicação com WS2812
    uint offset = pio_add_program(pio, &ws2812_program); // carrega o programa WS2812 no PIO
    ws2812_program_init(pio, 0, offset, WS2812_PIN, 800000, false); // inicializa WS2812 a 800kHz

    // configuração do botão A para alternar modos
    gpio_init(BOTAO_A); // inicializa o pino do botão A
    gpio_set_dir(BOTAO_A, GPIO_IN); // configura como entrada
    gpio_pull_up(BOTAO_A); // ativa pull-up interno

    // configuração do botão B para modo BOOTSEL
    gpio_init(BOTAO_B); // inicializa o pino do botão B
    gpio_set_dir(BOTAO_B, GPIO_IN); // configura como entrada
    gpio_pull_up(BOTAO_B); // ativa pull-up interno

    // configura as interrupções para os botões
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BOTAO_B, GPIO_IRQ_EDGE_FALL, true);

    // criação da fila para compartilhamento de dados do joystick
    xFilaDadosJoystick = xQueueCreate(5, sizeof(DadosJoystick)); // fila com capacidade para 5 elementos

    // criação das tarefas do FreeRTOS
    xTaskCreate(vTaskJoystick, "Joystick", 256, NULL, 1, NULL); // tarefa para ler o joystick
    xTaskCreate(vTaskDisplay, "Display", 512, NULL, 1, NULL); // tarefa para atualizar o display
    xTaskCreate(vTaskMatriz, "Matriz", 256, NULL, 1, NULL); // tarefa para controlar a matriz de LEDs
    xTaskCreate(vTaskLedRGB, "LED RGB", 256, NULL, 1, NULL); // tarefa para controlar o LED RGB
    xTaskCreate(vTaskBuzzer, "Buzzer", 256, NULL, 1, NULL); // tarefa para controlar o buzzer

    vTaskStartScheduler(); // inicia o escalonador do FreeRTOS
    while (1); 
}
