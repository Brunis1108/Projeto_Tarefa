#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include <stdio.h>
#include "pico/bootrom.h"
#include "ws2812.pio.h"
#include "hardware/pwm.h"

// Definições de pinos e constantes
#define WS2812_PIN 7          // Pino para a matriz de LEDs WS2812
#define NUM_LEDS 25           // Número total de LEDs na matriz

#define I2C_PORT i2c1         // Configuração I2C para o display
#define I2C_SDA 14            // Pino SDA
#define I2C_SCL 15            // Pino SCL
#define endereco 0x3C         // Endereço I2C do display

#define botaoB 6              // Pino do botão B (para modo BOOTSEL)
#define botaoA 5              // Pino do botão A (para alternar modos)

#define BUZZER_PIN 10         // Pino do buzzer
#define REST 0                // Define repouso para o buzzer

// Variáveis globais
bool noturno = false;         // Flag para modo noturno
volatile int estado_semaforo = 0;  // Estado atual do semáforo (0=vermelho, 1=verde, 2=amarelo)
PIO pio = pio0;               // Controlador PIO para os LEDs
int sm = 0;                   // Máquina de estado PIO

// Protótipos de funções
void put_pixel(uint32_t pixel_grb);
void gpio_irq_handler(uint gpio, uint32_t events);
void acender_leds(uint32_t r, uint32_t g, uint32_t b);
bool espera_com_interrupcao(int tempo_ms);
void buzzer_init();
void buzzer_play_note(int freq, int duration_ms);

/**
 * Tarefa: Leitura do Botão A
 * Verifica o estado do botão A para alternar entre modos normal/noturno
 */
void vLeBotaoTask() {
    bool botao_pressionado = false;

    while (1) {
        uint gpio_state = gpio_get(botaoA); // Lê o estado do botão A

        if (gpio_state == 0 && !botao_pressionado) { // Botão pressionado e ainda não tratado
            printf("Botao A pressionado\n");
            noturno = !noturno; // Alterna o modo
            botao_pressionado = true;
        } else if (gpio_state != 0 && botao_pressionado) { // Botão solto
            botao_pressionado = false;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Pequeno delay para debounce
    }
}

/**
 * Tarefa: Controle do Semáforo
 * Gerencia os estados do semáforo de acordo com o modo selecionado
 */
void vSemaforoTask() {
    while (1) {
        if (!noturno) {
            // Modo Normal - Ciclo completo do semáforo
            printf("Modo Normal\n");

            // Vermelho
            estado_semaforo = 0;
            acender_leds(50, 0, 0); // LEDs vermelhos
            if (!espera_com_interrupcao(8500)) continue; // Espera 8.5s (verificando interrupção)

            // Verde
            estado_semaforo = 1;
            acender_leds(0, 50, 0); // LEDs verdes
            if (!espera_com_interrupcao(5000)) continue; // Espera 5s

            // Amarelo
            estado_semaforo = 2;
            acender_leds(50, 35, 0); // LEDs amarelos
            if (!espera_com_interrupcao(3000)) continue; // Espera 3s

        } else {
            // Modo Noturno - Apenas amarelo piscando
            printf("Modo Noturno\n");

            estado_semaforo = 1;
            acender_leds(50, 35, 0); // LEDs amarelos acesos
            if (!espera_com_interrupcao(1000)) continue;

            acender_leds(0, 0, 0); // LEDs apagados
            if (!espera_com_interrupcao(1000)) continue;
        }
    }
}

/**
 * Tarefa: Controle do Buzzer
 * Gera os sinais sonoros de acordo com o estado do semáforo
 */
void vBuzzerTask() {
    while (1) {
        if (!noturno) {
            // Modo Normal - Sons diferentes para cada estado
            switch(estado_semaforo) {
                case 0: // Vermelho
                    buzzer_play_note(440, 500); // Tom contínuo curto
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    break;
                case 1: // Verde
                    buzzer_play_note(800, 700); // Beep curto
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;
                case 2: // Amarelo
                    buzzer_play_note(500, 500); // Beep rápido intermitente
                    vTaskDelay(pdMS_TO_TICKS(200));
                    break;
            }
        } else {
            // Modo Noturno - Beep lento
            buzzer_play_note(500, 600);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

/**
 * Função principal
 * Inicializa hardware e cria tarefas do FreeRTOS
 */
int main()
{
    stdio_init_all(); // Inicializa comunicação serial
    sleep_ms(2000);   // Delay para estabilização

    // Configuração dos botões
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_init(botaoA);
    gpio_set_dir(botaoA, GPIO_IN);
    gpio_pull_up(botaoA);
    
    // Inicializa hardware
    buzzer_init();
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, false);

    // Configura interrupção para o botão B (modo BOOTSEL)
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Cria tarefas do FreeRTOS
    xTaskCreate(vLeBotaoTask, "Botao Task", 512, NULL, 1, NULL);
    xTaskCreate(vSemaforoTask, "Semaforo Task", 512, NULL, 1, NULL);
    xTaskCreate(vBuzzerTask, "Buzzer Task", 256, NULL, 1, NULL);
    
    // Inicia o escalonador de tarefas
    vTaskStartScheduler();
    panic_unsupported(); // Nunca deve chegar aqui
}

/**
 * Manipulador de interrupção para o botão B
 * Entra no modo BOOTSEL quando o botão B é pressionado
 */
void gpio_irq_handler(uint gpio, uint32_t events) {
    reset_usb_boot(0, 0);
}

/**
 * Envia um valor de cor para o LED WS2812
 */
void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

/**
 * Converte valores RGB de 8 bits para formato 32 bits
 */
uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(g) << 16) | ((uint32_t)(r) << 8) | (uint32_t)(b);
}

/**
 * Controla a matriz de LEDs
 * Acende os LEDs centrais com a cor especificada e apaga os demais
 */
void acender_leds(uint32_t r, uint32_t g, uint32_t b) {
    for (int i = 0; i < NUM_LEDS; ++i) {
        // Acende apenas os LEDs centrais (formando um quadrado)
        if((i >=6 && i<=8) || (i>=11 && i<=13) || (i>=16 && i<=18)) {
            put_pixel(urgb_u32(r, g, b));
        } else {
            put_pixel(urgb_u32(0, 0, 0)); // Apaga outros LEDs
        }
    }
}

/**
 * Função de espera que verifica se o modo foi alterado
 * Retorna false se o modo foi alterado durante a espera
 */
bool espera_com_interrupcao(int tempo_ms) {
    const int intervalo = 50;
    int esperou = 0;
    bool modo_atual = noturno;

    while (esperou < tempo_ms) {
        vTaskDelay(pdMS_TO_TICKS(intervalo));
        esperou += intervalo;

        // Verifica se o modo foi alterado
        if (noturno != modo_atual) {
            return false;
        }
    }
    return true;
}

/**
 * Inicializa o pino do buzzer
 */
void buzzer_init() {
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
}

/**
 * Toca uma nota musical no buzzer
 * freq: frequência em Hz (0 para silêncio)
 * duration_ms: duração em milissegundos
 */
void buzzer_play_note(int freq, int duration_ms) {
    if (freq == REST) {
        gpio_put(BUZZER_PIN, 0);
        sleep_ms(duration_ms);
        return;
    }

    // Calcula período e número de ciclos
    uint32_t period_us = 1000000 / freq;
    uint32_t cycles = (freq * duration_ms) / 1000;

    // Gera onda quadrada para o tom
    for (uint32_t i = 0; i < cycles; i++) {
        gpio_put(BUZZER_PIN, 1);
        sleep_us(period_us / 2);
        gpio_put(BUZZER_PIN, 0);
        sleep_us(period_us / 2);
    }
}