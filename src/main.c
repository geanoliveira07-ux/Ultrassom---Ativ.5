#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <pwm_z42.h>
#include "ultrassom.h"
// ==============================================================================================================================
// DEFINIÇÕES E VELOCIDADES
// ==============================================================================================================================
#define TPM_MODULE          1000
#define DISTANCIA_PARADA    26.5    // Distância de segurança em cm

// Motor B está girando menos em relação ao Motor A, devido a defeito mecânico. Por essa razão, tem-se que abaixar a potência do Motor A para
// equilibrar as velocidades de rotação de ambos. A relação de ambos para o equilíbrio é de:
//
// MOTOR B = 100% da potência -----> MOTOR A = 91,23% da potência      (Sob velocidades máximas)
//
// Desta forma, basta utilizar proporcionalidade (regra de 3) para definir a potência do Motor A sobre qualquer valor para o Motor B, para qual
// estejam em equilíbrio

// Ajuste das velocidades (Lembre-se de fornecer o suficiente para "VEL_FRENTE/A" para haver torque, para assim, quebrar a inércia do motor)
uint16_t VEL_FRENTE             = TPM_MODULE;            // diminuir velocidade
uint16_t VEL_FRENTE_A           = TPM_MODULE * 0.9123;   // diminuir velocidade
uint16_t VEL_CURVA_REVERSA      = TPM_MODULE;            
uint16_t VEL_CURVA_REVERSA_A    = TPM_MODULE * 0.9123;
uint16_t VEL_PARADO             = 0;

// Definição das portas e pinos dos sensores a serem utilizadas
#define PORTA_A         DT_NODELABEL(gpioa)
#define PORTA_E         DT_NODELABEL(gpioe)
#define SENSOR_A_PIN    1                       // PTA1     (Sensor da Esquerda)
#define SENSOR_B_PIN    30                      // PTE30    (Sensor da Direita)

// Definição dos LEDs via DeviceTree
static const struct gpio_dt_spec led_blue   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green  = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
// ==============================================================================================================================
// MÁQUINA DE ESTADOS
// ==============================================================================================================================
typedef enum {
    PARADO,
    FRENTE,
    CURVA_ESQUERDA,     // Sensor A perdeu linha -> vira à esquerda
    CURVA_DIREITA,      // Sensor B perdeu linha -> vira à direita
    OBSTACULO
} carrinho_estado_t;
// ==============================================================================================================================
// HELPERS (Simplificador de comandos, configura os pinos a serem usados e integra a ação desses pinos através de um único comando)
// ==============================================================================================================================
static inline void set_velocidade(uint16_t motor_a, uint16_t motor_b){
    pwm_tpm_CnV(TPM2, 0, motor_a);
    pwm_tpm_CnV(TPM2, 1, motor_b);
}
static inline void set_led(int blue, int green){
    gpio_pin_set_dt(&led_blue, blue);
    gpio_pin_set_dt(&led_green, green);
}
// Controla a direção alterando a Ponte H dinamicamente
static inline void set_direcao(const struct device *dev_a, const struct device *dev_e, bool motor_a_frente, bool motor_b_frente) {
    if (motor_a_frente) {               // Motor A (Pinos PTE0 e PTE1)
        gpio_pin_set(dev_e, 0, 1);      // IN1 = 0                  
        gpio_pin_set(dev_e, 1, 0);      // IN2 = 1                  
    } else {
        gpio_pin_set(dev_e, 0, 0);      // IN1 = 1 (Ré)              
        gpio_pin_set(dev_e, 1, 1);      // IN2 = 0 (Ré)              
    }
    if (motor_b_frente) {               // Motor B (Pinos PTA16 e PTA17)
        gpio_pin_set(dev_a, 16, 1);     // IN3 = 0                  
        gpio_pin_set(dev_a, 17, 0);     // IN4 = 1                  
    } else {
        gpio_pin_set(dev_a, 16, 0);     // IN3 = 1 (Ré)            
        gpio_pin_set(dev_a, 17, 1);     // IN4 = 0 (Ré)            
    }
}
// ==============================================================================================================================
// MAIN
// ==============================================================================================================================
int main(void) {
                                                                    printk("[SISTEMA] Iniciando integração total do Carrinho...\n");
    // 1. Inicializa PWM (TPM2) para os Motores
    pwm_tpm_Init   (TPM2, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM2, 0, TPM_PWM_H, GPIOB, 2);                  // ENA - Motor A
    pwm_tpm_Ch_Init(TPM2, 1, TPM_PWM_H, GPIOB, 3);                  // ENB - Motor B
                                                                    printk("[DEGUB] Motores inicalizados.\n");
    // 2. Inicializa Ultrassom
    ultrassom_init();
                                                                    printk("[DEBUG] Ultrassom_init finalizada.\n");
    // 3. Configura GPIOs para uso (Sensores e Direção)
    const struct device *gpioa_dev = DEVICE_DT_GET(PORTA_A);
    const struct device *gpioe_dev = DEVICE_DT_GET(PORTA_E);
    if(!device_is_ready(gpioa_dev) || !device_is_ready(gpioe_dev)){
                                                                    printk("[ERRO] Porta A ou E nao pronta!\n");
        return 0;
    }
    // Direção: ambos os motores para frente
    gpio_pin_configure(gpioe_dev, 0,  GPIO_OUTPUT_INACTIVE);        // IN1
    gpio_pin_configure(gpioe_dev, 1,  GPIO_OUTPUT_INACTIVE);        // IN2
    gpio_pin_configure(gpioa_dev, 16, GPIO_OUTPUT_INACTIVE);        // IN3
    gpio_pin_configure(gpioa_dev, 17, GPIO_OUTPUT_INACTIVE);        // IN4
    // Sensores com pull-up (retorna 0 quando detecta linha)
    gpio_pin_configure(gpioa_dev, SENSOR_A_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure(gpioe_dev, SENSOR_B_PIN, GPIO_INPUT | GPIO_PULL_UP);
                                                                    printk("[DEGUB] Configuração das GPIOs finalizada.\n");
    // 4. Configura os LEDS para uso
    if(!gpio_is_ready_dt(&led_blue) || !gpio_is_ready_dt(&led_green)) return 0;
    gpio_pin_configure_dt(&led_blue,    GPIO_OUTPUT_INACTIVE);                
    gpio_pin_configure_dt(&led_green,   GPIO_OUTPUT_INACTIVE);
                                                                    printk("[DEGUB] Configuração dos LEDs finalizada.\n");
    // 5. Define estado inicial para a máquina de estados e a váriavel volátil de verificação do ultrassom
    carrinho_estado_t estado = PARADO;
    carrinho_estado_t ultimo_estado = FRENTE;   // Memória da última ação válida
    uint32_t contador = 0;              
    uint32_t tempo_perdido = 0;                 // Contador de tempo fora da pista
    uint32_t bloqueio_esquerda_timer = 0;
                                                                    printk("[DEGUB] Entrando no loop principal.\n");
// ==============================================================================================================================
// LEITURAS
// ==============================================================================================================================
    while (1) {
        // Dispara o trigger a cada 50ms (5 ciclos de 10ms)
        if (contador % 5 == 0)  ultrassom_trigger();

        uint32_t dist = ultrassom_get_distancia();

        // ========= Leitura dos Sensores IR (Lógica invertida: 0 = detecta linha) =============
        bool esq_na_linha = (gpio_pin_get(gpioa_dev, SENSOR_A_PIN) == 0);
        bool dir_na_linha = (gpio_pin_get(gpioe_dev, SENSOR_B_PIN) == 0);

        // =============================== Transição de estados  ===============================
        //      A   B       |       Ação
        //      1   1       |       Frente          - Ambos na linha
        //      1   0       |       Curva Dir.      - B perdeu  -   corrige para direita
        //      0   1       |       Curva Esq.      - A perdeu  -   corrige para esquerda
        //      1   1       |       Parado          - linha perdida completamente
       
        if (bloqueio_esquerda_timer > 0)    bloqueio_esquerda_timer--;      // Decrementa o temporizador de bloqueio a cada ciclo de 10ms

        // Prioridade 1: Segurança (Obstáculo)
        if (dist > 0 && dist < DISTANCIA_PARADA){
            estado = OBSTACULO;
            tempo_perdido = 0;                      // Reseta o tempo pois ele não perdeu a linha
        }
        // Prioridade 2: Seguidor de Linha
        else if (esq_na_linha && dir_na_linha){
            estado = FRENTE;
                                                    // LÓGICA DA TARJA: Se estava virando à direita e achou a tarja dupla...
                                                    // Ativa o bloqueio: Ignora a esquerda por 25 ciclos (250 ms)               [10]
            if (ultimo_estado == CURVA_DIREITA){ bloqueio_esquerda_timer = 10; }
            ultimo_estado = FRENTE;                 // Salva na memória
            tempo_perdido = 0;                      // Zera o cronômetro de perda
        }      
        else if (esq_na_linha && !dir_na_linha){
                                                    // LÓGICA DE DEFESA: O sensor pediu para virar à esquerda!
                                                    // Mas o temporizador diz que acabamos de cruzar a tarja. É um falso positivo!
                                                    // Ignora a ordem de virar à esquerda e força o carrinho a ir para frente para pular a fita.
            if (bloqueio_esquerda_timer > 0){ estado = FRENTE; }
            else{
                estado = CURVA_DIREITA;             // Passou o tempo de segurança, é uma curva à esquerda verdadeira.
                ultimo_estado = CURVA_DIREITA;
            }
            tempo_perdido = 0;    
        }    
        else if (!esq_na_linha && dir_na_linha){
                                                    // LÓGICA DE DEFESA: O sensor pediu para virar à esquerda!                
                                                    // Mas o temporizador diz que acabamos de cruzar a tarja. É um falso positivo!
                                                    // Ignora a ordem de virar à esquerda e força o carrinho a ir para frente para pular a fita.
            if (bloqueio_esquerda_timer > 0){ estado = FRENTE; }
            else{
                estado = CURVA_ESQUERDA;            // Passou o tempo de segurança, é uma curva à esquerda verdadeira.
                ultimo_estado = CURVA_ESQUERDA;
            }
            tempo_perdido = 0;              
        }
        else{
            if(tempo_perdido < 50){                 // Verifica se está perdido a menos de 100 ciclos (100 * 10ms = 1 segundo)  [50]
                    estado = FRENTE;                // Mantém a última manobra para tentar se recuperar
                    tempo_perdido++;
            }
            else    estado = PARADO;                // Se passou de 1 segundo sem ver a linha. Para
        }
// ================================================== AÇÃO DOS ESTADOS ==========================================================
        switch (estado){
            case OBSTACULO:
                set_direcao(gpioa_dev, gpioe_dev, true, true);      // Obstáculo detectado: Para os motores IMEDIATAMENTE
                set_led(1, 0);                                      // Verde
                set_velocidade(VEL_PARADO, VEL_PARADO);
                if (contador % 50 == 0)                             printk("ALERTA: Obstaculo a %u cm!\n", dist);
                break;
            case FRENTE:
                set_direcao(gpioa_dev, gpioe_dev, true, true);      // Ambos os sensores na linha -> velocidade máxima      
                set_led(1, 1);                                      // Ciano
                set_velocidade(VEL_FRENTE_A, VEL_FRENTE);
                break;
            case CURVA_DIREITA:
                set_direcao(gpioa_dev, gpioe_dev, true, false);     // Sensor B perdeu linha -> motor B inverte (dir.), motor A continua (esq.)    
                set_led(0, 1);                                      // Verde
                set_velocidade(VEL_CURVA_REVERSA_A, VEL_CURVA_REVERSA);
                k_busy_wait(100);
                break;
            case CURVA_ESQUERDA:
                set_direcao(gpioa_dev, gpioe_dev, false, true);     // Sensor A perdeu linha -> motor A inverte (esq.), motor B continua (dir.)            
                set_led(1, 0);                                      // Azul
                set_velocidade(VEL_CURVA_REVERSA_A, VEL_CURVA_REVERSA);
                k_busy_wait(100);
                break;
            case PARADO:
                set_direcao(gpioa_dev, gpioe_dev, true, true);      // Ambos os sensores na linha -> velocidade máxima      
                set_led(1, 1);                                      // Ciano
                set_velocidade(VEL_FRENTE_A, VEL_FRENTE);
                k_busy_wait(100);
                set_direcao(gpioa_dev, gpioe_dev, true, true);      // Nenhum sensor detecta linha -> para tudo    
                set_led(0, 0);                                      // Apagado
                set_velocidade(VEL_PARADO, VEL_PARADO);
                break;
        }
        contador++;
        k_msleep(10); // Loop de 100Hz
    }
}
//======================================================================
//PINAGEM FÍSICA
//======================================================================
//                              Microcontrolar         Ponte H
//MOTOR A - PWM                 PTB2                   ENA          (Cinza)
//MOTOR B - PWM                 PTB3                   ENB          (Marrom)
//MOTOR A - DIREÇÃO 1           PTE0                   IN1          (Branco)
//MOTOR A - DIREÇÃO 2           PTE1                   IN2          (Cinza)
//MOTOR B - DIREÇÃO 1           PTA16                  IN3          (Laranja)
//MOTOR B - DIREÇÃO 2           PTA17                  IN4          (Roxo)
//SENSOR A (CONTROLA MOTOR A)   PTA1            
//SENSOR B (CONTROLA MOTOR B)   PTE30
//ULTRASSOM TRIGGER             PTD1
//ULTRASSOM ECHO                PTD3