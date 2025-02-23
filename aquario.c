#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "pico/binary_info.h"
#include "include/ssd1306.h"

const uint I2C_SDA = 14;
const uint I2C_SCL = 15;

#include "ws2818b.pio.h" // Biblioteca gerada pelo arquivo .pio durante compilação.
#define LED_COUNT 25 // Define número de LEDs da matrix.
#define LED_PIN 7 // Define GPIO da matix.
#define DS18B20_PIN 0 // Definição do GPIO onde o DS18B20 está conectado
#define tecla_b_ 6 // GPIO 6 para a tecla B
#define BUZZER_PIN 21 // Pino do Buzzer
#define BUZZER_FREQUENCY 10000 // frequencia do buzzer

float temperatura_ = 28;  // temperatura do sistema.
float variacao_ = 2; // tolerância de variação 
float erro_ = 2; // limite de falha 

// Definição de pixel GRB para a matriz de LED - função dos exemplos do BitDogLab. 
struct pixel_t 
{
  uint8_t G, R, B; // Três valores de 8-bits compõem um pixel.
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t; // Mudança de nome de "struct pixel_t" para "npLED_t" por clareza.
npLED_t leds[LED_COUNT]; // // Declaração do buffer de pixels que formam a matriz.

PIO np_pio; // Variáveis para uso da máquina PIO.
uint sm;

void ds18b20_init()  // Função para configurar o GPIO para o sensor de temperatura 
{
  gpio_init(DS18B20_PIN);
  gpio_set_dir(DS18B20_PIN, GPIO_OUT);
  gpio_put(DS18B20_PIN, 1); // Mantém o barramento em estado alto
} 

bool ds18b20_reset() // Função para enviar um pulso de reset ao sensor de temperatura
{
    gpio_set_dir(DS18B20_PIN, GPIO_OUT);
    gpio_put(DS18B20_PIN, 0);
    sleep_us(480); // Pulso de reset
    gpio_set_dir(DS18B20_PIN, GPIO_IN);
    sleep_us(70);
    bool presence = !gpio_get(DS18B20_PIN);
    sleep_us(410);
    return presence;
}

void ds18b20_write_bit(bool bit) // Função para escrever um bit no barramento 1-Wire para o sensor de temperatura 
{
    gpio_set_dir(DS18B20_PIN, GPIO_OUT);
    gpio_put(DS18B20_PIN, 0);
    sleep_us(bit ? 5 : 60);
    gpio_put(DS18B20_PIN, 1);
    sleep_us(bit ? 55 : 5);
}

bool ds18b20_read_bit() // Função para ler um bit do barramento 1-Wire do sensor de temperatura 
{
    gpio_set_dir(DS18B20_PIN, GPIO_OUT);
    gpio_put(DS18B20_PIN, 0);
    sleep_us(2);
    gpio_set_dir(DS18B20_PIN, GPIO_IN);
    sleep_us(10);
    bool bit = gpio_get(DS18B20_PIN);
    sleep_us(50);
    return bit;
}

void ds18b20_write_byte(uint8_t byte) // Função para escrever um byte no sensor de temperatura 
{
    for (int i = 0; i < 8; i++) 
    {
        ds18b20_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

uint8_t ds18b20_read_byte()  // Função para ler um byte do sensor de temperatura
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (ds18b20_read_bit()) {
            byte |= (1 << i);
        }
    }
    return byte;
}

float ds18b20_get_temperature() // Função para ler a temperatura do sensor DS18B20
{
    if (!ds18b20_reset()) 
    {
        printf("Erro: DS18B20 não encontrado!\n");
        return -1000;
    }
    
    ds18b20_write_byte(0xCC); // Skip ROM (ignorar endereço)
    ds18b20_write_byte(0x44); // Iniciar conversão de temperatura
    sleep_ms(750); // Tempo de conversão (750ms para 12 bits)
    
    if (!ds18b20_reset())
    {
        printf("Erro: DS18B20 não encontrado!\n");
        return -1000;
    }
    
    ds18b20_write_byte(0xCC); // Skip ROM
    ds18b20_write_byte(0xBE); // Comando para ler scratchpad
    uint8_t lsb = ds18b20_read_byte();
    uint8_t msb = ds18b20_read_byte();
    int16_t temp = (msb << 8) | lsb;
    return temp / 16.0; // Conversão para Celsius
}

void npInit(uint pin) // Inicializa a máquina PIO para controle da matriz de LEDs.
{
  uint offset = pio_add_program(pio0, &ws2818b_program); // Cria programa PIO.
  np_pio = pio0;
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) 
  {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true); 
  }

  ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);  // Inicia programa na máquina PIO.

  for (uint i = 0; i < LED_COUNT; ++i) // Limpa buffer de pixels.
  {
    leds[i].R = 0;
    leds[i].G = 0;
    leds[i].B = 0;
  }
}

void npSetLED(const uint index, const uint8_t r, const uint8_t g, const uint8_t b)  // Atribui uma cor RGB a um LED.
{
  leds[index].R = r;
  leds[index].G = g;
  leds[index].B = b;
}

void npClear()  // Limpa o buffer de pixels.
{
  for (uint i = 0; i < LED_COUNT; ++i)
    npSetLED(i, 0, 0, 0);
}

void npWrite() // Escreve os dados do buffer nos LEDs.
{
  // Escreve cada dado de 8-bits dos pixels em sequência no buffer da máquina PIO.
  for (uint i = 0; i < LED_COUNT; ++i) {
    pio_sm_put_blocking(np_pio, sm, leds[i].G);
    pio_sm_put_blocking(np_pio, sm, leds[i].R);
    pio_sm_put_blocking(np_pio, sm, leds[i].B);
  }
  sleep_us(100); // Espera 100us, sinal de RESET do datasheet.
}

void status(int x_) // Mostra o status do sistema na matriz de led (1 - OK, 2 - Alerta, 3 - Falha)
{
  if (x_ == 1)
  {
  npClear();
  npSetLED(6, 0, 100, 0);
  npSetLED(12, 0, 100, 0);
  npSetLED(14, 0, 100, 0);
  npSetLED(18, 0, 100, 0);
  npSetLED(20, 0, 100, 0);
  }
  if (x_ == 2)
  {
    npClear();
    npSetLED(22, 100, 100, 0);
    npSetLED(17, 100, 100, 0);
    npSetLED(12, 100, 100, 0);
    npSetLED(2, 100, 100, 0);
  }
  if (x_ == 3)
  {
    npClear();
    npSetLED(4, 100, 0, 0);
    npSetLED(6, 100, 0, 0);
    npSetLED(12, 100, 0, 0);
    npSetLED(18, 100, 0, 0);
    npSetLED(20, 100, 0, 0);
    npSetLED(24, 100, 0, 0);
    npSetLED(16, 100, 0, 0);
    npSetLED(8, 100, 0, 0);
    npSetLED(0, 100, 0, 0);
  }
  if (x_ == 4)
  {
    npClear();
    npSetLED(13, 0, 0, 100);
    npSetLED(17, 0, 0, 100);
    npSetLED(11, 0, 0, 100);
    npSetLED(7, 0, 0, 100);      
  }
  npWrite(); // Escreve os dados nos LEDs.
}
 
int atualiza_display(float temp_)  // Prepara a área de renderização para o display 
{
    struct render_area frame_area = {start_column : 0, end_column : ssd1306_width - 1, start_page : 0, end_page : ssd1306_n_pages - 1};
    calculate_render_area_buffer_length(&frame_area);

uint8_t ssd[ssd1306_buffer_length];      // zera o display inteiro
memset(ssd, 0, ssd1306_buffer_length);   
render_on_display(ssd, &frame_area);

char temp_str[5];
char temp_str_x[5] = "#C";
snprintf(temp_str, sizeof(temp_str), "%.2f", temp_);  // Converte o float temp_ para string para apresentar no displau=y 

char *text[] = { "A Q U A S Y S ", " ", "Temp", strcat(temp_str, temp_str_x) };  // Array de strings para apresentar no display 

int y = 0;
for (uint i = 0; i < count_of(text); i++)
{
  ssd1306_draw_string(ssd, 5, y, text[i]);
  y += 8;
}
render_on_display(ssd, &frame_area);
sleep_ms(2000);
}

void pwm_init_buzzer(uint pin)  // Incializa o GPIO do Buzzer
{
  gpio_set_function(pin, GPIO_FUNC_PWM);  // Configurar o pino como saída de PWM
  uint slice_num = pwm_gpio_to_slice_num(pin); // Obter o slice do PWM associado ao pino
  pwm_config config = pwm_get_default_config(); // Configurar o PWM com frequência desejada
  pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096)); // Divisor de clock
  pwm_init(slice_num, &config, true);
  pwm_set_gpio_level(pin, 0);  // Iniciar o PWM no nível baixo
}

void beep(uint pin, uint duration_ms)  // Definição de uma função para emitir um beep com duração especificada, função de exemplo do BitDogLab
{
  uint slice_num = pwm_gpio_to_slice_num(pin);  // Obter o slice do PWM associado ao pino
  pwm_set_gpio_level(pin, 2048); // Configurar o duty cycle para 50% (ativo)
  sleep_ms(duration_ms); // Temporização
  pwm_set_gpio_level(pin, 0); // Desativar o sinal PWM (duty cycle 0)
  sleep_ms(100); // Pausa de 100ms
}

 /******************************************************************************************************************************************************************************/

int main() 
{
  stdio_init_all();
  ds18b20_init(); // Inicializa do sensor de tempeatura 
  gpio_init(tecla_b_); // Inicializa o GPIO 6
  gpio_set_dir(tecla_b_, GPIO_IN); // Define o GPIO 6 como entrada
  gpio_pull_up(tecla_b_); // Seleciona o GPIO 6 com pull-up interno
  npInit(LED_PIN); // Inicializa matriz de LEDs NeoPixel.
  npClear(); 
  i2c_init(i2c1, ssd1306_i2c_clock * 1000);  // incializa o barramento I2C do display oled ssd1306
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); 
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
  pwm_init_buzzer(BUZZER_PIN);  // Inicialização do Buzzer 
  ssd1306_init();  // Processo de inicialização completo do OLED SSD1306
  status(1); // Mostra o status inicial no display, evita que o display fique apagado caso a temperatura este dentro do range de OK para alerta 0.2 graus

  while (1)  // laço de repetição de leitura 
  {
    float temp_sensor_ = ds18b20_get_temperature();
    atualiza_display(temp_sensor_);   
    printf("Temperatura: %.2f °C\n", temp_sensor_);
    printf("Pressione a tecla B para configurar...\n");
    printf("\n");

    if (temp_sensor_ >= (temperatura_ + variacao_))
       { 
        status(2);
        printf(" Alerta! : Temperatura alta \n");
        printf("Temperatura: %.2f °C\n", temp_sensor_);
        printf("\n");
        sleep_ms(2000);
       }
   
    if (temp_sensor_ >= (temperatura_ + (variacao_ + erro_)))
       { 
        status(3);
        printf(" Falha! : Temperatura alta \n");
        printf("Temperatura: %.2f °C\n", temp_sensor_);
        printf("\n");
        beep(BUZZER_PIN, 500);
        sleep_ms(2000);
       }
    
    if (temp_sensor_ <= (temperatura_ - variacao_))
       { 
        status(2);
        printf(" Alerta! : Temperatura Baixa \n");
        printf("Temperatura: %.2f °C\n", temp_sensor_);
        printf("\n");
        sleep_ms(2000);
       }
    
    if (temp_sensor_ <= (temperatura_ - (variacao_ + erro_)))
       { 
        status(3);
        printf(" Falha! : Temperatura Baixa \n");
        printf("Temperatura: %.2f °C\n", temp_sensor_);
        printf("\n");
        beep(BUZZER_PIN, 500);
        sleep_ms(2000);
       }
        
       if ((temp_sensor_ > (temperatura_ - (variacao_ - 0.3))) && (temp_sensor_ < (temperatura_ + (variacao_ - 0.3))))
       {
           status(1);
           float a = (temperatura_ - (variacao_ - 0.3));
           float b = (temperatura_ + (variacao_ - 0.3));
           //printf(" -: %.2f | +: %.2f \n", a, b);
           sleep_ms(2000);
       }
       if (gpio_get(tecla_b_) == 0) // Configuração de outra temperatura, a padrão é 28. 
        {
            sleep_ms(100); // Tempo para evitar debounce 
            if (gpio_get(tecla_b_) == 0) 
            {
              status(4);
              printf("Aquasys - Projeto Final EmbarcaTech.\n");
              printf("\n");
              printf("Menu de configuração do sistema.\n");
              printf("Temperatura cadastrada: %.2f ºC, Variação: %.2f ºC \n", temperatura_, variacao_);
              printf("Entre com a temperatura do aquário: ");
              scanf("%f", &temperatura_);  // Captura a entrada do usuário
              while(getchar() != '\n'); // Limpa o buffer de entrada
              printf("Temperatura inserida: %.2f ºC\n", temperatura_);  
              sleep_ms(2000);
              status(1);
            }
        }
  }
}
