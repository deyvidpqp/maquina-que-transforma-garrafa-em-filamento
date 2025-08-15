
#include <xc.h>
#include <stdio.h>
#include <math.h>
#include "LCD_4VIAS.h"

// CONFIGURAÇÕES
#define _XTAL_FREQ 4000000

// ==== Definições do motor de passos ====
#define STEP1 LATBbits.LATB0
#define STEP2 LATBbits.LATB1
#define STEP3 LATBbits.LATB2
#define STEP4 LATBbits.LATB3

// ==== Definições dos botões/encoder ====
#define ENC_A PORTAbits.RA4   // Canal A encoder
#define ENC_B PORTAbits.RA5   // Canal B encoder
#define ENC_BTN PORTAbits.RA3 // Botão do encoder

// ==== Definições do termistor ====
#define BETA 3950.0
#define R0 100000.0
#define T0 298.15
#define SERIE_RESISTOR 100000.0

// ==== Protótipos ====
void ADC_Init(void);
unsigned int ADC_Read(unsigned char canal);
float temperatura_C(unsigned int ADC_value);
void passo_motor(unsigned char motor);
void lcd_inicializa(void);
void lcd_envia_nibble(int dado);
void lcd_envia_comando(int comando);
void lcd_envia_dado(int dado);
void lcd_cursor(int linha, int coluna);
void lcd_envia_texto(char* str);

// ==== Estados do sistema ====
typedef enum {
    ST_IDLE = 0,
    ST_AJUSTE_TEMP = 1,
    ST_AJUSTE_VEL = 2
} EstadoAjuste;

void main(void) {
    unsigned int adc_value, adc_temp;
    unsigned int speed, speed_manual = 0;
    char buffer[17];
    unsigned char step_seq = 0;
    unsigned int delay_ms;
    float temperatura, temperatura_anterior = 0.0f;
    unsigned int real_speed;
    unsigned char ligado = 0;
    unsigned int debounce = 0;

    // Para encoder
    unsigned char enc_btn_last = 1, enc_btn_count = 0, enc_btn_confirm = 0;
    unsigned char enc_a_last = 0, enc_b_last = 0;
    int ajuste_temp = 25, ajuste_vel = 100; // valor configurável
    EstadoAjuste estado_ajuste = ST_IDLE;

    // Inicialização
    ADC_Init();
    lcd_inicializa();
    TRISB = 0xF0;      // RB0-RB3 como saída (motor)
    LATB = 0x00;
    TRISA0 = 1;        // Potenciômetro
    TRISA1 = 1;        // NTC
    TRISA3 = 1;        // Botão encoder
    TRISA4 = 1;        // Encoder A
    TRISA5 = 1;        // Encoder B

    lcd_cursor(0,0);
    lcd_envia_texto(" AGUARDANDO LIGAR ");

    while(1) {
         // ----- Temperatura -----
        adc_temp = ADC_Read(1); // Supondo NTC em AN1 (RA1), altere se necessário!
        temperatura = temperatura_C(adc_temp);

        lcd_cursor(0,0);
        sprintf(buffer, "Temp:%5.1fC", temperatura);
        lcd_envia_texto(buffer);

        lcd_cursor(0,12);
        if (temperatura > temperatura_anterior + 0.1f) {
            lcd_envia_texto("UP ");
        } else if (temperatura < temperatura_anterior - 0.1f) {
            lcd_envia_texto("DN ");
        } else {
            lcd_envia_texto("STB");
        }
        temperatura_anterior = temperatura;

        // ----- Velocidade -----
        adc_value = ADC_Read(0); // Potenciômetro em AN0

        // Mapeia valor do ADC (0-1023) para velocidade (ex: 10 a 300 steps/s)
        speed = 10 + ((adc_value * 290) / 1023);
        
        int ajuste_temp = 25, ajuste_vel = 100; // valor configurável

        // Velocidade final = ADC + ajuste manual
        real_speed = speed + speed_manual;
        if(real_speed < 10) real_speed = 10;
        if(real_speed > 500) real_speed = 500;

        delay_ms = 1000 / real_speed;

        // Mostra velocidade na linha 2
        lcd_cursor(1,0);
        sprintf(buffer,"Vel:%3u stp/s", real_speed);
        lcd_envia_texto(buffer);

         // Gira motor de passos
        passo_motor(step_seq);
        step_seq = (step_seq + 1) % 4;
        // Corrigido: delay_ms variável
        for(unsigned int i = 0; i < delay_ms; i++) {
            __delay_ms(1);
        }
        // === BOTÃO ENCODER: LIGA/DESLIGA & CONTAGEM DE CLIQUES ===
        if(!ENC_BTN && enc_btn_last) { // click detectado
            __delay_ms(30); // debounce
            if(!ENC_BTN) {
                enc_btn_count++;
                enc_btn_confirm = 0;
                if(!ligado) {
                    ligado = 1;
                    lcd_cursor(1, 0);
                    lcd_envia_texto("   SISTEMA LIGADO ");
                }
            }
        }
        if(ENC_BTN && !enc_btn_last && enc_btn_count>0) {
            // Botão solto após click
            if(enc_btn_count == 2 && ligado) {
                estado_ajuste = ST_AJUSTE_TEMP;
                enc_btn_confirm = 0;
            }
            else if(enc_btn_count == 3 && ligado) {
                estado_ajuste = ST_AJUSTE_VEL;
                enc_btn_confirm = 0;
            }
            else if(enc_btn_count == 1 && ligado && estado_ajuste != ST_IDLE) {
                // Confirma ajuste
                estado_ajuste = ST_IDLE;
                enc_btn_confirm = 1;
            }
            enc_btn_count = 0;
        }
        enc_btn_last = ENC_BTN;

        // === SISTEMA DESLIGADO ===
        if(!ligado) {
            LATB = 0x00;
            lcd_cursor(0, 0);
            lcd_envia_texto(" AGUARDANDO LIGAR ");
            estado_ajuste = ST_IDLE;
            __delay_ms(200);
            continue;
        }

        // --- Ajuste via ENCODER ROTATIVO ---
        unsigned char enc_a = ENC_A;
        unsigned char enc_b = ENC_B;

        if(estado_ajuste == ST_AJUSTE_TEMP && !enc_btn_confirm) {
            lcd_cursor(0,0);
            sprintf(buffer,"AJUSTE TEMP:%2dC", ajuste_temp);
            lcd_envia_texto(buffer);
            lcd_cursor(1,0);
            lcd_envia_texto("GIRE p/ AJUSTAR  ");
            // Rotação do encoder: esquerda/direita
            if(enc_a != enc_a_last) {
                if(enc_b != enc_a) { // Direita
                    ajuste_temp++;
                    if(ajuste_temp > 99) ajuste_temp = 99;
                } else { // Esquerda
                    ajuste_temp--;
                    if(ajuste_temp < 0) ajuste_temp = 0;
                }
                lcd_cursor(0,0);
                sprintf(buffer,"AJUSTE TEMP:%2dC", ajuste_temp);
                lcd_envia_texto(buffer);
                __delay_ms(100);
            }
        } else if(estado_ajuste == ST_AJUSTE_VEL && !enc_btn_confirm) {
            lcd_cursor(0,0);
            sprintf(buffer,"AJUSTE VEL:%3drpm", ajuste_vel);
            lcd_envia_texto(buffer);
            lcd_cursor(1,0);
            lcd_envia_texto("GIRE p/ AJUSTAR  ");
            if(enc_a != enc_a_last) {
                if(enc_b != enc_a) { // Direita
                    ajuste_vel += 10;
                    if(ajuste_vel > 500) ajuste_vel = 500;
                } else {
                    ajuste_vel -= 10;
                    if(ajuste_vel < 10) ajuste_vel = 10;
                }
                lcd_cursor(0,0);
                sprintf(buffer,"AJUSTE VEL:%3drpm", ajuste_vel);
                lcd_envia_texto(buffer);
                __delay_ms(100);
            }
        } else if(estado_ajuste == ST_IDLE) {
            // ----- Temperatura -----
            adc_temp = ADC_Read(1);
            temperatura = temperatura_C(adc_temp);

            lcd_cursor(0,0);
            sprintf(buffer, "Temp: %5.1fC", temperatura);
            lcd_envia_texto(buffer);

            lcd_cursor(0,12);
            if (temperatura > temperatura_anterior + 0.1f) {
                lcd_envia_texto("UP ");
            } else if (temperatura < temperatura_anterior - 0.1f) {
                lcd_envia_texto("DN ");
            } else {
                lcd_envia_texto("STB");
            }
            temperatura_anterior = temperatura;

            // ----- Velocidade -----
            adc_value = ADC_Read(0);
            speed = 10 + ((adc_value * 290) / 1023);

            // Usa valor ajustado se estiver em modo ajuste
            real_speed = ajuste_vel;
            delay_ms = 1000 / real_speed;

            // Mostra velocidade na linha 2
            lcd_cursor(1,0);
            sprintf(buffer,"Vel:%3u stp/s", real_speed);
            lcd_envia_texto(buffer);

            // Gira motor de passos
            passo_motor(step_seq);
            step_seq = (step_seq + 1) % 4;
            for(unsigned int i = 0; i < delay_ms; i++) {
                __delay_ms(1);
            }
        }

        enc_a_last = enc_a;
        enc_b_last = enc_b;
    }
}

// ==== Funções do motor ====
void passo_motor(unsigned char motor) {
    switch(motor){
        case 0: STEP1=1; STEP2=0; STEP3=0; STEP4=0; break;
        case 1: STEP1=0; STEP2=1; STEP3=0; STEP4=0; break;
        case 2: STEP1=0; STEP2=0; STEP3=1; STEP4=0; break;
        case 3: STEP1=0; STEP2=0; STEP3=0; STEP4=1; break;
    }
}

// ==== ADC ====
void ADC_Init(void) {
    ADCON1 = 0x80; // Vref = Vdd, resultado à direita
    TRISA0 = 1;    // AN0 como entrada (pot)
    TRISA1 = 1;    // AN1 como entrada (NTC)
    ADCON0 = 0x41; // Liga ADC, canal 0
}

unsigned int ADC_Read(unsigned char canal) {
    if(canal > 7) return 0;
    ADCON0 &= 0xC5;
    ADCON0 |= (canal << 3);
    __delay_us(10);
    GO_nDONE = 1;
    while(GO_nDONE);
    return ((unsigned int)ADRESH << 8) | ADRESL;
}

// ==== Temperatura ====
float temperatura_C(unsigned int ADC_value) {
    float Vout, Rt, tempK, tempC;
    float vref = 5.0f;
    Vout = (ADC_value * vref) / 1023.0f;
    Rt = (SERIE_RESISTOR * Vout) / (vref - Vout);
    tempK = 1.0f / ((1.0f/T0) + (1.0f/BETA) * logf(Rt/R0));
    tempC = tempK - 273.15f;
    return tempC;
}

// ==== LCD 4 VIAS ====
void lcd_inicializa(void){
    LCD_RS = 0;
    __delay_ms(100);
    lcd_envia_nibble(3);
    __delay_ms(5);
    lcd_envia_nibble(3);
    __delay_ms(10);
    lcd_envia_nibble(3);
    lcd_envia_nibble(2);
    lcd_envia_comando(0x28);
    lcd_envia_comando(0x0C);
    lcd_envia_comando(0x01);
    lcd_envia_comando(0x06);   
}

void lcd_envia_nibble(int dado){
    LCD_D4 = (dado & 1) ? 1:0;
    LCD_D5 = (dado & 2) ? 1:0;
    LCD_D6 = (dado & 4) ? 1:0;
    LCD_D7 = (dado & 8) ? 1:0;
    LCD_E = 1;
    __delay_us(1);
    LCD_E = 0;  
}

void lcd_envia_comando(int comando){
    LCD_RS = 0;
    lcd_envia_nibble(comando >> 4);
    lcd_envia_nibble(comando & 0x0F);
    __delay_ms(2);
}

void lcd_envia_dado(int dado){
    LCD_RS = 1;
    lcd_envia_nibble(dado >> 4);
    lcd_envia_nibble(dado & 0x0F);
    __delay_ms(1);
}

void lcd_cursor(int linha, int coluna){
    if(linha == 0) lcd_envia_comando(0x80 + coluna);
    if(linha == 1) lcd_envia_comando(0xC0 + coluna);
}

void lcd_envia_texto(char* str){
    int tamanho = 0;
    while(*str != 0 && tamanho++ < 16) lcd_envia_dado(*str++);  
}