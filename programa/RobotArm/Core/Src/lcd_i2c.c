#include "i2c.h"
#include "lcd_i2c.h"

// Dirección I2C efectiva del backpack (se autodetecta en Lcd_Init). Arranca con
// la configurada por defecto; si el módulo está en otra, la corregimos al vuelo.
static uint8_t lcd_addr = LCD_ADDRESS;

// 1 = el LCD respondió el ACK en el arranque; 0 = no está / no responde.
// Si es 0, TODAS las escrituras se saltan: así un LCD ausente o mal cableado NO
// congela el bucle principal (antes cada transmisión sin ACK bloqueaba 100ms, y
// con varias por refresco el sistema entero se ponía lentísimo o "colgado").
static uint8_t lcd_present = 0;

// Timeout corto por transmisión: si el LCD no engancha, degrada rápido en vez de
// frenar el firmware 100ms por byte.
#define LCD_I2C_TIMEOUT 20

// Direcciones típicas (formato 8-bit HAL) de los backpacks I2C para HD44780:
//   0x4E = PCF8574  en 0x27   |   0x7E = PCF8574A en 0x3F
static const uint8_t LCD_CANDIDATE_ADDR[] = { LCD_ADDRESS, 0x4E, 0x7E };

void Lcd_Send_Cmd(char cmd)
{
	if (!lcd_present) return; // LCD ausente: no bloquear el sistema
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xF0);
	data_l = ((cmd<<4) & 0xF0);
	data_t[0] = data_u|0x0C;
	data_t[1] = data_u|0x08;
	data_t[2] = data_l|0x0C;
	data_t[3] = data_l|0x08;
	HAL_I2C_Master_Transmit(&hi2c1, lcd_addr,(uint8_t*) data_t, 4, LCD_I2C_TIMEOUT);
}

void Lcd_Send_Char(char data)
{
	if (!lcd_present) return; // LCD ausente: no bloquear el sistema
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xF0);
	data_l = ((data<<4) & 0xF0);
	data_t[0] = data_u|0x0D;
	data_t[1] = data_u|0x09;
	data_t[2] = data_l|0x0D;
	data_t[3] = data_l|0x09;
	HAL_I2C_Master_Transmit(&hi2c1, lcd_addr,(uint8_t*) data_t, 4, LCD_I2C_TIMEOUT);
}

// Sondea el bus I2C buscando el backpack del LCD. Prueba la dirección configurada
// y las dos típicas. Deja lcd_addr/lcd_present listos. Devuelve 1 si lo encontró.
uint8_t Lcd_Probe(void)
{
    for (uint8_t i = 0; i < sizeof(LCD_CANDIDATE_ADDR); i++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, LCD_CANDIDATE_ADDR[i], 2, 20) == HAL_OK) {
            lcd_addr = LCD_CANDIDATE_ADDR[i];
            lcd_present = 1;
            return 1;
        }
    }
    lcd_present = 0; // No respondió nadie: seguimos sin LCD, sin colgarnos.
    return 0;
}

uint8_t Lcd_IsPresent(void)  { return lcd_present; }
uint8_t Lcd_GetAddress(void) { return lcd_addr; }

void Lcd_Init(void)
{
    // 0. AUTODETECCIÓN: ¿está el backpack en el bus? ¿en qué dirección?
    //    Una dirección I2C equivocada es la causa #1 de "el LCD no responde".
    Lcd_Probe();
    if (!lcd_present) return; // Sin LCD: no intentamos inicializar (no bloquea)

    // 1. Espera inicial de seguridad (el datasheet pide >40ms tras VCC sube a 2.7V)
    HAL_Delay(50);

    // 2. SECUENCIA MÁGICA DE RESET (Hitachi HD44780)
    // Intentamos forzar modo 8-bit tres veces para resetear la máquina de estados interna
    // Nota: Usamos Lcd_Send_Cmd(0x30) repetidamente.
    // Aunque tu función manda 2 nibbles, el LCD interpretará el reset correctamente.

    Lcd_Send_Cmd(0x30);
    HAL_Delay(5);  // Esperar > 4.1ms

    Lcd_Send_Cmd(0x30);
    HAL_Delay(1);  // Esperar > 100us

    Lcd_Send_Cmd(0x30);
    HAL_Delay(10);

    // 3. Ahora sí, pasamos a modo 4-bits
    Lcd_Send_Cmd(0x20); // Function Set: 4-bit interface
    HAL_Delay(10);

    // 4. Configuración Estándar (La que ya tenías)
    Lcd_Send_Cmd(0x28); // 4-bit, 2 líneas, 5x8 puntos
    HAL_Delay(1);

    Lcd_Send_Cmd(0x08); // Display OFF (para evitar parpadeos mientras configuramos)
    HAL_Delay(1);

    Lcd_Send_Cmd(0x01); // Clear Display
    HAL_Delay(2);       // Este comando tarda más, dale 2ms mínimo

    Lcd_Send_Cmd(0x06); // Entry Mode: Increment cursor, no shift
    HAL_Delay(1);

    Lcd_Send_Cmd(0x0C); // Display ON, Cursor OFF, Blink OFF
}

void Lcd_Clear(void)
{
	Lcd_Send_Cmd(0x01);
	HAL_Delay(2);
}

void Lcd_Set_Cursor(int row, int col)
{
	uint8_t address;
	switch(row)
	{
		case 1:
			address = 0x00;
			break;
		case 2:
			address = 0x40;
			break;
		case 3:
			address = 0x14;
			break;
		case 4:
			address = 0x54;
			break;
	}
	address += col - 1;
	Lcd_Send_Cmd(0x80 | address);

}

void Lcd_Send_String(char *str)
{
	while(*str) Lcd_Send_Char(*str++);
}

void Lcd_Shift_Right(void)
{
	Lcd_Send_Cmd(0x1C);
}

void Lcd_Shift_Left(void)
{
	Lcd_Send_Cmd(0x18);
}

void Lcd_Blink(void)
{
	Lcd_Send_Cmd(0x0F);
}

void Lcd_NoBlink(void)
{
	Lcd_Send_Cmd(0x0C);
}

void Lcd_CGRAM_CreateChar(unsigned char pos, const char*msg)
{
    if(pos < 8)
    {
        Lcd_Send_Cmd(0x40 + (pos*8));
        for(unsigned char i=0; i<8; i++)
        {
            Lcd_Send_Char(msg[i]);
        }
    }
}

void Lcd_CGRAM_WriteChar(char pos)
{
    Lcd_Send_Char(pos);
}
