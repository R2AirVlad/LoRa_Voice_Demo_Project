
#pragma once // контроль за тем, чтобы конкретный исходный файл при компиляции подключался строго один раз.

// lora modulation parameters
#define LORA_RADIO_FREQ         432.000 // Частота середины чирпа
#define LORA_RADIO_BW           500.0 // Полоса частот чирпа 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250 и 500 kHz.
#define LORA_RADIO_SF           10 // Фактор разброса от 6 (быстро но не помехоустойчиво) до 12 (медленно, но помехоустойчиво).
#define LORA_RADIO_CR           7 // Соотношение кодирования. Диапазон от 5 (меньше дублирующих данных - бастрее) до 8 (больше дублирующих данных - медленнее).
#define LORA_RADIO_PWR          2 // Мощьность излучения передатчика - от 2 до 17 dBm. (1.6 - 50 млВт)
#define LORA_RADIO_SYNC         0x34 // LoRa слово синхронизации. Может быть установлено для определения различных сетей связи.
#define LORA_PREAMBLE_LENGTH    8 // Длинна преамбулы LoRa в количестве символов. Реальная длина приамбулы больше на величину 4.25 символа, чем установленное тут значение.  Допустимо от 6 до 6553
#define LORA_GAIN               0 // Регулировка усиления МШУ предусилителя модема. Устанавливается в диапазоне от 1 до 6, где 1 - максимальное усиление. Установка 0 (нуля) говорит о включении АРУ (рекомендуется).
#define LORA_RADIO_CRC          true // Включаем контроль четности пакетов
#define LORA_FHSS_HOP_PER       10 // Включаем прыжки по частоте и устанавливаем период прыжков

// Подготовка данных для ЛоРа 
#define LORA_RADIO_BUF_LEN      256   // Размер буфера пакета ЛоРа, 256 - максимум, что может передать ЛоРа за один такт
#define LORA_RADIO_QUEUE_LEN    512   // Длинна очереди

#define LORA_RADIO_TASK_RX_BIT  0x01  // Флаг-Бит нужен для идентификатора источника прерывания RTOS при приеме

// Настройка подключения пинов усилителя I2S MAX98357A к пинам контроллера. Вывод GAIN необходимо соединить с GND.
// Напряжение питания 3,3 В (вывод Vin) можно взять с любого подходящего вывода платы, а вывод GND соединить с GND платы.
#define AUDIO_SPEAKER_BCLK      4 // соединяем вывод BCLK с GPIO 4 платы
#define AUDIO_SPEAKER_LRC       15 // соединяем вывод LRC с GPIO 15 платы
#define AUDIO_SPEAKER_DIN       2  // соединяем вывод DIN с GPIO 2 платы

// Настройка звука
#define AUDIO_CODEC2_MODE	      CODEC2_MODE_1600 // Скорость Codec2
#define AUDIO_SAMPLE_RATE       8000    // Скорость сэмплирования
#define AUDIO_MAX_PACKET_SIZE   48      // Максимальный размер пакета, включая множественные кадры звуковых данных

#define AUDIO_TASK_PLAY_BIT     0x01    // Бит флага для Задачи (таска) старта воспроизведения звука

#define UNUSE_PIN                   (0)

#define I2C_SDA                     21 // экран
#define I2C_SCL                     22 // экран


#define RADIO_SCLK_PIN              5 //SPI радио
#define RADIO_MISO_PIN              19 //SPI радио
#define RADIO_MOSI_PIN              27 //SPI радио
#define RADIO_CS_PIN                18 // радио
#define RADIO_DIO0_PIN              26 // радио 
#define RADIO_RST_PIN               23 // радио
#define RADIO_DIO1_PIN              33 // резерв
#define RADIO_BUSY_PIN              32 // радио


#define BOARD_LED                   25
#define LED_ON                      HIGH

#define ADC_PIN                     35








