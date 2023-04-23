/*
Проект "LoRa Voice" задуман как эксперимент для проверки алгоритмов реализации передачи голоса через модемы LoRa.
 ЦЕЛИ ПРОЕКТА:
 - реализовать и оптимизировать методы приема/передачи голоса;
 - определить оптимальные настройки модема и кодека, обеспечивающие максимальную помехоустойчивость при приемлемом качестве голоса; 
 - описать максимально доступно алгоритмы работы в целях их развития и совершенствования, вовлечения бОльшей аудитории в процесс разработки;
 - определить пеерспективы данного подхода для использования в прикладных устройствах; 

Основным преимуществом метода передачи голоса через модуляцию LoRa является экстремально высокая помехоустойчивость. 
А это свойство, благодаря запатентованной модуляции LoRa, позволяет достигать наверное лучших показателей деальности 
связи в сравниии с традиционными методами модуляции FSK, GMSK, PSK/OFDM, FHSS (при прочих равных условиях). 
При проведении экспериментов, комплекс приемник/передачик демострировал уверенное декодирование пакетов 
при соотношении сигнал/шум -17 дб (уровень шума больше полезного сигнала на 17 дб то есть в 7 раз по напряжению). 
При этом, для данных условий теоретическое значение предела Шеннона составляет примерно -24 дб.

Подробнее про LoRa - https://itechinfo.ru/content/обзор-технологии-lora

В качестве эквпериментального объекта выбрана распространенная платформа разработки от компании LILYGO®  - TTGO LoRa32 версия 1.6.1 на 433 Мгц. 
Эта плата основана на процессоре ESP32 и включает в себя модем SX1278, OLED 0.96 дюйа и SD карт-ридер. 
Ссылка: https://aliexpress.ru/item/32872078587.html?sku_id=12000031557075306&spm

Для реализации передатчика вам потребуется данная плата и микрофон I2S INMP441. 

Все настройки устройства вынесены для удобства в отдельный файл utilities.h - там же содержатся указания по пинам подлючения микрофона.

Для установки платы необходимо использовать настройки:
json ESP32: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
Выбор платы: TTGO LoRa32-OLED
В Меню Инструменты Arduino IDE: Eraise All Flash Before Sketch Uploaded - Enable; Flash Frequency - 40Mhz; Board Revition: TTGO LoRa32 (1.6.1); Upload Speed: 115200

ВНИМАНИЕ: Чтобы не нарушать правила ГКРЧ не превышайте выходную мощность передатчика более 10 миливатт = +10дбм 

*/

#include <Arduino.h> // стандартная бибилиотека ESP32
#include <SPI.h> // стандартная бибилиотека ESP32
#include <Wire.h> // стандартная бибилиотека ESP32
#include <FS.h> // стандартная бибилиотека ESP32
#include <U8g2lib.h> // бибилиотека дисплея: https://github.com/olikraus/u8g2
#include <RadioLib.h> // библиотека модема SX1278: https://github.com/jgromes/RadioLib
#include <driver/i2s.h> //драйвер устройств шины I2S - стандартный Ардуино драйвер
#include <codec2.h> // голосовой кодек, очень сильно сжимает звук: https://github.com/sh123/esp32_codec2_arduino
#include <CircularBuffer.h> // кольцевой буфер, действует по принципу первый вошел - первый вышел, нужен для преобразования потока данных с кодека в пакеты ЛоРа: https://github.com/rlogiacco/CircularBuffer
#include "utilities.h" // настройки


U8G2_SSD1306_128X64_NONAME_F_HW_I2C *u8g2 = nullptr; //установка дисплея


//Создаем 2 кольцевых буфера для использования с ЛоРа: задаем тип данных буфера целые числа 0-255, задаем размер буфера.
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_tx_queue_; //декларируем, что этот буфер для очереди передачи
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_tx_queue_index_; //декларируем, что этот буфер для индексов передачи

// Декларируем с типом байт (целые числа 0-255) переменные для буферов приема/передачи
byte lora_radio_tx_buf_[LORA_RADIO_BUF_LEN];  // буфер передачи

TaskHandle_t lora_task_;    // Создание Задачи FREE RTOS для ЛоРа, который обеспечивает передачу пакетов. 

SX1278 lora_radio_ = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN); // определяем пины модуля ЛоРа

TaskHandle_t audio_task_;       /// таск записи звука

// Настройка Codec2
struct CODEC2* c2_;             // Создание структуры codec2, в которой есть разнотипные переменные. Создаем указатель( * )на CODEC2 называем его с2_
int c2_samples_per_frame_;      // Декларируем 32 битную (4 байта) переменную сколько чистых сэмплов в одном кадре 
int c2_bytes_per_frame_;        // Декларируем 32 битную (4 байта) переменную сколько чистых байт в одном кадре
int16_t *c2_samples_;           // Декларируем 16 битную (2 байта) переменную размера буфера для чистых сэмплов
uint8_t *c2_bits_;              // Декларируем 8 битную (1 байт) переменную размера буфера для закодированных кадров



void setup()

{
  Serial.begin(115200); // стартуем серийный порт
  Serial.println("initBoard"); // пишем в серийный порт, что начинается инициализация
  SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN); // стартуем SPI для модуля ЛоРа
  Wire.begin(I2C_SDA, I2C_SCL); // и для дисплея
   
//включаем светодиод на плате
  gpio_hold_dis(GPIO_NUM_4);
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, LED_ON);

//запускаем OLED и рисуем заставку LoRaVoice
  Wire.beginTransmission(0x3C);
    if (Wire.endTransmission() == 0) {
        Serial.println("Started OLED");
        u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
        u8g2->begin();
        u8g2->clearBuffer();
        u8g2->setFlipMode(0);
        u8g2->setFontMode(1); // Прозрачность
        u8g2->setDrawColor(1);
        u8g2->setFontDirection(0);
        u8g2->firstPage();
        do {
            u8g2->setFont(u8g2_font_inb19_mr);
            u8g2->drawStr(0, 30, "LoRa");
            u8g2->drawHLine(2, 35, 47);
            u8g2->drawHLine(3, 36, 47);
            u8g2->drawVLine(45, 32, 12);
            u8g2->drawVLine(46, 33, 12);
            u8g2->setFont(u8g2_font_inb16_mf); // вибираем шрифт
            u8g2->drawStr(52, 57, "Voice");
        } while ( u8g2->nextPage() );
        u8g2->sendBuffer();
        u8g2->setFont(u8g2_font_fur11_tf); // вибираем шрифт
        delay(3000);
    }

// Немного ждем для прохождения внутренних процессов платы.
    delay(1500);

// запускаем и настраиваем модем SX1278 
    int state = lora_radio_.begin(LORA_RADIO_FREQ, LORA_RADIO_BW, LORA_RADIO_SF, LORA_RADIO_CR, LORA_RADIO_SYNC, LORA_RADIO_PWR, LORA_PREAMBLE_LENGTH, LORA_GAIN);
    lora_radio_.setCRC(LORA_RADIO_CRC);
    lora_radio_.setFHSSHoppingPeriod(LORA_FHSS_HOP_PER);

// Настройка входа драйвера I2S для записи звука с внешнего микрофона I2S, стандартная настройка, как и устройства воспроизведения. https://docs.espressif.com/projects/esp-idf/en/v3.3/api-reference/peripherals/i2s.html
  i2s_config_t i2s_mic_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // бывают разные версии микрофона, обычно все работает, но если нет, можно поменять на I2S_CHANNEL_FMT_ONLY_RIGHT
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll=0,
    .tx_desc_auto_clear= true,
    .fixed_mclk=-1
  };
//Натраиваем пины микрофона
  i2s_pin_config_t i2s_mic_pin_config = {
    .bck_io_num = AUDIO_MIC_SCK,
    .ws_io_num = AUDIO_MIC_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = AUDIO_MIC_SD
  };
//Тут и ниже, стандартная обратока запуска I2S, если есть ошибки - пишем в лог  
  if (i2s_driver_install(I2S_NUM_1, &i2s_mic_config, 0, NULL) != ESP_OK) {
    Serial.println(F("Failed to install i2s mic driver"));
  }
  if (i2s_set_pin(I2S_NUM_1, &i2s_mic_pin_config) != ESP_OK) {
    Serial.println(F("Failed to set i2s mic pins"));    
  }
  
  // Запускаем Задачу по звуку
  xTaskCreate(&audio_task, "audio_task", 32000, NULL, 5, &audio_task_); 

  // Точно также запускаем Задачу по ЛоРа, но с меньшей глубиной стэка
  xTaskCreate(&lora_task, "lora_task", 16000, NULL, 5, &lora_task_);

//проверяем что бибилиотека радио не вернула ошибкок, если вернула, пишем на экране
  if (state != RADIOLIB_ERR_NONE) {
    u8g2->clearBuffer();        
    u8g2->drawStr(0, 12, "Initializing: FAIL!");
    u8g2->sendBuffer();
   }

//сообщение в серийный порт, что есть ошибки в инициализации экрана
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("Display success!"));
    } else {
        Serial.print(F("Display failed, code "));
        Serial.println(state);
        while (true);
    }

Serial.println(F("Board setup completed"));   

} //конец установок

void lora_task(void *param) {
  
Serial.println(F("  Lora task started"));   

  // Ждем уведомления от подпрограммы управления прерываниями, читаем данные с ЛоРа и передаем в обработку звука
  while (true) { 
    uint32_t lora_status_bits = 0; //декларируем и сбрасываем 32биное значение переменной в ноль и дальше смотрим, что выведет таск, проверяющий наличие бит-статуса
    xTaskNotifyWaitIndexed(0, 0x00, ULONG_MAX, &lora_status_bits, portMAX_DELAY); // ждем нулевую нотификацию, не очищаем любой бит-статус на входе, сбрасывем бит-статус в ноль на выходе, передаем значение нотификации в переменную, блокируем на неопределнный срок
    Serial.print(F("Lora task bits - "));
    Serial.println(lora_status_bits);
    if (lora_status_bits & LORA_RADIO_TASK_TX_BIT) { //если бит-статус из обработчика прерываний совпадает с бит-статусом передачи = 0x02
      while (lora_radio_tx_queue_index_.size() > 0) { //пока значения бит индекса в круговом буфере больше нуля, то есть у нас есть полный пакет 48 байт с кодека для передачи
          // берем размер пакета и читаем его
          int tx_bytes_cnt = lora_radio_tx_queue_index_.shift(); // забираем из очереди кругового буфера значение размера пакета (в настрйках указали 48 байт)
          for (int i = 0; i < tx_bytes_cnt; i++) {  // выталкиваем накопившиеся байты с кодека по кадому байту все 48 байт в буфер передачи и передаем весь массив за раз
              lora_radio_tx_buf_[i] = lora_radio_tx_queue_.shift();
          }
          // и передаем пакет в ЛоРа
          int lora_radio_state = lora_radio_.transmit(lora_radio_tx_buf_, tx_bytes_cnt); // передаем в трансивер значения каждого байта покета, который нужно отправить (48 байт)
          if (lora_radio_state != RADIOLIB_ERR_NONE) {
             Serial.println(F("Lora radio transmit failed"));// если есть ошибка, возвращаем ее тип в лог
          }
             Serial.print(F("Transmitted packet-"));
             Serial.println(tx_bytes_cnt);
          //Рисуем на экране пареметры скорости передачи пакета
          char buf[256];
            u8g2->clearBuffer();
            u8g2->drawStr(0, 12, "Transmitting: OK!");
            snprintf(buf, sizeof(buf), "Rate:%.2f bps", lora_radio_.getDataRate());
            u8g2->drawStr(5, 30, buf);
            u8g2->sendBuffer();
       vTaskDelay(1); //держим задачу 1 милисекунду, чтобы ЛоРа успела его обработать и передать, потом берем следующий пакет
     } // цикл передачи пакета завершен
    } // цикл опредления бита передачи завершен
    else {
    Serial.println(F("Lora task bit not 0x02 - cant transmit"));   
    }
  } // бесконечный цикл задачи обработки прерываения
} // цикл задачи ЛоРа завершен

void audio_task(void *param) {
  
Serial.println(F("Audio task started"));

  // Создание codec2
  c2_ = codec2_create(AUDIO_CODEC2_MODE); // инициализируем кодек2 с режимом скорсти потока, называем покороче - с2_
  if (c2_ == NULL) {
     Serial.println(F("Failed to create Codec2")); // если кодек2 не стартанул, пишем в лог ошибку и не идем дальше
    return;
  } else {  // иначе, указываем все нужные параметры кодека - все стандартно, в принципе, как в примерах
    c2_samples_per_frame_ = codec2_samples_per_frame(c2_); // достаем из библиотеки кодека сколько у него сэмплов в одном кадре
    c2_bytes_per_frame_ = codec2_bytes_per_frame(c2_); // достаем из библиотеки кодека сколько у него байт в одном кадре
    c2_samples_ = (int16_t*)malloc(sizeof(int16_t) * c2_samples_per_frame_); // вычисляем сэмплы кодека, выделяем динамичесикую пямять умножаем на количество сэмплов в кадре
    c2_bits_ = (uint8_t*)malloc(sizeof(uint8_t) * c2_bytes_per_frame_); // вычисляем биты кодека, выделяем динамичесикую пямять умножаем на количество байт в кадре
    Serial.println(F("Codec2 constructed"));
    Serial.print(F("Semples per frame-"));
    Serial.println(c2_samples_per_frame_);
    Serial.print(F("Bytes per frame-"));
    Serial.println(c2_bytes_per_frame_);// пишем в лог, что кодек стартовал с этими параметрами
  }
int packet_size = 0; // декларируем переменную размера пакета. Она нужна для определения, что идет передача. В начале сбрасываем 32биное значение переменной в ноль и дальше смотрим, что выведет Задача (таск), проверяющий наличие бит-статуса    

while (true) {
       // отправляем пакет если накопилось достаточное количество кодированных кадров
        if (packet_size + c2_bytes_per_frame_ > AUDIO_MAX_PACKET_SIZE) { // если количество байт на кадр превышает максимальное значение размера пакета (в настройках задали что 48 = 16 раз по 3 байт на кадр) - срабатывает все, что ниже
          lora_radio_tx_queue_index_.push(packet_size); // толкаем в очередь кругового буфера информацию, что у нас есть 48 байт для передачи в очереди tx_queue 
          uint32_t lora_tx_bits = LORA_RADIO_TASK_TX_BIT; // вводим переменную равную биту старта Задачи передачи
          xTaskNotify(lora_task_, lora_tx_bits, eSetBits); // выводим уведомление в задачу передачи ЛоРА - разрешаем передачу пакета
          packet_size = 0; //снова сбрасываем размер пакета в ноль, поскольку отправили один полный пакет.
          Serial.println(F("Packet Done")); // пишем, что пакет записан
        }
        // чтение и кодирование одного кадра кодек2 
        size_t bytes_read; 
        i2s_read(I2S_NUM_1, c2_samples_, sizeof(uint16_t) * c2_samples_per_frame_, &bytes_read, portMAX_DELAY); // читаем данные с I2S микрофона в DMA: номер потока 1, прочитанные с i2S сэмплы передаем в кодек2, количество байт прочитано, блок Задачи пока не будет места в очереди
        codec2_encode(c2_, c2_bits_, c2_samples_); // кодируем кодеком2: передаем настройки, на выходе получаем кодированный битовый поток, принимаем сэмплы с i2S
        for (int i = 0; i < c2_bytes_per_frame_; i++) {
          lora_radio_tx_queue_.push(c2_bits_[i]); // толкаем в кольцевой буфер поочереди все байты кадра, пока не передадим весь кадр. Если кодек сказал, что на кадр 3 байта, значит будет три 8-ми бытных слова в очереди кадра.
        }
        packet_size += c2_bytes_per_frame_; // считаем размер пакета (это разумный размер что пролазит в ЛоРА за один такт). Пакет вычисляем как накопленная сумма байт на кадр с кодека (к предыдущему значению прибавляем 3+3=6+3=9+3=12 и тд)
        vTaskDelay(1); // даем веремя на запись
      } // конец цикла
}
  


void loop()
//Отдыхаем
{
    delay(100);
}


