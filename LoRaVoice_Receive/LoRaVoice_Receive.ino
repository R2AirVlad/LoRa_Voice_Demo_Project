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

Для реализации приемника вам потребуется данная плата i2s усилитель MAX98357A и любой малогабаритный динамик на 8 Ом. 

Для улучшения динамических характеристик приемника рекомендуется подключить на антенный вход модуль гребенчатого полосового фильтра на диапазон 430-440 Мгц.

Все настройки устройства вынесены для удобства в отдельный файл utilities.h - там же содержатся указания по пинам подлючения усилителя.

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

//Создаем 2 колцевых буфера для использования с ЛоРа: задаем тип данных буфера целые числа 0-255, задаем размер буфера.
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_rx_queue_; //декларируем, что этот буфер для очереди приема
CircularBuffer<uint8_t, LORA_RADIO_QUEUE_LEN> lora_radio_rx_queue_index_; //декларируем, что этот буфер для индексов приема

// Декларируем с типом байт (целые числа 0-255) переменные для буферов приема
byte lora_radio_rx_buf_[LORA_RADIO_BUF_LEN];  // буфер приема


TaskHandle_t lora_task_;    // Создание Задачи FREE RTOS для ЛоРа, который обеспечивает прием. 

volatile bool lora_enable_isr_ = true;  // Создаем логическую переменную, которая может изменяться программой для обработки прерывания. Служебная подпрограмма прерывания (ISR) меняет значение.

SX1278 lora_radio_ = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

TaskHandle_t audio_task_;       /// таск записи/воспроизведения звука

// Настройка Codec2
struct CODEC2* c2_;             // Создание структуры codec2, в которой есть разнотипные переменные. Создаем указатель( * )на CODEC2 называем его с2_
int c2_samples_per_frame_;      // Декларируем 32 битную (4 байта) переменную сколько сырых сэмплов в одном кадре
int c2_bytes_per_frame_;        // Декларируем 32 битную (4 байта) переменную сколько сырых байт в одном кадре (эта переменная отсутсвовала в первоначальном коде)
int16_t *c2_samples_;           // Декларируем 16 битную (2 байта) переменную размера буфера для сырых сэмплов
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
  lora_radio_.setDio0Action(onLoraDataAvailableIsr); //Атрибут ЛоРа модуля (пин DIO0), управлет программой обработки прерываний (ISR) - модуль ЛоРа отвечает, что пакет принят    
  lora_radio_.setFHSSHoppingPeriod(LORA_FHSS_HOP_PER);


 // Настройка выхода драйвера I2S для воспроизведения звука на внешнем усилителе с I2S интерфейсом, стандартная настройка. https://docs.espressif.com/projects/esp-idf/en/v3.3/api-reference/peripherals/i2s.html
  i2s_config_t i2s_speaker_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB), // отправляем метку MSB чтобы устройства знали сколько бит в слове
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Флаг используется для определния прерывания 
    .dma_buf_count = 8, // Количество буферовпрямого доступа к памяти
    .dma_buf_len = 1024, // Длина буфера прямого доступа к памят
    .use_apll=0, // если включить, получаем более точный контроль отсчетов, для голоса не особо нужно
    .tx_desc_auto_clear= true, // Очищает дискриптор передачи, помогает избежать шума, если нет полезного сигнала
    .fixed_mclk=-1    // нужно всегда ставить так, если битность сигнала меньше 64
  };
  //Тут указываем выходы пинов на усилитель
  i2s_pin_config_t i2s_speaker_pin_config = {
    .bck_io_num = AUDIO_SPEAKER_BCLK,
    .ws_io_num = AUDIO_SPEAKER_LRC,
    .data_out_num = AUDIO_SPEAKER_DIN,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
//Тут и ниже, стандартная обратока запуска I2S, если есть ошибки - пишем в лог  
  if (i2s_driver_install(I2S_NUM_0, &i2s_speaker_config, 0, NULL) != ESP_OK) {
    Serial.println(F("Failed to install i2s speaker driver"));
  }
  if (i2s_set_pin(I2S_NUM_0, &i2s_speaker_pin_config) != ESP_OK) {
    Serial.println(F("Failed to set i2s speaker pins"));    
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
  

  // Запуск приема
    Serial.print(F("[SX1278] Starting to receive - "));
    state = lora_radio_.startReceive(); //важно в первый раз вызвать эту функцию, чтобы модем перешел в режим приема
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("Receive starded!"));
        u8g2->clearBuffer();        
        u8g2->drawStr(0, 16, "Receive starded!");
        u8g2->sendBuffer();
        delay (2000);
        u8g2->clearBuffer();        
        u8g2->drawStr(0, 16, "Waiting packet!");
        u8g2->sendBuffer();
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

Serial.println(F("Board setup completed"));   

} //конец установок

// Вызываем обработчик прерываний ISR когда новый пакет принят модулем ЛоРа
ICACHE_RAM_ATTR void onLoraDataAvailableIsr() { //объявляем стандартную функцию аппаратного прерывания ESP32, запускаем прерывание при первом успешном декодировании пакета модулем ЛоРа
  if (!lora_enable_isr_) return; // если этот атрибут не равен true (равен false) - возвращаем true (логически оборотная функция)
  BaseType_t xHigherPriorityTaskWoken; // Обявляем переменную логического Выхода прерывания RTOS
  uint32_t lora_rx_bit = LORA_RADIO_TASK_RX_BIT; // задаем переменную бита приема равной значению бита приема из задаваемых параемтров (0x01)
  // Уведомление от обработчика перываний, что пришел новый пакет данных от ЛоРа. 
  // Задача разблокируется, когда указанный флаг-бит станет активный, и поэтому задача должна сама проверить нужную комбинацию бит.
  //(Ссылка на хендл Задачи по ЛОРА от которой придет уведомление, изменяемое RTOS значение сигнал-бита, атрибут говорит что ждем битовый формат, логическое изменяемое значение - выход нотификации)
  xTaskNotifyFromISR(lora_task_, lora_rx_bit, eSetBits, &xHigherPriorityTaskWoken);
  Serial.print(F("Lora task ISR bits - "));
  Serial.println(lora_rx_bit);
}


void lora_task(void *param)  {
  
Serial.println(F("  Lora task started"));
  
  while (true) {
    uint32_t lora_status_bits = 0; //декларируем и сбрасываем 32биное значение переменной в ноль и дальше смотрим, что выведет таск, проверяющий наличие бит-статуса приема
    xTaskNotifyWaitIndexed(0, 0x00, ULONG_MAX, &lora_status_bits, portMAX_DELAY); // ждем нулевую нотификацию, не очищаем любой бит-статус на входе, сбрасывем бит-статус в ноль на выходе, передаем значение нотификации в переменную, блокируем на неопределнный срок
    //Serial.print(F("Lora task bits - ")); //Используем только для отладки, выключаем, чтобы не засорять память при обычной работе
    //Serial.println(lora_status_bits); //Используем только для отладки, выключаем, чтобы не засорять память при обычной работе
    
    // ЛоРа прием данных
  if (lora_status_bits & LORA_RADIO_TASK_RX_BIT) { //если бит-статус из обработчика прерываний совпадает с бит-статусом приема 0х01
    int packet_size = lora_radio_.getPacketLength(); // делаем переменную размера пакета равной данным полученным из радиолиб ЛоРа 
    if (packet_size > 0) { // если размер пакета больше нуля
      int state = lora_radio_.readData(lora_radio_rx_buf_, packet_size); // устанавливаем тип данных 32 бит, перводим ЛоРа в состояние чтения данных и передаем в переменную буфера приема хэндла Задачи. Принимаем только пакеты целиком.
      if (state == RADIOLIB_ERR_NONE) { // если не возвращается ошибки идем дальше
        // обрабатывам целый пакет
        if (packet_size % c2_bytes_per_frame_ == 0) { // если значение пакета кратно длине кадра кодек2, то есть проверена полнота пакета (48 байт)
          for (int i = 0; i < packet_size; i++) {
              lora_radio_rx_queue_.push(lora_radio_rx_buf_[i]); //толкаем байты пакета последовательно в круговой буфер приема, толкаем до тех пор пока не дотолкаем весь пакет
          }
          lora_radio_rx_queue_index_.push(packet_size); // толкаем в круговой буфер приема индекса размер принятого пакета (48 байт)
          uint32_t audio_play_bit = AUDIO_TASK_PLAY_BIT; // декларируем 32 биную переменную и приравнивем ее старт-биту начала декодирования кодеком 0х01
          xTaskNotify(audio_task_, audio_play_bit, eSetBits); // отправляем уведомление в задачу аудио таска в битовом формате, что мы приняли успешно полный пакет и кодек может начать его декодировать
         // Serial.print(F("Received packet, size - ")); //пишем, что принят пакет и его размер. Используем только для отладки, выключаем, чтобы не засорять память при обычной работе
         // Serial.println(packet_size); // //Используем только для отладки, выключаем, чтобы не засорять память при обычной работе
          //Рисуем на экране пареметры приема пакета          
          char buf[256];
          u8g2->clearBuffer();
          u8g2->drawStr(0, 12, "Received 48 bytes");
          snprintf(buf, sizeof(buf), "SNR:%.2f", lora_radio_.getSNR());
          u8g2->drawStr(0, 40, buf);
          snprintf(buf, sizeof(buf), "RSSI:%.2f", lora_radio_.getRSSI());
          u8g2->drawStr(0, 54, buf);        
          u8g2->sendBuffer();     
 
        } else {
          Serial.print(F("Audio packet of wrong size, expected multiple-"));// иначе пишем ошибку в лог и указываем значение байт на кадр для кодека и снова начинаем слушать модем
          Serial.println(c2_bytes_per_frame_);
        }
       } else {       
          Serial.print(F("Read data error: "));// ошибка чтения потока даннных, указание типа с радиолиба и снова начинаем слушать модем
          Serial.println(state);
       }
        //Переводим ЛоРа снова в состояние приема. 
        state = lora_radio_.startReceive(); 
        Serial.println(F("Receive started"));        
    } // размер пакета > 0
    else {
          Serial.println(F("Waiting packet"));// ждем пакета
          vTaskDelay(100);
         }    
  }// конец ЛоРа прием

 } // бесконечный цикл задачи обработки прерываения

}// цикл задачи ЛоРа завершен

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

size_t bytes_written; // декларируем  переменные с типом размера в байтах

while (true) {

  uint32_t audio_bits = 0; // декларируем и сбрасываем 32биное значение переменной в ноль и дальше смотрим, что выведет таск, проверяющий наличие бит-статуса
  xTaskNotifyWaitIndexed(0, 0x00, ULONG_MAX, &audio_bits, portMAX_DELAY); // ждем нулевую нотификацию, не очищаем любой бит-статус на входе, сбрасывем бит-статус в ноль на выходе, передаем значение нотификации в переменную, блокируем на неопределнный срок
 // Serial.print(F("Audio task bits - ")); //Используем только для отладки, выключаем, чтобы не засорять память при обычной работе
 // Serial.println(audio_bits); //Используем только для отладки, выключаем, чтобы не засорять память при обычной работе

 // прием - декод - воспроизведение звука
  if (audio_bits & AUDIO_TASK_PLAY_BIT) {  // если из Задачи возвращается значение статус-бита воспроизведения
  Serial.println(F("Playing audio"));  
      // пока пакеты приема идут
      while (lora_radio_rx_queue_index_.size() > 0) { // пока в очереди кругового буфера есть пакеты с длинной больше нуля
        int packet_size = lora_radio_rx_queue_index_.shift(); // передаем из очереди обработки пакетов приема в переменную размера пакета значения
       // Serial.print(F("Playing packet, size - ")); // пишем, что играем пакет и его размер. Используем только для отладки, выключаем, чтобы не засорять память при обычной работе
       // Serial.println(packet_size); //Используем только для отладки, выключаем, чтобы не засорять память при обычной работе
        // делим на кадры, декодируем и воспроизводим
        for (int i = 0; i < packet_size; i++) {
          c2_bits_[i % c2_bytes_per_frame_] = lora_radio_rx_queue_.shift(); // достаем каждый следующий очередной бит из очереди кругового буфера звука толкаем его в кодек и затем воспроизводим
          if (i % c2_bytes_per_frame_ == c2_bytes_per_frame_ - 1) { // если текущее итое значение байта кадра равно предыдущему, значит кадры закончились и можно декодить звук
            codec2_decode(c2_, c2_samples_, c2_bits_); // декодим: кодеком с настроенными параметрами, выход декодированных сэмплов кодека, входной битовый поток с очереди буфера
            i2s_write(I2S_NUM_0, c2_samples_, sizeof(uint16_t) * c2_samples_per_frame_, &bytes_written, portMAX_DELAY); // передаем в I2S: номер потока 0, записанные сэмплы, из величины размеченной динамической памяти, количество байт записано, блокируем на неопределнный срок
            vTaskDelay(1); // придерживаем Задачу, пока I2S воспроизведет пакет
          }
        }
      } // пока есть байты приема с ЛоРа 
    } else { // декод и вопроизведение звук
          Serial.println(F("Nothing to play")); // пакета от модема пока нет и нет уведомления от задачи ЛоРа - воспроизводить нечего
          delay(100);
       } 

} // бесконечный цикл воспроизведения пакета

} // цикл задачи Аудио завершен


void loop()
//Отдыхаем
{
delay(100);
}


