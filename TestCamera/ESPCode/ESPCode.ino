#include <WiFi.h>      // Библиотека для работы с Wi-Fi на ESP32
#include <WiFiUdp.h>   // Библиотека для работы с UDP-протоколом поверх Wi-Fi

// Имя Wi-Fi сети, которую создаёт ESP32 (режим точки доступа)
const char* ssid = "ESP32_AP";

// Пароль для подключения к этой сети (минимум 8 символов)
const char* password = "12345678";

// Создаём объект UDP для приёма и отправки UDP-пакетов
WiFiUDP udp;

// Порт, который будет слушать ESP32
// Должен совпадать с портом, на который отправляет ПК
const unsigned int localPort = 239;

// Буфер для хранения входящих данных
// Мы ожидаем 4 байта: 2 байта для X и 2 байта для Y
uint8_t buffer[4];

void setup()
{
    Serial.begin(115200);
    WiFi.softAP(ssid, password); // Создаем сеть WiFi
    Serial.print("IP ESP32: "); 
    Serial.println(WiFi.softAPIP()); // Выводим IP-адрес ESP32 в Serial Monitor
    udp.begin(localPort); // Открываем UDP-порт и начинаем его слушать
}

void loop()
{
    int packetSize = udp.parsePacket();// Проверяем, пришёл ли UDP-пакет, если да, пишем его размер
    if (packetSize == 4) // Проверяем, что размер пакета ровно 4 байта
    {
        udp.read(buffer, 4); // Читаем 4 байта из входящего пакета в массив buffer
        /*
        Восстанавливаем координату X из двух байтов
        buffer[0] — старший байт
        buffer[1] — младший байт
        Сдвиг << 8 возвращает старший байт на своё место
        */
        uint16_t x = (buffer[0] << 8) | buffer[1];

        // Аналогично собираем координату Y
        uint16_t y = (buffer[2] << 8) | buffer[3];
        // Выводим полученные координаты в Serial Monitor
        Serial.print("X: ");
        Serial.print(x);
        Serial.print("  Y: ");
        Serial.println(y);
    }
    // Если пакет другого размера или его нет — просто продолжаем цикл
}
