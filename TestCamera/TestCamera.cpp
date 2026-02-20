#include <opencv2/opencv.hpp> // подключение библиотеки opencv
#include <vector>
#include <winsock2.h> // основные сетевые типы и функции (сокеты)
#include <ws2tcpip.h> // удобные функции для IP-адресов и портов
#pragma comment(lib, "ws2_32.lib") // библиотека линковки WinSock (чтобы всё собралось).

// Глобальные переменные сокета и адреса получателя
SOCKET senderSocket = INVALID_SOCKET;  // сокет для отправки
sockaddr_in receiverAddr{};            // структура с IP и портом получателя

struct MinMax // структура для переменных
{
    int min;
    int max;
};

int Smin = 148, Vmin = 103;
MinMax H = { 96, 117 };
MinMax A = { 110, 120 };
MinMax B = { 95, 120 };

struct Packet
{
    uint16_t x;  // координата X
    uint16_t y;  // координата Y
};

void initObjectControls() // функция для создания трекбаров
{
    cv::namedWindow("Controls_Object", cv::WINDOW_NORMAL); // создаем окно
    cv::resizeWindow("Controls_Object", 350, 260); // устанавливаем размер окна

    cv::createTrackbar("H min", "Controls_Object", &H.min, 179); // создаем трекбары
    cv::createTrackbar("H max", "Controls_Object", &H.max, 179);
    cv::createTrackbar("S min", "Controls_Object", &Smin, 255);
    cv::createTrackbar("V min", "Controls_Object", &Vmin, 255);

    cv::createTrackbar("A min", "Controls_Object", &A.min, 255);
    cv::createTrackbar("A max", "Controls_Object", &A.max, 255);
    cv::createTrackbar("B min", "Controls_Object", &B.min, 255);
    cv::createTrackbar("B max", "Controls_Object", &B.max, 255);
}

bool initUDP() // инициализация UDP-соединения
{
    WSADATA wsa; // структура для запуска WinSock
    // запускаем WinSock версии 2.2
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
        return false; // если ошибка — выходим

    // создаём UDP-сокет (IPv4, датаграммы, протокол UDP)
    senderSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    /*
    * 1 параметр - тип адреса:
    * AF_INET = IPv4.
    * AF_INET6 → IPv6
    *
    * 2 параметр - тип сокета:
    * SOCK_DGRAM = датаграммы → это UDP.
    * SOCK_STREAM → TCP (соединение, подтверждения, контроль доставки)
    *
    * 3 параметр - Какой именно протокол использовать.
    * IPPROTO_UDP → UDP
    * IPPROTO_TCP → TCP
    */

    // проверяем, создан ли сокет
    if (senderSocket == INVALID_SOCKET)
        return false;

    receiverAddr.sin_family = AF_INET;      // используем IPv4
    receiverAddr.sin_port = htons(239);     // порт получателя (перевод в сетевой формат)

    // преобразуем строковый IP в бинарный формат
    inet_pton(AF_INET, "192.168.4.1", &receiverAddr.sin_addr);

    return true; // всё успешно
}

void shutdownUDP() // корректное завершение работы UDP
{
    // если сокет открыт — закрываем его
    if (senderSocket != INVALID_SOCKET)
        closesocket(senderSocket);

    senderSocket = INVALID_SOCKET; // помечаем как закрытый

    WSACleanup(); // освобождаем ресурсы WinSock
}

void sendData(const void* data, size_t size)
{
    sendto(
        senderSocket,                              // сокет
        reinterpret_cast<const char*>(data),       // указатель на данные
        static_cast<int>(size),                    // размер данных
        0,                                         // флаги
        reinterpret_cast<sockaddr*>(&receiverAddr),// адрес получателя
        sizeof(receiverAddr)                       // размер структуры адреса
    );
}



int main()
{
    cv::Mat frameBGR, frameHSV, frameLAB, frameLabHSV; // объект класса Mat для хранения текущего кадра изображения
    cv::VideoCapture video(0, cv::CAP_DSHOW); // создаем объект класса VideoCapture: 0 — индекс камеры, CAP_DSHOW — backend DirectShow
    if (!video.isOpened()) return -1; // если камера не открылась — завершаем программу с кодом ошибки -1
    if (!initUDP()) return -2; // инициализируем UDP
    initObjectControls();

    while (true) // бесконечно
    {
        video >> frameBGR; // получаем кадр из видеопотока
        if (frameBGR.empty()) break; // если кадр пустой — выходим из цикла

        cv::cvtColor(frameBGR, frameHSV, cv::COLOR_BGR2HSV); // преобразуем полученный кадр в HSV
        cv::GaussianBlur(frameHSV,
            frameHSV,
            cv::Size(15, 15),
            0); // размываем кадр и переписываем в ту же переменную
        cv::inRange(frameHSV,
            cv::Scalar(H.min, Smin, Vmin),
            cv::Scalar(H.max, 255, 255),
            frameHSV); // ищем только вхождения нужного цвета (преобразуем в бинарную маску) и переписываем в ту же переменную

        cv::cvtColor(frameBGR, frameLAB, cv::COLOR_BGR2Lab);// преобразуем полученный кадр в Lab
        std::vector<cv::Mat> vectorFrameLab; // создаем массив типа Mat (вектор - динамический массив). Нужны библиотеки vector и iostream
        cv::Mat maskA, maskB; // создаем маски для осей А и В
        cv::split(frameLAB, vectorFrameLab); // разделяем кадр в Lab на потоки
        cv::inRange(vectorFrameLab[1], A.min, A.max, maskA); // проверяем вхождения в поток А
        cv::inRange(vectorFrameLab[2], B.min, B.max, maskB); // проверяем вхождения в поток И
        frameLAB = maskA & maskB; // объединяем результат в единую маску и записываем его

        frameLabHSV = frameHSV | frameLAB; // объединяем маски HSV и Lab в единый кадр

        std::vector<std::vector<cv::Point>> contour; // создаем вектор векторов координат для контуров объекта. Вектор векторов - это двухмерный массив.
        cv::findContours( //Ищем контуры объекта
            frameLabHSV, // Откуда читаем
            contour, // куда выводим
            cv::RETR_EXTERNAL, // режим извлечения контуров. Берёт только внешние контуры. Варианты: RETR_LIST, RETR_TREE, RETR_CCOMP
            cv::CHAIN_APPROX_SIMPLE); // метод хранения точек контура. Оставляет только углы и ключевые точки. Вариант: CHAIN_APPROX_NONE (хранит каждый пиксель)

        double maxArea = 0; // переменная для хранения максимального контура
        int maxIndex = -1; // номер максимального контура

        for (int n = 0; n < contour.size(); n++) // пока не перебрали все контуры
        {
            double area = cv::contourArea(contour[n]); // пишем текущий контур
            if (area > maxArea) // если он больше максимального
            {
                maxArea = area; // обновляем максимальный
                maxIndex = n; // запоминаем индекс
            }
        }

        if (maxIndex != -1 && maxArea > 200) // если объект обнаружен и больше 200, то ищем его центр
        {
            cv::Point2f objectCenter; // создаем переменную для записи координат
            Packet pack; // создаем пакет для формирования данных для отправки
            float objectRadius = 0; // создаем переменную для радиуса круга
            cv::minEnclosingCircle(contour[maxIndex], objectCenter, objectRadius); // рассчитываем минимальную описанную окружность для найденного контура объекта
            cv::circle(frameBGR, objectCenter, (int)objectRadius, cv::Scalar(255, 0, 0), 2); // рисуем саму окружность на изначальном изображении
            cv::circle(frameBGR, objectCenter, 3, cv::Scalar(0, 255, 0), -1); // рисуем центральную точку на изначальном изображении
            cv::putText(frameBGR, // добавляем надпись. что это робот
                "Robot",
                objectCenter + cv::Point2f(-objectRadius, objectRadius),
                cv::FONT_HERSHEY_COMPLEX,
                0.6,
                cv::Scalar(0, 0, 255));
            int xi = (int)std::lround(objectCenter.x); // округляем и записываем координату центра по Х 
            int yi = (int)std::lround(objectCenter.y); // округляем и записываем координату центра по У 

            xi = std::clamp(xi, 0, 65535); // ограничиваем диапазон числа, чтобы он не мог быть больше 2-х байтов
            yi = std::clamp(yi, 0, 65535);

            pack.x = htons((uint16_t)xi); // упаковываем число. преобразовав его в 2 байта
            pack.y = htons((uint16_t)yi); // упаковываем число. преобразовав его в 2 байта

            sendData(&pack, sizeof(pack)); // отправляем число по UDP при помощи созданной функции
        }

        imshow("OriginalVideo", frameBGR); // отображаем кадр в окне с именем OriginalVideo
        imshow("HSVVideo", frameHSV); // отображаем кадр в окне с именем HSVVideo
        imshow("LABVideo", frameLAB); // отображаем кадр в окне с именем LabVideo
        imshow("LabHSVVideo", frameLabHSV); // отображаем кадр в окне с именем LabHSVVideo
        if (cv::waitKey(1) == 27) break; // ожидание 1 мс и проверка нажатия клавиши ESC
    }
    shutdownUDP(); // отключаем UDP
}