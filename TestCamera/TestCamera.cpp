/*
#include <opencv2/opencv.hpp>   // Подключение основной библиотеки OpenCV
#include <iostream>             // Подключение библиотеки для вывода в консоль

// Глобальные переменные для хранения изображений
cv::Mat frame;      // Кадр с камеры в формате BGR
cv::Mat frameLab;   // Тот же кадр, но преобразованный в цветовое пространство Lab
cv::Mat frameHSV;   // Тот же кадр, но преобразованный в цветовое пространство HSV

// Функция-обработчик событий мыши
// Она автоматически вызывается OpenCV при клике в окне
void onMouse(int event, int x, int y, int, void*)
{
    if (event == cv::EVENT_LBUTTONDOWN)// Проверяем, нажата ли левая кнопка мыши
    {
        // Получаем значение пикселя в координатах (x, y)
        // Vec3b — это вектор из трёх байтов
        cv::Vec3b pixelLAB = frameLab.at<cv::Vec3b>(y, x);
        cv::Vec3b pixelHSV = frameHSV.at<cv::Vec3b>(y, x);

        // Извлекаем значения каналов
        int L = pixelLAB[0];   // Канал яркости (Lightness)
        int A = pixelLAB[1];   // Канал зелёный ↔ красный
        int B = pixelLAB[2];   // Канал синий ↔ жёлтый

        int H = pixelHSV[0];   // Канал Hue
        int S = pixelHSV[1];   // Канал Saturation
        int V = pixelHSV[2];   // Канал Value

        // Выводим значения каналов в консоль
        std::cout << "L: " << L
            << "  A: " << A
            << "  B: " << B
            << std::endl;
        std::cout << "H: " << H
            << "  S: " << S
            << "  V: " << V
            << std::endl;
    }
}

int main()
{

    cv::VideoCapture cap(0); // Создаём объект для захвата видео с камеры
    if (!cap.isOpened())// Проверяем, удалось ли открыть камеру
        return -1;  // Если нет — завершаем программу
    cv::namedWindow("LabHSV");// Создаём окно с именем "LabHSV"
    cv::setMouseCallback("LabHSV", onMouse);// Назначаем функцию обработки клика мыши для окна "LabHSV"
    while (true)// Бесконечный цикл захвата видео
    {
        cap >> frame;// Захватываем новый кадр с камеры
        cv::cvtColor(frame, frameLab, cv::COLOR_BGR2Lab);// Преобразуем BGR в Lab
        cv::cvtColor(frame, frameHSV, cv::COLOR_BGR2HSV);// Преобразуем BGR в HSV
        cv::imshow("LabHSV", frame);// Отображаем исходное изображение (не Lab!)
        if (cv::waitKey(1) == 27)
            break;                // Если нажали ESC — выходим из цикла
    }
    return 0;  // Завершение программы
}

*/

#include <opencv2/opencv.hpp> // подключение библиотеки opencv
#include <vector>

struct MinMax
{
    int min;
    int max;
};

int main()
{
    cv::Mat frameBGR, frameHSV, frameLAB, frameLabHSV; // объект класса Mat для хранения текущего кадра изображения
    cv::VideoCapture video(0, cv::CAP_DSHOW); // создаем объект класса VideoCapture: 0 — индекс камеры, CAP_DSHOW — backend DirectShow
    int Smin = 148, Vmin = 103;
    MinMax H = { 96, 117 };
    MinMax A = { 110, 120 };
    MinMax B = { 95, 120 };
    if (!video.isOpened()) return -1; // если камера не открылась — завершаем программу с кодом ошибки -1

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
        cv:: Mat maskA, maskB; // создаем маски для осей А и В
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

        if (maxIndex != -1 && maxArea > 200)
        {
            cv::Point2f objectCenter;
            float objectRadius = 0;
            cv::minEnclosingCircle(contour[maxIndex], objectCenter, objectRadius);
            cv::circle(frameBGR, objectCenter, (int)objectRadius, cv::Scalar(255, 0, 0), 2);
            cv::circle(frameBGR, objectCenter, 3, cv::Scalar(0, 255, 0), -1);
            cv::putText(frameBGR, 
                "Robot", 
                objectCenter + cv::Point2f(-objectRadius, objectRadius),
                cv::FONT_HERSHEY_COMPLEX,
                0.6,
                cv::Scalar(0, 0, 255));
        }

        imshow("OriginalVideo", frameBGR); // отображаем кадр в окне с именем OriginalVideo
        imshow("HSVVideo", frameHSV); // отображаем кадр в окне с именем HSVVideo
        imshow("LABVideo", frameLAB); // отображаем кадр в окне с именем LabVideo
        imshow("LabHSVVideo", frameLabHSV); // отображаем кадр в окне с именем LabHSVVideo
        if (cv::waitKey(1) == 27) break; // ожидание 1 мс и проверка нажатия клавиши ESC
    }
}