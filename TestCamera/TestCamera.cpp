#include <opencv2/opencv.hpp> // подключение библиотеки opencv
#include <vector>
#include <iostream>

struct MinMax
{
    int min;
    int max;
};

int main()
{
    cv::Mat frame, frameHSV, frameLAB, frameLabHSV; // объект класса Mat для хранения текущего кадра изображения
    cv::VideoCapture video(0, cv::CAP_DSHOW); // создаем объект класса VideoCapture: 0 — индекс камеры, CAP_DSHOW — backend DirectShow
    int Smin = 148, Vmin = 103;
    MinMax H = { 96, 117 };
    MinMax A = { 111, 173 };
    MinMax B = { 81, 116 };
    if (!video.isOpened()) return -1; // если камера не открылась — завершаем программу с кодом ошибки -1

    while (true) // бесконечно
    {
        video >> frame; // получаем кадр из видеопотока
        if (frame.empty()) break; // если кадр пустой — выходим из цикла

        cv::cvtColor(frame, frameHSV, cv::COLOR_BGR2HSV);
        cv::GaussianBlur(frameHSV,
                         frameHSV, 
                         cv::Size(15, 15), 
                         0);
        cv::inRange(frameHSV,
                    cv::Scalar(H.min, Smin, Vmin), 
                    cv::Scalar(H.max, 255, 255), 
                    frameHSV);

        cv::cvtColor(frame, frameLAB, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> vectorFrameLab;
        cv:: Mat maskA, maskB;
        cv::split(frameLAB, vectorFrameLab);
        cv::inRange(vectorFrameLab[1], A.min, A.max, maskA);
        cv::inRange(vectorFrameLab[2], B.min, B.max, maskB);
        frameLAB = maskA & maskB;

        frameLabHSV = frameHSV | frameLAB;

        imshow("OriginalVideo", frame); // отображаем кадр в окне с именем OriginalVideo
        imshow("HSVVideo", frameHSV); // отображаем кадр в окне с именем HSVVideo
        imshow("LABVideo", frameLAB); // отображаем кадр в окне с именем LabVideo
        imshow("LabHSVVideo", frameLabHSV); // отображаем кадр в окне с именем LabHSVVideo
        if (cv::waitKey(1) == 27) break; // ожидание 1 мс и проверка нажатия клавиши ESC
    }
}