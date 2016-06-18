#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    cv::Mat image = cv::imread("/home/Imagens/imagem1.jpg", CV_LOAD_IMAGE_ANYCOLOR);
    if(!image.data)
    {
      std::cout << "ERRO" << std::endl;
    }
    cv::namedWindow("window", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", image);
    cv::waitKey(0);
    /*
    cv::Mat scene = cv::imread("/home/Code/Trabalhos/reconhecimento_de_objetos/C++/images/ambiente.jpg", CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat object = cv::imread("home/Code/Trabalhos/reconhecimento_de_objetos/C++/images/objeto.jpg", CV_LOAD_IMAGE_GRAYSCALE);
   // analise se realment as imagens foram carregadas
    if(!scene.data || !object.data)
    {
        std::cout <<"ERRO::erro ao carregar as imagens, nenhuma imagem no Mat" << std::endl;

    }
    //Passo 1 criar ORB e
    cv::Ptr<cv::Feature2D> orb; //Cria uma classe para características 2d
    orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20); //cria ORB mais em http://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html
    std::vector<cv::KeyPoint> kp_scene, kp_object; //cria vetores para armazenars os pontos de interesse
    //Passo 2 detectar pontos chave e computá-los
    cv::Mat dc_scene, dc_object;   //cria vetor de descritores para cena e objeto
    orb->detectAndCompute(scene, cv::Mat(), kp_scene, dc_scene, false); //detecta e computa pontos de interesse e descritores
    orb->detectAndCompute(object, cv::Mat(), kp_object, dc_object, false); //detecta e computa pontos de interesse e descritores
    //Passo 3 realizar o match
    std::vector<cv::DMatch> matches;
    cv::BFMatcher bfmMatcher(cv::NORM_HAMMING);
    bfmMatcher.match(dc_scene, dc_object, matches);
    // Desenhar os matches
    std::cout << "Quantidade de matches:" << matches.size() << std::endl;
    std::cout << "Distancia: "<< matches.back().distance << std::endl;
    if (matches.back().distance > 50)
    {
        std::cout << "Imagem parecida, objeto achado" << std::endl;
    }
    cv::Mat imMatches;
    cv::drawMatches(scene,kp_scene,object,kp_object,matches,imMatches);
    // mostrar as imagens
    cv::namedWindow("match", CV_WINDOW_AUTOSIZE);
    cv::imshow("match", imMatches);
    cv::waitKey(0);
*/
   std::cout << "Hello World " << std::endl;

}

MainWindow::~MainWindow()
{
    delete ui;
}
