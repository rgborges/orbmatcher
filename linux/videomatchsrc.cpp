/* FACULDADE SENAI MARIANO FERRAZ
 * Núcleo de automação industrial
 * Projeto de iniciação científica : reconhecimento de objetos em sistemas embarcados
 *
 * BATISTA, rafael
 * rafael430g@gmail.com
 *
 *
 *
 * Este programa tem como objetivo identifica e localizar um objeto em um determinado
 * ambiente utilizadno a bilioteca open cv.
 *
 * uso:./videoMatcher <objeto.jpg>
 * sendo objeto.jpg o diretório da imagem do objeto a ser procurado
 *
 *
 *
 * */






#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

//parametros do orb

int nfeatures = 500;
float scaleFactor = 1.2f;
int nlevels = 8;
int edgeThreshold = 31;
int firstLevel = 0;
int WTA_K = 2;
int scoreType = ORB::HARRIS_SCORE;
int patchSize = 31;
int fastThreshold = 20;


int main(int argc, char *argv[])
{
    VideoCapture cap(0); // abre câmera padrão
    Mat objeto = imread(argv[1], CV_LOAD_IMAGE_COLOR);// le imagem do primeiro argumento
    if(!cap.isOpened())
    {
        cout << "ERRO: não foi possivel ler a camera" << endl;
        return -1;
    }
    if(!objeto.data)
    {
        cout << "ERRO: erro ao ler as imagens" << endl;
        return -1;
    }
    if(argc > 2)
    {
            if(strcmp(argv[2], "-p") == 0)
            {
               cout << "modo parêmetro .. [ok]" << endl;
               cout << "configure os parametros do ORB>>1 2 3 4 5 6 7 8" << endl;
               cout << "sendo: " << endl
                    << "1. numero de features "<< endl
                    << "2. fator escala " << endl
                    << "3. numero de niveis" << endl
                    << "4. filtro de borda"  << endl
                    << "5. primeiro nivel"   << endl
                    << "6. WTA_K"            << endl
                    << "7. tipo de score "   << endl
                    << "8. tamanho do pacote"<< endl
                    << "ORB>>";
            }
            if(strcmp(argv[2], "-nf") == 0)
            {
               nfeatures = atoi(argv[3]); //converte argumento 3 am inteiro
               cout << "algortimo de dectção de features: ORB" << endl;
               cout << "numero de features:" << nfeatures << endl;
            }

            //Passo 1 adiquirir as imagens
    }
    while(true)
    {
            Mat ambiente;             //array para armazenar o ambiente
            cap >> ambiente;          // captura a imagem gravada e transmite para o array Mat

            //Passo 2 Detectar e computar pontos de interesse
            vector<KeyPoint> kp_objeto, kp_ambiente; // cria vetor para armazenar os pontos de interce
            Mat dc_objeto, dc_ambiente;              // array mat oara armazenar os descritores das imagens
            Ptr<Feature2D> fp;                        // cria um ponteiro de pontos 2d
            fp = ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
            fp->detectAndCompute(objeto, Mat(), kp_objeto, dc_objeto, false);
            fp->detectAndCompute(ambiente, Mat(), kp_ambiente, dc_ambiente, false);
            //Passo 3 realizar os matches
            vector<DMatch> matches;
            BFMatcher bfmMatcher(NORM_HAMMING);
            bfmMatcher.match(dc_objeto, dc_ambiente, matches);
            //Passo 4 Achar bons matches
            double max_dist = 0; double min_dist = 100;

             //-- Quick calculation of max and min distances between keypoints
             for( int i = 0; i < dc_objeto.rows; i++ )
             { double dist = matches[i].distance;
               if( dist < min_dist ) min_dist = dist;
               if( dist > max_dist ) max_dist = dist;
             }

            // printf("-- Max dist : %f \n", max_dist );
            // printf("-- Min dist : %f \n", min_dist );

             //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
             std::vector< DMatch > good_matches;

             for( int i = 0; i < dc_objeto.rows; i++ )
             { if( matches[i].distance < 3*min_dist )
                { good_matches.push_back( matches[i]); }
             }

            Mat imMatch;
            cv::drawMatches(objeto, kp_objeto, ambiente, kp_ambiente, good_matches, imMatch);
           /************************************código de exemplo - open cv : http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html */
            //Passo 5 detectar objeto
            vector<Point2f> obj, amb;

            for( int i = 0; i < good_matches.size(); i++ )
            {
              //-- Get the keypoints from the good matches
              obj.push_back( kp_objeto[ good_matches[i].queryIdx ].pt );
              amb.push_back( kp_ambiente[ good_matches[i].trainIdx ].pt );
            }

            Mat H = findHomography(obj, amb, CV_RANSAC);

            //Achar os cantos da imagem 1 ( o objeto a ser detectado)

            vector<Point2f> obj_cantos(4);
            obj_cantos[0] = cvPoint(0,0);
            obj_cantos[1] = cvPoint(objeto.cols, 0);
            obj_cantos[2] = cvPoint( objeto.cols, objeto.rows);
            obj_cantos[3] = cvPoint(0, objeto.rows);

            vector<Point2f> amb_cantos(4);

            perspectiveTransform(obj_cantos, amb_cantos, H);

            //Desenhar linhas entre os cantos (os objetos mapeados no ambiente - imagem 2)


            line( imMatch, amb_cantos[0] +  Point2f( objeto.cols, 0), amb_cantos[1] + Point2f( objeto.cols, 0), Scalar(0, 255, 0), 4);
            line( imMatch, amb_cantos[1] +  Point2f( objeto.cols, 0), amb_cantos[2] + Point2f( objeto.cols, 0), Scalar(0, 255, 0), 4);
            line( imMatch, amb_cantos[2] +  Point2f( objeto.cols, 0), amb_cantos[3] + Point2f( objeto.cols, 0), Scalar(0, 255, 0), 4);
            line( imMatch, amb_cantos[3] +  Point2f( objeto.cols, 0), amb_cantos[0] + Point2f( objeto.cols, 0), Scalar(0, 255, 0), 4);



            imshow("match", imMatch);

            if(waitKey(30) >= 0)
            {
                break;
            }
         }
            return 0;



}


//referencias: exemplos do open cv
