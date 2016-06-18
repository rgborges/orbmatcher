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

#include <stdio.h>

// constantes de cor no console

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"


// ORB é o featiurte detector padrão

int nfeatures = 500;
float scaleFactor = 2.0f;
int nlevels = 8;
int edgeThreshold = 31;
int firstLevel = 0;
int WTA_K = 2;
int scoreType = ORB::HARRIS_SCORE;
int patchSize = 31;
int fastThreshold = 20;



int main(int argc, char *argv[])
{



        Mat objeto = imread(argv[1], CV_LOAD_IMAGE_ANYCOLOR); //recebe imagem objeto
        Mat ambiente = imread(argv[2], CV_LOAD_IMAGE_ANYCOLOR); // recebe imagem ambiente
        if(!objeto.data || !objeto.data)
        {
           printf("%sERRO: erro ao abrir as imagens , cheque os argumentos\n", KRED);
        }
 /*

        if(strcmp(argv[3], "-nf") == 0)
        {
            nfeatures = atoi(argv[3]);
            cout << "numero de features configurado em " << nfeatures << endl;
        }
        else if(strcmp(argv[3], "-p") == 0)
        {
            cout << "configurar ORB digite o parametros ORB(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, patchSize, fastThreshold)" << endl;
            int *tnFeatures = new int;
            float *tscaleFactor = new float;
            int *tnLevles = new int;
            int *tedgeThreshold = new int;
            int *tfirstLevel = new int;
            int *tWTA_K = new int;
            int *tpatchSize = new int;
            int *tfastThreshold = new int;
            scanf("%i,%f,%i,%i,%i,%i,%i,%i", tnFeatures, tscaleFactor, tnLevles, tedgeThreshold, tfirstLevel,tWTA_K,tpatchSize,tfastThreshold);
            nfeatures = *tnFeatures;
            scaleFactor = *tscaleFactor;
            nlevels = *tnLevles;
            edgeThreshold = *tedgeThreshold;
            firstLevel = *tfirstLevel;
            WTA_K = *tWTA_K;
            patchSize = *tpatchSize;
            fastThreshold = *tfastThreshold;
            delete tnFeatures;
            delete tscaleFactor;
            delete tnLevles;
            delete tedgeThreshold;
            delete tfirstLevel;
            delete tWTA_K;
            delete tpatchSize;
            delete tfastThreshold;
        }

*/
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

                 Point2f center_point1 = (amb_cantos[0] +  Point2f( objeto.cols, 0) +  amb_cantos[1] + Point2f( objeto.cols, 0)) / 2.0;
                 Point2f center_point2 = (amb_cantos[2] +  Point2f( objeto.cols, 0) + amb_cantos[3] + Point2f( objeto.cols, 0)) / 2.0;

                 std::cout << "center p1: " << center_point1 << " center p2:" << center_point2;


                 // o centro da imagem é a metade da distancia entre centro 1 e 2

                 Point2f center_object = (center_point1 - center_point2) / 2.0;


                 std::cout << " object_center: " << center_object << std::endl;
                cv::namedWindow("image 1", WINDOW_AUTOSIZE);
                imshow("match", imMatch);
                waitKey(0);







    return 0;


    //referencias: exemplos do open cv
 }

