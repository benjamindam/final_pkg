#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "ros/console.h"
#include "linefollowertest/pos.h"
#include "linefollowertest/docking.h"


class LineDetect {
 public:
    cv::Mat img;  /// Input image in opencv matrix format
    cv::Mat img_filt;  /// Filtered image in opencv matrix format
    int dir;  /// Direction message to be published
/**
*@brief Callback used to subscribe to the image topic from the Turtlebot and convert to opencv image format
*@param msg is the image message for ROS
*@return none
*/
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
/**
*@brief Function that applies Gaussian filter in the input image 
*@param input is the image from the turtlebot in opencv matrix format
*@return Mat of Gaussian filtered image in opencv matrix format
*/
    cv::Mat Gauss(cv::Mat input);
/**
*@brief Function to perform line detection using color thresholding,image masking and centroid detection to publish direction 
*@param input is the Filtered input image in opencv matrix format
*@return int direction which returns the direction the turtlebot should head in
*/
    int colorthresh(cv::Mat input);

 private:
    cv::Scalar LowerYellow;
    cv::Scalar UpperYellow;
    cv::Mat img_hsv;
    cv::Mat img_mask;
};

void LineDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  /*
Vi bruger cv_bridge for at konvertere ROS billeder til billeder som openCV kan forstå
Vi laver en variabel cv_bridge::CvImagePtr som lagrer vores ros billede i variablen cv_ptr
  */
  cv_bridge::CvImagePtr cv_ptr; 
  try {
    /*
    Vi bruger nu cv_bridge::toCvCopy til at konvertere vores billede som ligger i variabel msg, til BGR8 farverummet
    Dette kan openCV arbejde med
    */
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    img = cv_ptr->image;//Nu ligger vi vores nye billede over i variabel img
    cv::waitKey(30); //Vi venter i 30 milisekunder
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); //Hvis ikke vi kan konvertere, melder vi fejl
  }
}


int LineDetect::colorthresh(cv::Mat input) {//typen cv::Mat, er en måde for OpenCV at lagre store vektorer såsom billeder
  // Initializaing variables
  /*
  Laver en variabel s, med typen cv::Size, som lagrer højde og bredde på en vektor.
  Størrelsen defineres som size() af input, som er vores filtrerede billede
  */
  cv::Size s = input.size(); 
/*
laver en vektor v, som er konstrueret af en vektor, som er af type cv::point. cv::point er punkter med x og y koordinater.
Så vi laver en vektor bygget op af punkter.
*/  
std::vector<std::vector<cv::Point> > v; 
/*
Vi laver nu 3 variabler w, h og c_x. w definerer vi som s.width, som er en værdi i sv::Size s; Det samme med h 
som er s.height.
c_x definerer vi som 0.0
*/
  auto w = s.width;
  auto h = s.height;
  auto c_x = 0.0;
  // Detect all objects within the HSV range

  /*
  cv::cvtColor(InputArray src, OutputArray dst, int code) konverterer et billede fra et colorspace til et andet
  Her giver vi 3 variabler:
  src – input image: 8-bit unsigned, 16-bit unsigned ( CV_16UC... ), or single-precision floating-point.
  dst – output image of the same size and depth as src.
  code – color space conversion code.
  Vi giver vores billede som input, output er LineDetect::img_hsv. Som er defineret i linedetect.hpp og med type cv::Mat
  vi konverterer til HSV, og bruger derfor CV_BGR2HSV, da vi konverterer fra BGR til HSV. Mere information her: 
  https://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html
  */
  cv::cvtColor(input, LineDetect::img_hsv, CV_BGR2HSV);

/*
Vi angiver nu to vektorer, med 3 værdier i hver. Både LowerYellow og UpperYellow er i linedetect.hpp defineret som
cv::Scalar variabel-typer. cv::Scalar er variabler som kan holde op til 4 værdier.
Vi bruger disse til at definere, hvilket rum indenfor HSV farverummet, vi vil beholde. 
Dette gør vi for at isolere vores gule linje.
*/
  LineDetect::LowerYellow = {20, 120, 80};
  LineDetect::UpperYellow = {30, 255, 220};

  /*
  cv::inRange tjekker om værdier i en vektor eller et array ligge indenfor to andre værdier.
  void cv::inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)
  Vi ligger vores img_hsv billede ind, som er vores billede i HSV faverummet. Vi bruger som lowerb og upperb
  vores LowerYellow og UpperYellow værdier. Vi outputter billedet i LineDetect::img_mask, som ligesom de andre billeder er 
  en cv::Mat type, og som er defineret i Linedetect.hpp.
  */
  cv::inRange(LineDetect::img_hsv, LowerYellow, UpperYellow, LineDetect::img_mask);
  /*
  Vi vil ikke bruge hele billedet, da der kan være streget på vægge osv. Vi laver derfor en rektangel, 
  som vi definerer, ikke skal være en del af billedet
  */
  img_mask(cv::Rect(0, 0, w, 0.8*h)) = 0;
  /*
 
 void cv::findContours(InputOutputArray image, OutputArrayOfArrays contours, OutputArray hierarchy,
 int mode, int method, Point offset = Point())	

  Vi bruger denne funktion til at finde linjer som argrænser vores gule linje. Vi bruger vores img_mask som input
Punkterne til linjerne som bliver fundet, skal lagres i en vektor defineret som std::vector<std::vector<cv::Point> >
Derfor vi har lavet vektoren v, og denne gemmer vi disse værdier i. 
Vi bruger RETR_LIST til at få punkterne og CHAIN_APPROX_NONE for at lagre værdierne

Læs mere om disse værdier her: https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a

  */
  cv::findContours(LineDetect::img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

/*
Vi siger nu, at hvis der er contours, altså at der er elementer i vores vektor v, så kører vi det næste
*/

  if (v.size() != 0) {
    //Vi initialiserer 3 værdier, area, idx & count
  auto area = 0;
  auto idx = 0;
  auto count = 0;
/*
Så længe vores count værdi er mindre end størrelsen af vektoren v. Tester vi om area er mindre end størrelsen 
på count værdien i v i bytes. Hvis den er det, så bliver idx sat til count, og area til størrelsen af count værdien i v.
Derefter plusses count med 1.
*/
  while (count < v.size()) {
    if (area < v[count].size()) {
       idx = count;
       area = v[count].size();
    }
    count++;
  }
/*
Vi bruger nu funktionen boundingRect() til at udregne et rektangel ud fra punkterne i vores vektor v. 
Vi bruger kun punkterne op til tallet som idx er. 

*/

  cv::Rect rect = boundingRect(v[idx]);

  /*
  Vi definerer du 3 punkter, pt1, pt2 & pt3
  */

  cv::Point pt1, pt2, pt3;

  /*
  Vi tager nu og og angiver pt1 til rektanglets start, x og y værdier. Dette er værdierne i øverste venstre hjørne
  Vi angiver pt2 koordinater til rektanglets start x og y værdier, dog plusset med bredden og højden af rektanglet. 
  pt3 bliver til koordinaterne som pt1 har, med henholdsvis plusset og minusset med 5.
  Dette forskyder punktet, og her vil vi have teksten til at stå
  */
  pt1.x = rect.x;
  pt1.y = rect.y;
  pt2.x = rect.x + rect.width;
  pt2.y = rect.y + rect.height;
  pt3.x = pt1.x+5;
  pt3.y = pt1.y-5;
  /*
  Vi bruger nu punkterne  til at tegne rektanglet, med funktionen rectangle(). Vi giver vores kamerabillede som input.
  Vi kommer vores to rektangel punkter ind, og angiver at stregen skal være i rgb farverummet 255, 0, 0 som er rød.
  Til sidst angives at vi vil tegne med en tykkelse på 2
  */
  rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
  /*
Vi bruger nu cv::putText() til at skrive på billedet. Vi skriver på input, som er billedet direkte fra kameraet.
Vi angiver at der skal stå "Linje fundet", og det skal sættes i punktet pt3. Vi vælger skrifttypen CV_FONT_HERSHEY_COMPLEX.
1 tallet er skalering af skriften. Til sidst vælger vi farven på teksten
  */
  cv::putText(input, "Linje fundet", pt3, CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));
  }




/*
Vi er interesserede i at robotten kun scanner det som er lige foran dem, så at hvis der er noget 
i højre eller venstre side af billedet som kan distrahere kameraet, så kan den køre forkert. Vi definerer derfor at i højre 
og venstre side er der et rektangel hvori robotten ikke leder efter gule pixels
*/

  
  img_mask(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
  img_mask(cv::Rect(0, 0, 0.3*w, h)) = 0;


/*
med cv::Moments kan vi beregne hvor linjen er i forhold til center. af billedet
Ud fra dette kan vi bestemme om robotten skal køre til ventre, for at linjen igen er i midten, eller om den skal til højre
Vi laver en variabel cv::Moments M. vi kører funktionen cv::moments, og indsætter vores billede. Vi får så forskellige
værdier ud. 
*/

  // Perform centroid detection of line
  cv::Moments M = cv::moments(LineDetect::img_mask);
  if (M.m00 > 0) {
    cv::Point p1(M.m10/M.m00, M.m01/M.m00);
    cv::circle(LineDetect::img_mask, p1, 5, cv::Scalar(155, 200, 0), -1);
  }

  c_x = M.m10/M.m00;
  // Tolerance to chooise directions
  auto tol = 15;
  auto count = cv::countNonZero(img_mask); //Denne funktion tæller hvor mange værdier i img_mask, som ikke er 0
  // Turn left if centroid is to the left of the image center minus tolerance
  // Turn right if centroid is to the right of the image center plus tolerance
  // Go straight if centroid is near image center
  if (c_x < w/2-tol) {
    LineDetect::dir = 0; //Kør til venstre
  } else if (c_x > w/2+tol) {
    LineDetect::dir = 2; //Kør til højre
  } else {
    LineDetect::dir = 1; //Kør ligeud
  }

  // Hvis der ikke er nogle værdier over 0, så er der ikke nogen linje, og vi skal derfor lede efter en
  if (count < 50) {
    LineDetect::dir = 3;
  }
  // Output images viewed by the turtlebot
  cv::namedWindow("Turtlebot View"); //cv::namedWindow laver et vindue med navn "Turtlebot View"
  imshow("Turtlebot img_mask", img_mask);
  imshow("Turtlebot img_hsv", img_hsv);
  imshow("Turtlebot input", input); //imshow tager vinduet "Turtlebot View og viser billedet input"
  return LineDetect::dir; //Returner dir.
}






int main(int argc, char **argv) {
    ros::init(argc, argv, "detection"); //Initialser en ros node med navn "detection"
    ros::NodeHandle n; //start nodehandle n
    LineDetect det; //laver et komponent i call LineDetect med navn "det"

    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw",
        1, &LineDetect::imageCallback, &det); //Starter en publisher på image_raw, med en kø på 1, og med callback funktion 
        //imageCallback, som ligger i linedetect.cpp, med besked det. 
    ros::Publisher dirPub = n.advertise<linefollowertest::pos>("direction", 1); //Starter en publisher på custom besked pos til topic "direction"
        linefollowertest::pos msg; //Laver en variabel med type Linefollower::pos, som hedder msg.

    while (ros::ok()) {
        if (!det.img.empty()) {//hvis der er et værdier (et billede) i variabel det.img, kør følgende
            /*
            Kør følgende funktion og lig det i variablen det.img_filt. 
            Prototypen til gauss() er i linedetect.hpp. Funktionen køres i linedetect.cpp
            Her ligges et gaussisk filter på billedet, som laver kontrasten om
            */
            //det.img_filt = det.Gauss(det.img);
            /*
            Vi tager nu det filtrede billede, og kører den gennem colorthresh(). Igen er prototypen i linedetect.hpp,
            og funktionen i linedetect.cpp. 
            */
            msg.direction = det.colorthresh(det.img);
            /*
            colorthresh tilbagemelder en værdi, som er den værdi som skal få turtlebotten til at dreje, eller fortsætte
            Denne publisher vi til dirpub, med vores custom besked. 
            */
            dirPub.publish(msg);
            }
        ros::spinOnce(); //Vi kører igen
    }
    // Closing image viewer
    cv::destroyWindow("Turtlebot View"); //Når ros ikke længere kører, lukkes billedvinduet
    cv::destroyWindow("Turtlebot Linje");
}
