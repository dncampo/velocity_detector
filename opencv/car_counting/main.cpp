#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/highgui/highgui_c.h>
#include<opencv2/imgproc/imgproc.hpp>

#include<iostream>
#include<iomanip>
#include<sys/time.h>
#include<unistd.h>

#include "Blob.h"

using namespace std;

// global variables ///////////////////////////////////////////////////////////////////////////////
const cv::Scalar SCALAR_BLACK = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar SCALAR_WHITE = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar SCALAR_YELLOW = cv::Scalar(0.0, 255.0, 255.0);
const cv::Scalar SCALAR_GREEN = cv::Scalar(0.0, 255.0, 0.0);
const cv::Scalar SCALAR_RED = cv::Scalar(0.0, 0.0, 255.0);

// function prototypes ////////////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(vector<Blob> &existingBlobs, vector<Blob> &currentFrameBlobs);
void addBlobToExistingBlobs(Blob &currentFrameBlob, vector<Blob> &existingBlobs, int &intIndex);
void addNewBlob(Blob &currentFrameBlob, vector<Blob> &existingBlobs);
double distanceBetweenPoints(cv::Point point1, cv::Point point2);
void drawAndShowContours(cv::Size imageSize, vector<vector<cv::Point> > contours, string strImageName);
void drawAndShowContours(cv::Size imageSize, vector<Blob> blobs, string strImageName);
bool checkIfBlobsCrossedTheLine(vector<Blob> &blobs, int &intHorizontalLinePosition, int &carCount, int);
void drawBlobInfoOnImage(vector<Blob> &blobs, cv::Mat &imgFrame2Copy);
void drawCarCountOnImage(int &carCount, cv::Mat &imgFrame2Copy);

const float LINE1_POSITION_PERCENTAGE = 0.9;
const float LINE2_POSITION_PERCENTAGE = 0.3;
const float DISTANCE_BTW_LINES = 3.048;
const bool SHOW_CAR_COUNT = true;
const bool SHOW_BLOB_INFO = true;
const bool SWOW_DEBUG_INFO = false;

int main(void) {
    cv::VideoCapture capVideo;

    cv::Mat imgFrame1;
    cv::Mat imgFrame2;

    vector<Blob> blobs;

    cv::Point crossingLine[2];
    cv::Point crossingLine2[2];

    int carCount = 0;

    capVideo.open("../_vids/original_bridge.mp4");

    if (!capVideo.isOpened()) {
        cout << "error reading video file" << endl << endl;
        return(0);
    }

    capVideo.read(imgFrame1);
    capVideo.read(imgFrame2);

    int intHorizontalLinePosition = (int)round((double)imgFrame1.rows * LINE1_POSITION_PERCENTAGE);
    int intHorizontalLinePosition2 = (int)round((double)imgFrame1.rows * LINE2_POSITION_PERCENTAGE);

    crossingLine[0].x = 0;
    crossingLine[0].y = intHorizontalLinePosition;

    crossingLine[1].x = imgFrame1.cols - 1;
    crossingLine[1].y = intHorizontalLinePosition;

    crossingLine2[0].x = 0;
    crossingLine2[0].y = intHorizontalLinePosition2;

    crossingLine2[1].x = imgFrame1.cols - 1;
    crossingLine2[1].y = intHorizontalLinePosition2;

    char chCheckForEscKey = 0;
    bool blnFirstFrame = true;
    int frameCount = 2;
    while (capVideo.isOpened() && chCheckForEscKey != 27) {
        vector<Blob> currentFrameBlobs;

        cv::Mat imgFrame1Copy = imgFrame1.clone();
        cv::Mat imgFrame2Copy = imgFrame2.clone();

        cv::Mat imgDifference;
        cv::Mat imgThresh;

        cv::cvtColor(imgFrame1Copy, imgFrame1Copy, CV_BGR2GRAY);
        cv::cvtColor(imgFrame2Copy, imgFrame2Copy, CV_BGR2GRAY);

        cv::GaussianBlur(imgFrame1Copy, imgFrame1Copy, cv::Size(5, 5), 0);
        cv::GaussianBlur(imgFrame2Copy, imgFrame2Copy, cv::Size(5, 5), 0);

        cv::absdiff(imgFrame1Copy, imgFrame2Copy, imgDifference);
        cv::threshold(imgDifference, imgThresh, 30, 255.0, CV_THRESH_BINARY);
        if (SWOW_DEBUG_INFO) {
            cv::imshow("imgThresh", imgThresh);
        }

        cv::Mat structuringElement3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat structuringElement5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::Mat structuringElement7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::Mat structuringElement15x15 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));

        for (unsigned int i = 0; i < 2; i++) {
            cv::dilate(imgThresh, imgThresh, structuringElement5x5);
            cv::dilate(imgThresh, imgThresh, structuringElement5x5);
            cv::erode(imgThresh, imgThresh, structuringElement5x5);
        }
        cv::Mat imgThreshCopy = imgThresh.clone();
        vector<vector<cv::Point> > contours;
        cv::findContours(imgThreshCopy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (SWOW_DEBUG_INFO) {
            drawAndShowContours(imgThresh.size(), contours, "imgContours");
        }
        vector<vector<cv::Point> > convexHulls(contours.size());
        for (unsigned int i = 0; i < contours.size(); i++) {
            cv::convexHull(contours[i], convexHulls[i]);
        }

        if (SWOW_DEBUG_INFO) {
            drawAndShowContours(imgThresh.size(), convexHulls, "imgConvexHulls");
        }
        for (auto &convexHull : convexHulls) {
            Blob possibleBlob(convexHull);
            if (possibleBlob.currentBoundingRect.area() > 400 &&
                possibleBlob.dblCurrentAspectRatio > 0.2 &&
                possibleBlob.dblCurrentAspectRatio < 4.0 &&
                possibleBlob.currentBoundingRect.width > 30 &&
                possibleBlob.currentBoundingRect.height > 30 &&
                possibleBlob.dblCurrentDiagonalSize > 60.0 &&
                (cv::contourArea(possibleBlob.currentContour) / (double)possibleBlob.currentBoundingRect.area()) > 0.50) {
                currentFrameBlobs.push_back(possibleBlob);
            }
        }

        if (SWOW_DEBUG_INFO) {
            drawAndShowContours(imgThresh.size(), currentFrameBlobs, "imgCurrentFrameBlobs");
        }

        if (blnFirstFrame == true) {
            for (auto &currentFrameBlob : currentFrameBlobs) {
                blobs.push_back(currentFrameBlob);
            }
        } else {
            matchCurrentFrameBlobsToExistingBlobs(blobs, currentFrameBlobs);
        }
        if (SWOW_DEBUG_INFO) {
            drawAndShowContours(imgThresh.size(), blobs, "imgBlobs");
        }
        imgFrame2Copy = imgFrame2.clone();          // get another copy of frame 2 since we changed the previous frame 2 copy in the processing above

        if (SHOW_BLOB_INFO) {
            drawBlobInfoOnImage(blobs, imgFrame2Copy);
        }
        bool blnAtLeastOneBlobCrossedTheLine[2];
        int falseCarCount = 0;
        blnAtLeastOneBlobCrossedTheLine[0] = checkIfBlobsCrossedTheLine(blobs, intHorizontalLinePosition, carCount, 0);
        blnAtLeastOneBlobCrossedTheLine[1] = checkIfBlobsCrossedTheLine(blobs, intHorizontalLinePosition2, falseCarCount, 1);

        if (blnAtLeastOneBlobCrossedTheLine[0] == true) {
            cv::line(imgFrame2Copy, crossingLine[0], crossingLine[1], SCALAR_GREEN, 2);
        } else {
            cv::line(imgFrame2Copy, crossingLine[0], crossingLine[1], SCALAR_RED, 2);
        }
        if (blnAtLeastOneBlobCrossedTheLine[1] == true) {
            cv::line(imgFrame2Copy, crossingLine2[0], crossingLine2[1], SCALAR_GREEN, 2);
        } else {
            cv::line(imgFrame2Copy, crossingLine2[0], crossingLine2[1], SCALAR_RED, 2);
        }
        if (SHOW_CAR_COUNT) {
            drawCarCountOnImage(carCount, imgFrame2Copy);
        }
        cv::imshow("imgFrame2Copy", imgFrame2Copy);

        //cv::waitKey(0);                 // uncomment this line to go frame by frame for debugging
        // now we prepare for the next iteration
        currentFrameBlobs.clear();
        imgFrame1 = imgFrame2.clone();           // move frame 1 up to where frame 2 is
        if ((capVideo.get(CV_CAP_PROP_POS_FRAMES) + 1) < capVideo.get(CV_CAP_PROP_FRAME_COUNT)) {
            capVideo.read(imgFrame2);
        } else {
            cout << "end of video\n";
            break;
        }
        blnFirstFrame = false;
        frameCount++;
        chCheckForEscKey = cv::waitKey(1);
    }
    if (chCheckForEscKey != 27) {               // if the user did not press esc (i.e. we reached the end of the video)
        cv::waitKey(0);                         // hold the windows open to allow the "end of video" message to show
    }
    // note that if the user did press esc, we don't need to hold the windows open, we can simply let the program end which will close the windows
    return(0);
}

void matchCurrentFrameBlobsToExistingBlobs(vector<Blob> &existingBlobs, vector<Blob> &currentFrameBlobs) {
    for (auto &existingBlob : existingBlobs) {
        existingBlob.blnCurrentMatchFoundOrNewBlob = false;
        existingBlob.predictNextPosition();
    }
    for (auto &currentFrameBlob : currentFrameBlobs) {
        int intIndexOfLeastDistance = 0;
        double dblLeastDistance = 100000.0;
        for (unsigned int i = 0; i < existingBlobs.size(); i++) {
            if (existingBlobs[i].blnStillBeingTracked == true) {
                double dblDistance = distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);
                if (dblDistance < dblLeastDistance) {
                    dblLeastDistance = dblDistance;
                    intIndexOfLeastDistance = i;
                }
            }
        }
        if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 0.5) {
            addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
        } else {
            addNewBlob(currentFrameBlob, existingBlobs);
        }
    }
    for (auto &existingBlob : existingBlobs) {
        if (existingBlob.blnCurrentMatchFoundOrNewBlob == false) {
            existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
        }
        if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
            existingBlob.blnStillBeingTracked = false;
        }
    }
}

void addBlobToExistingBlobs(Blob &currentFrameBlob, vector<Blob> &existingBlobs, int &intIndex) {
    existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
    existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;

    existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());

    existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
    existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;

    existingBlobs[intIndex].blnStillBeingTracked = true;
    existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
}

void addNewBlob(Blob &currentFrameBlob, vector<Blob> &existingBlobs) {
    currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;
    existingBlobs.push_back(currentFrameBlob);
}

double distanceBetweenPoints(cv::Point point1, cv::Point point2) {
    int intX = abs(point1.x - point2.x);
    int intY = abs(point1.y - point2.y);
    return(sqrt(pow(intX, 2) + pow(intY, 2)));
}

void drawAndShowContours(cv::Size imageSize, vector<vector<cv::Point> > contours, string strImageName) {
    cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);
    cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);
    cv::imshow(strImageName, image);
}

void drawAndShowContours(cv::Size imageSize, vector<Blob> blobs, string strImageName) {
    cv::Mat image(imageSize, CV_8UC3, SCALAR_BLACK);
    vector<vector<cv::Point> > contours;
    for (auto &blob : blobs) {
        if (blob.blnStillBeingTracked == true) {
            contours.push_back(blob.currentContour);
        }
    }
    cv::drawContours(image, contours, -1, SCALAR_WHITE, -1);
    cv::imshow(strImageName, image);
}

long int getTs() {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

bool checkIfBlobsCrossedTheLine(vector<Blob> &blobs, int &intHorizontalLinePosition, int &carCount, int line) {
    bool blnAtLeastOneBlobCrossedTheLine = false;
    for (int i=0; i<blobs.size(); i++) {
        if (blobs[i].blnStillBeingTracked == true && blobs[i].centerPositions.size() >= 2) {
            int prevFrameIndex = (int)blobs[i].centerPositions.size() - 2;
            int currFrameIndex = (int)blobs[i].centerPositions.size() - 1;

            if (blobs[i].centerPositions[prevFrameIndex].y > intHorizontalLinePosition && blobs[i].centerPositions[currFrameIndex].y <= intHorizontalLinePosition) {
                carCount++;
                blnAtLeastOneBlobCrossedTheLine = true;
                blobs[i].ts[line] = getTs();
            }
        }

    }
    return blnAtLeastOneBlobCrossedTheLine;
}

void drawBlobInfoOnImage(vector<Blob> &blobs, cv::Mat &imgFrame2Copy) {
    for (unsigned int i = 0; i < blobs.size(); i++) {
        if (blobs[i].blnStillBeingTracked == true) {
            cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_BLACK, 2);

            int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
            double dblFontScale = blobs[i].dblCurrentDiagonalSize / 60.0;
            int intFontThickness = (int)round(dblFontScale * 1.0);

            Blob blob = blobs[i];
            cv::Point speedLabelPos = blob.centerPositions.back();
            speedLabelPos.y += 50;;

            cv::putText(imgFrame2Copy, to_string(i), blob.centerPositions.back(), intFontFace, dblFontScale / 2, SCALAR_RED, intFontThickness);
            if (blob.getSpeed(DISTANCE_BTW_LINES) != 0) {
                stringstream stream;
                stream << fixed << setprecision(1) << blob.getSpeed(DISTANCE_BTW_LINES);
                string speedLabel = stream.str();
                cv::putText(imgFrame2Copy, speedLabel + " Km/h" , speedLabelPos, intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
            }
        }
    }
}

void drawCarCountOnImage(int &carCount, cv::Mat &imgFrame2Copy) {
    int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
    double dblFontScale = (imgFrame2Copy.rows * imgFrame2Copy.cols) / 300000.0;
    int intFontThickness = (int)round(dblFontScale * 1.5);

    cv::Size textSize = cv::getTextSize(to_string(carCount), intFontFace, dblFontScale, intFontThickness, 0);

    cv::Point ptTextBottomLeftPosition;

    ptTextBottomLeftPosition.x = imgFrame2Copy.cols - 1 - (int)((double)textSize.width * 1.25);
    ptTextBottomLeftPosition.y = (int)((double)textSize.height * 1.25);

    cv::putText(imgFrame2Copy, to_string(carCount), ptTextBottomLeftPosition, intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
}
