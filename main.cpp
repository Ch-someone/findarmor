#include <iostream>
#include <fstream>
#include <thread>
#include "armordetect1.h"

using namespace std;

int main()

{


    ArmorDetect ArmorDetect;

     std::thread th1(&ArmorDetect::produce,&ArmorDetect);
     std::thread th2(&ArmorDetect::consume,&ArmorDetect);
     th1.join();
     th2.join();

    return 0;
}

//
//#include <opencv2/opencv.hpp>
//#include <string>
//
//int main() {
//
//
//    VideoCapture cap(0);
//
////    cap.set(cv::CAP_MODE_GRAY, cv::CAP_MODE_GRAY);
//
//    while (true){
//
//        string train_filename = "/home/dji/Desktop/findarmor_2023/numsImg/";
//        string train_filenames[10];
//        int train_labels[10];
//        for (int i = 0; i < 10; i++) {
//            train_filenames[i] = train_filename + "number_" + to_string(i) + ".png";
//            train_labels[i] = i;
////            cout << train_filenames[i] << " v " << train_labels[i] << endl;
//        }
//
//        string test_filenames[2] = {"/home/dji/Desktop/findarmor_2023/numsImg/number_0.png",
//                                    "/home/dji/Desktop/findarmor_2023/numsImg/number_7.png"};
//
//        // 将图像转换为特征向量
//        std::vector<cv::Mat> trainImages; // 训练图像数组
//        std::vector<int> trainLabels; // 训练图像标签数组
//        std::vector<cv::Mat> testImages; // 测试图像数组
//
//        // 读取训练数据
//        for (int i = 0; i < 10; i++) {
//            cv::Mat image = cv::imread(train_filenames[i]);
//
//            cv::resize(image, image, cv::Size(28, 28)); // 调整图像大小为28x28
//            trainImages.push_back(image);
//            trainLabels.push_back(train_labels[i]);
//        }
//
//        // 创建特征矩阵
//        cv::Mat trainData;
//        for (int i = 0; i < trainImages.size(); i++) {
//            cv::Mat image = trainImages[i].reshape(1, 1);
//            trainData.push_back(image);
//        }
//        trainData.convertTo(trainData, CV_32F);
//
//        Mat image;
//        cap.read(image);
//
//        cv::resize(image, image, Size(28, 28));
//        testImages.push_back(image);
//
//        Mat testData;
//        Mat img = testImages[0].reshape(1, 1);
//        testData.push_back(img);
//
//        testData.convertTo(testData, CV_32F);
//
//        // 创建标签矩阵
//        cv::Mat trainLabelsMat(trainLabels.size(), 1, CV_32S, trainLabels.data());
//
//        // 创建SVM对象
//        cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
//        svm->setType(cv::ml::SVM::C_SVC);
//        svm->setKernel(cv::ml::SVM::RBF);
//
//        // 训练SVM模型
//        svm->train(trainData, cv::ml::ROW_SAMPLE, trainLabelsMat);
//
//        cv::Mat p;
//        svm->predict(testData, p);
//
//        // 处理分类结果
//        for (int i = 0; i < p.rows; i++) {
//            int predictedLabel = p.at<float>(i, 0);
//            // 处理预测结果
//            // ...
//            cout << predictedLabel << endl;
//        }
//        resize(image, image, Size(300, 300));
//        imshow("1", image);
//        waitKey(1);
//    }
//    return 0;
//}
