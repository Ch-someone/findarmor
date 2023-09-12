////
//// Created by dji on 5/19/23.
////
//
//#include <iostream>
//#include <opencv2/opencv.hpp>
//
//int main() {
//    // 创建黑底图像
//    cv::Mat image(100, 100, CV_8UC1, cv::Scalar(0));
//
//    // 设置字体类型和尺寸
//    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
//    double fontScale = 2;
//
//    // 遍历生成数字图片
//    for (int i = 0; i <= 9; i++) {
//        // 清空图像
//        image.setTo(0);
//
//        // 在图像中心位置绘制白色数字
//        std::string text = std::to_string(i);
//        int baseline = 0;
//        cv::Size textSize = cv::getTextSize(text, fontFace, fontScale, 2, &baseline);
//        cv::Point textOrg((image.cols - textSize.width) / 2, (image.rows + textSize.height) / 2);
//        cv::putText(image, text, textOrg, fontFace, fontScale, cv::Scalar(255), 2, cv::LINE_AA);
//
//        // 显示图像
//        cv::imshow("Number Image", image);
//        cv::waitKey(0);
//
//        // 保存图像
//        std::string filename = "number_" + std::to_string(i) + ".png";
//        cv::imwrite(filename, image);
//    }
//
//    return 0;
//}