#include "armordetect1.h"
//************大恒相机驱动库*************
#include "GxIAPI.h"
#include "DxImageProc.h"
#include "anglesolve1.h"
#include <fstream>
#include<iostream>
#include <chrono>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
static std::mutex mutex1;

#define DETECT_DEBUG
#define TIME_COUNT

//float last_pitch;

// 计算当前时刻距离 1970年1月1号 的毫秒数
#ifdef TIME_COUNT
int getCurrentTime(){
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#endif //TIME_COUNT
void FrameBuffer::init(int size)
{
    _frames.resize(size);
    _tailIdx = 0;
    _headIdx = 0;
    _lastGetTimeStamp = 0.0;
}
bool FrameBuffer::push(const Frame& aframe)
{
    const int newHeadIdx = (_headIdx + 1) % _frames.size();

    unique_lock<timed_mutex> lock(_mutexes[newHeadIdx],chrono::milliseconds(2));

    if(!lock.owns_lock())
        return false;
    
    _frames[newHeadIdx] = aframe;
    if(newHeadIdx == _tailIdx)
        _tailIdx = (_tailIdx + 1) % _frames.size();
    _headIdx = newHeadIdx;

    return true;
}
bool FrameBuffer::getLatest(Mat& aframe)
{
    volatile const int headIdx = _headIdx;

    unique_lock<timed_mutex> lock(_mutexes[headIdx],chrono::milliseconds(5));
    if(!_frames.size())
    {
        return false;
    }



    if(!lock.owns_lock() ||
       _frames[headIdx].img.empty() || 
       _frames[headIdx].timeStamp == _lastGetTimeStamp)
    {
        return false;
    }
    aframe = _frames[headIdx].img;
    _lastGetTimeStamp = _frames[headIdx].timeStamp;

    return true;

}
cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode) {
    using std::swap;
    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;
    if (mode == 1) {
        if (width < height) {
            swap(width, height);
            angle += 90.0;
        }
    }

    while (angle >= 90.0)
        angle -= 180.0;
    while (angle < -90.0)
        angle += 180.0;

    if (mode == 0) {
        if (angle >= 45.0) {
            swap(width, height);
            angle -= 90.0;
        }
        else if (angle < -45.0) {
            swap(width, height);
            angle += 90.0;
        }
    }
    return rec;
}
float predict(float*w,float b,float*X)
{
    int num = 0;
    for (int i =0; i<5;i++)
    {
        num += w[i]*X[i];
    }
    num += b;
    float A = 1/(1+exp(-num));
    return A;
}
float getDistance(Point2f point0, Point2f point1)
{
    float distance;
    distance = powf((point0.x - point1.x), 2) + powf((point0.y - point1.y), 2);
    distance = sqrtf(distance);
    return distance;
}

void ArmorDetect::produce()
{

    auto startTime = chrono::high_resolution_clock::now();
    Angle_Solve Angle_Solve;
    KalmanFilter KF(4, 2, 0);
    //Mat processNoise(stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    //这里没有控制矩阵B，默认为0
    setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0]测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪音，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪音，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
    char *pRGB24Buf = NULL;
    ArmorDetect ArmorDetect;
    _buffer.init(6);
    GX_STATUS status = GX_STATUS_SUCCESS;
    GX_DEV_HANDLE hDevice = NULL;
    uint32_t nDeviceNum = 0;
    //初始化库
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS)
    {
//        cout << "Library Open failed!"<<endl;
        return;
    }
    //枚举设备列表
    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0))
    {
        cout << "Devices Open failed!"<<endl;
        return;
    }
    //cout << "Open success!"<<endl;

    //打开第一个设备
    status = GXOpenDeviceByIndex(1, &hDevice);
    if (status == GX_STATUS_SUCCESS)
    {
        int64_t nDecimationH= 1;
        int64_t nDecimationV= 1;
        //设 置 水 平 和 垂 直 Decimation 模 式 为 Sum 模 式
        status = GXSetEnum(hDevice,GX_ENUM_BINNING_HORIZONTAL_MODE,GX_BINNING_HORIZONTAL_MODE_SUM);
        status = GXSetEnum(hDevice,GX_ENUM_BINNING_VERTICAL_MODE,GX_BINNING_VERTICAL_MODE_SUM);
        status = GXSetInt(hDevice, GX_INT_DECIMATION_HORIZONTAL, nDecimationH);
        status = GXSetInt(hDevice, GX_INT_DECIMATION_VERTICAL, nDecimationV);
        //设 置 一 个 offset 偏 移 为 (320,272) ,640x480 尺 寸 的 区 域
        GX_STATUS status = GX_STATUS_SUCCESS;
        int64_t nWidth= 640;
        int64_t nHeight= 1000;
        int64_t nOffsetX = 320;
        int64_t nOffsetY = 450;
        status = GXSetInt(hDevice, GX_INT_WIDTH, nWidth);
        status = GXSetInt(hDevice, GX_INT_HEIGHT, nHeight);
        status = GXSetInt(hDevice, GX_INT_OFFSET_X, nOffsetX);
        status = GXSetInt(hDevice, GX_INT_OFFSET_Y, nOffsetY);
        // 使能采集帧率调节模式
        status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
        // 设置采集帧率,假设设置为 210.0（每秒采集 210 张图像）, 用户按照实际需求设此值
        status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 210.0);
        //设置曝光时间
        double exposure_time = 5000.0000;
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, exposure_time);
        //设 置 曝 光 延 迟 为 2us0.
        double dExposureValue = 2.0;
        status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_DELAY, dExposureValue);
        //自 动 白 平 衡 设 置
        status = GXSetEnum(hDevice,GX_ENUM_BALANCE_WHITE_AUTO,GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        //2023 5 21 : gammaParam
        bool bIsImplemented = false;
        double dGammaParam;
        int64_t nContrastParam = 0;
        int64_t nColorCorrectionParam = 0;
        int nGammaLutLength;
        int nContrastLutLength;
        GX_STATUS GxStatus = GXIsImplemented(hDevice, GX_INT_CONTRAST_PARAM,
                                             &bIsImplemented);
        if (bIsImplemented)
        {
            //获取 Gamma 调节参数
            GxStatus = GXGetFloat (hDevice, GX_FLOAT_GAMMA_PARAM, &dGammaParam);
            if (GxStatus!= GX_STATUS_SUCCESS)
            {
                cout << "Gamma failed!"<<endl;
                return;
            }
        }
        //获取对比度调节参数
        GxStatus = GXGetInt (hDevice, GX_INT_CONTRAST_PARAM, &nContrastParam);
        if (GxStatus != GX_STATUS_SUCCESS)
        {
            cout << "Contrast failed!"<<endl;
            return;
        }
        //获取颜色校正调节参数
        GxStatus = GXGetInt (hDevice, GX_INT_COLOR_CORRECTION_PARAM,
                             &nColorCorrectionParam);
        if (GxStatus != GX_STATUS_SUCCESS)
        {
            cout << "ColorCorrection failed!"<<endl;
            return;
        }


        dGammaParam = 1.55;
        nContrastParam = -40;

        //获取 Gamma 查找表的长度
        VxInt32 DxStatus= DxGetGammatLut(dGammaParam, NULL, &nGammaLutLength);
        if (DxStatus != DX_OK)
        {
            cout << "Gamma failed!"<<endl;
            return;
        }
        //为 Gamma 查找表申请空间
        unsigned  char* pGammaLut;
        pGammaLut= new unsigned char[nGammaLutLength];
        if (pGammaLut== NULL)
        {
            cout << "Gamma failed!"<<endl;
            return;
        }
        //获取对比度查找表的长度
        DxStatus= DxGetContrastLut(nContrastParam, NULL, &nContrastLutLength);
        if (DxStatus != DX_OK)
        {
            return;
        }
        //为对比度查找表申请空间
        unsigned char* pContrastLut;
        pContrastLut = new unsigned char[nContrastLutLength];
        if (pContrastLut == NULL)
        {
            DxStatus= DX_NOT_ENOUGH_SYSTEM_MEMORY;
            return;
        }
        //计算对比度查找表
        DxStatus = DxGetContrastLut(nContrastParam, pContrastLut, &nContrastLutLength);
        if (DxStatus != DX_OK)
        {
            return;
        }
        //计算 Gamma 查找表
        DxStatus = DxGetGammatLut(dGammaParam, pGammaLut, &nGammaLutLength);
        if (DxStatus != DX_OK)
        {
            cout << "Gamma failed!"<<endl;
            if (pGammaLut!= NULL)
            {
                delete []pGammaLut;
                pGammaLut= NULL;
            }
            if(pContrastLut != NULL)
            {
                delete[] pContrastLut;
                pContrastLut = NULL;
            }
            return;
        }




        //获取图像采集帧率
        double current_frame;
        status = GXGetFloat(hDevice,GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE,&current_frame);
        cout << "fps:"<<current_frame << endl;
        //定义 GXDQAllBuf 的传入参数
        PGX_FRAME_BUFFER pFrameBuffer[5];
        //定 义 实 际 填 充 图 像 个 数
        uint32_t nFrameCount = 0;
        //开始循环采集图像
        Mat frame;
        for(;;)
        {
            int produceTimeStamp1 = getCurrentTime();
            status = GXStreamOn(hDevice);
            if (status == GX_STATUS_SUCCESS)
            {
                //调 用 GXDQAllBufs 获 取 队 列 中 所 有 图 像
                status = GXDQAllBufs(hDevice, pFrameBuffer, 5, &nFrameCount, 1000);



                if (status == GX_STATUS_SUCCESS)
                {
                    for(uint32_t i = 0; i < nFrameCount; i++)
                    {

                        if(pFrameBuffer[i] != NULL && pFrameBuffer[i]->nStatus == GX_FRAME_STATUS_SUCCESS)
                        {
                            //图像获取成功
                            //对图像进行处理...

                            #ifdef TIME_COUNT
                            ArmorDetect.clocker += 1;
                            #endif //TIME_COUNT

                            frame.create(pFrameBuffer[i]->nHeight, pFrameBuffer[i]->nWidth, CV_8UC3);

                            pRGB24Buf = new char[(size_t)(pFrameBuffer[i]->nWidth*pFrameBuffer[i]->nHeight*3)];
                            //false图像为不反转
                            DxStatus = DxRaw8toRGB24(pFrameBuffer[i]->pImgBuf,pRGB24Buf,pFrameBuffer[i]->nWidth,pFrameBuffer[i]->nHeight,RAW2RGB_NEIGHBOUR,DX_PIXEL_COLOR_FILTER(BAYERBG),false);
                            DxStatus = DxImageImprovment(pRGB24Buf,pRGB24Buf,pFrameBuffer[i]->nWidth,pFrameBuffer[i]->nHeight,
                                                         nColorCorrectionParam, pContrastLut, pGammaLut);
                            if (DxStatus != DX_OK){
    //                            cout << "DxStatus != DX_OK" << endl;
                                if (pRGB24Buf != NULL)
                                {
                                    delete []pRGB24Buf;
                                    pRGB24Buf = NULL;
                                }
                                return ;
                            }

                            memcpy(frame.data, pRGB24Buf, (pFrameBuffer[i]->nHeight * pFrameBuffer[i]->nWidth * 3));

                            if (pRGB24Buf != NULL)
                            {
                                delete []pRGB24Buf;
                                pRGB24Buf = NULL;
                            }
                            double timeStamp = (chrono::duration_cast<chrono::duration<double>>(chrono::high_resolution_clock::now()-startTime)).count();
                            int produceTimeStamp2 = getCurrentTime();
                            _buffer.push(Frame{frame,timeStamp});
                            int produceTimeStamp3 = getCurrentTime();
    //                        cout << "One Single Frame's lock time" << produceTimeStamp3 - produceTimeStamp2 << endl;
                        }
                    }
                    //调 用 GXQAllBufs 将 获 取 到 的 所 有 图 像 buf 放 回 库 中 继 续 采 图
                    status = GXQAllBufs(hDevice);
                }

            }
        #ifdef DETECT_DEBUG
        if(ArmorDetect.c == 27){
            break;
        }
        #endif //DETECT_DEBUG

        int produceTimeStamp4 = getCurrentTime();
        cout << "All produceTime:" << (int)(produceTimeStamp4 - produceTimeStamp1) << endl;
        }
        if (pGammaLut!= NULL)
        {
            delete []pGammaLut;
            pGammaLut= NULL;
        }
        if (pContrastLut!= NULL)
        {
            delete []pContrastLut;
            pContrastLut = NULL;
        }
        status = GXStreamOff(hDevice);
    }
    status = GXCloseDevice(hDevice);
    status = GXCloseLib();
}
void ArmorDetect::consume()
{
    bool last_con = 0;
    Mat frame;
    KalmanFilter KF(4, 2, 0);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    //这里没有控制矩阵B，默认为0
    setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0]测量矩阵
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪音，单位阵
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪音，单位阵
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差协方差矩阵，初始化为单位阵
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值
    int count = 0;
    while(true)
    {
        int consumeTimeStamp1 = getCurrentTime();
        if(!_buffer.getLatest(frame))
            continue;
        if(findarmor(frame, KF, measurement, count) == false)
        {
            last_con = 0;
            count = count+1;
            continue;
        }
        last_con = 1;
        int consumeTimeStamp2 = getCurrentTime();
        cout << "consuming interval: "<< consumeTimeStamp2 - consumeTimeStamp1 << endl;
    }
}
int ArmorDetect::armordetect(){
    return 0;
}
bool ArmorDetect::findarmor(Mat frame, KalmanFilter KF, Mat measurement, int count){
//    printf("\033c");
    cout << "-----------------------"<<endl;
    float height;
    //cout<<"last_pitch: "<< last_pitch << endl;
    Angle_Solve Angle_Solve;
    Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    //RED range1
    Scalar lower1(0, 43, 150);
    Scalar upper1(25, 255, 255);
    //RED range2
    Scalar lower2(150, 43, 150);
    Scalar upper2(180, 255, 255);
    //BLUE range
    Scalar lower3(100, 43, 150);
    Scalar upper3(124, 255, 255);

    Mat mask,copy,mask_red2;
    imshow("raw_frame", frame);
    waitKey(1);
    frame.copyTo(copy);
    cv::cvtColor(frame, mask, cv::COLOR_BGR2HSV);
    cv::cvtColor(frame, mask_red2, cv::COLOR_BGR2HSV);


//    imshow("mask", mask);
//    waitKey(1);
    split(frame, channels);
    //cannot show
    if (armorParam.enemy_color == "RED") {
        grayImage = channels.at(2) - channels.at(0);  // Get red-blue image;
        threshold_param = armorParam.red_brightness_threshold;
        inRange(mask, lower1, upper1, mask);
        inRange(mask_red2, lower2, upper2, mask_red2);
        bitwise_or(mask,mask_red2,mask);
//        imshow("grayImage",grayImage);
    } else {
        grayImage = channels.at(0) - channels.at(2);  // Get blue-red image;
        threshold_param = armorParam.blue_brightness_threshold;
        inRange(mask, lower3, upper3, mask);
//        imshow("grayImage",grayImage);

    }
//    imshow("mask", mask);
    threshold(grayImage, binBrightImage_light, threshold_param, 255, cv::THRESH_BINARY);
    bitwise_and(binBrightImage_light, mask, binBrightImage_light);
    imshow("mask", binBrightImage_light);
    dilate(binBrightImage_light, binBrightImage_light, element);
    vector<vector<Point>> lightContours;
    vector<Vec4i> lightContours2;
    vector<LightDescriptor> lightInfos;
    findContours(binBrightImage_light.clone(), lightContours, lightContours2,RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat testm = mask.clone();
    if(lightContours2.size())
    {
    for(int i=0;i >= 0;i = lightContours2[i][0])
    {
        drawContours(testm,lightContours,i,(0,0,255),CV_FILLED,8,lightContours2);
    }
    
    }
//    imshow("contours",testm);
//    waitKey(1);
Mat result1 = frame.clone();
    for (const auto& contour : lightContours){
        float lightContourArea = contourArea(contour);
        if (contour.size() < armorParam.light_min_size || lightContourArea < armorParam.light_min_area)continue;
        //RotatedRect lightRec = fitEllipse(contour);
        RotatedRect lightRec = minAreaRect(contour);
        adjustRec(lightRec, 0);
        if (lightRec.size.width / lightRec.size.height > armorParam.light_width_height_ratio || lightContourArea / lightRec.size.area() < armorParam.light_contour_rect_ratio)
            continue;
        lightRec.size.width *= armorParam.light_extend_ratio;
        lightRec.size.height *= armorParam.light_extend_ratio;
 //       circle(frame, lightRec.center, 2, Scalar(255, 0, 0), 2);//wo jia de
        lightInfos.push_back(LightDescriptor(lightRec));
    }
    if (lightInfos.empty())
        return false;
    //按照中心的 x 坐标进行排序
    sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2)
        {
            return ld1.center.x < ld2.center.x;
        });
    vector<Target> targets;
    float meanLen;
    vector<Point3f> POINT_3D;
    //最终识别筛选后的灯条（前期筛选的好的话，灯条数组的长度 5～6 个）
//    cout << "light_num: "<<lightInfos.size() << endl;
    for (size_t i = 0; i < lightInfos.size(); i++){
        for (size_t j = i + 1; (j < lightInfos.size()); j++){
            //矩形的四个顶点（Pointf）
            Point tl, tr, bl, br;
            Target target;
            //识别出的左右灯条
            const LightDescriptor& leftLight = lightInfos[i];
            const LightDescriptor& rightLight = lightInfos[j];
            float lenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
            if (lenDiff_ratio > armorParam.light_max_height_diff_ratio){
//                cout<<"lenDiff"<<endl;
                continue;
            }
            float dis = sqrt(powf((leftLight.center.x - rightLight.center.x), 2) + powf((leftLight.center.y - rightLight.center.y), 2));
            meanLen = (leftLight.length + rightLight.length) / 2;
            float yDiff = abs(leftLight.center.y - rightLight.center.y);
            float yDiff_ratio = yDiff / meanLen;
            float xDiff = abs(leftLight.center.x - rightLight.center.x);
            float xDiff_ratio = xDiff / meanLen;
            float ratio = dis / meanLen;
//            cout<<lenDiff_ratio<<"    "<<dis<<"   "<<yDiff_ratio<<"   "<<xDiff_ratio<<"   "<<ratio<<endl;


              float X[5] = {lenDiff_ratio,dis,yDiff_ratio,xDiff_ratio,ratio};
              float w[5] = {-1.96231213,-0.02850556,-6.53832968,0.04956654,-5.15525653};
              float A = predict(w,19.701902069923097,X);

              if (A <= 0.45){

                  continue;
              }





//              cout << "light " << i << " light " << j << " ratio: "<<ratio<<endl;
              if(ratio > armorParam.big_armor_min_aspect_ratio && ratio < armorParam.big_armor_max_aspect_ratio){
//                  cout << "BIG" << endl;
                  POINT_3D = Angle_Solve.POINT_3D_OF_ARMOR_BIG;
              }
              else if(ratio > armorParam.small_armor_min_aspect_ratio && ratio < armorParam.small_armor_max_aspect_ratio){
//                  cout << "SMALL" << endl;
                  POINT_3D = Angle_Solve.POINT_3D_OF_ARMOR_SMALL;
              }
              else{
                  continue;
              }
            Point pt = Point((leftLight.center.x + rightLight.center.x) / 2, (leftLight.center.y + rightLight.center.y) / 2);
            //circle(frame, pt, 2, Scalar(0, 0, 255), 2);
            if(leftLight.pts[1].x > rightLight.pts[1].x){
               tl = Point(rightLight.pts[1].x,rightLight.pts[1].y);
               tr = Point(leftLight.pts[2].x,leftLight.pts[2].y);
               bl = Point(rightLight.pts[0].x,rightLight.pts[0].y);
               br = Point(leftLight.pts[3].x,leftLight.pts[3].y);
            } else {
               tl = Point(leftLight.pts[1].x,leftLight.pts[1].y);
               tr = Point(rightLight.pts[2].x,rightLight.pts[2].y);
               bl = Point(leftLight.pts[0].x,leftLight.pts[0].y);
               br = Point(rightLight.pts[3].x,rightLight.pts[3].y);
            }



            target.TargetAngle = (leftLight.angle + rightLight.angle) / 2;
            target.armor4dot.push_back(tl);
            target.armor4dot.push_back(tr);
            target.armor4dot.push_back(br);
            target.armor4dot.push_back(bl);
            target.center = pt;
            targets.push_back(target);


            float height1 = tl.y - bl.y;
            if (height1 < 0){
                height1 = -height1;
            }
            float height2 = tr.y - br.y;
            if (height2 < 0){
                height2 = -height2;
            }
            height = (height1+height2)/2.0;
            //calAngle(cameraMatrix, distCoeffs, pt.x, pt.y);
        }
    }
    if (targets.empty()) return false;
    //             detect the armor is or not a num armor
    //             SVM
    for(auto &item : targets){
        Point tl, tr, bl, br;
        tl = item.armor4dot[0];
        tr = item.armor4dot[1];
        bl = item.armor4dot[2];
        br = item.armor4dot[3];

        float centerX = (((tl.x + br.x) / 2) + (tr.x + bl.x) / 2) / 2;
        float centerY = (((tl.y + br.y) / 2) + (tr.y + bl.y) / 2) / 2;
        Point2f selectCenter = Point2f(centerX,centerY);
        float lightLen2 = getDistance(tl,bl) + getDistance(tr,br);
        Size2f selectSize = Size2f(lightLen2,lightLen2);
        float picAngle = item.TargetAngle;
        RotatedRect select;
        select = RotatedRect(selectCenter,selectSize,picAngle);
        Point2f vertices[4];
        select.points(vertices);
        swap(vertices[0],vertices[1]);
        swap(vertices[1],vertices[2]);
//        for(int i=0;i<4;i++)
//            cout << vertices[i] << endl;
//        cout << "angle" << picAngle<<endl;
//        cout << tl <<endl;
//        cout << tr <<endl;
//        cout << bl <<endl;
//        cout << br <<endl;
        float widf = 25,heightf = 25;
        Point2f vertices_dst[4] = {{0.0f,0.0f},{widf,0.0f},{0.0f,heightf},{widf,heightf}};

        Mat per_Mat = getPerspectiveTransform(vertices,vertices_dst);

        Mat svmFrame;
        warpPerspective(copy,svmFrame,per_Mat,Size((int)widf,(int)heightf));

        imshow("svmFrame", svmFrame);
        if (numsDetectSVM(svmFrame, xmlPath))
            item.with_num = true;
        else
            item.with_num = false;
    }
    sort(targets.begin(), targets.end(), [](const Target& pt1, const Target& pt2)
    {
        if(pt1.with_num != pt2.with_num)
            return pt1.with_num > pt2.with_num;
        else
            return abs(pt1.center.x - 320) < abs(pt2.center.x - 320);

    });
    //const Target& true_target = targets[0];
    Target& true_target = targets[0];


    #ifdef TIME_COUNT
    // 每50帧记录一次
    if(clocker % 50 == 1){
        average_time = (getCurrentTime() - previous_time) / 50.0;
        previous_time = getCurrentTime();
         cout << "average_time" << average_time << "-----------------------------------------------" << endl;
    }
    #endif //TIME_COUNT

    #ifdef DETECT_DEBUG
    Mat result2 = frame.clone();
    float true_x[1000];
    float true_y[1000];
    string text_x = to_string(true_target.center.x);
    string text_y = to_string(true_target.center.y);
    string text = "(" + text_x + "," + text_y + ")";
    putText(result1, text, Point(true_target.center.x, true_target.center.y), FONT_HERSHEY_PLAIN, 1.5, Scalar::all(255), 1, 8, 0);


    circle(result1, true_target.center, 2, Scalar(0, 0, 255), 2);
    for(int i=0;i<true_target.armor4dot.size();i++) {
        cv::line(result1, true_target.armor4dot[i], true_target.armor4dot[(i+1)%4], Scalar(0, 255, 0), 3);
    }
    putText(result1, "armor", true_target.armor4dot[0], FONT_HERSHEY_PLAIN, 1.5, Scalar::all(255), 1, 8, 0);
    imshow("result",result1);
    waitKey(1);
    #endif //DETECT_DEBUG

    float* angle_and_distance = Angle_Solve.calPnP(POINT_3D,true_target.armor4dot,cameraMatrix,distCoeffs);
    angle_and_distance = Angle_Solve.compensateOffset(angle_and_distance, 2000);//测量枪管和镜头前后距离(mm)
    //angle_and_distance[1] =Z 11.0;
    //angle_and_distance = Angle_Solve.compensateGravity(angle_and_distance, 13.5);//重力补偿(mm)
    //float distance = angle_and_distance[2];
    float distance_true = -3.3686624*log(height) + 14.21;
    //angle_and_distance[2] = distance_true;
    //angle_and_distance[1] = -1.4;


//    cout<<"height:   "<<height<<endl;
//    cout<<"distance:   "<<angle_and_distance[2]<<endl;
//    cout<<"yaw: "<< angle_and_distance[0] <<endl;
//    cout<<"pitch: "<< angle_and_distance[1] << endl;
    Serial test;
    test.try1(angle_and_distance);
    // std::thread thread(openThread, &angle_and_distance);

    delete[] angle_and_distance;
    return true;
}



bool ArmorDetect::numsDetectSVM(Mat frame, string filename) {
    Ptr<ml::SVM> svm = ml::StatModel::load<ml::SVM>(filename);
    int cnt = 0;
    Mat mask,grayImage,image = frame.clone();
    //resize(frame, frame, Size(25, 25));
    resize(image, mask, Size(25, 25));
//
    //range1
    Scalar lower1(0, 0, 180);
    Scalar upper1(180, 20, 255);
    cvtColor(mask,mask,COLOR_BGR2GRAY);
//
    adaptiveThreshold(mask,mask,255,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,3,0);
    mask.convertTo(mask, CV_32F);
    imshow("2zhi", mask);
    mask = mask.reshape(1, 1);

    Mat p;
    svm->predict(mask, p);
    if(int(p.at<float>(0,0))) {
        cout<<"is Armor!"<<endl;
        return true;
    }else{
        cout<<"Not Armor!"<<endl;
        return false;
    }
}



