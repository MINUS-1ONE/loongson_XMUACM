//  Created by Linzaer on 2019/11/15.
//  Copyright © 2019 Linzaer. All rights reserved.

#include "recognizer.h"
#include "json.hpp"
#include "httplib.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <thread>
#include <sstream>

using namespace std;
using namespace httplib;
using json = nlohmann::json;

std::string pwd = "";

int main(int argc, char **argv)
{
    Recognizer recognizer(argv[1],argv[2]);

    int size = 341;

    cv::Mat frame,boxplot(cv::Size(size*3,600),CV_8UC3,cv::Scalar(0,0,0));
    cv::VideoCapture cap(0);
    cv::Mat face = cv::imread("Aspendove.png");
    recognizer.Register(face,"Aspendove");

    std::vector<cv::Mat> breads
    {
        cv::imread("./bread/bread1.jpg"),
        cv::imread("./bread/bread2.jpg"),
        cv::imread("./bread/bread3.jpg")
    };
    
    cv::resize(breads[0],breads[0],cv::Size(size,size));
    cv::resize(breads[1],breads[1],cv::Size(size,size));
    cv::resize(breads[2],breads[2],cv::Size(size,size));

    std::vector<cv::Mat> breads_gray;
    breads_gray.resize(3);
    cv::cvtColor(breads[0],breads_gray[0],CV_BGR2GRAY);
    cv::cvtColor(breads[1],breads_gray[1],CV_BGR2GRAY);
    cv::cvtColor(breads[2],breads_gray[2],CV_BGR2GRAY);
    cv::merge(std::vector<cv::Mat>{breads_gray[0],breads_gray[0],breads_gray[0]},breads_gray[0]);
    cv::merge(std::vector<cv::Mat>{breads_gray[1],breads_gray[1],breads_gray[1]},breads_gray[1]);
    cv::merge(std::vector<cv::Mat>{breads_gray[2],breads_gray[2],breads_gray[2]},breads_gray[2]);

    Client cli(argv[3]);
    Server svr;
    svr.Post("/plan", [&](const Request &req, Response &res)
    {
        auto svrres = cli.Post("/plan");

        res.set_content(svrres->body, "text/plain");
    });
    svr.Post("/drive", [&](const Request &req, Response &res)
    {
        cli.Post("/drive", req.get_param_value("state"), "text/plain");
        if(req.get_param_value("state") == "drive")
        {
            std::cout << "drive now" << std::endl;
        }
        else std::cout << "stop now" << std::endl;
    });
    svr.Post("/select", [&](const Request &req, Response &res)
    {
        //cli.Post("/dirve", req.body, "text/plain");
        stringstream ss(req.body);
        std::vector<int> state;
        state.resize(3);
        std::cout << req.body << std::endl;
        sscanf(req.body.c_str(),"0=%d&1=%d&2=%d",&state[0],&state[1],&state[2]);
        std::cout << state[2] << std::endl;
        for(int i = 0; i != 3; ++i)
        {
            if(state[i])
                breads[i].copyTo(boxplot(cv::Rect(cv::Point(i*size,(600-size)/2),cv::Size(size,size))));
            else
                breads_gray[i].copyTo(boxplot(cv::Rect(cv::Point(i*size,(600-size)/2),cv::Size(size,size))));
        }
       // ss >> "data=" >> state[0] >> "%2C" >> state[1] >> "%2C" >> state[2];
    });
    svr.Get("/currentpose", [&](const Request &req, Response &res)
    {
        auto svrres = cli.Get("/currentpose");
        res.set_content(svrres->body, "text/plain");
    });
    svr.Post("/multipart", [&](const Request &req, Response &res)
    {
        const MultipartFormData &file = req.get_file_value("uploadFace");

        file.content.data();
        file.content.length();

        cv::Mat rawData(1,file.content.length(), CV_8UC1, (void*)file.content.data());
        cv::Mat face;
        cv::imdecode(rawData,CV_LOAD_IMAGE_COLOR,&face);
        
        cv::imshow("X",face);
        cv::waitKey(0);
    });
    bool ret = svr.set_mount_point("/RegisteredFace",(pwd+"/RegisteredFace").c_str());
    if(!ret)
    {
        std::cerr << ((pwd+"/RegisteredFace"+"not ready").c_str()) << std::endl;
    }
    svr.Get("/hello",[](const Request & /*req*/, Response &res) {
        const char *fmt = "Hello loongix";
        char buf[BUFSIZ];
        snprintf(buf, sizeof(buf), fmt, res.status);
        res.set_content(buf, "text/html");
    });


    breads_gray[0].copyTo(boxplot(cv::Rect(cv::Point(0,(600-size)/2),cv::Size(size,size))));
    breads_gray[1].copyTo(boxplot(cv::Rect(cv::Point(size,(600-size)/2),cv::Size(size,size))));
    breads_gray[2].copyTo(boxplot(cv::Rect(cv::Point(size*2,(600-size)/2),cv::Size(size,size))));

    std::thread tsvr([&]()
    {
        svr.listen("::", 2334);
    });
    std::thread tocv([&]
    {
        while(true)
        {
            auto start = chrono::steady_clock::now();
            cap>>frame;
            auto end = chrono::steady_clock::now();
            chrono::duration<double> elapsed = end - start;

            recognizer.TryFind(frame);

            //cout << "all time: " << elapsed.count() << " s" << endl;
            cv::imshow("UltraFace", frame);
            
            cv::imshow("Boxes",boxplot);
            
            cv::waitKey(1);
        }
    });

    tsvr.join();
    tocv.join();
    return 0;
}
