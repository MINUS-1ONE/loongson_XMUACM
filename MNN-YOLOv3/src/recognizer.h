#include "UltraFace.hpp"
#include "mobilefacenet.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <map>
#include <vector>


class Recognizer
{
    UltraFace detector;
    Mobilefacenet extractor;
    std::map<std::string,std::vector<float>> info;
public:
    Recognizer(const std::string& ultramodel,const std::string& mbfacemodel)
    :detector(ultramodel, 640, 480, 4, 0.65),extractor(mbfacemodel)
    {

    }
    void Register(cv::Mat& img, const std::string& name)
    {
        std::vector<FaceInfo> face_info;
        detector.detect(img, face_info);
        std::vector<float> feature;

        for (auto face : face_info)
        {
            cv::Point pt1(face.x1, face.y1);
            cv::Point pt2(face.x2, face.y2);
            //cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
            
            extractor.ExtractFeature(img(cv::Rect(pt1,pt2)),&feature);
            break;  // PROCESS ONLY ONE FACE
        }
        info[name] = feature;
    }
    std::string TryFind(cv::Mat& img)
    {
        std::vector<FaceInfo> face_info;
        detector.detect(img, face_info);
        std::vector<float> feature;
        if(face_info.empty())return "";
        cv::Point pt1,pt2;
        for (auto face : face_info)
        {
            pt1 = cv::Point(face.x1, face.y1);
            pt2 = cv::Point(face.x2, face.y2);
            cv::rectangle(img, pt1, pt2, cv::Scalar(0, 0, 255), 2);
            
            extractor.ExtractFeature(img(cv::Rect(pt1,pt2)),&feature);
            break;  // PROCESS ONLY ONE FACE
        }
        for(auto iter = info.begin();iter != info.end(); ++iter)
        {
            float sim = cosine_similarity_vectors(feature,iter->second);
            if (sim > 0.6)
            {
                cv::rectangle(img, pt1, pt2, cv::Scalar(0, 255, 0), 2);
                std::cout<< sim << std::endl;
            }
        }

        return "";
    }
    float cosine_similarity_vectors(std::vector<float> A, std::vector<float>B)
    {
        float mul = 0.0;
        float d_a = 0.0;
        float d_b = 0.0 ;

        if (A.size() != B.size())
        {
            std::cout<<A.size()<<" "<<B.size()<<std::endl;
            throw std::logic_error("Vector A and Vector B are not the same size");
        }

        // Prevent Division by zero
        if (A.size() < 1)
        {
            throw std::logic_error("Vector A and Vector B are empty");
        }

        std::vector<float>::iterator B_iter = B.begin();
        std::vector<float>::iterator A_iter = A.begin();
        for( ; A_iter != A.end(); A_iter++ , B_iter++ )
        {
            mul += *A_iter * *B_iter;
            d_a += *A_iter * *A_iter;
            d_b += *B_iter * *B_iter;
        }

        if (d_a == 0.0f || d_b == 0.0f)
        {
            throw std::logic_error(
                    "cosine similarity is not defined whenever one or both "
                    "input vectors are zero-vectors.");
        }

        return mul / (sqrt(d_a) * sqrt(d_b));
    }
};
