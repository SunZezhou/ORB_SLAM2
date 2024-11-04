/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
 
#include "FrameDrawer.h"
#include "Tracking.h"
 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
 
#include<mutex>
#include <tuple>
#include "ORBmatcher.h"
namespace ORB_SLAM2
{
 
FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mIm_r = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}
 
std::tuple<cv::Mat, cv::Mat, std::vector<cv::KeyPoint>, std::vector<cv::KeyPoint>,  vector<cv::DMatch> > FrameDrawer::DrawFrame()
{
    cv::Mat im,im1,im2;
    vector<cv::KeyPoint> vIniKeys,lastKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<bool> vbVO1, vbMap1; // Tracked MapPoints in reference frame
    int state; // Tracking state
    vector<cv::DMatch>  matchess;
    vector<cv::KeyPoint> nkeys1;
    vector<cv::KeyPoint> nkeys2;
    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;
 
        mIm.copyTo(im);
        mIm.copyTo(im2);
        mIm_r.copyTo(im1);
 
        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
             lastKeys=mvlastkeys;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
             vbVO1 = mvbVO1;
            vbMap1 = mvbMap1;
            lastKeys=mvlastkeys;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
             lastKeys=mvlastkeys;
        }
    } // destroy scoped mutex -> release mutex
 
    if(im.channels()<3) //this should be always true
       { 
        // cvtColor(im,im,CV_GRAY2BGR);
        // cvtColor(im1,im1,CV_GRAY2BGR);
        // cvtColor(im2,im2,CV_GRAY2BGR);
        }
 
    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                       cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
         
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
 
                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im2,pt1,pt2,cv::Scalar(0,255,0));
                   cv::circle(im2,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im2,pt1,pt2,cv::Scalar(255,0,0));
                   cv::circle(im2,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
        const int n1 =  lastKeys.size();
        for(int i=0;i<n1;i++)
        {
             if(vbVO1[i] || vbMap1[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=lastKeys[i].pt.x-r;
                pt1.y=lastKeys[i].pt.y-r;
                pt2.x=lastKeys[i].pt.x+r;
                pt2.y=lastKeys[i].pt.y+r;
                 
             if(vbMap1[i])
                {
                    cv::rectangle(im1,pt1,pt2,cv::Scalar(0,255,0));
                   cv::circle(im1,lastKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                   cv::rectangle(im1,pt1,pt2,cv::Scalar(255,0,0));
                   cv::circle(im1,lastKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                }
            }
        }
    }
 
    //cv::Mat imWithInfo;
    //DrawTextInfo(im,state, imWithInfo);
 
    for(size_t i1=0, iend=vMatches12.size(); i1<iend;i1++)
    {
            int i2 = vMatches12[i1];
            if (i2<0)
            {
                nkeys1.push_back(lastKeys[i1]);
                continue;
            }
            cv::DMatch mm;
            mm.queryIdx = nkeys1.size();
            mm.trainIdx = i2;
            matchess.push_back(mm);
            nkeys1.push_back(lastKeys[i1]);
            nkeys2.push_back(vCurrentKeys[i2]);
    }
 
 
    return std::make_tuple(im1,im2,lastKeys,vCurrentKeys,matchess);
}
 
 
void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }
 
    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);
 
    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
 
}
 
void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    mIm.copyTo(mIm_r);
    pTracker->mImGray.copyTo(mIm);
 
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mvlastkeys=store_lastframe.mvKeys;
    N = mvCurrentKeys.size();
    N1 = mvlastkeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mvbVO1 = vector<bool>(N1,false);
    mvbMap1 = vector<bool>(N1,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
 
 
    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N1;i++)
        {
            MapPoint* pMP1 = store_lastframe.mvpMapPoints[i];
            if(pMP1)
            {
                if(!store_lastframe.mvbOutlier[i])
                {
                    if(pMP1->Observations()>0)
                        mvbMap1[i]=true;
                    else
                        mvbVO1[i]=true;
                }
            }
        }
 
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
          
    }
    // Find correspondences
    ORBmatcher matcher(0.6,false);
    vPrevMatched.resize(store_lastframe.mvKeysUn.size());
    for(size_t i=0; i<store_lastframe.mvKeysUn.size(); i++)
        vPrevMatched[i]=store_lastframe.mvKeysUn[i].pt;
    int nmatches = matcher.SearchForInitialization(store_lastframe,pTracker->mCurrentFrame,vPrevMatched,vMatches12,100);
    std::cout <<"points: "<<nmatches << std::endl;
    mState=static_cast<int>(pTracker->mLastProcessedState);
    store_lastframe=pTracker->mCurrentFrame;
     
}
 
} //namespace ORB_SLAM