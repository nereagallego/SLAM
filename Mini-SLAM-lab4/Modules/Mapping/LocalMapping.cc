/**
* This file is part of Mini-SLAM
*
* Copyright (C) 2021 Juan J. Gómez Rodríguez and Juan D. Tardós, University of Zaragoza.
*
* Mini-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Mini-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with Mini-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include "Mapping/LocalMapping.h"
#include "Optimization/g2oBundleAdjustment.h"
#include "Matching/DescriptorMatching.h"
#include "Utils/Geometry.h"

using namespace std;

LocalMapping::LocalMapping() {

}

LocalMapping::LocalMapping(Settings& settings, std::shared_ptr<Map> pMap) {
    settings_ = settings;
    pMap_ = pMap;
}

void LocalMapping::doMapping(std::shared_ptr<KeyFrame> &pCurrKeyFrame) {
    //Keep input keyframe
    currKeyFrame_ = pCurrKeyFrame;

    if(!currKeyFrame_)
        return;

    //Remove redundant MapPoints
    mapPointCulling();

    //Triangulate new MapPoints
    triangulateNewMapPoints();

    checkDuplicatedMapPoints();

    //Run a local Bundle Adjustment
    localBundleAdjustment(pMap_.get(),currKeyFrame_->getId());
}

void LocalMapping::mapPointCulling() {
    /*
     * Your code for Lab 4 - Task 4 here!
     */
}

void LocalMapping::triangulateNewMapPoints() {
    //Get a list of the best covisible KeyFrames with the current one
    vector<pair<ID,int>> vKeyFrameCovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());

    vector<int> vMatches(currKeyFrame_->getMapPoints().size());

    //Get data from the current KeyFrame
    shared_ptr<CameraModel> calibration1 = currKeyFrame_->getCalibration();
    Sophus::SE3f T1w = currKeyFrame_->getPose();

    int nTriangulated = 0;

    for(pair<ID,int> pairKeyFrame_Obs : vKeyFrameCovisible){
        int commonObservations = pairKeyFrame_Obs.second;
        if(commonObservations < 20)
            continue;

        shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(pairKeyFrame_Obs.first);
        if(pKF->getId() == currKeyFrame_->getId())
            continue;

        //Check that baseline between KeyFrames is not too short
        Eigen::Vector3f vBaseLine = currKeyFrame_->getPose().inverse().translation() - pKF->getPose().inverse().translation();
        float medianDepth = pKF->computeSceneMedianDepth();
        float ratioBaseLineDepth = vBaseLine.norm() / medianDepth;

        if(ratioBaseLineDepth < 0.01){
            continue;
        }

        Sophus::SE3f T2w = pKF->getPose();

        Sophus::SE3f T21 = T2w*T1w.inverse();
        Eigen::Matrix<float,3,3> E = computeEssentialMatrixFromPose(T21);

        //Match features between the current and the covisible KeyFrame
        //TODO: this can be further improved using the orb vocabulary
        int nMatches = searchForTriangulation(currKeyFrame_.get(),pKF.get(),settings_.getMatchingForTriangulationTh(),
                settings_.getEpipolarTh(),E,vMatches);

        vector<cv::KeyPoint> vTriangulated1, vTriangulated2;
        vector<int> vMatches_;
        //Try to triangulate a new MapPoint with each match
        for(size_t i = 0; i < vMatches.size(); i++){
            if(vMatches[i] != -1){
                cv::Point2f p1 = currKeyFrame_->getKeyPoint(i).pt;
                cv::Point2f p2 = pKF->getKeyPoint(vMatches[i]).pt;

                Eigen::Vector3f xn1 = calibration1->unproject(p1).normalized();
                Eigen::Vector3f xn2 = pKF->getCalibration()->unproject(p2).normalized();

                Eigen::Vector3f x3D;
                triangulate(xn1,xn2,T1w,T2w,x3D);

                // Reproject 3D point to the camera frame
                Eigen::Vector3f x3Dcam = T1w*x3D;

                // Normalize the point
                x3Dcam.normalize();

                // Convert to 2D point
                cv::Point2f x2D = calibration1->project(x3Dcam);

                // Reproject the 3D point to the camera frame of the second camera
                Eigen::Vector3f x3Dcam2 = T2w*x3D;
                x3Dcam2.normalize();

                // Convert to 2D point 
                cv::Point2f x2D2 = pKF->getCalibration()->project(x3Dcam2);

                // cout << "Triangulated point: " << x3D.transpose() << endl;
                // cout << "Point p1 " << p1 << " p2 " << x2D << " p2' " << x2D2 << endl;
                // cout << "Reprojection error: " << squaredReprojectionError(p1,x2D) << " " << squaredReprojectionError(p2,x2D2) << endl;
                // cout << "Parallax: " << cosRayParallax(xn1,xn2) << endl;
                if(x3D(2) > 0 && abs(cosRayParallax(xn1 ,xn2)) < 0.998 && squaredReprojectionError(p1,x2D) < 2.0 && squaredReprojectionError(p2,x2D2) < 2.0){
                    cout << "Triangulated new MapPoint" << endl;
                    vTriangulated1.push_back(currKeyFrame_->getKeyPoint(i));
                    vTriangulated2.push_back(pKF->getKeyPoint(vMatches[i]));
                    vMatches_.push_back(vMatches[i]);

                    shared_ptr<MapPoint> pMP(new MapPoint(x3D));

                    // Add the new mappoint into the map
                    pMap_->insertMapPoint(pMP);

                    // Adds the observation of the mappoint in the current keyframe
                    pMap_->addObservation(currKeyFrame_->getId(), pMP->getId(), vMatches[i]);

                    nTriangulated++;

                    
                }
            }
        }
    }
}

void LocalMapping::checkDuplicatedMapPoints() {
    vector<pair<ID,int>> vKFcovisible = pMap_->getCovisibleKeyFrames(currKeyFrame_->getId());
    vector<shared_ptr<MapPoint>> vCurrMapPoints = currKeyFrame_->getMapPoints();

    for(int i = 0; i < vKFcovisible.size(); i++){
        if(vKFcovisible[i].first == currKeyFrame_->getId())
            continue;
        int nFused = fuse(pMap_->getKeyFrame(vKFcovisible[i].first),settings_.getMatchingFuseTh(),vCurrMapPoints,pMap_.get());
        pMap_->checkKeyFrame(vKFcovisible[i].first);
        pMap_->checkKeyFrame(currKeyFrame_->getId());
    }
}
