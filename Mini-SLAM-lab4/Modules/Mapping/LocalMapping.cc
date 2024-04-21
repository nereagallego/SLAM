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
    // detect bad MapPoints and remove them from the map
    

    // Map maintenance. Eliminate MapPoints seen only once, more than two steps ago.
    vector<shared_ptr<MapPoint>> vMapPoints = currKeyFrame_->getMapPoints();

    for(int i = 0; i < vMapPoints.size(); i++){
        shared_ptr<MapPoint> pMP = vMapPoints[i];
        if(pMP){
            int nObs = pMap_->getNumberOfObservations(pMP->getId());
            // cout << "MapPoint " << pMP->getId() << " has " << nObs << " observations" << endl;
            if(nObs == 1){
                cout << "MapPoint " << pMP->getId() << " only seen once" << endl;
                
                // If the MapPoint is seen once but less than 3 KeyFrames before, it is kept in the map
                if(pMap_->isMapPointInKeyFrame(pMP->getId(), currKeyFrame_->getId())
                    || pMap_->isMapPointInKeyFrame(pMP->getId(), currKeyFrame_->getId()-1)
                    || pMap_->isMapPointInKeyFrame(pMP->getId(), currKeyFrame_->getId()-2)){
                    continue;
                }

                // Search the keyframe that contains the MapPoint and remove the observation
                for (int j = currKeyFrame_->getId()-3; j > 0; j--){
                    shared_ptr<KeyFrame> pKF = pMap_->getKeyFrame(j);
                    if(pMap_->isMapPointInKeyFrame(pMP->getId(), currKeyFrame_->getId())){
                        pMap_->removeObservation(pKF->getId(),pMP->getId());
                        pMap_->removeMapPoint(pMP->getId());
                        cout << "MapPoint " << pMP->getId() << " removed" << endl;
                        break;
                    }
                }
            }
        }
    }
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
                // The next step is to try to triangulate a 3D point from each match and check that it is geometrically correct (it has been triangulated in front of both cameras, has been observed with enough parallax and its reprojection error is low)
                cv::KeyPoint kp1 = currKeyFrame_->getKeyPoint(i);
                cv::KeyPoint kp2 = pKF->getKeyPoint(vMatches[i]);

                Eigen::Vector3f xn1 = calibration1->unproject(kp1.pt).normalized();
                Eigen::Vector3f xn2 = pKF->getCalibration()->unproject(kp2.pt).normalized();

                Eigen::Vector3f x3D;
                triangulate(xn1,xn2,T1w,T2w,x3D);

                Eigen::Vector3f x3Dw = T1w * x3D;
                Eigen::Vector3f x3Dc = T2w * x3D;

                // Check that the point is in front of both cameras
                if(x3Dw.z() < 0 || x3Dc.z() < 0)
                    continue;

                // Unproject rays and check parallax
                Eigen::Vector3f ray1 = T1w.inverse().rotationMatrix() * xn1;
                Eigen::Vector3f ray2 = T2w.inverse().rotationMatrix() * xn2;

                float cosParallax = cosRayParallax(ray1.normalized(),ray2.normalized());

                if(cosParallax < 0 || cosParallax > settings_.getMinCos())
                    continue;

                // Check reprojection error
                
                cv::Point2f reprojection1 = calibration1->project(x3Dw);
                
                cv::Point2f reprojection2 = pKF->getCalibration()->project(x3Dc);

                float squaredError1 = squaredReprojectionError(kp1.pt,reprojection1);
                float squaredError2 = squaredReprojectionError(kp2.pt,reprojection2);
                

                if(squaredError1 > 4 || squaredError2 > 4)
                    continue;

                // Add the new MapPoint to the Map
                shared_ptr<MapPoint> pMP(new MapPoint(x3D));
                pMap_->insertMapPoint(pMP);

                // Set the map point in the frames
                currKeyFrame_->setMapPoint(i,pMP);
                pKF->setMapPoint(vMatches[i],pMP);

                // Add the observation to the Map
                pMap_->addObservation(currKeyFrame_->getId(),pMP->getId(),i);
                pMap_->addObservation(pKF->getId(),pMP->getId(),vMatches[i]);

                nTriangulated++;

                vTriangulated1.push_back(kp1);
                vTriangulated2.push_back(kp2);
                vMatches_.push_back(vMatches[i]);

            }
        }
    }
    cout << "Triangulated " << nTriangulated << " new MapPoints" << endl;
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
