#pragma once

#include <stdlib.h>
#include <set>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "../../RaisimGymEnv.hpp"
namespace raisim {
    class ENVIRONMENT : public RaisimGymEnv {
    public:
        explicit ENVIRONMENT(const std::string &resourceDir, const Yaml::Node &cfg, bool visualizable) :
                RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable), normDist_(0,1) {
          world_ = std::make_unique<raisim::World>();

          //add objects
          gripper_ = world_->addArticulatedSystem(resourceDir + "/ur5_description/urdf/ur5_no3.urdf");
          gripper_->setName("mygripper");
          gripper_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
          ball = world_->addSphere(ballrad, 1);
          ball->setPosition(Eigen::Vector3d(0.6,0.4,0.02));
          auto ground = world_->addGround();
          gcDim_ = gripper_->getGeneralizedCoordinateDim();
          gvDim_ = gripper_->getDOF();
          gJoints = 4;
          nJoints = gvDim_; // 12
          //joint 개수 11  gcDim 11 -> 로봇팔 gc 5 gv5 로봇 손 6개 7번 개씩무한대 6,7,9,10 능동 5, 10 수동
          //index 6,7,8,9가 로봇 손의 관절에 해당하는 부분임 6,7이 left inner knuckle, inner finger 8,9가 right inner knuckle, inner finger
          gc_.setZero(gcDim_);
          gc_init_.setZero(gcDim_);
          gv_.setZero(gvDim_);
          gv_init_.setZero(gvDim_);
          pTarget_.setZero(gcDim_);
          vTarget_.setZero(gvDim_);
          gTarget4_.setZero(gJoints);
          pTarget12_.setZero(nJoints);

          basePos_.setZero();
          gc_init_ <<0.0108, 0.0518, -0.2194, -0.1152, -0.0478, 0.0811, 0.000, 0.0, 0.000, 0.00, 0.0;

          Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_); //gc 19 gv 18
          jointPgain.setZero(); jointPgain.tail(nJoints).setConstant(50.0);
          jointPgain({6,11})<<0,0;
          jointDgain.setZero(); jointDgain.tail(nJoints).setConstant(0.2);
          jointDgain({6,11})<<0,0;
          jointPgain({0,1,2,3})<<500,300,200,100;
          gripper_->setPdGains(jointPgain, jointDgain);
          gripper_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_)); // 팔과 그리퍼의 p d gain을 동일하게 해
          obDim_ = 33; //3+12+12+3=24 -> 공위치 3, 움직이는 관절 수 12*2, 손 base 위치 3, 공속도 3
          actionDim_ = nJoints;
          actionMean_.setZero(actionDim_);
          actionStd_.setZero(actionDim_);
          obDouble_.setZero(obDim_);
          obDouble_.setZero(obDim_);

          rewards_.initializeFromConfigurationFile (cfg["reward"]);
          idx1 = gripper_->getBodyIdx("left_inner_finger");
          idx2 = gripper_->getBodyIdx("right_inner_finger");
          handIndices.insert(gripper_->getBodyIdx("left_inner_knuckle"));
          handIndices.insert(gripper_->getBodyIdx("left_inner_finger"));
          handIndices.insert(gripper_->getBodyIdx("right_inner_knuckle"));
          handIndices.insert(gripper_->getBodyIdx("right_inner_finger"));

          /// action scaling
          actionMean_ = gc_init_.tail(nJoints);
          double action_std;
          READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
          actionStd_.setConstant(action_std);
          if (visualizable_) {
            server_ = std::make_unique<raisim::RaisimServer>(world_.get());
            server_->launchServer();
            server_->focusOn(gripper_);
          }
        }
        void init() final{}
        void reset() final{
          gripper_ -> setState(gc_init_, gv_init_);
          updateObservation();
          ball->setPosition(Eigen::Vector3d(0.8,0.4,0.03));
          ball->setVelocity(0,0,0,0,0,0);
        }

        float step(const Eigen::Ref<EigenVec>& action) final{
          pTarget12_ = action.cast<double>();
          pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
          pTarget12_ += actionMean_;

          pTarget_.tail(nJoints)= pTarget12_;
          gripper_ ->setPdTarget(pTarget_, vTarget_);

          for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
            if(server_) server_->lockVisualizationServerMutex();
            world_->integrate();
            if(server_) server_->unlockVisualizationServerMutex();
          }

          updateObservation();
          //rewards_.record("torque", std::min(1.0,gripper_ ->getGeneralizedForce().squaredNorm()));
          rewards_.record("ballheight", std::max(0.02, ball->getPosition()[2]));
          //rewards_.record("balldist", std::max(0.003, baseDist_.squaredNorm()));
          rewards_.record("touches",0);
          for (auto& contact: ball->getContacts()){
            if(contact.getPairObjectIndex()==gripper_->getIndexInWorld()){
                rewards_.record("touches", 0.25, true);
            }
          }
          return rewards_.sum();
        }

        void updateObservation(){
          gripper_->getState(gc_,gv_);
          gripper_->getBodyPosition(idx1,leftPos_);
          //std::cout<<idx1<<leftPos_<<std::endl;
          gripper_->getBodyPosition(idx2, rightPos_);
          basePos_<<(leftPos_[0]+rightPos_[0])/2.0, (leftPos_[1]+rightPos_[1])/2.0,(leftPos_[2]+rightPos_[2])/2.0;
          baseDist_<<ball->getPosition()-basePos_;

          obDouble_<<ball->getPosition(),ball->getLinearVelocity(),
          gc_.tail(12),
          gv_.tail(12),
          basePos_;
          //std::cout<<gripper_->getGeneralizedForce()<<std::endl;

          //std::cout<<std::min(0.03,baseDist_.squaredNorm())

        }

        void observe(Eigen::Ref<EigenVec> ob)final {
          ob = obDouble_.cast<float>();
        }

        bool isTerminalState(float& terminalReward) final{
          terminalReward = float(terminalRewardCoeff_);
          if (ball->getPosition()[2]>0.5){
            return true;
          }
          terminalReward = 0.f;
          return false;
          //for(auto& contact: ball->getContacts())
          //  if(0 == contact.getIndexInObjectContactList())
          //      return false;
          //return true;
        }
    private:
        int gcDim_, gvDim_, gJoints, nJoints;
        bool visualizable_ = false;
        raisim::ArticulatedSystem *gripper_;
        raisim::SingleBodyObject *ball;
        Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, gTarget4_, vTarget_, pTarget12_;
        double terminalRewardCoeff_ = 10.;
        float ballrad = 0.02;
        Eigen::VectorXd actionMean_, actionStd_, obDouble_;
        Eigen::Vector3d bodyLinearVel_, bodyAngularVel_, basePos_;
        Vec<3> leftPos_, rightPos_;
        std::set<size_t> handIndices;
        size_t idx1, idx2;
        Eigen::Vector3d baseDist_, ballPos_;
        std::normal_distribution<double> normDist_;

    };
}
