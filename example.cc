#include <stdlib.h>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "../raisimGymTorch/raisimGymTorch/env/RaisimGymEnv.hpp"
namespace raisim {
    class ENVIRONMENT : public RaisimGymEnv {
    public:
        explicit ENVIRONMENT(const std::string &resourceDir, const Yaml::Node &cfg, bool visualizable) :
                RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) {
          raisim::World world;

          //add objects
          gripper_ = world.addArticulatedSystem(resourceDir + "/ur5_description/urdf/ur5_no2.urdf");
          gripper_->setName("mygripper");
          gripper_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
          auto ball = world.addSphere(0.1, 1);
          ball->setPosition(Eigen::Vector3d(0,-5,0));
          auto ground = world.addGround();
          ball->setPosition(Eigen::Vector3d(0,-3.0,0));
          gcDim_ = gripper_->getGeneralizedCoordinateDim();
          gvDim_ = gripper_->getDOF();
          gJoints = 4;
          nJoints = gvDim_ - 2;
          //joint 개수 11  gcDim 11 -> 로봇팔 gc 5 gv5 로봇 손 gc 6. gv 6
          //index 6,7,8,9가 로봇 손의 관절에 해당하는 부분임 6,7이 left inner knuckle, inner finger 8,9가 right inner knuckle, inner finger
          gc_.setZero(gcDim_);
          gc_init_.setZero(gcDim_);
          gv_.setZero(gvDim_);
          gv_init_.setZero(gvDim_);
          pTarget_.setZero(gcDim_);
          vTarget_.setZero(gvDim_);
          gTarget4_.setZero(gJoints);
          gc_init_ << 0, 0, 0, 0, 0, 0.0055, 0.0062, 0.0005, -0.0012, 0.0005, 0.0054;

          Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
          jointPgain.setZero();
          jointPgain.setConstant(50.0);
          jointDgain.setZero();
          jointDgain.setConstant(0.2);
          gripper_->setPdGains(jointPgain, jointDgain);
          gripper_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_)); // 팔과 그리퍼의 p d gain을 동일하게 해
          obDim_ = 20;
          actionDim_ = nJoints;
          actionMean_.setZero(actionDim_);
          actionStd_.setZero(actionDim_);
          obDouble_.setZero(obDim_);
          obDouble_.setZero(obDim_);

          handIndices.insert(gripper_->getBodyIdx("robotiq_85_base_link"));
          handIndices.insert(gripper_->getBodyIdx("left_inner_knuckle"));
          handIndices.insert(gripper_->getBodyIdx("left_inner_finger"));
          handIndices.insert(gripper_->getBodyIdx("right_inner_knuckle"));
          handIndices.insert(gripper_->getBodyIdx("right_inner_finger"));

          /// action scaling
          //actionMean_ = gc_init_.tail(nJoints_);
          //double action_std;
          //READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
          //actionStd_.setConstant(action_std);
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
        }

        float set(const Eigen::Ref<EigenVec>& action) final{
          pTarget_ = action.cast<double>();
          pTarget_ = pTarget_.cwiseProduct(actionStd_);
          pTarget_ += actionMean_;
          gripper_ ->setPdTarget(pTarget_, vTarget_);

          for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
            if(server_) server_->lockVisualizationServerMutex();
            world_->integrate();
            if(server_) server_->unlockVisualizationServerMutex();
          }

          updateObservation();
          rewards_.record("torque", gripper_ ->getGeneralizedForce().squaredNorm());

          return rewards_.sum();
        }

        void updateObservation(){
          gripper_->getState(gc_,gv_);
          //raisim::Vect
        }

        void observe(Eigen::Ref<EigenVec> ob)final {
          ob = obDouble_.cast<float>();
        }

        bool isTerminalState(float& terminalReward) final{
          terminalReward = float(terminalRewardCoeff_);

          for(auto& contact: gripper_->getContacts())
            if()
        }
    private:
        int gcDim_, gvDim_, gJoints, nJoints;
        bool visualizable_ = false;
        raisim::ArticulatedSystem *gripper_;
        Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, gTarget4_, vTarget_;
        double terminalRewardCoeff_ = -10.;
        Eigen::VectorXd actionMean_, actionStd_, obDouble_;
        Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
        std::set<size_t> handIndices;
    };
}
