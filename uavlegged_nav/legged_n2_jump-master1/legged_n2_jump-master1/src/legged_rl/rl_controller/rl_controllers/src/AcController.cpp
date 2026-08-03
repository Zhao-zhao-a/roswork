#include "rl_controllers/AcController.h"
#include <pluginlib/class_list_macros.hpp>
#include "rl_controllers/RotationTools.h"
#include <algorithm>
#include <random>

namespace legged
{

  void AcController::handleWalkMode()
  {
    // compute observation & actions
    if (std::cout.fail())
    {
      std::cerr << "std::cout is in a bad state!" << std::endl;
      // 可能需要清除错误状态
      std::cout.clear();
    }
    // 摔倒保护
    if (propri_.projectedGravity(2) >= -0.3)
    {
      std::cout << "摔倒保护" << std::endl;
      mode_ = Mode::DEFAULT;
    }
    if (loopCount_ % robotCfg_.controlCfg.decimation == 0)
    {
      computeObservation();
      computeActions();
      // limit action range
      scalar_t actionMin = -robotCfg_.clipActions;
      scalar_t actionMax = robotCfg_.clipActions;
      std::transform(actions_.begin(), actions_.end(), actions_.begin(),
                     [actionMin, actionMax](scalar_t x)
                     { return std::max(actionMin, std::min(actionMax, x)); });
    }


    // 初始化随机数引擎（使用硬件种子）
    // std::random_device rd;
    // std::mt19937 gen(rd());
    
    // // 定义分布：[0, 1)
    // std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    // float random_num = dis(gen);
    // set action
    for (int i = 0; i < actionsSize_; i++)
    {
      std::string partName = hybridJointHandles_[i].getName();
      // actions_[i] = random_num * actions_[i] + (1 - random_num) * lastActions_(i, 0);
      scalar_t pos_des = actions_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(i);
      double stiffness = robotCfg_.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
      double damping = robotCfg_.controlCfg.damping[partName]; // 根据关节名称获取阻尼
      // std::cout << "joint_name: " << partName << "kp:" << stiffness << " kd:" << damping << std::endl;
      hybridJointHandles_[i].setCommand(pos_des, 0, stiffness, damping, 0);
      lastActions_(i, 0) = actions_[i];
    }
  
  }

  void AcController::handleJumpMode()
  {
    // compute observation & actions
    if (std::cout.fail())
    {
      std::cerr << "std::cout is in a bad state!" << std::endl;
      // 可能需要清除错误状态
      std::cout.clear();
    }
    // 摔倒保护
    if (propri_.projectedGravity(2) >= -0.3 && jump_phase_ > 0.8)
    {
      std::cout << "摔倒保护" << std::endl;
      mode_ = Mode::DEFAULT;
    }
    if (loopCount_ % robotCfg_.controlCfg.decimation == 0)
    {
      computeObservationJump();
      computeActionsJump();
      // limit action range
      scalar_t actionMin = -robotCfg_.clipActions;
      scalar_t actionMax = robotCfg_.clipActions;
      std::transform(actionsJump_.begin(), actionsJump_.end(), actionsJump_.begin(),
                     [actionMin, actionMax](scalar_t x)
                     { return std::max(actionMin, std::min(actionMax, x)); });
    }

    // 初始化随机数引擎（使用硬件种子）
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // 定义分布：[0, 1)
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    float random_num = dis(gen);
    // set action
    for (int i = 0; i < actionsSize_; i++)
    {
      std::string partName = hybridJointHandles_[i].getName();
      actionsJump_[i] = random_num * actionsJump_[i] + (1 - random_num) * lastActionsJump_(i, 0);
      scalar_t pos_des = actionsJump_[i] * robotCfg_.controlCfg.actionScale + defaultJointAngles_(i);
      double stiffness = robotCfg_.controlCfg.stiffness[partName]; // 根据关节名称获取刚度
      double damping = robotCfg_.controlCfg.damping[partName]; // 根据关节名称获取阻尼
      // std::cout << "joint_name: " << partName << "kp:" << stiffness << " kd:" << damping << std::endl;
      hybridJointHandles_[i].setCommand(pos_des, 0, stiffness, damping, 0);
      lastActionsJump_(i, 0) = actionsJump_[i];
    }
  
  }

  bool AcController::loadModel(ros::NodeHandle &nh)
  {
    std::string policyFilePath;
    std::string policyFileJumpPath;
    
    if (!nh.getParam("/policyFile", policyFilePath))
    {
      ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
      return false;
    }
    policyFilePath_ = policyFilePath;
    ROS_INFO_STREAM("Load Walk Onnx model from path : " << policyFilePath);

    if (!nh.getParam("/policyFileJump", policyFileJumpPath))
    {
      ROS_ERROR_STREAM("Get policy path fail from param server, some error occur!");
      return false;
    }
    policyFileJumpPath_ = policyFileJumpPath;
    
    ROS_INFO_STREAM("Load Jump Onnx model from path : " << policyFileJumpPath_);

    // create env
    onnxEnvPrt_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "LeggedOnnxController"));
        
    // create session
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetInterOpNumThreads(1);
    policySessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyFilePath.c_str(), sessionOptions);
    policyJumpSessionPtr_ = std::make_unique<Ort::Session>(*onnxEnvPrt_, policyFileJumpPath.c_str(), sessionOptions);

    // get input and output info
    policyInputNames_.clear();
    policyOutputNames_.clear();
    policyJumpInputNames_.clear();
    policyJumpOutputNames_.clear();
    
    policyInputShapes_.clear();
    policyOutputShapes_.clear();
    policyJumpInputShapes_.clear();
    policyJumpOutputShapes_.clear();

    Ort::AllocatorWithDefaultOptions allocator;

    ROS_INFO_STREAM("count: " << policySessionPtr_->GetOutputCount());

    // -------------------------------- Walk --------------------------------
    for (int i = 0; i < policySessionPtr_->GetInputCount(); i++) {
      auto policyInputnamePtr = policySessionPtr_->GetInputNameAllocated(i, allocator);
      policyInputNodeNameAllocatedStrings.push_back(std::move(policyInputnamePtr));
      policyInputNames_.push_back(policyInputNodeNameAllocatedStrings.back().get());
      policyInputShapes_.push_back(policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> policyShape = policySessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      std::cerr << "Policy Shape: [";
      for (size_t j = 0; j < policyShape.size(); ++j)
      {
          std::cout << policyShape[j];
          if (j != policyShape.size() - 1)
          {
              std::cerr << ", ";
          }
      }
      std::cout << "]" << std::endl;
    }

    for (int i = 0; i < policySessionPtr_->GetOutputCount(); i++)
    {
      auto policyOutputnamePtr = policySessionPtr_->GetOutputNameAllocated(i, allocator);
      policyOutputNodeNameAllocatedStrings.push_back(std::move(policyOutputnamePtr));
      policyOutputNames_.push_back(policyOutputNodeNameAllocatedStrings.back().get());
      std::cout << policySessionPtr_->GetOutputNameAllocated(i, allocator).get() << std::endl;
      policyOutputShapes_.push_back(policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> policyShape = policySessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    }

    // -------------------------------- Jump --------------------------------
    for (int i = 0; i < policyJumpSessionPtr_->GetInputCount(); i++) {
      auto policyJumpInputnamePtr = policyJumpSessionPtr_->GetInputNameAllocated(i, allocator);
      policyJumpInputNodeNameAllocatedStrings.push_back(std::move(policyJumpInputnamePtr));
      policyJumpInputNames_.push_back(policyJumpInputNodeNameAllocatedStrings.back().get());
      policyJumpInputShapes_.push_back(policyJumpSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> policyJumpShape = policyJumpSessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
      std::cerr << "policy Shape: [";
      for (size_t j = 0; j < policyJumpShape.size(); ++j)
      {
          std::cout << policyJumpShape[j];
          if (j != policyJumpShape.size() - 1)
          {
              std::cerr << ", ";
          }
      }
      std::cout << "]" << std::endl;
    }

    for (int i = 0; i < policyJumpSessionPtr_->GetOutputCount(); i++)
    {
      auto policyJumpOutputnamePtr = policyJumpSessionPtr_->GetOutputNameAllocated(i, allocator);
      policyJumpOutputNodeNameAllocatedStrings.push_back(std::move(policyJumpOutputnamePtr));
      policyJumpOutputNames_.push_back(policyJumpOutputNodeNameAllocatedStrings.back().get());
      std::cout << policyJumpSessionPtr_->GetOutputNameAllocated(i, allocator).get() << std::endl;
      policyJumpOutputShapes_.push_back(policyJumpSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
      std::vector<int64_t> policyJumpShape = policyJumpSessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    }

    ROS_INFO_STREAM("Load Onnx model successfully !!!");
    return true;
  }

  bool AcController::loadRLCfg(ros::NodeHandle &nh)
  {
    RLRobotCfg::InitState &initState = robotCfg_.initState;
    RLRobotCfg::ControlCfg &controlCfg = robotCfg_.controlCfg;
    RLRobotCfg::ObsScales &obsScales = robotCfg_.obsScales;

    int error = 0;

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l1_joint", initState.leg_l1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l2_joint", initState.leg_l2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l3_joint", initState.leg_l3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l4_joint", initState.leg_l4_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_l5_joint", initState.leg_l5_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r1_joint", initState.leg_r1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r2_joint", initState.leg_r2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r3_joint", initState.leg_r3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r4_joint", initState.leg_r4_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/leg_r5_joint", initState.leg_r5_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_l1_joint", initState.arm_l1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_l2_joint", initState.arm_l2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_l3_joint", initState.arm_l3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_l4_joint", initState.arm_l4_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_r1_joint", initState.arm_r1_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_r2_joint", initState.arm_r2_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_r3_joint", initState.arm_r3_joint));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/init_state/default_joint_angle/arm_r4_joint", initState.arm_r4_joint));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/stiffness", controlCfg.stiffness));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/damping", controlCfg.damping));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/action_scale", controlCfg.actionScale));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/decimation", controlCfg.decimation));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/control/cycle_time", controlCfg.cycle_time));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_observations", robotCfg_.clipObs));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/clip_scales/clip_actions", robotCfg_.clipActions));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/lin_vel", obsScales.linVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/ang_vel", obsScales.angVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_pos", obsScales.dofPos));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/dof_vel", obsScales.dofVel));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/height_measurements", obsScales.heightMeasurements));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/normalization/obs_scales/quat", obsScales.quat));

    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/actions_size", actionsSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size", observationSize_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/observations_size_Jump", observationSizeJump_));
    error += static_cast<int>(!nh.getParam("/LeggedRobotCfg/size/stack_size", stackSize_));

    actions_.resize(actionsSize_);
    actionsJump_.resize(actionsSize_);
    policyObservations_.resize(observationSize_ * stackSize_);
    policyObservationsJump_.resize(observationSizeJump_ * stackSize_);

    std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
    std::fill(policyObservationsJump_.begin(), policyObservationsJump_.end(), 0.0f);

    command_.x = 0;
    command_.y = 0;
    command_.yaw = 0;
    baseLinVel_.setZero();
    basePosition_.setZero();
    std::vector<scalar_t> defaultJointAngles{
        robotCfg_.initState.arm_l1_joint, robotCfg_.initState.arm_l2_joint, robotCfg_.initState.arm_l3_joint, robotCfg_.initState.arm_l4_joint, 
        robotCfg_.initState.leg_l1_joint, robotCfg_.initState.leg_l2_joint, robotCfg_.initState.leg_l3_joint,
        robotCfg_.initState.leg_l4_joint, robotCfg_.initState.leg_l5_joint,
        robotCfg_.initState.arm_r1_joint, robotCfg_.initState.arm_r2_joint, robotCfg_.initState.arm_r3_joint, robotCfg_.initState.arm_r4_joint,
        robotCfg_.initState.leg_r1_joint, robotCfg_.initState.leg_r2_joint, robotCfg_.initState.leg_r3_joint,
        robotCfg_.initState.leg_r4_joint, robotCfg_.initState.leg_r5_joint
      };
    
    lastActions_.resize(actionsSize_);
    lastActions_.setZero();

    lastActionsJump_.resize(actionsSize_);
    lastActionsJump_.setZero();

    const int inputSize = observationSize_ * stackSize_;
    proprioHistoryBuffer_.resize(inputSize);

    const int inputSizeJump = observationSizeJump_ * stackSize_;
    proprioHistoryBufferJump_.resize(inputSizeJump);

    defaultJointAngles_.resize(actionsSize_);
    for (int i = 0; i < actionsSize_; i++)
    {
      defaultJointAngles_(i) = defaultJointAngles[i];
    }

    return (error == 0);
  }

  void AcController::computeActions()
  {

    std::vector<Ort::Value> policyInputValues;
    policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, policyObservations_.data(), policyObservations_.size(),
                                                                         policyInputShapes_[0].data(), policyInputShapes_[0].size()));
    // run inference
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues;
    outputValues = policySessionPtr_->Run(runOptions, policyInputNames_.data(), policyInputValues.data(), 1, policyOutputNames_.data(), 1);

    if (isfirstCompAct_){
      // for (int i = 0; i < policyObservations_.size(); ++i) {
      //   std::cout << policyObservations_[i] << " ";
      //   if ((i + 1) % observationSize_ == 0) {
      //       std::cout << std::endl;
      //   }
      // }
      isfirstCompAct_ = false;
      isfirstCompActJump_ = true;
    }

    for (int i = 0; i < actionsSize_; i++)
    {
      actions_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }

  }

  void AcController::computeActionsJump()
  {

    std::vector<Ort::Value> policyInputValues;
    policyInputValues.push_back(Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, policyObservationsJump_.data(), policyObservationsJump_.size(),
                                                                         policyJumpInputShapes_[0].data(), policyJumpInputShapes_[0].size()));
    // run inference
    Ort::RunOptions runOptions;
    std::vector<Ort::Value> outputValues;
    outputValues = policyJumpSessionPtr_->Run(runOptions, policyJumpInputNames_.data(), policyInputValues.data(), 1, policyJumpOutputNames_.data(), 1);

    if (isfirstCompActJump_){
      for (int i = 0; i < policyObservationsJump_.size(); ++i) {
        std::cout << policyObservationsJump_[i] << " ";
        if ((i + 1) % observationSizeJump_ == 0) {
            std::cout << std::endl;
        }
      }
      isfirstCompAct_ = true;
      isfirstCompActJump_ = false;
    }

    for (int i = 0; i < actionsSize_; i++)
    {
      actionsJump_[i] = *(outputValues[0].GetTensorMutableData<tensor_element_t>() + i);
    }

  }

  void AcController::computeObservation()
  {
    RLRobotCfg::ObsScales &obsScales = robotCfg_.obsScales;
    // command
    vector_t command(3);

    if (abs(command_.x) < 0.4) command_.x = 0.0;
    if (abs(command_.y) < 0.4) command_.y = 0.0;
    if (abs(command_.yaw) < 0.4) command_.yaw = 0.0;

    if (command_.x < -0.8) command_.x = -0.8;

    command[0] = command_.x * obsScales.linVel;
    command[1] = command_.y * obsScales.linVel;
    command[2] = command_.yaw * obsScales.angVel;

    // actions
    vector_t actions(lastActions_);

    matrix_t commandScaler = Eigen::DiagonalMatrix<scalar_t, 3>(obsScales.linVel, obsScales.linVel, obsScales.angVel);

    vector_t proprioObs(observationSize_);

    proprioObs << command, // 3
        propri_.baseAngVel * obsScales.angVel,  // 3
        propri_.projectedGravity(0) * obsScales.quat,  // 1
        propri_.projectedGravity(1) * obsScales.quat,  // 1
        propri_.projectedGravity(2) * obsScales.quat,  // 1
        (propri_.jointPos - defaultJointAngles_) * obsScales.dofPos,  // 18
        propri_.jointVel * obsScales.dofVel,  // 18
        actions;  // 18


    if (isfirstRecObs_)
    {
      for (
         int i = observationSize_ - actionsSize_; i < observationSize_; i++)
      {
        proprioObs(i,0) = 0.0;
      }

       for (size_t i = 0; i < stackSize_; i++)
      {
        proprioHistoryBuffer_.segment(i * observationSize_, observationSize_) = proprioObs.cast<tensor_element_t>();
      }
      isfirstRecObs_ = false;
      std::cout <<"isfirstRecObs_" << isfirstRecObs_<<std::endl;

      std::fill(policyObservations_.begin(), policyObservations_.end(), 0.0f);
      }

    proprioHistoryBuffer_.head(proprioHistoryBuffer_.size() - observationSize_) =
        proprioHistoryBuffer_.tail(proprioHistoryBuffer_.size() - observationSize_);
    proprioHistoryBuffer_.tail(observationSize_) = proprioObs.cast<tensor_element_t>();

    for (size_t i = 0; i < (observationSize_ * stackSize_); i++){
      policyObservations_[i] = static_cast<tensor_element_t>(proprioHistoryBuffer_[i]);
    }

    scalar_t obsMin = -robotCfg_.clipObs;
    scalar_t obsMax = robotCfg_.clipObs;
    std::transform(policyObservations_.begin(), policyObservations_.end(), policyObservations_.begin(),
                   [obsMin, obsMax](scalar_t x)
                   { return std::max(obsMin, std::min(obsMax, x)); });
  }

  void AcController::computeObservationJump()
  {
    RLRobotCfg::ObsScales &obsScales = robotCfg_.obsScales;
    // command
    vector_t command(2);
    if (jump_toggle) jump_phase_ = (phase_ - record_phase) / robotCfg_.controlCfg.cycle_time;
    else jump_phase_ = 0;
    if (jump_phase_ >= 0.99) 
    {
      jump_toggle = false;
      mode_ = Mode::WALK;
      isfirstRecObs_ = true;
    }

    command[0] = sin(2 * M_PI * jump_phase_);
    command[1] = cos(2 * M_PI * jump_phase_);

    // actions
    vector_t actionsJump(lastActionsJump_);
    vector_t proprioObs(observationSizeJump_);

    proprioObs << command, // 2
        propri_.baseAngVel * obsScales.angVel,  // 3
        propri_.projectedGravity(0) * obsScales.quat,  // 1
        propri_.projectedGravity(1) * obsScales.quat,  // 1
        propri_.projectedGravity(2) * obsScales.quat,  // 1
        (propri_.jointPos - defaultJointAngles_) * obsScales.dofPos,  // 18
        propri_.jointVel * obsScales.dofVel,  // 18
        actionsJump;  // 18


    if (isfirstRecObsJump_)
    {
      for (
         int i = observationSizeJump_ - actionsSize_; i < observationSizeJump_; i++)
      {
        proprioObs(i,0) = 0.0;
      }

       for (size_t i = 0; i < stackSize_; i++)
      {
        proprioHistoryBufferJump_.segment(i * observationSizeJump_, observationSizeJump_) = proprioObs.cast<tensor_element_t>();
      }
      isfirstRecObsJump_ = false;
      std::cout <<"isfirstRecObsJump_" << isfirstRecObsJump_ <<std::endl;

      std::fill(policyObservationsJump_.begin(), policyObservationsJump_.end(), 0.0f);
      }

    proprioHistoryBufferJump_.head(proprioHistoryBufferJump_.size() - observationSizeJump_) =
        proprioHistoryBufferJump_.tail(proprioHistoryBufferJump_.size() - observationSizeJump_);
    proprioHistoryBufferJump_.tail(observationSizeJump_) = proprioObs.cast<tensor_element_t>();

    for (size_t i = 0; i < (observationSizeJump_ * stackSize_); i++){
      policyObservationsJump_[i] = static_cast<tensor_element_t>(proprioHistoryBufferJump_[i]);
    }

    scalar_t obsMin = -robotCfg_.clipObs;
    scalar_t obsMax = robotCfg_.clipObs;
    std::transform(policyObservationsJump_.begin(), policyObservationsJump_.end(), policyObservationsJump_.begin(),
                   [obsMin, obsMax](scalar_t x)
                   { return std::max(obsMin, std::min(obsMax, x)); });
  }

} // namespace legged


PLUGINLIB_EXPORT_CLASS(legged::AcController, controller_interface::ControllerBase)
