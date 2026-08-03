#pragma once

#include "rl_controllers/RLControllerBase.h"

namespace legged
{

  class AcController : public RLControllerBase
  {
    using tensor_element_t = float;

  public:
    AcController() : memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)){}

    ~AcController() override = default;

  protected:
    bool loadModel(ros::NodeHandle &nh) override;
    bool loadRLCfg(ros::NodeHandle &nh) override;
    void computeActions() override;
    void computeActionsJump() override;
    void computeObservation() override;
    void computeObservationJump() override;
    void handleWalkMode() override;
    void handleJumpMode() override;

  private:
    // onnx policy model
    std::string policyFilePath_;
    std::string policyFileJumpPath_;
    
    std::shared_ptr<Ort::Env> onnxEnvPrt_;
    std::unique_ptr<Ort::Session> policySessionPtr_;
    std::unique_ptr<Ort::Session> policyJumpSessionPtr_;
  
    std::vector<const char *> policyInputNames_;
    std::vector<const char *> policyOutputNames_;
    std::vector<const char *> policyJumpInputNames_;
    std::vector<const char *> policyJumpOutputNames_;
    
    std::vector<Ort::AllocatedStringPtr> policyInputNodeNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> policyOutputNodeNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> policyJumpInputNodeNameAllocatedStrings;
    std::vector<Ort::AllocatedStringPtr> policyJumpOutputNodeNameAllocatedStrings;
    
    std::vector<std::vector<int64_t>> policyInputShapes_;
    std::vector<std::vector<int64_t>> policyOutputShapes_;
    std::vector<std::vector<int64_t>> policyJumpInputShapes_;
    std::vector<std::vector<int64_t>> policyJumpOutputShapes_;

    vector3_t baseLinVel_;
    vector3_t basePosition_;
    vector_t lastActions_;
    vector_t lastActionsJump_;
    vector_t defaultJointAngles_;

    int actionsSize_;
    int observationSize_;
    int observationSizeJump_;
    int stackSize_;
    std::vector<tensor_element_t> actions_;
    std::vector<tensor_element_t> actionsJump_;
    std::vector<tensor_element_t> policyObservations_;
    std::vector<tensor_element_t> policyObservationsJump_;

    Ort::MemoryInfo memoryInfo;
    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBuffer_;
    Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> proprioHistoryBufferJump_;
    
    bool isfirstCompAct_{true};
    bool isfirstCompActJump_{true};
  };

} // namespace legged