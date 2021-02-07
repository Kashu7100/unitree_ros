module A1Robot
  using CxxWrap
  using StaticArrays

  # define datastructure
  # /home/biorobotics/Documents/unitree_legged_sdk/include/unitree_legged_sdk/comm.h
  mutable struct Cartesian
    x::Float32
    y::Float32
    z::Float32
    Cartesian() = new()
  end

  mutable struct IMU
    quaternion::SVector{3,Float32}
    gyroscope::SVector{3,Float32}
    accelerometer::SVector{3,Float32}
    rpy::SVector{3,Float32}
    temperature::Int8
    IMU() = new()
  end

  mutable struct LED
    r::UInt8
    g::UInt8
    b::UInt8
    function LED() 
        new(0,0,0)
    end
  end
  mutable struct MotorState
    mode::UInt8
    q::Float32
    dq::Float32
    ddq::Float32
    tauEst::Float32
    q_raw::Float32
    dq_raw::Float32
    ddq_raw::Float32
    temperature::Int8
    reserve::SVector{2,UInt32}
    MotorState() = new()
  end

  mutable struct MotorCmd
    mode::UInt8
    q::Float32
    dq::Float32
    tau::Float32
    Kp::Float32
    Kd::Float32
    reserve::SVector{3,UInt32}
    function MotorCmd() 
        new(0,0.0,0.0,0.0,0.0,0.0,zeros(SVector{3,UInt32}))
    end
  end

  # this is not identical to unitree_sdk's datastructure 
  mutable struct LowState
    # levelFlag::UInt8
    # commVersion::UInt16
    # robotID::UInt16
    # SN::UInt32
    # bandWidth::UInt8
    imu::IMU
    motorState::Array{MotorState,1}
    footForce::Array{Int16,1}
    # footForceEst::SVector{4,Int16}
    # tick::UInt32
    # wirelessRemote::SVector{40,UInt8}
    # reserve::UInt32
    # crc::UInt32
    function LowState() 
        # new(0,0,0,0,0,IMU(),@SVector[MotorState() for i in 1:20], zeros(SVector{4,Int16}), zeros(SVector{4,Int16}),0, zeros(SVector{40,UInt8}),0,0)
        new(IMU(), [MotorState() for i in 1:12], zeros(Int16,4))
      end
  end

  # mutable struct LowCmd
  #   levelFlag::UInt8
  #   commVersion::UInt16
  #   robotID::UInt16
  #   SN::UInt32
  #   bandWidth::UInt8
  #   motorCmd::SVector{20,MotorCmd}
  #   led::SVector{4,LED}
  #   wirelessRemote::SVector{40,UInt8}
  #   reserve::UInt32
  #   crc::UInt32
  #   function LowCmd() 
  #       new(0,0,0,0,0,@SVector[MotorCmd() for i in 1:20], @SVector[LED() for i in 1:4], zeros(SVector{40,UInt8}),0,0)
  #   end
  # end

  # C_ means these indices are for C++
  const C_FR_ = 0
  const C_FL_ = 1
  const C_RR_ = 2
  const C_RL_ = 3

  const C_FR_0 = 0;     
  const C_FR_1 = 1;      
  const C_FR_2 = 2;

  const C_FL_0 = 3;
  const C_FL_1 = 4;
  const C_FL_2 = 5;

  const C_RR_0 = 6;
  const C_RR_1 = 7;
  const C_RR_2 = 8;

  const C_RL_0 = 9;
  const C_RL_1 = 10;
  const C_RL_2 = 11;

  @wrapmodule("/home/biorobotics/ros_workspaces/unitree_ws/build/unitree_legged_real/lib/libjulia_a1_interface.so")

  function __init__()
    @initcxx
  end
  export LowState
end



@show A1Robot.doc()
A1Robot.doc()
b = A1Robot.RobotInterface()
# must intialize the robot 
A1Robot.InitSend(b)
fbk_state = A1Robot.LowState()
# @show d = A1Robot.ReceiveObservation(b)
# c = A1Robot.IMU()

while true
    sleep(0.01)
    # there is actually an index mismatch
    # @show A1Robot.getFootForce(b,A1Robot.FR_)
    fbk_state.footForce[1]=A1Robot.getFootForce(b,A1Robot.C_FR_)
    fbk_state.footForce[2]=A1Robot.getFootForce(b,A1Robot.C_FL_)
    fbk_state.footForce[3]=A1Robot.getFootForce(b,A1Robot.C_RR_)
    fbk_state.footForce[4]=A1Robot.getFootForce(b,A1Robot.C_RL_)

    # k = zeros(Float32,12)
    for i=1:12
      # k[i] = A1Robot.getMotorStateQ(b,i-1)
      fbk_state.motorState[i].q  = A1Robot.getMotorStateQ(b,i-1)
      fbk_state.motorState[i].dq  = A1Robot.getMotorStateDQ(b,i-1)
      fbk_state.motorState[i].ddq  = A1Robot.getMotorStateDDQ(b,i-1)
      fbk_state.motorState[i].tauEst  = A1Robot.getMotorStateTau(b,i-1)
    end
    fbk_state.imu = A1Robot.getIMU(b)

    # now fbk_state are filled with most of the sensor data from the robot
    @show fbk_state.imu.rpy
end