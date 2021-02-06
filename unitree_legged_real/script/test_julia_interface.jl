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
    function MotorState() 
        new(0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,zeros(SVector{2,UInt32}))
    end
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

  mutable struct LowState
    levelFlag::UInt8
    commVersion::UInt16
    robotID::UInt16
    SN::UInt32
    bandWidth::UInt8
    imu::IMU
    motorState::SVector{20,MotorState}
    footForce::SVector{4,Int16}
    footForceEst::SVector{4,Int16}
    tick::UInt32
    wirelessRemote::SVector{40,UInt8}
    reserve::UInt32
    crc::UInt32
    function LowState() 
        new(0,0,0,0,0,IMU(),@SVector[MotorState() for i in 1:20], zeros(SVector{4,Int16}), zeros(SVector{4,Int16}),0, zeros(SVector{40,UInt8}),0,0)
    end
  end

  mutable struct LowCmd
    levelFlag::UInt8
    commVersion::UInt16
    robotID::UInt16
    SN::UInt32
    bandWidth::UInt8
    motorCmd::SVector{20,MotorCmd}
    led::SVector{4,LED}
    wirelessRemote::SVector{40,UInt8}
    reserve::UInt32
    crc::UInt32
    function LowCmd() 
        new(0,0,0,0,0,@SVector[MotorCmd() for i in 1:20], @SVector[LED() for i in 1:4], zeros(SVector{40,UInt8}),0,0)
    end
  end

  @wrapmodule("/home/biorobotics/ros_workspaces/unitree_ws/build/unitree_legged_real/lib/libjulia_a1_interface.so")

  function __init__()
    @initcxx
  end
  
end



@show A1Robot.doc()
A1Robot.doc()
b = A1Robot.RobotInterface()
# @show d = A1Robot.ReceiveObservation(b)
# c = A1Robot.IMU()

while true
    sleep(0.01)
    @show A1Robot.getFootForce(b,0)
end