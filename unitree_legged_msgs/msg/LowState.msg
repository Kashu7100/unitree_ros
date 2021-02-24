uint8 levelFlag
uint16 commVersion              # Old version Aliengo does not have
uint16 robotID                  # Old version Aliengo does not have
uint32 SN                       # Old version Aliengo does not have
uint8 bandWidth                 # Old version Aliengo does not have
IMU imu
MotorState[20] motorState
int16[4] footForce              # force sensors     # Old version Aliengo is different
int16[4] footForceEst           # force sensors     # Old version Aliengo does not have
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wirelessRemote        # wireless commands
uint32 reserve                  # Old version Aliengo does not have
uint32 crc

# Old version Aliengo does not have:
CartesianMsg[4] eeForceRaw
CartesianMsg[4] eeForce          #it's a 1-DOF force infact, but we use 3-DOF here just for visualization 
CartesianMsg position            # will delete
CartesianMsg velocity            # will delete
CartesianMsg velocity_w            # will delete
