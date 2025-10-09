# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use
# The quaternion uses the Hamilton convention, and the order is q(w, x, y, z)

uint64 timestamp                # time since system start (microseconds)
float32[4] q                    # Quaternion rotation from the FRD body frame to the NED earth frame

