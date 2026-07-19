#!/bin/bash
# Must be run with sudo

# Array of hardware addresses: Pin 15, Pin 32, Pin 33
ADDRESSES=("3280000" "32e0000" "32c0000")
CLOCKS=("pwm1" "pwm7" "pwm5")

for i in ${!ADDRESSES[@]}; do
    ADDR=${ADDRESSES[$i]}
    CLK=${CLOCKS[$i]}
    
    # 1. Lower parent clock for 50Hz support
    echo 2000000 > /sys/kernel/debug/bpmp/debug/clk/$CLK/rate
    
    # 2. Find chip and export
    CHIP=$(ls -l /sys/class/pwm/ | grep "$ADDR" | awk '{print $9}')
    if [ ! -d "/sys/class/pwm/$CHIP/pwm0" ]; then
        echo 0 > /sys/class/pwm/$CHIP/export
    fi
    
    # 3. Set 20ms period, center duty cycle, and enable
    echo 20000000 > /sys/class/pwm/$CHIP/pwm0/period
    echo 1500000 > /sys/class/pwm/$CHIP/pwm0/duty_cycle
    echo 1 > /sys/class/pwm/$CHIP/pwm0/enable
    
    # 4. Grant write permissions to all users for ROS 2 access
    chmod 666 /sys/class/pwm/$CHIP/pwm0/duty_cycle
done

echo "Hardware PWM configured and permissions unlocked for ROS 2."