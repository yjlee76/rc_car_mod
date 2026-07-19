#!/bin/bash
# Must be run with sudo

# Array of hardware addresses: Pin 15, Pin 32, Pin 33
ADDRESSES=("3280000" "32e0000" "32c0000")

for ADDR in "${ADDRESSES[@]}"; do
    # Find the corresponding pwmchip
    CHIP=$(ls -l /sys/class/pwm/ | grep "$ADDR" | awk '{print $9}')
    
    # Check if the chip was found and if the pwm0 directory is currently exported
    if [ ! -z "$CHIP" ] && [ -d "/sys/class/pwm/$CHIP/pwm0" ]; then
        echo "Shutting down $CHIP (Hardware Address: $ADDR)..."
        
        # 1. Disable the PWM signal
        echo 0 > /sys/class/pwm/$CHIP/pwm0/enable
        
        # 2. Unexport the pin to remove the pwm0 directory and release it
        echo 0 > /sys/class/pwm/$CHIP/unexport
    else
        echo "Hardware address $ADDR is already unexported or not active."
    fi
done

echo "All PWM signals disabled and hardware pins released."