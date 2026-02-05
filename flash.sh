#!/bin/bash
#
# ESPBatterySense Flash Script
# Compiles once, then flashes to multiple ESP8266 devices
#

SKETCH_PATH="./Arduino Code/ESPBatterySense/ESPBatterySense"
BOARD="esp8266:esp8266:generic"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================"
echo "  ESPBatterySense Flash Tool"
echo "========================================"
echo ""

# Compile first
echo -e "${YELLOW}Compiling firmware...${NC}"
arduino-cli compile --fqbn "$BOARD" "$SKETCH_PATH"

if [ $? -ne 0 ]; then
    echo -e "${RED}Compilation failed!${NC}"
    exit 1
fi

echo -e "${GREEN}Compilation successful!${NC}"
echo ""

# Flash loop
DEVICE_COUNT=0

while true; do
    echo "========================================"
    echo "  Ready to flash device #$((DEVICE_COUNT + 1))"
    echo "========================================"
    echo ""
    echo "1. Connect the ESP8266 to USB"
    echo "2. Put it in flash mode (GPIO0 to GND, then reset)"
    echo "3. Press ENTER when ready (or 'q' to quit)"
    echo ""

    read -r input

    if [ "$input" = "q" ] || [ "$input" = "Q" ]; then
        echo ""
        echo -e "${GREEN}Done! Flashed $DEVICE_COUNT device(s).${NC}"
        exit 0
    fi

    # Find USB serial ports (exclude Bluetooth)
    echo -e "${YELLOW}Detecting serial ports...${NC}"
    PORTS=$(ls /dev/cu.usbserial* 2>/dev/null)

    if [ -z "$PORTS" ]; then
        echo -e "${RED}No USB serial ports found! Is the device connected?${NC}"
        echo ""
        continue
    fi

    # Count ports
    PORT_COUNT=$(echo "$PORTS" | wc -l | tr -d ' ')

    if [ "$PORT_COUNT" -eq 1 ]; then
        PORT="$PORTS"
        echo "Found port: $PORT"
    else
        echo "Multiple ports found:"
        echo "$PORTS" | nl
        echo ""
        echo "Enter port number (or full path):"
        read -r port_selection

        if [[ "$port_selection" =~ ^[0-9]+$ ]]; then
            PORT=$(echo "$PORTS" | sed -n "${port_selection}p")
        else
            PORT="$port_selection"
        fi
    fi

    if [ -z "$PORT" ]; then
        echo -e "${RED}Invalid port selection${NC}"
        continue
    fi

    echo ""
    echo -e "${YELLOW}Flashing to $PORT...${NC}"
    echo ""

    arduino-cli upload -p "$PORT" --fqbn "$BOARD" "$SKETCH_PATH"

    if [ $? -eq 0 ]; then
        DEVICE_COUNT=$((DEVICE_COUNT + 1))
        echo ""
        echo -e "${GREEN}SUCCESS! Device #$DEVICE_COUNT flashed.${NC}"
        echo -e "${YELLOW}Power cycle the device to start it.${NC}"
    else
        echo ""
        echo -e "${RED}Flash failed! Check connections and try again.${NC}"
    fi

    echo ""
done
