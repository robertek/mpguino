### PATHS
PROJECT_DIR 		= .
ARDMK_DIR 		= $(PROJECT_DIR)/Arduino-Makefile
AVR_TOOLS_DIR 		= /usr
AVRDDUDE 		= /usr/bin/avrdude

### BOARD
BOARD_TAG 		= atmega328

### CFLAGS
CFLAGS_STD 		= -std=c11
CXXFLAGS_STD 		= -std=c++11

### MONITOR
MONITOR_BAUDRATE 	= 115200
MONITOR_PORT 		= /dev/tty.usbmodem*

### path to Arduino.mk
include $(ARDMK_DIR)/Arduino.mk

