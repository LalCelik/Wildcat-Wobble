PROJECT_NAME = $(shell basename "$(realpath ./)")

# Configurations
NRF_IC = nrf52833
SDK_VERSION = 16
SOFTDEVICE_MODEL = blank

# Source and header files
APP_HEADER_PATHS += . ./driver ./estimation
APP_SOURCE_PATHS += . ./driver ./estimation
APP_SOURCES = $(notdir $(wildcard ./*.c ./driver/*.c ./estimation/*.c))

# Path to base of nRF52x-base repo
NRF_BASE_DIR = ../../software/nrf52x-base/

# Include board Makefile (if any)
#include ../../../software/boards/microbit_v2/Board.mk
include ../../software/boards/microbit_v2/Board.mk

# Include main Makefile
include $(NRF_BASE_DIR)/make/AppMakefile.mk
