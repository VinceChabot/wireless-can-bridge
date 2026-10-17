# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)


# The Seeed XIAO nRF54L15 board definition is not part of NCS v3.1.0; it is kept in
# boards/seeed/xiao_nrf54l15 so the project builds without any extra -DBOARD_ROOT.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Bridge)

target_sources(app PRIVATE
	src/main.c
	src/can_communication.c
	src/esb_communication.c
	src/esb_link.c
	src/esb_afh.c
	src/esb_pairing.c
	src/esb_ui.c
)

target_include_directories(app PRIVATE include)
