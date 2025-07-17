#!/usr/bin/env bash
set -ex

DESIGN_RTL_DIR=3rdparty/cv32e40p/rtl

sv2v -D SYNTHESIS -D PULP_FPGA_EMUL \
	--top cv32e40p_top \
	-w amaranth_cv32e40p/verilog/cv32e40p_conv_sv2v.v \
	-I ${DESIGN_RTL_DIR}/include \
	-I ${DESIGN_RTL_DIR}/../bhv \
	-I ${DESIGN_RTL_DIR}/../bhv/include \
	-I ${DESIGN_RTL_DIR}/../sva \
	${DESIGN_RTL_DIR}/include/cv32e40p_apu_core_pkg.sv \
	${DESIGN_RTL_DIR}/include/cv32e40p_fpu_pkg.sv \
	${DESIGN_RTL_DIR}/include/cv32e40p_pkg.sv \
	${DESIGN_RTL_DIR}/cv32e40p_if_stage.sv \
	${DESIGN_RTL_DIR}/cv32e40p_cs_registers.sv \
	${DESIGN_RTL_DIR}/cv32e40p_register_file_ff.sv \
	${DESIGN_RTL_DIR}/cv32e40p_load_store_unit.sv \
	${DESIGN_RTL_DIR}/cv32e40p_id_stage.sv \
	${DESIGN_RTL_DIR}/cv32e40p_aligner.sv \
	${DESIGN_RTL_DIR}/cv32e40p_decoder.sv \
	${DESIGN_RTL_DIR}/cv32e40p_compressed_decoder.sv \
	${DESIGN_RTL_DIR}/cv32e40p_fifo.sv \
	${DESIGN_RTL_DIR}/cv32e40p_prefetch_buffer.sv \
	${DESIGN_RTL_DIR}/cv32e40p_hwloop_regs.sv \
	${DESIGN_RTL_DIR}/cv32e40p_mult.sv \
	${DESIGN_RTL_DIR}/cv32e40p_int_controller.sv \
	${DESIGN_RTL_DIR}/cv32e40p_ex_stage.sv \
	${DESIGN_RTL_DIR}/cv32e40p_alu_div.sv \
	${DESIGN_RTL_DIR}/cv32e40p_alu.sv \
	${DESIGN_RTL_DIR}/cv32e40p_ff_one.sv \
	${DESIGN_RTL_DIR}/cv32e40p_popcnt.sv \
	${DESIGN_RTL_DIR}/cv32e40p_apu_disp.sv \
	${DESIGN_RTL_DIR}/cv32e40p_controller.sv \
	${DESIGN_RTL_DIR}/cv32e40p_obi_interface.sv \
	${DESIGN_RTL_DIR}/cv32e40p_prefetch_controller.sv \
	${DESIGN_RTL_DIR}/cv32e40p_sleep_unit.sv \
	${DESIGN_RTL_DIR}/cv32e40p_core.sv \
	harness/dummy_clock_gate.v \
	${DESIGN_RTL_DIR}/cv32e40p_top.sv
