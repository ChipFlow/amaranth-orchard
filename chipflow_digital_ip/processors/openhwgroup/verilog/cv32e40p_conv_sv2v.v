module cv32e40p_if_stage (
	clk,
	rst_n,
	m_trap_base_addr_i,
	u_trap_base_addr_i,
	trap_addr_mux_i,
	boot_addr_i,
	dm_exception_addr_i,
	dm_halt_addr_i,
	req_i,
	instr_req_o,
	instr_addr_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_rdata_i,
	instr_err_i,
	instr_err_pmp_i,
	instr_valid_id_o,
	instr_rdata_id_o,
	is_compressed_id_o,
	illegal_c_insn_id_o,
	pc_if_o,
	pc_id_o,
	is_fetch_failed_o,
	clear_instr_valid_i,
	pc_set_i,
	mepc_i,
	uepc_i,
	depc_i,
	pc_mux_i,
	exc_pc_mux_i,
	m_exc_vec_pc_mux_i,
	u_exc_vec_pc_mux_i,
	csr_mtvec_init_o,
	jump_target_id_i,
	jump_target_ex_i,
	hwlp_jump_i,
	hwlp_target_i,
	halt_if_i,
	id_ready_i,
	if_busy_o,
	perf_imiss_o
);
	reg _sv2v_0;
	parameter COREV_PULP = 0;
	parameter PULP_OBI = 0;
	parameter PULP_SECURE = 0;
	parameter FPU = 0;
	parameter ZFINX = 0;
	input wire clk;
	input wire rst_n;
	input wire [23:0] m_trap_base_addr_i;
	input wire [23:0] u_trap_base_addr_i;
	input wire [1:0] trap_addr_mux_i;
	input wire [31:0] boot_addr_i;
	input wire [31:0] dm_exception_addr_i;
	input wire [31:0] dm_halt_addr_i;
	input wire req_i;
	output wire instr_req_o;
	output wire [31:0] instr_addr_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	input wire [31:0] instr_rdata_i;
	input wire instr_err_i;
	input wire instr_err_pmp_i;
	output reg instr_valid_id_o;
	output reg [31:0] instr_rdata_id_o;
	output reg is_compressed_id_o;
	output reg illegal_c_insn_id_o;
	output wire [31:0] pc_if_o;
	output reg [31:0] pc_id_o;
	output reg is_fetch_failed_o;
	input wire clear_instr_valid_i;
	input wire pc_set_i;
	input wire [31:0] mepc_i;
	input wire [31:0] uepc_i;
	input wire [31:0] depc_i;
	input wire [3:0] pc_mux_i;
	input wire [2:0] exc_pc_mux_i;
	input wire [4:0] m_exc_vec_pc_mux_i;
	input wire [4:0] u_exc_vec_pc_mux_i;
	output wire csr_mtvec_init_o;
	input wire [31:0] jump_target_id_i;
	input wire [31:0] jump_target_ex_i;
	input wire hwlp_jump_i;
	input wire [31:0] hwlp_target_i;
	input wire halt_if_i;
	input wire id_ready_i;
	output wire if_busy_o;
	output wire perf_imiss_o;
	wire if_valid;
	wire if_ready;
	wire prefetch_busy;
	reg branch_req;
	reg [31:0] branch_addr_n;
	wire fetch_valid;
	reg fetch_ready;
	wire [31:0] fetch_rdata;
	reg [31:0] exc_pc;
	reg [23:0] trap_base_addr;
	reg [4:0] exc_vec_pc_mux;
	wire fetch_failed;
	wire aligner_ready;
	wire instr_valid;
	wire illegal_c_insn;
	wire [31:0] instr_aligned;
	wire [31:0] instr_decompressed;
	wire instr_compressed_int;
	localparam cv32e40p_pkg_EXC_PC_DBD = 3'b010;
	localparam cv32e40p_pkg_EXC_PC_DBE = 3'b011;
	localparam cv32e40p_pkg_EXC_PC_EXCEPTION = 3'b000;
	localparam cv32e40p_pkg_EXC_PC_IRQ = 3'b001;
	localparam cv32e40p_pkg_TRAP_MACHINE = 2'b00;
	localparam cv32e40p_pkg_TRAP_USER = 2'b01;
	always @(*) begin : EXC_PC_MUX
		if (_sv2v_0)
			;
		case (trap_addr_mux_i)
			cv32e40p_pkg_TRAP_MACHINE: trap_base_addr = m_trap_base_addr_i;
			cv32e40p_pkg_TRAP_USER: trap_base_addr = u_trap_base_addr_i;
			default: trap_base_addr = m_trap_base_addr_i;
		endcase
		case (trap_addr_mux_i)
			cv32e40p_pkg_TRAP_MACHINE: exc_vec_pc_mux = m_exc_vec_pc_mux_i;
			cv32e40p_pkg_TRAP_USER: exc_vec_pc_mux = u_exc_vec_pc_mux_i;
			default: exc_vec_pc_mux = m_exc_vec_pc_mux_i;
		endcase
		case (exc_pc_mux_i)
			cv32e40p_pkg_EXC_PC_EXCEPTION: exc_pc = {trap_base_addr, 8'h00};
			cv32e40p_pkg_EXC_PC_IRQ: exc_pc = {trap_base_addr, 1'b0, exc_vec_pc_mux, 2'b00};
			cv32e40p_pkg_EXC_PC_DBD: exc_pc = {dm_halt_addr_i[31:2], 2'b00};
			cv32e40p_pkg_EXC_PC_DBE: exc_pc = {dm_exception_addr_i[31:2], 2'b00};
			default: exc_pc = {trap_base_addr, 8'h00};
		endcase
	end
	localparam cv32e40p_pkg_PC_BOOT = 4'b0000;
	localparam cv32e40p_pkg_PC_BRANCH = 4'b0011;
	localparam cv32e40p_pkg_PC_DRET = 4'b0111;
	localparam cv32e40p_pkg_PC_EXCEPTION = 4'b0100;
	localparam cv32e40p_pkg_PC_FENCEI = 4'b0001;
	localparam cv32e40p_pkg_PC_HWLOOP = 4'b1000;
	localparam cv32e40p_pkg_PC_JUMP = 4'b0010;
	localparam cv32e40p_pkg_PC_MRET = 4'b0101;
	localparam cv32e40p_pkg_PC_URET = 4'b0110;
	always @(*) begin
		if (_sv2v_0)
			;
		branch_addr_n = {boot_addr_i[31:2], 2'b00};
		case (pc_mux_i)
			cv32e40p_pkg_PC_BOOT: branch_addr_n = {boot_addr_i[31:2], 2'b00};
			cv32e40p_pkg_PC_JUMP: branch_addr_n = jump_target_id_i;
			cv32e40p_pkg_PC_BRANCH: branch_addr_n = jump_target_ex_i;
			cv32e40p_pkg_PC_EXCEPTION: branch_addr_n = exc_pc;
			cv32e40p_pkg_PC_MRET: branch_addr_n = mepc_i;
			cv32e40p_pkg_PC_URET: branch_addr_n = uepc_i;
			cv32e40p_pkg_PC_DRET: branch_addr_n = depc_i;
			cv32e40p_pkg_PC_FENCEI: branch_addr_n = pc_id_o + 4;
			cv32e40p_pkg_PC_HWLOOP: branch_addr_n = hwlp_target_i;
			default:
				;
		endcase
	end
	assign csr_mtvec_init_o = (pc_mux_i == cv32e40p_pkg_PC_BOOT) & pc_set_i;
	assign fetch_failed = 1'b0;
	cv32e40p_prefetch_buffer #(
		.PULP_OBI(PULP_OBI),
		.COREV_PULP(COREV_PULP)
	) prefetch_buffer_i(
		.clk(clk),
		.rst_n(rst_n),
		.req_i(req_i),
		.branch_i(branch_req),
		.branch_addr_i({branch_addr_n[31:1], 1'b0}),
		.hwlp_jump_i(hwlp_jump_i),
		.hwlp_target_i(hwlp_target_i),
		.fetch_ready_i(fetch_ready),
		.fetch_valid_o(fetch_valid),
		.fetch_rdata_o(fetch_rdata),
		.instr_req_o(instr_req_o),
		.instr_addr_o(instr_addr_o),
		.instr_gnt_i(instr_gnt_i),
		.instr_rvalid_i(instr_rvalid_i),
		.instr_err_i(instr_err_i),
		.instr_err_pmp_i(instr_err_pmp_i),
		.instr_rdata_i(instr_rdata_i),
		.busy_o(prefetch_busy)
	);
	always @(*) begin
		if (_sv2v_0)
			;
		fetch_ready = 1'b0;
		branch_req = 1'b0;
		if (pc_set_i)
			branch_req = 1'b1;
		else if (fetch_valid) begin
			if (req_i && if_valid)
				fetch_ready = aligner_ready;
		end
	end
	assign if_busy_o = prefetch_busy;
	assign perf_imiss_o = !fetch_valid && !branch_req;
	always @(posedge clk or negedge rst_n) begin : IF_ID_PIPE_REGISTERS
		if (rst_n == 1'b0) begin
			instr_valid_id_o <= 1'b0;
			instr_rdata_id_o <= 1'sb0;
			is_fetch_failed_o <= 1'b0;
			pc_id_o <= 1'sb0;
			is_compressed_id_o <= 1'b0;
			illegal_c_insn_id_o <= 1'b0;
		end
		else if (if_valid && instr_valid) begin
			instr_valid_id_o <= 1'b1;
			instr_rdata_id_o <= instr_decompressed;
			is_compressed_id_o <= instr_compressed_int;
			illegal_c_insn_id_o <= illegal_c_insn;
			is_fetch_failed_o <= 1'b0;
			pc_id_o <= pc_if_o;
		end
		else if (clear_instr_valid_i) begin
			instr_valid_id_o <= 1'b0;
			is_fetch_failed_o <= fetch_failed;
		end
	end
	assign if_ready = fetch_valid & id_ready_i;
	assign if_valid = ~halt_if_i & if_ready;
	cv32e40p_aligner aligner_i(
		.clk(clk),
		.rst_n(rst_n),
		.fetch_valid_i(fetch_valid),
		.aligner_ready_o(aligner_ready),
		.if_valid_i(if_valid),
		.fetch_rdata_i(fetch_rdata),
		.instr_aligned_o(instr_aligned),
		.instr_valid_o(instr_valid),
		.branch_addr_i({branch_addr_n[31:1], 1'b0}),
		.branch_i(branch_req),
		.hwlp_addr_i(hwlp_target_i),
		.hwlp_update_pc_i(hwlp_jump_i),
		.pc_o(pc_if_o)
	);
	cv32e40p_compressed_decoder #(
		.FPU(FPU),
		.ZFINX(ZFINX)
	) compressed_decoder_i(
		.instr_i(instr_aligned),
		.instr_o(instr_decompressed),
		.is_compressed_o(instr_compressed_int),
		.illegal_instr_o(illegal_c_insn)
	);
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_cs_registers (
	clk,
	rst_n,
	hart_id_i,
	mtvec_o,
	utvec_o,
	mtvec_mode_o,
	utvec_mode_o,
	mtvec_addr_i,
	csr_mtvec_init_i,
	csr_addr_i,
	csr_wdata_i,
	csr_op_i,
	csr_rdata_o,
	fs_off_o,
	frm_o,
	fflags_i,
	fflags_we_i,
	fregs_we_i,
	mie_bypass_o,
	mip_i,
	m_irq_enable_o,
	u_irq_enable_o,
	csr_irq_sec_i,
	sec_lvl_o,
	mepc_o,
	uepc_o,
	mcounteren_o,
	debug_mode_i,
	debug_cause_i,
	debug_csr_save_i,
	depc_o,
	debug_single_step_o,
	debug_ebreakm_o,
	debug_ebreaku_o,
	trigger_match_o,
	pmp_addr_o,
	pmp_cfg_o,
	priv_lvl_o,
	pc_if_i,
	pc_id_i,
	pc_ex_i,
	csr_save_if_i,
	csr_save_id_i,
	csr_save_ex_i,
	csr_restore_mret_i,
	csr_restore_uret_i,
	csr_restore_dret_i,
	csr_cause_i,
	csr_save_cause_i,
	hwlp_start_i,
	hwlp_end_i,
	hwlp_cnt_i,
	mhpmevent_minstret_i,
	mhpmevent_load_i,
	mhpmevent_store_i,
	mhpmevent_jump_i,
	mhpmevent_branch_i,
	mhpmevent_branch_taken_i,
	mhpmevent_compressed_i,
	mhpmevent_jr_stall_i,
	mhpmevent_imiss_i,
	mhpmevent_ld_stall_i,
	mhpmevent_pipe_stall_i,
	apu_typeconflict_i,
	apu_contention_i,
	apu_dep_i,
	apu_wb_i
);
	reg _sv2v_0;
	parameter N_HWLP = 2;
	parameter APU = 0;
	parameter A_EXTENSION = 0;
	parameter FPU = 0;
	parameter ZFINX = 0;
	parameter PULP_SECURE = 0;
	parameter USE_PMP = 0;
	parameter N_PMP_ENTRIES = 16;
	parameter NUM_MHPMCOUNTERS = 1;
	parameter COREV_PULP = 0;
	parameter COREV_CLUSTER = 0;
	parameter DEBUG_TRIGGER_EN = 1;
	input wire clk;
	input wire rst_n;
	input wire [31:0] hart_id_i;
	output wire [23:0] mtvec_o;
	output wire [23:0] utvec_o;
	output wire [1:0] mtvec_mode_o;
	output wire [1:0] utvec_mode_o;
	input wire [31:0] mtvec_addr_i;
	input wire csr_mtvec_init_i;
	input wire [11:0] csr_addr_i;
	input wire [31:0] csr_wdata_i;
	localparam cv32e40p_pkg_CSR_OP_WIDTH = 2;
	input wire [1:0] csr_op_i;
	output wire [31:0] csr_rdata_o;
	output wire fs_off_o;
	output wire [2:0] frm_o;
	localparam cv32e40p_pkg_C_FFLAG = 5;
	input wire [4:0] fflags_i;
	input wire fflags_we_i;
	input wire fregs_we_i;
	output wire [31:0] mie_bypass_o;
	input wire [31:0] mip_i;
	output wire m_irq_enable_o;
	output wire u_irq_enable_o;
	input wire csr_irq_sec_i;
	output wire sec_lvl_o;
	output wire [31:0] mepc_o;
	output wire [31:0] uepc_o;
	output wire [31:0] mcounteren_o;
	input wire debug_mode_i;
	input wire [2:0] debug_cause_i;
	input wire debug_csr_save_i;
	output wire [31:0] depc_o;
	output wire debug_single_step_o;
	output wire debug_ebreakm_o;
	output wire debug_ebreaku_o;
	output wire trigger_match_o;
	output wire [(N_PMP_ENTRIES * 32) - 1:0] pmp_addr_o;
	output wire [(N_PMP_ENTRIES * 8) - 1:0] pmp_cfg_o;
	output wire [1:0] priv_lvl_o;
	input wire [31:0] pc_if_i;
	input wire [31:0] pc_id_i;
	input wire [31:0] pc_ex_i;
	input wire csr_save_if_i;
	input wire csr_save_id_i;
	input wire csr_save_ex_i;
	input wire csr_restore_mret_i;
	input wire csr_restore_uret_i;
	input wire csr_restore_dret_i;
	input wire [5:0] csr_cause_i;
	input wire csr_save_cause_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_start_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_end_i;
	input wire [(N_HWLP * 32) - 1:0] hwlp_cnt_i;
	input wire mhpmevent_minstret_i;
	input wire mhpmevent_load_i;
	input wire mhpmevent_store_i;
	input wire mhpmevent_jump_i;
	input wire mhpmevent_branch_i;
	input wire mhpmevent_branch_taken_i;
	input wire mhpmevent_compressed_i;
	input wire mhpmevent_jr_stall_i;
	input wire mhpmevent_imiss_i;
	input wire mhpmevent_ld_stall_i;
	input wire mhpmevent_pipe_stall_i;
	input wire apu_typeconflict_i;
	input wire apu_contention_i;
	input wire apu_dep_i;
	input wire apu_wb_i;
	localparam NUM_HPM_EVENTS = 16;
	localparam MTVEC_MODE = 2'b01;
	localparam MAX_N_PMP_ENTRIES = 16;
	localparam MAX_N_PMP_CFG = 4;
	localparam N_PMP_CFG = ((N_PMP_ENTRIES % 4) == 0 ? N_PMP_ENTRIES / 4 : (N_PMP_ENTRIES / 4) + 1);
	localparam MSTATUS_UIE_BIT = 0;
	localparam MSTATUS_SIE_BIT = 1;
	localparam MSTATUS_MIE_BIT = 3;
	localparam MSTATUS_UPIE_BIT = 4;
	localparam MSTATUS_SPIE_BIT = 5;
	localparam MSTATUS_MPIE_BIT = 7;
	localparam MSTATUS_SPP_BIT = 8;
	localparam MSTATUS_MPP_BIT_LOW = 11;
	localparam MSTATUS_MPP_BIT_HIGH = 12;
	localparam MSTATUS_FS_BIT_LOW = 13;
	localparam MSTATUS_FS_BIT_HIGH = 14;
	localparam MSTATUS_MPRV_BIT = 17;
	localparam MSTATUS_SD_BIT = 31;
	localparam [1:0] MXL = 2'd1;
	function automatic [31:0] sv2v_cast_32;
		input reg [31:0] inp;
		sv2v_cast_32 = inp;
	endfunction
	localparam [31:0] MISA_VALUE = (((((((((((A_EXTENSION << 0) | 4) | 0) | 0) | (sv2v_cast_32((FPU == 1) && (ZFINX == 0)) << 5)) | 256) | 4096) | 0) | 0) | (PULP_SECURE << 20)) | (sv2v_cast_32(COREV_PULP || COREV_CLUSTER) << 23)) | (sv2v_cast_32(MXL) << 30);
	localparam PULP_PERF_COUNTERS = 0;
	reg [31:0] csr_wdata_int;
	reg [31:0] csr_rdata_int;
	reg csr_we_int;
	localparam cv32e40p_pkg_C_RM = 3;
	reg [2:0] frm_q;
	reg [2:0] frm_n;
	reg [4:0] fflags_q;
	reg [4:0] fflags_n;
	reg fcsr_update;
	reg [31:0] mepc_q;
	reg [31:0] mepc_n;
	reg [31:0] uepc_q;
	reg [31:0] uepc_n;
	wire [31:0] tmatch_control_rdata;
	wire [31:0] tmatch_value_rdata;
	wire [15:0] tinfo_types;
	reg [31:0] dcsr_q;
	reg [31:0] dcsr_n;
	reg [31:0] depc_q;
	reg [31:0] depc_n;
	reg [31:0] dscratch0_q;
	reg [31:0] dscratch0_n;
	reg [31:0] dscratch1_q;
	reg [31:0] dscratch1_n;
	reg [31:0] mscratch_q;
	reg [31:0] mscratch_n;
	reg [31:0] exception_pc;
	reg [6:0] mstatus_q;
	reg [6:0] mstatus_n;
	reg mstatus_we_int;
	reg [1:0] mstatus_fs_q;
	reg [1:0] mstatus_fs_n;
	reg [5:0] mcause_q;
	reg [5:0] mcause_n;
	reg [5:0] ucause_q;
	reg [5:0] ucause_n;
	reg [23:0] mtvec_n;
	reg [23:0] mtvec_q;
	reg [23:0] utvec_n;
	reg [23:0] utvec_q;
	reg [1:0] mtvec_mode_n;
	reg [1:0] mtvec_mode_q;
	reg [1:0] utvec_mode_n;
	reg [1:0] utvec_mode_q;
	wire [31:0] mip;
	reg [31:0] mie_q;
	reg [31:0] mie_n;
	reg [31:0] csr_mie_wdata;
	reg csr_mie_we;
	wire is_irq;
	reg [1:0] priv_lvl_n;
	reg [1:0] priv_lvl_q;
	reg [767:0] pmp_reg_q;
	reg [767:0] pmp_reg_n;
	reg [15:0] pmpaddr_we;
	reg [15:0] pmpcfg_we;
	localparam cv32e40p_pkg_MHPMCOUNTER_WIDTH = 64;
	reg [2047:0] mhpmcounter_q;
	reg [1023:0] mhpmevent_q;
	reg [1023:0] mhpmevent_n;
	reg [31:0] mcounteren_q;
	reg [31:0] mcounteren_n;
	reg [31:0] mcountinhibit_q;
	reg [31:0] mcountinhibit_n;
	wire [15:0] hpm_events;
	wire [2047:0] mhpmcounter_increment;
	wire [31:0] mhpmcounter_write_lower;
	wire [31:0] mhpmcounter_write_upper;
	wire [31:0] mhpmcounter_write_increment;
	assign is_irq = csr_cause_i[5];
	assign mip = mip_i;
	function automatic [1:0] sv2v_cast_315CD;
		input reg [1:0] inp;
		sv2v_cast_315CD = inp;
	endfunction
	always @(*) begin
		if (_sv2v_0)
			;
		csr_mie_wdata = csr_wdata_i;
		csr_mie_we = 1'b1;
		case (csr_op_i)
			sv2v_cast_315CD(2'b01): csr_mie_wdata = csr_wdata_i;
			sv2v_cast_315CD(2'b10): csr_mie_wdata = csr_wdata_i | mie_q;
			sv2v_cast_315CD(2'b11): csr_mie_wdata = ~csr_wdata_i & mie_q;
			sv2v_cast_315CD(2'b00): begin
				csr_mie_wdata = csr_wdata_i;
				csr_mie_we = 1'b0;
			end
		endcase
	end
	localparam cv32e40p_pkg_IRQ_MASK = 32'hffff0888;
	assign mie_bypass_o = ((csr_addr_i == 12'h304) && csr_mie_we ? csr_mie_wdata & cv32e40p_pkg_IRQ_MASK : mie_q);
	genvar _gv_j_1;
	localparam cv32e40p_pkg_MARCHID = 32'h00000004;
	localparam cv32e40p_pkg_MVENDORID_BANK = 25'h000000c;
	localparam cv32e40p_pkg_MVENDORID_OFFSET = 7'h02;
	generate
		if (PULP_SECURE == 1) begin : gen_pulp_secure_read_logic
			always @(*) begin
				if (_sv2v_0)
					;
				case (csr_addr_i)
					12'h001: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fflags_q} : {32 {1'sb0}});
					12'h002: csr_rdata_int = (FPU == 1 ? {29'b00000000000000000000000000000, frm_q} : {32 {1'sb0}});
					12'h003: csr_rdata_int = (FPU == 1 ? {24'b000000000000000000000000, frm_q, fflags_q} : {32 {1'sb0}});
					12'h300: csr_rdata_int = {14'b00000000000000, mstatus_q[0], 4'b0000, mstatus_q[2-:2], 3'b000, mstatus_q[3], 2'h0, mstatus_q[4], mstatus_q[5], 2'h0, mstatus_q[6]};
					12'h301: csr_rdata_int = MISA_VALUE;
					12'h304: csr_rdata_int = mie_q;
					12'h305: csr_rdata_int = {mtvec_q, 6'h00, mtvec_mode_q};
					12'h340: csr_rdata_int = mscratch_q;
					12'h341: csr_rdata_int = mepc_q;
					12'h342: csr_rdata_int = {mcause_q[5], 26'b00000000000000000000000000, mcause_q[4:0]};
					12'h344: csr_rdata_int = mip;
					12'hf14: csr_rdata_int = hart_id_i;
					12'hf11: csr_rdata_int = {cv32e40p_pkg_MVENDORID_BANK, cv32e40p_pkg_MVENDORID_OFFSET};
					12'hf12: csr_rdata_int = cv32e40p_pkg_MARCHID;
					12'hf13, 12'h343: csr_rdata_int = 'b0;
					12'h306: csr_rdata_int = mcounteren_q;
					12'h7a0, 12'h7a3, 12'h7a8, 12'h7aa: csr_rdata_int = 'b0;
					12'h7a1: csr_rdata_int = tmatch_control_rdata;
					12'h7a2: csr_rdata_int = tmatch_value_rdata;
					12'h7a4: csr_rdata_int = tinfo_types;
					12'h7b0: csr_rdata_int = dcsr_q;
					12'h7b1: csr_rdata_int = depc_q;
					12'h7b2: csr_rdata_int = dscratch0_q;
					12'h7b3: csr_rdata_int = dscratch1_q;
					12'hb00, 12'hb02, 12'hb03, 12'hb04, 12'hb05, 12'hb06, 12'hb07, 12'hb08, 12'hb09, 12'hb0a, 12'hb0b, 12'hb0c, 12'hb0d, 12'hb0e, 12'hb0f, 12'hb10, 12'hb11, 12'hb12, 12'hb13, 12'hb14, 12'hb15, 12'hb16, 12'hb17, 12'hb18, 12'hb19, 12'hb1a, 12'hb1b, 12'hb1c, 12'hb1d, 12'hb1e, 12'hb1f, 12'hc00, 12'hc02, 12'hc03, 12'hc04, 12'hc05, 12'hc06, 12'hc07, 12'hc08, 12'hc09, 12'hc0a, 12'hc0b, 12'hc0c, 12'hc0d, 12'hc0e, 12'hc0f, 12'hc10, 12'hc11, 12'hc12, 12'hc13, 12'hc14, 12'hc15, 12'hc16, 12'hc17, 12'hc18, 12'hc19, 12'hc1a, 12'hc1b, 12'hc1c, 12'hc1d, 12'hc1e, 12'hc1f: csr_rdata_int = mhpmcounter_q[(csr_addr_i[4:0] * 64) + 31-:32];
					12'hb80, 12'hb82, 12'hb83, 12'hb84, 12'hb85, 12'hb86, 12'hb87, 12'hb88, 12'hb89, 12'hb8a, 12'hb8b, 12'hb8c, 12'hb8d, 12'hb8e, 12'hb8f, 12'hb90, 12'hb91, 12'hb92, 12'hb93, 12'hb94, 12'hb95, 12'hb96, 12'hb97, 12'hb98, 12'hb99, 12'hb9a, 12'hb9b, 12'hb9c, 12'hb9d, 12'hb9e, 12'hb9f, 12'hc80, 12'hc82, 12'hc83, 12'hc84, 12'hc85, 12'hc86, 12'hc87, 12'hc88, 12'hc89, 12'hc8a, 12'hc8b, 12'hc8c, 12'hc8d, 12'hc8e, 12'hc8f, 12'hc90, 12'hc91, 12'hc92, 12'hc93, 12'hc94, 12'hc95, 12'hc96, 12'hc97, 12'hc98, 12'hc99, 12'hc9a, 12'hc9b, 12'hc9c, 12'hc9d, 12'hc9e, 12'hc9f: csr_rdata_int = mhpmcounter_q[(csr_addr_i[4:0] * 64) + 63-:32];
					12'h320: csr_rdata_int = mcountinhibit_q;
					12'h323, 12'h324, 12'h325, 12'h326, 12'h327, 12'h328, 12'h329, 12'h32a, 12'h32b, 12'h32c, 12'h32d, 12'h32e, 12'h32f, 12'h330, 12'h331, 12'h332, 12'h333, 12'h334, 12'h335, 12'h336, 12'h337, 12'h338, 12'h339, 12'h33a, 12'h33b, 12'h33c, 12'h33d, 12'h33e, 12'h33f: csr_rdata_int = mhpmevent_q[csr_addr_i[4:0] * 32+:32];
					12'hcc0: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_start_i[0+:32]);
					12'hcc1: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_end_i[0+:32]);
					12'hcc2: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_cnt_i[0+:32]);
					12'hcc4: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_start_i[32+:32]);
					12'hcc5: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_end_i[32+:32]);
					12'hcc6: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_cnt_i[32+:32]);
					12'h3a0: csr_rdata_int = (USE_PMP ? pmp_reg_q[128+:32] : {32 {1'sb0}});
					12'h3a1: csr_rdata_int = (USE_PMP ? pmp_reg_q[160+:32] : {32 {1'sb0}});
					12'h3a2: csr_rdata_int = (USE_PMP ? pmp_reg_q[192+:32] : {32 {1'sb0}});
					12'h3a3: csr_rdata_int = (USE_PMP ? pmp_reg_q[224+:32] : {32 {1'sb0}});
					12'h3b0, 12'h3b1, 12'h3b2, 12'h3b3, 12'h3b4, 12'h3b5, 12'h3b6, 12'h3b7, 12'h3b8, 12'h3b9, 12'h3ba, 12'h3bb, 12'h3bc, 12'h3bd, 12'h3be, 12'h3bf: csr_rdata_int = (USE_PMP ? pmp_reg_q[256 + (csr_addr_i[3:0] * 32)+:32] : {32 {1'sb0}});
					12'h000: csr_rdata_int = {27'b000000000000000000000000000, mstatus_q[4], 3'h0, mstatus_q[6]};
					12'h005: csr_rdata_int = {utvec_q, 6'h00, utvec_mode_q};
					12'hcd0: csr_rdata_int = (!COREV_PULP ? 'b0 : hart_id_i);
					12'h041: csr_rdata_int = uepc_q;
					12'h042: csr_rdata_int = {ucause_q[5], 26'h0000000, ucause_q[4:0]};
					12'hcd1: csr_rdata_int = (!COREV_PULP ? 'b0 : {30'h00000000, priv_lvl_q});
					default: csr_rdata_int = 1'sb0;
				endcase
			end
		end
		else begin : gen_no_pulp_secure_read_logic
			always @(*) begin
				if (_sv2v_0)
					;
				case (csr_addr_i)
					12'h001: csr_rdata_int = (FPU == 1 ? {27'b000000000000000000000000000, fflags_q} : {32 {1'sb0}});
					12'h002: csr_rdata_int = (FPU == 1 ? {29'b00000000000000000000000000000, frm_q} : {32 {1'sb0}});
					12'h003: csr_rdata_int = (FPU == 1 ? {24'b000000000000000000000000, frm_q, fflags_q} : {32 {1'sb0}});
					12'h300: csr_rdata_int = {((FPU == 1) && (ZFINX == 0) ? (mstatus_fs_q == 2'b11 ? 1'b1 : 1'b0) : 1'b0), 13'b0000000000000, mstatus_q[0], 2'b00, ((FPU == 1) && (ZFINX == 0) ? mstatus_fs_q : 2'b00), mstatus_q[2-:2], 3'b000, mstatus_q[3], 2'h0, mstatus_q[4], mstatus_q[5], 2'h0, mstatus_q[6]};
					12'h301: csr_rdata_int = MISA_VALUE;
					12'h304: csr_rdata_int = mie_q;
					12'h305: csr_rdata_int = {mtvec_q, 6'h00, mtvec_mode_q};
					12'h340: csr_rdata_int = mscratch_q;
					12'h341: csr_rdata_int = mepc_q;
					12'h342: csr_rdata_int = {mcause_q[5], 26'b00000000000000000000000000, mcause_q[4:0]};
					12'h344: csr_rdata_int = mip;
					12'hf14: csr_rdata_int = hart_id_i;
					12'hf11: csr_rdata_int = {cv32e40p_pkg_MVENDORID_BANK, cv32e40p_pkg_MVENDORID_OFFSET};
					12'hf12: csr_rdata_int = cv32e40p_pkg_MARCHID;
					12'hf13: csr_rdata_int = ((FPU || COREV_PULP) || COREV_CLUSTER ? 32'h00000001 : 'b0);
					12'h343: csr_rdata_int = 'b0;
					12'h7a0, 12'h7a3, 12'h7a8, 12'h7aa: csr_rdata_int = 'b0;
					12'h7a1: csr_rdata_int = tmatch_control_rdata;
					12'h7a2: csr_rdata_int = tmatch_value_rdata;
					12'h7a4: csr_rdata_int = tinfo_types;
					12'h7b0: csr_rdata_int = dcsr_q;
					12'h7b1: csr_rdata_int = depc_q;
					12'h7b2: csr_rdata_int = dscratch0_q;
					12'h7b3: csr_rdata_int = dscratch1_q;
					12'hb00, 12'hb02, 12'hb03, 12'hb04, 12'hb05, 12'hb06, 12'hb07, 12'hb08, 12'hb09, 12'hb0a, 12'hb0b, 12'hb0c, 12'hb0d, 12'hb0e, 12'hb0f, 12'hb10, 12'hb11, 12'hb12, 12'hb13, 12'hb14, 12'hb15, 12'hb16, 12'hb17, 12'hb18, 12'hb19, 12'hb1a, 12'hb1b, 12'hb1c, 12'hb1d, 12'hb1e, 12'hb1f, 12'hc00, 12'hc02, 12'hc03, 12'hc04, 12'hc05, 12'hc06, 12'hc07, 12'hc08, 12'hc09, 12'hc0a, 12'hc0b, 12'hc0c, 12'hc0d, 12'hc0e, 12'hc0f, 12'hc10, 12'hc11, 12'hc12, 12'hc13, 12'hc14, 12'hc15, 12'hc16, 12'hc17, 12'hc18, 12'hc19, 12'hc1a, 12'hc1b, 12'hc1c, 12'hc1d, 12'hc1e, 12'hc1f: csr_rdata_int = mhpmcounter_q[(csr_addr_i[4:0] * 64) + 31-:32];
					12'hb80, 12'hb82, 12'hb83, 12'hb84, 12'hb85, 12'hb86, 12'hb87, 12'hb88, 12'hb89, 12'hb8a, 12'hb8b, 12'hb8c, 12'hb8d, 12'hb8e, 12'hb8f, 12'hb90, 12'hb91, 12'hb92, 12'hb93, 12'hb94, 12'hb95, 12'hb96, 12'hb97, 12'hb98, 12'hb99, 12'hb9a, 12'hb9b, 12'hb9c, 12'hb9d, 12'hb9e, 12'hb9f, 12'hc80, 12'hc82, 12'hc83, 12'hc84, 12'hc85, 12'hc86, 12'hc87, 12'hc88, 12'hc89, 12'hc8a, 12'hc8b, 12'hc8c, 12'hc8d, 12'hc8e, 12'hc8f, 12'hc90, 12'hc91, 12'hc92, 12'hc93, 12'hc94, 12'hc95, 12'hc96, 12'hc97, 12'hc98, 12'hc99, 12'hc9a, 12'hc9b, 12'hc9c, 12'hc9d, 12'hc9e, 12'hc9f: csr_rdata_int = mhpmcounter_q[(csr_addr_i[4:0] * 64) + 63-:32];
					12'h320: csr_rdata_int = mcountinhibit_q;
					12'h323, 12'h324, 12'h325, 12'h326, 12'h327, 12'h328, 12'h329, 12'h32a, 12'h32b, 12'h32c, 12'h32d, 12'h32e, 12'h32f, 12'h330, 12'h331, 12'h332, 12'h333, 12'h334, 12'h335, 12'h336, 12'h337, 12'h338, 12'h339, 12'h33a, 12'h33b, 12'h33c, 12'h33d, 12'h33e, 12'h33f: csr_rdata_int = mhpmevent_q[csr_addr_i[4:0] * 32+:32];
					12'hcc0: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_start_i[0+:32]);
					12'hcc1: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_end_i[0+:32]);
					12'hcc2: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_cnt_i[0+:32]);
					12'hcc4: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_start_i[32+:32]);
					12'hcc5: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_end_i[32+:32]);
					12'hcc6: csr_rdata_int = (!COREV_PULP ? 'b0 : hwlp_cnt_i[32+:32]);
					12'hcd0: csr_rdata_int = (!COREV_PULP ? 'b0 : hart_id_i);
					12'hcd1: csr_rdata_int = (!COREV_PULP ? 'b0 : {30'h00000000, priv_lvl_q});
					12'hcd2: csr_rdata_int = ((FPU == 1) && (ZFINX == 1) ? 32'h00000001 : 32'h00000000);
					default: csr_rdata_int = 1'sb0;
				endcase
			end
		end
	endgenerate
	function automatic [1:0] sv2v_cast_2;
		input reg [1:0] inp;
		sv2v_cast_2 = inp;
	endfunction
	generate
		if (PULP_SECURE == 1) begin : gen_pulp_secure_write_logic
			always @(*) begin
				if (_sv2v_0)
					;
				fflags_n = fflags_q;
				frm_n = frm_q;
				mscratch_n = mscratch_q;
				mepc_n = mepc_q;
				uepc_n = uepc_q;
				depc_n = depc_q;
				dcsr_n = dcsr_q;
				dscratch0_n = dscratch0_q;
				dscratch1_n = dscratch1_q;
				mstatus_n = mstatus_q;
				mcause_n = mcause_q;
				ucause_n = ucause_q;
				exception_pc = pc_id_i;
				priv_lvl_n = priv_lvl_q;
				mtvec_n = (csr_mtvec_init_i ? mtvec_addr_i[31:8] : mtvec_q);
				utvec_n = utvec_q;
				mtvec_mode_n = mtvec_mode_q;
				utvec_mode_n = utvec_mode_q;
				pmp_reg_n[767-:512] = pmp_reg_q[767-:512];
				pmp_reg_n[255-:128] = pmp_reg_q[255-:128];
				pmpaddr_we = 1'sb0;
				pmpcfg_we = 1'sb0;
				mie_n = mie_q;
				if (FPU == 1) begin
					if (fflags_we_i)
						fflags_n = fflags_i | fflags_q;
				end
				case (csr_addr_i)
					12'h001:
						if (csr_we_int)
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
					12'h002:
						if (csr_we_int)
							frm_n = (FPU == 1 ? csr_wdata_int[2:0] : {3 {1'sb0}});
					12'h003:
						if (csr_we_int) begin
							fflags_n = (FPU == 1 ? csr_wdata_int[4:0] : {5 {1'sb0}});
							frm_n = (FPU == 1 ? csr_wdata_int[7:cv32e40p_pkg_C_FFLAG] : {3 {1'sb0}});
						end
					12'h300:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[MSTATUS_UIE_BIT], csr_wdata_int[MSTATUS_MIE_BIT], csr_wdata_int[MSTATUS_UPIE_BIT], csr_wdata_int[MSTATUS_MPIE_BIT], sv2v_cast_2(csr_wdata_int[MSTATUS_MPP_BIT_HIGH:MSTATUS_MPP_BIT_LOW]), csr_wdata_int[MSTATUS_MPRV_BIT]};
					12'h304:
						if (csr_we_int)
							mie_n = csr_wdata_int & cv32e40p_pkg_IRQ_MASK;
					12'h305:
						if (csr_we_int) begin
							mtvec_n = csr_wdata_int[31:8];
							mtvec_mode_n = {1'b0, csr_wdata_int[0]};
						end
					12'h340:
						if (csr_we_int)
							mscratch_n = csr_wdata_int;
					12'h341:
						if (csr_we_int)
							mepc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h342:
						if (csr_we_int)
							mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
					12'h7b0:
						if (csr_we_int) begin
							dcsr_n[15] = csr_wdata_int[15];
							dcsr_n[13] = 1'b0;
							dcsr_n[12] = csr_wdata_int[12];
							dcsr_n[11] = csr_wdata_int[11];
							dcsr_n[10] = 1'b0;
							dcsr_n[9] = 1'b0;
							dcsr_n[4] = 1'b0;
							dcsr_n[2] = csr_wdata_int[2];
							dcsr_n[1-:2] = (csr_wdata_int[1:0] == 2'b11 ? 2'b11 : 2'b00);
						end
					12'h7b1:
						if (csr_we_int)
							depc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h7b2:
						if (csr_we_int)
							dscratch0_n = csr_wdata_int;
					12'h7b3:
						if (csr_we_int)
							dscratch1_n = csr_wdata_int;
					12'h3a0:
						if (csr_we_int) begin
							pmp_reg_n[128+:32] = csr_wdata_int;
							pmpcfg_we[3:0] = 4'b1111;
						end
					12'h3a1:
						if (csr_we_int) begin
							pmp_reg_n[160+:32] = csr_wdata_int;
							pmpcfg_we[7:4] = 4'b1111;
						end
					12'h3a2:
						if (csr_we_int) begin
							pmp_reg_n[192+:32] = csr_wdata_int;
							pmpcfg_we[11:8] = 4'b1111;
						end
					12'h3a3:
						if (csr_we_int) begin
							pmp_reg_n[224+:32] = csr_wdata_int;
							pmpcfg_we[15:12] = 4'b1111;
						end
					12'h3b0, 12'h3b1, 12'h3b2, 12'h3b3, 12'h3b4, 12'h3b5, 12'h3b6, 12'h3b7, 12'h3b8, 12'h3b9, 12'h3ba, 12'h3bb, 12'h3bc, 12'h3bd, 12'h3be, 12'h3bf:
						if (csr_we_int) begin
							pmp_reg_n[256 + (csr_addr_i[3:0] * 32)+:32] = csr_wdata_int;
							pmpaddr_we[csr_addr_i[3:0]] = 1'b1;
						end
					12'h000:
						if (csr_we_int)
							mstatus_n = {csr_wdata_int[MSTATUS_UIE_BIT], mstatus_q[5], csr_wdata_int[MSTATUS_UPIE_BIT], mstatus_q[3], sv2v_cast_2(mstatus_q[2-:2]), mstatus_q[0]};
					12'h005:
						if (csr_we_int) begin
							utvec_n = csr_wdata_int[31:8];
							utvec_mode_n = {1'b0, csr_wdata_int[0]};
						end
					12'h041:
						if (csr_we_int)
							uepc_n = csr_wdata_int;
					12'h042:
						if (csr_we_int)
							ucause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
				endcase
				case (1'b1)
					csr_save_cause_i: begin
						case (1'b1)
							csr_save_if_i: exception_pc = pc_if_i;
							csr_save_id_i: exception_pc = pc_id_i;
							csr_save_ex_i: exception_pc = pc_ex_i;
							default:
								;
						endcase
						case (priv_lvl_q)
							2'b00:
								if (~is_irq) begin
									priv_lvl_n = 2'b11;
									mstatus_n[3] = mstatus_q[6];
									mstatus_n[5] = 1'b0;
									mstatus_n[2-:2] = 2'b00;
									if (debug_csr_save_i)
										depc_n = exception_pc;
									else
										mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
								else if (~csr_irq_sec_i) begin
									priv_lvl_n = 2'b00;
									mstatus_n[4] = mstatus_q[6];
									mstatus_n[6] = 1'b0;
									if (debug_csr_save_i)
										depc_n = exception_pc;
									else
										uepc_n = exception_pc;
									ucause_n = csr_cause_i;
								end
								else begin
									priv_lvl_n = 2'b11;
									mstatus_n[3] = mstatus_q[6];
									mstatus_n[5] = 1'b0;
									mstatus_n[2-:2] = 2'b00;
									if (debug_csr_save_i)
										depc_n = exception_pc;
									else
										mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
							2'b11:
								if (debug_csr_save_i) begin
									dcsr_n[1-:2] = 2'b11;
									dcsr_n[8-:3] = debug_cause_i;
									depc_n = exception_pc;
								end
								else begin
									priv_lvl_n = 2'b11;
									mstatus_n[3] = mstatus_q[5];
									mstatus_n[5] = 1'b0;
									mstatus_n[2-:2] = 2'b11;
									mepc_n = exception_pc;
									mcause_n = csr_cause_i;
								end
							default:
								;
						endcase
					end
					csr_restore_uret_i: begin
						mstatus_n[6] = mstatus_q[4];
						priv_lvl_n = 2'b00;
						mstatus_n[4] = 1'b1;
					end
					csr_restore_mret_i:
						case (mstatus_q[2-:2])
							2'b00: begin
								mstatus_n[6] = mstatus_q[3];
								priv_lvl_n = 2'b00;
								mstatus_n[3] = 1'b1;
								mstatus_n[2-:2] = 2'b00;
							end
							2'b11: begin
								mstatus_n[5] = mstatus_q[3];
								priv_lvl_n = 2'b11;
								mstatus_n[3] = 1'b1;
								mstatus_n[2-:2] = 2'b00;
							end
							default:
								;
						endcase
					csr_restore_dret_i: priv_lvl_n = dcsr_q[1-:2];
					default:
						;
				endcase
			end
		end
		else begin : gen_no_pulp_secure_write_logic
			always @(*) begin
				if (_sv2v_0)
					;
				if (FPU == 1) begin
					fflags_n = fflags_q;
					frm_n = frm_q;
					if (ZFINX == 0) begin
						mstatus_fs_n = mstatus_fs_q;
						fcsr_update = 1'b0;
					end
				end
				mscratch_n = mscratch_q;
				mepc_n = mepc_q;
				uepc_n = 'b0;
				depc_n = depc_q;
				dcsr_n = dcsr_q;
				dscratch0_n = dscratch0_q;
				dscratch1_n = dscratch1_q;
				mstatus_we_int = 1'b0;
				mstatus_n = mstatus_q;
				mcause_n = mcause_q;
				ucause_n = 1'sb0;
				exception_pc = pc_id_i;
				priv_lvl_n = priv_lvl_q;
				mtvec_n = (csr_mtvec_init_i ? mtvec_addr_i[31:8] : mtvec_q);
				utvec_n = 1'sb0;
				pmp_reg_n[767-:512] = 1'sb0;
				pmp_reg_n[255-:128] = 1'sb0;
				pmp_reg_n[127-:128] = 1'sb0;
				pmpaddr_we = 1'sb0;
				pmpcfg_we = 1'sb0;
				mie_n = mie_q;
				mtvec_mode_n = mtvec_mode_q;
				utvec_mode_n = 1'sb0;
				case (csr_addr_i)
					12'h001:
						if (FPU == 1) begin
							if (csr_we_int) begin
								fflags_n = csr_wdata_int[4:0];
								if (ZFINX == 0)
									fcsr_update = 1'b1;
							end
						end
					12'h002:
						if (FPU == 1) begin
							if (csr_we_int) begin
								frm_n = csr_wdata_int[2:0];
								if (ZFINX == 0)
									fcsr_update = 1'b1;
							end
						end
					12'h003:
						if (FPU == 1) begin
							if (csr_we_int) begin
								fflags_n = csr_wdata_int[4:0];
								frm_n = csr_wdata_int[7:cv32e40p_pkg_C_FFLAG];
								if (ZFINX == 0)
									fcsr_update = 1'b1;
							end
						end
					12'h300:
						if (csr_we_int) begin
							mstatus_n = {csr_wdata_int[MSTATUS_UIE_BIT], csr_wdata_int[MSTATUS_MIE_BIT], csr_wdata_int[MSTATUS_UPIE_BIT], csr_wdata_int[MSTATUS_MPIE_BIT], sv2v_cast_2(csr_wdata_int[MSTATUS_MPP_BIT_HIGH:MSTATUS_MPP_BIT_LOW]), csr_wdata_int[MSTATUS_MPRV_BIT]};
							if ((FPU == 1) && (ZFINX == 0)) begin
								mstatus_we_int = 1'b1;
								mstatus_fs_n = sv2v_cast_2(csr_wdata_int[MSTATUS_FS_BIT_HIGH:MSTATUS_FS_BIT_LOW]);
							end
						end
					12'h304:
						if (csr_we_int)
							mie_n = csr_wdata_int & cv32e40p_pkg_IRQ_MASK;
					12'h305:
						if (csr_we_int) begin
							mtvec_n = csr_wdata_int[31:8];
							mtvec_mode_n = {1'b0, csr_wdata_int[0]};
						end
					12'h340:
						if (csr_we_int)
							mscratch_n = csr_wdata_int;
					12'h341:
						if (csr_we_int)
							mepc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h342:
						if (csr_we_int)
							mcause_n = {csr_wdata_int[31], csr_wdata_int[4:0]};
					12'h7b0:
						if (csr_we_int) begin
							dcsr_n[15] = csr_wdata_int[15];
							dcsr_n[13] = 1'b0;
							dcsr_n[12] = 1'b0;
							dcsr_n[11] = csr_wdata_int[11];
							dcsr_n[10] = 1'b0;
							dcsr_n[9] = 1'b0;
							dcsr_n[4] = 1'b0;
							dcsr_n[2] = csr_wdata_int[2];
							dcsr_n[1-:2] = 2'b11;
						end
					12'h7b1:
						if (csr_we_int)
							depc_n = csr_wdata_int & ~32'b00000000000000000000000000000001;
					12'h7b2:
						if (csr_we_int)
							dscratch0_n = csr_wdata_int;
					12'h7b3:
						if (csr_we_int)
							dscratch1_n = csr_wdata_int;
				endcase
				if (FPU == 1) begin
					if (fflags_we_i)
						fflags_n = fflags_i | fflags_q;
					if (ZFINX == 0) begin
						if (((fregs_we_i && !(mstatus_we_int && (mstatus_fs_n != 2'b11))) || fflags_we_i) || fcsr_update)
							mstatus_fs_n = 2'b11;
					end
				end
				case (1'b1)
					csr_save_cause_i: begin
						case (1'b1)
							csr_save_if_i: exception_pc = pc_if_i;
							csr_save_id_i: exception_pc = pc_id_i;
							csr_save_ex_i: exception_pc = pc_ex_i;
							default:
								;
						endcase
						if (debug_csr_save_i) begin
							dcsr_n[1-:2] = 2'b11;
							dcsr_n[8-:3] = debug_cause_i;
							depc_n = exception_pc;
						end
						else begin
							priv_lvl_n = 2'b11;
							mstatus_n[3] = mstatus_q[5];
							mstatus_n[5] = 1'b0;
							mstatus_n[2-:2] = 2'b11;
							mepc_n = exception_pc;
							mcause_n = csr_cause_i;
						end
					end
					csr_restore_mret_i: begin
						mstatus_n[5] = mstatus_q[3];
						priv_lvl_n = 2'b11;
						mstatus_n[3] = 1'b1;
						mstatus_n[2-:2] = 2'b11;
					end
					csr_restore_dret_i: priv_lvl_n = dcsr_q[1-:2];
					default:
						;
				endcase
			end
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		csr_wdata_int = csr_wdata_i;
		csr_we_int = 1'b1;
		case (csr_op_i)
			sv2v_cast_315CD(2'b01): csr_wdata_int = csr_wdata_i;
			sv2v_cast_315CD(2'b10): csr_wdata_int = csr_wdata_i | csr_rdata_o;
			sv2v_cast_315CD(2'b11): csr_wdata_int = ~csr_wdata_i & csr_rdata_o;
			sv2v_cast_315CD(2'b00): begin
				csr_wdata_int = csr_wdata_i;
				csr_we_int = 1'b0;
			end
		endcase
	end
	assign csr_rdata_o = csr_rdata_int;
	assign m_irq_enable_o = mstatus_q[5] && !(dcsr_q[2] && !dcsr_q[11]);
	assign u_irq_enable_o = mstatus_q[6] && !(dcsr_q[2] && !dcsr_q[11]);
	assign priv_lvl_o = priv_lvl_q;
	assign sec_lvl_o = priv_lvl_q[0];
	assign fs_off_o = ((FPU == 1) && (ZFINX == 0) ? (mstatus_fs_q == 2'b00 ? 1'b1 : 1'b0) : 1'b0);
	assign frm_o = (FPU == 1 ? frm_q : {3 {1'sb0}});
	assign mtvec_o = mtvec_q;
	assign utvec_o = utvec_q;
	assign mtvec_mode_o = mtvec_mode_q;
	assign utvec_mode_o = utvec_mode_q;
	assign mepc_o = mepc_q;
	assign uepc_o = uepc_q;
	assign mcounteren_o = (PULP_SECURE ? mcounteren_q : {32 {1'sb0}});
	assign depc_o = depc_q;
	assign pmp_addr_o = pmp_reg_q[767-:512];
	assign pmp_cfg_o = pmp_reg_q[127-:128];
	assign debug_single_step_o = dcsr_q[2];
	assign debug_ebreakm_o = dcsr_q[15];
	assign debug_ebreaku_o = dcsr_q[12];
	generate
		if (PULP_SECURE == 1) begin : gen_pmp_user
			for (_gv_j_1 = 0; _gv_j_1 < N_PMP_ENTRIES; _gv_j_1 = _gv_j_1 + 1) begin : CS_PMP_CFG
				localparam j = _gv_j_1;
				wire [8:1] sv2v_tmp_C98C8;
				assign sv2v_tmp_C98C8 = pmp_reg_n[128 + (((j / 4) * 32) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (8 * ((j % 4) + 1)) - 1 : (((8 * ((j % 4) + 1)) - 1) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)) - 1))-:(((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)];
				always @(*) pmp_reg_n[0 + (j * 8)+:8] = sv2v_tmp_C98C8;
				wire [(((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1) * 1:1] sv2v_tmp_864C8;
				assign sv2v_tmp_864C8 = pmp_reg_q[0 + (j * 8)+:8];
				always @(*) pmp_reg_q[128 + (((j / 4) * 32) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (8 * ((j % 4) + 1)) - 1 : (((8 * ((j % 4) + 1)) - 1) + (((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)) - 1))-:(((8 * ((j % 4) + 1)) - 1) >= (8 * (j % 4)) ? (((8 * ((j % 4) + 1)) - 1) - (8 * (j % 4))) + 1 : ((8 * (j % 4)) - ((8 * ((j % 4) + 1)) - 1)) + 1)] = sv2v_tmp_864C8;
			end
			for (_gv_j_1 = 0; _gv_j_1 < N_PMP_ENTRIES; _gv_j_1 = _gv_j_1 + 1) begin : CS_PMP_REGS_FF
				localparam j = _gv_j_1;
				always @(posedge clk or negedge rst_n)
					if (rst_n == 1'b0) begin
						pmp_reg_q[0 + (j * 8)+:8] <= 1'sb0;
						pmp_reg_q[256 + (j * 32)+:32] <= 1'sb0;
					end
					else begin
						if (pmpcfg_we[j])
							pmp_reg_q[0 + (j * 8)+:8] <= (USE_PMP ? pmp_reg_n[0 + (j * 8)+:8] : {8 {1'sb0}});
						if (pmpaddr_we[j])
							pmp_reg_q[256 + (j * 32)+:32] <= (USE_PMP ? pmp_reg_n[256 + (j * 32)+:32] : {32 {1'sb0}});
					end
			end
			always @(posedge clk or negedge rst_n)
				if (rst_n == 1'b0) begin
					uepc_q <= 1'sb0;
					ucause_q <= 1'sb0;
					utvec_q <= 1'sb0;
					utvec_mode_q <= MTVEC_MODE;
					priv_lvl_q <= 2'b11;
				end
				else begin
					uepc_q <= uepc_n;
					ucause_q <= ucause_n;
					utvec_q <= utvec_n;
					utvec_mode_q <= utvec_mode_n;
					priv_lvl_q <= priv_lvl_n;
				end
		end
		else begin : gen_no_pmp_user
			wire [768:1] sv2v_tmp_78853;
			assign sv2v_tmp_78853 = 1'sb0;
			always @(*) pmp_reg_q = sv2v_tmp_78853;
			wire [32:1] sv2v_tmp_6C649;
			assign sv2v_tmp_6C649 = 1'sb0;
			always @(*) uepc_q = sv2v_tmp_6C649;
			wire [6:1] sv2v_tmp_1A9D8;
			assign sv2v_tmp_1A9D8 = 1'sb0;
			always @(*) ucause_q = sv2v_tmp_1A9D8;
			wire [24:1] sv2v_tmp_CC076;
			assign sv2v_tmp_CC076 = 1'sb0;
			always @(*) utvec_q = sv2v_tmp_CC076;
			wire [2:1] sv2v_tmp_C675D;
			assign sv2v_tmp_C675D = 1'sb0;
			always @(*) utvec_mode_q = sv2v_tmp_C675D;
			wire [2:1] sv2v_tmp_16422;
			assign sv2v_tmp_16422 = 2'b11;
			always @(*) priv_lvl_q = sv2v_tmp_16422;
		end
	endgenerate
	localparam cv32e40p_pkg_DBG_CAUSE_NONE = 3'h0;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			if (FPU == 1) begin
				frm_q <= 1'sb0;
				fflags_q <= 1'sb0;
				if (ZFINX == 0)
					mstatus_fs_q <= 2'b00;
			end
			mstatus_q <= 7'b0000110;
			mepc_q <= 1'sb0;
			mcause_q <= 1'sb0;
			depc_q <= 1'sb0;
			dcsr_q <= {23'h200000, cv32e40p_pkg_DBG_CAUSE_NONE, 6'b000011};
			dscratch0_q <= 1'sb0;
			dscratch1_q <= 1'sb0;
			mscratch_q <= 1'sb0;
			mie_q <= 1'sb0;
			mtvec_q <= 1'sb0;
			mtvec_mode_q <= MTVEC_MODE;
		end
		else begin
			if (FPU == 1) begin
				frm_q <= frm_n;
				fflags_q <= fflags_n;
				if (ZFINX == 0)
					mstatus_fs_q <= mstatus_fs_n;
			end
			if (PULP_SECURE == 1)
				mstatus_q <= mstatus_n;
			else
				mstatus_q <= {1'b0, mstatus_n[5], 1'b0, mstatus_n[3], 3'b110};
			mepc_q <= mepc_n;
			mcause_q <= mcause_n;
			depc_q <= depc_n;
			dcsr_q <= dcsr_n;
			dscratch0_q <= dscratch0_n;
			dscratch1_q <= dscratch1_n;
			mscratch_q <= mscratch_n;
			mie_q <= mie_n;
			mtvec_q <= mtvec_n;
			mtvec_mode_q <= mtvec_mode_n;
		end
	generate
		if (DEBUG_TRIGGER_EN) begin : gen_trigger_regs
			reg tmatch_control_exec_q;
			reg [31:0] tmatch_value_q;
			wire tmatch_control_we;
			wire tmatch_value_we;
			assign tmatch_control_we = (csr_we_int & debug_mode_i) & (csr_addr_i == 12'h7a1);
			assign tmatch_value_we = (csr_we_int & debug_mode_i) & (csr_addr_i == 12'h7a2);
			always @(posedge clk or negedge rst_n)
				if (!rst_n) begin
					tmatch_control_exec_q <= 'b0;
					tmatch_value_q <= 'b0;
				end
				else begin
					if (tmatch_control_we)
						tmatch_control_exec_q <= csr_wdata_int[2];
					if (tmatch_value_we)
						tmatch_value_q <= csr_wdata_int[31:0];
				end
			assign tinfo_types = 4;
			assign tmatch_control_rdata = {28'h2800104, PULP_SECURE == 1, tmatch_control_exec_q, 2'b00};
			assign tmatch_value_rdata = tmatch_value_q;
			assign trigger_match_o = tmatch_control_exec_q & (pc_id_i[31:0] == tmatch_value_q[31:0]);
		end
		else begin : gen_no_trigger_regs
			assign tinfo_types = 'b0;
			assign tmatch_control_rdata = 'b0;
			assign tmatch_value_rdata = 'b0;
			assign trigger_match_o = 'b0;
		end
	endgenerate
	assign hpm_events[0] = 1'b1;
	assign hpm_events[1] = mhpmevent_minstret_i;
	assign hpm_events[2] = mhpmevent_ld_stall_i;
	assign hpm_events[3] = mhpmevent_jr_stall_i;
	assign hpm_events[4] = mhpmevent_imiss_i;
	assign hpm_events[5] = mhpmevent_load_i;
	assign hpm_events[6] = mhpmevent_store_i;
	assign hpm_events[7] = mhpmevent_jump_i;
	assign hpm_events[8] = mhpmevent_branch_i;
	assign hpm_events[9] = mhpmevent_branch_taken_i;
	assign hpm_events[10] = mhpmevent_compressed_i;
	assign hpm_events[11] = (COREV_CLUSTER ? mhpmevent_pipe_stall_i : 1'b0);
	assign hpm_events[12] = (!APU ? 1'b0 : apu_typeconflict_i && !apu_dep_i);
	assign hpm_events[13] = (!APU ? 1'b0 : apu_contention_i);
	assign hpm_events[14] = (!APU ? 1'b0 : apu_dep_i && !apu_contention_i);
	assign hpm_events[15] = (!APU ? 1'b0 : apu_wb_i);
	wire mcounteren_we;
	wire mcountinhibit_we;
	wire mhpmevent_we;
	assign mcounteren_we = csr_we_int & (csr_addr_i == 12'h306);
	assign mcountinhibit_we = csr_we_int & (csr_addr_i == 12'h320);
	assign mhpmevent_we = csr_we_int & (((((((((((((((((((((((((((((csr_addr_i == 12'h323) || (csr_addr_i == 12'h324)) || (csr_addr_i == 12'h325)) || (csr_addr_i == 12'h326)) || (csr_addr_i == 12'h327)) || (csr_addr_i == 12'h328)) || (csr_addr_i == 12'h329)) || (csr_addr_i == 12'h32a)) || (csr_addr_i == 12'h32b)) || (csr_addr_i == 12'h32c)) || (csr_addr_i == 12'h32d)) || (csr_addr_i == 12'h32e)) || (csr_addr_i == 12'h32f)) || (csr_addr_i == 12'h330)) || (csr_addr_i == 12'h331)) || (csr_addr_i == 12'h332)) || (csr_addr_i == 12'h333)) || (csr_addr_i == 12'h334)) || (csr_addr_i == 12'h335)) || (csr_addr_i == 12'h336)) || (csr_addr_i == 12'h337)) || (csr_addr_i == 12'h338)) || (csr_addr_i == 12'h339)) || (csr_addr_i == 12'h33a)) || (csr_addr_i == 12'h33b)) || (csr_addr_i == 12'h33c)) || (csr_addr_i == 12'h33d)) || (csr_addr_i == 12'h33e)) || (csr_addr_i == 12'h33f));
	genvar _gv_incr_gidx_1;
	generate
		for (_gv_incr_gidx_1 = 0; _gv_incr_gidx_1 < 32; _gv_incr_gidx_1 = _gv_incr_gidx_1 + 1) begin : gen_mhpmcounter_increment
			localparam incr_gidx = _gv_incr_gidx_1;
			assign mhpmcounter_increment[incr_gidx * 64+:64] = mhpmcounter_q[incr_gidx * 64+:64] + 1;
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		mcounteren_n = mcounteren_q;
		mcountinhibit_n = mcountinhibit_q;
		mhpmevent_n = mhpmevent_q;
		if (PULP_SECURE && mcounteren_we)
			mcounteren_n = csr_wdata_int;
		if (mcountinhibit_we)
			mcountinhibit_n = csr_wdata_int;
		if (mhpmevent_we)
			mhpmevent_n[csr_addr_i[4:0] * 32+:32] = csr_wdata_int;
	end
	genvar _gv_wcnt_gidx_1;
	generate
		for (_gv_wcnt_gidx_1 = 0; _gv_wcnt_gidx_1 < 32; _gv_wcnt_gidx_1 = _gv_wcnt_gidx_1 + 1) begin : gen_mhpmcounter_write
			localparam wcnt_gidx = _gv_wcnt_gidx_1;
			assign mhpmcounter_write_lower[wcnt_gidx] = csr_we_int && (csr_addr_i == (12'hb00 + wcnt_gidx));
			assign mhpmcounter_write_upper[wcnt_gidx] = ((!mhpmcounter_write_lower[wcnt_gidx] && csr_we_int) && (csr_addr_i == (12'hb80 + wcnt_gidx))) && 1'd1;
			if (!PULP_PERF_COUNTERS) begin : gen_no_pulp_perf_counters
				if (wcnt_gidx == 0) begin : gen_mhpmcounter_mcycle
					assign mhpmcounter_write_increment[wcnt_gidx] = (!mhpmcounter_write_lower[wcnt_gidx] && !mhpmcounter_write_upper[wcnt_gidx]) && !mcountinhibit_q[wcnt_gidx];
				end
				else if (wcnt_gidx == 2) begin : gen_mhpmcounter_minstret
					assign mhpmcounter_write_increment[wcnt_gidx] = ((!mhpmcounter_write_lower[wcnt_gidx] && !mhpmcounter_write_upper[wcnt_gidx]) && !mcountinhibit_q[wcnt_gidx]) && hpm_events[1];
				end
				else if ((wcnt_gidx > 2) && (wcnt_gidx < (NUM_MHPMCOUNTERS + 3))) begin : gen_mhpmcounter
					assign mhpmcounter_write_increment[wcnt_gidx] = ((!mhpmcounter_write_lower[wcnt_gidx] && !mhpmcounter_write_upper[wcnt_gidx]) && !mcountinhibit_q[wcnt_gidx]) && |(hpm_events & mhpmevent_q[(wcnt_gidx * 32) + 15-:16]);
				end
				else begin : gen_mhpmcounter_not_implemented
					assign mhpmcounter_write_increment[wcnt_gidx] = 1'b0;
				end
			end
			else begin : gen_pulp_perf_counters
				assign mhpmcounter_write_increment[wcnt_gidx] = ((!mhpmcounter_write_lower[wcnt_gidx] && !mhpmcounter_write_upper[wcnt_gidx]) && !mcountinhibit_q[wcnt_gidx]) && |(hpm_events & mhpmevent_q[(wcnt_gidx * 32) + 15-:16]);
			end
		end
	endgenerate
	genvar _gv_cnt_gidx_1;
	generate
		for (_gv_cnt_gidx_1 = 0; _gv_cnt_gidx_1 < 32; _gv_cnt_gidx_1 = _gv_cnt_gidx_1 + 1) begin : gen_mhpmcounter
			localparam cnt_gidx = _gv_cnt_gidx_1;
			if ((cnt_gidx == 1) || (cnt_gidx >= (NUM_MHPMCOUNTERS + 3))) begin : gen_non_implemented
				wire [64:1] sv2v_tmp_0C563;
				assign sv2v_tmp_0C563 = 'b0;
				always @(*) mhpmcounter_q[cnt_gidx * 64+:64] = sv2v_tmp_0C563;
			end
			else begin : gen_implemented
				always @(posedge clk or negedge rst_n)
					if (!rst_n)
						mhpmcounter_q[cnt_gidx * 64+:64] <= 'b0;
					else if (mhpmcounter_write_lower[cnt_gidx])
						mhpmcounter_q[(cnt_gidx * 64) + 31-:32] <= csr_wdata_int;
					else if (mhpmcounter_write_upper[cnt_gidx])
						mhpmcounter_q[(cnt_gidx * 64) + 63-:32] <= csr_wdata_int;
					else if (mhpmcounter_write_increment[cnt_gidx])
						mhpmcounter_q[cnt_gidx * 64+:64] <= mhpmcounter_increment[cnt_gidx * 64+:64];
			end
		end
	endgenerate
	genvar _gv_evt_gidx_1;
	generate
		for (_gv_evt_gidx_1 = 0; _gv_evt_gidx_1 < 32; _gv_evt_gidx_1 = _gv_evt_gidx_1 + 1) begin : gen_mhpmevent
			localparam evt_gidx = _gv_evt_gidx_1;
			if ((evt_gidx < 3) || (evt_gidx >= (NUM_MHPMCOUNTERS + 3))) begin : gen_non_implemented
				wire [32:1] sv2v_tmp_D8BE3;
				assign sv2v_tmp_D8BE3 = 'b0;
				always @(*) mhpmevent_q[evt_gidx * 32+:32] = sv2v_tmp_D8BE3;
			end
			else begin : gen_implemented
				if (1) begin : gen_tie_off
					wire [16:1] sv2v_tmp_84EE5;
					assign sv2v_tmp_84EE5 = 'b0;
					always @(*) mhpmevent_q[(evt_gidx * 32) + 31-:16] = sv2v_tmp_84EE5;
				end
				always @(posedge clk or negedge rst_n)
					if (!rst_n)
						mhpmevent_q[(evt_gidx * 32) + 15-:16] <= 'b0;
					else
						mhpmevent_q[(evt_gidx * 32) + 15-:16] <= mhpmevent_n[(evt_gidx * 32) + 15-:16];
			end
		end
	endgenerate
	genvar _gv_en_gidx_1;
	generate
		for (_gv_en_gidx_1 = 0; _gv_en_gidx_1 < 32; _gv_en_gidx_1 = _gv_en_gidx_1 + 1) begin : gen_mcounteren
			localparam en_gidx = _gv_en_gidx_1;
			if (((PULP_SECURE == 0) || (en_gidx == 1)) || (en_gidx >= (NUM_MHPMCOUNTERS + 3))) begin : gen_non_implemented
				wire [1:1] sv2v_tmp_A6AAD;
				assign sv2v_tmp_A6AAD = 'b0;
				always @(*) mcounteren_q[en_gidx] = sv2v_tmp_A6AAD;
			end
			else begin : gen_implemented
				always @(posedge clk or negedge rst_n)
					if (!rst_n)
						mcounteren_q[en_gidx] <= 'b0;
					else
						mcounteren_q[en_gidx] <= mcounteren_n[en_gidx];
			end
		end
	endgenerate
	genvar _gv_inh_gidx_1;
	generate
		for (_gv_inh_gidx_1 = 0; _gv_inh_gidx_1 < 32; _gv_inh_gidx_1 = _gv_inh_gidx_1 + 1) begin : gen_mcountinhibit
			localparam inh_gidx = _gv_inh_gidx_1;
			if ((inh_gidx == 1) || (inh_gidx >= (NUM_MHPMCOUNTERS + 3))) begin : gen_non_implemented
				wire [1:1] sv2v_tmp_91B94;
				assign sv2v_tmp_91B94 = 'b0;
				always @(*) mcountinhibit_q[inh_gidx] = sv2v_tmp_91B94;
			end
			else begin : gen_implemented
				always @(posedge clk or negedge rst_n)
					if (!rst_n)
						mcountinhibit_q[inh_gidx] <= 'b1;
					else
						mcountinhibit_q[inh_gidx] <= mcountinhibit_n[inh_gidx];
			end
		end
	endgenerate
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_register_file (
	clk,
	rst_n,
	scan_cg_en_i,
	raddr_a_i,
	rdata_a_o,
	raddr_b_i,
	rdata_b_o,
	raddr_c_i,
	rdata_c_o,
	waddr_a_i,
	wdata_a_i,
	we_a_i,
	waddr_b_i,
	wdata_b_i,
	we_b_i
);
	parameter ADDR_WIDTH = 5;
	parameter DATA_WIDTH = 32;
	parameter FPU = 0;
	parameter ZFINX = 0;
	input wire clk;
	input wire rst_n;
	input wire scan_cg_en_i;
	input wire [ADDR_WIDTH - 1:0] raddr_a_i;
	output wire [DATA_WIDTH - 1:0] rdata_a_o;
	input wire [ADDR_WIDTH - 1:0] raddr_b_i;
	output wire [DATA_WIDTH - 1:0] rdata_b_o;
	input wire [ADDR_WIDTH - 1:0] raddr_c_i;
	output wire [DATA_WIDTH - 1:0] rdata_c_o;
	input wire [ADDR_WIDTH - 1:0] waddr_a_i;
	input wire [DATA_WIDTH - 1:0] wdata_a_i;
	input wire we_a_i;
	input wire [ADDR_WIDTH - 1:0] waddr_b_i;
	input wire [DATA_WIDTH - 1:0] wdata_b_i;
	input wire we_b_i;
	localparam NUM_WORDS = 2 ** (ADDR_WIDTH - 1);
	localparam NUM_FP_WORDS = 2 ** (ADDR_WIDTH - 1);
	localparam NUM_TOT_WORDS = (FPU ? (ZFINX ? NUM_WORDS : NUM_WORDS + NUM_FP_WORDS) : NUM_WORDS);
	reg [(NUM_WORDS * DATA_WIDTH) - 1:0] mem;
	reg [(NUM_FP_WORDS * DATA_WIDTH) - 1:0] mem_fp;
	wire [ADDR_WIDTH - 1:0] waddr_a;
	wire [ADDR_WIDTH - 1:0] waddr_b;
	wire [NUM_TOT_WORDS - 1:0] we_a_dec;
	wire [NUM_TOT_WORDS - 1:0] we_b_dec;
	assign rdata_a_o = (raddr_a_i[5] ? mem_fp[raddr_a_i[4:0] * DATA_WIDTH+:DATA_WIDTH] : mem[raddr_a_i[4:0] * DATA_WIDTH+:DATA_WIDTH]);
	assign rdata_b_o = (raddr_b_i[5] ? mem_fp[raddr_b_i[4:0] * DATA_WIDTH+:DATA_WIDTH] : mem[raddr_b_i[4:0] * DATA_WIDTH+:DATA_WIDTH]);
	assign rdata_c_o = (raddr_c_i[5] ? mem_fp[raddr_c_i[4:0] * DATA_WIDTH+:DATA_WIDTH] : mem[raddr_c_i[4:0] * DATA_WIDTH+:DATA_WIDTH]);
	assign waddr_a = waddr_a_i;
	assign waddr_b = waddr_b_i;
	genvar _gv_gidx_1;
	generate
		for (_gv_gidx_1 = 0; _gv_gidx_1 < NUM_TOT_WORDS; _gv_gidx_1 = _gv_gidx_1 + 1) begin : gen_we_decoder
			localparam gidx = _gv_gidx_1;
			assign we_a_dec[gidx] = (waddr_a == gidx ? we_a_i : 1'b0);
			assign we_b_dec[gidx] = (waddr_b == gidx ? we_b_i : 1'b0);
		end
	endgenerate
	genvar _gv_i_1;
	genvar _gv_l_1;
	always @(posedge clk or negedge rst_n)
		if (~rst_n)
			mem[0+:DATA_WIDTH] <= 32'b00000000000000000000000000000000;
		else
			mem[0+:DATA_WIDTH] <= 32'b00000000000000000000000000000000;
	generate
		for (_gv_i_1 = 1; _gv_i_1 < NUM_WORDS; _gv_i_1 = _gv_i_1 + 1) begin : gen_rf
			localparam i = _gv_i_1;
			always @(posedge clk or negedge rst_n) begin : register_write_behavioral
				if (rst_n == 1'b0)
					mem[i * DATA_WIDTH+:DATA_WIDTH] <= 32'b00000000000000000000000000000000;
				else if (we_b_dec[i] == 1'b1)
					mem[i * DATA_WIDTH+:DATA_WIDTH] <= wdata_b_i;
				else if (we_a_dec[i] == 1'b1)
					mem[i * DATA_WIDTH+:DATA_WIDTH] <= wdata_a_i;
			end
		end
		if ((FPU == 1) && (ZFINX == 0)) begin : gen_mem_fp_write
			for (_gv_l_1 = 0; _gv_l_1 < NUM_FP_WORDS; _gv_l_1 = _gv_l_1 + 1) begin : genblk1
				localparam l = _gv_l_1;
				always @(posedge clk or negedge rst_n) begin : fp_regs
					if (rst_n == 1'b0)
						mem_fp[l * DATA_WIDTH+:DATA_WIDTH] <= 1'sb0;
					else if (we_b_dec[l + NUM_WORDS] == 1'b1)
						mem_fp[l * DATA_WIDTH+:DATA_WIDTH] <= wdata_b_i;
					else if (we_a_dec[l + NUM_WORDS] == 1'b1)
						mem_fp[l * DATA_WIDTH+:DATA_WIDTH] <= wdata_a_i;
				end
			end
		end
		else begin : gen_no_mem_fp_write
			wire [NUM_FP_WORDS * DATA_WIDTH:1] sv2v_tmp_6E2A2;
			assign sv2v_tmp_6E2A2 = 'b0;
			always @(*) mem_fp = sv2v_tmp_6E2A2;
		end
	endgenerate
endmodule
module cv32e40p_load_store_unit (
	clk,
	rst_n,
	data_req_o,
	data_gnt_i,
	data_rvalid_i,
	data_err_i,
	data_err_pmp_i,
	data_addr_o,
	data_we_o,
	data_be_o,
	data_wdata_o,
	data_rdata_i,
	data_we_ex_i,
	data_type_ex_i,
	data_wdata_ex_i,
	data_reg_offset_ex_i,
	data_load_event_ex_i,
	data_sign_ext_ex_i,
	data_rdata_ex_o,
	data_req_ex_i,
	operand_a_ex_i,
	operand_b_ex_i,
	addr_useincr_ex_i,
	data_misaligned_ex_i,
	data_misaligned_o,
	data_atop_ex_i,
	data_atop_o,
	p_elw_start_o,
	p_elw_finish_o,
	lsu_ready_ex_o,
	lsu_ready_wb_o,
	busy_o
);
	reg _sv2v_0;
	parameter PULP_OBI = 0;
	input wire clk;
	input wire rst_n;
	output wire data_req_o;
	input wire data_gnt_i;
	input wire data_rvalid_i;
	input wire data_err_i;
	input wire data_err_pmp_i;
	output wire [31:0] data_addr_o;
	output wire data_we_o;
	output wire [3:0] data_be_o;
	output wire [31:0] data_wdata_o;
	input wire [31:0] data_rdata_i;
	input wire data_we_ex_i;
	input wire [1:0] data_type_ex_i;
	input wire [31:0] data_wdata_ex_i;
	input wire [1:0] data_reg_offset_ex_i;
	input wire data_load_event_ex_i;
	input wire [1:0] data_sign_ext_ex_i;
	output wire [31:0] data_rdata_ex_o;
	input wire data_req_ex_i;
	input wire [31:0] operand_a_ex_i;
	input wire [31:0] operand_b_ex_i;
	input wire addr_useincr_ex_i;
	input wire data_misaligned_ex_i;
	output reg data_misaligned_o;
	input wire [5:0] data_atop_ex_i;
	output wire [5:0] data_atop_o;
	output wire p_elw_start_o;
	output wire p_elw_finish_o;
	output wire lsu_ready_ex_o;
	output wire lsu_ready_wb_o;
	output wire busy_o;
	localparam DEPTH = 2;
	wire trans_valid;
	wire trans_ready;
	wire [31:0] trans_addr;
	wire trans_we;
	wire [3:0] trans_be;
	wire [31:0] trans_wdata;
	wire [5:0] trans_atop;
	wire resp_valid;
	wire [31:0] resp_rdata;
	wire resp_err;
	reg [1:0] cnt_q;
	reg [1:0] next_cnt;
	wire count_up;
	wire count_down;
	wire ctrl_update;
	wire [31:0] data_addr_int;
	reg [1:0] data_type_q;
	reg [1:0] rdata_offset_q;
	reg [1:0] data_sign_ext_q;
	reg data_we_q;
	reg data_load_event_q;
	wire [1:0] wdata_offset;
	reg [3:0] data_be;
	reg [31:0] data_wdata;
	wire misaligned_st;
	wire load_err_o;
	wire store_err_o;
	reg [31:0] rdata_q;
	always @(*) begin
		if (_sv2v_0)
			;
		case (data_type_ex_i)
			2'b00:
				if (misaligned_st == 1'b0)
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b1111;
						2'b01: data_be = 4'b1110;
						2'b10: data_be = 4'b1100;
						2'b11: data_be = 4'b1000;
					endcase
				else
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b0000;
						2'b01: data_be = 4'b0001;
						2'b10: data_be = 4'b0011;
						2'b11: data_be = 4'b0111;
					endcase
			2'b01:
				if (misaligned_st == 1'b0)
					case (data_addr_int[1:0])
						2'b00: data_be = 4'b0011;
						2'b01: data_be = 4'b0110;
						2'b10: data_be = 4'b1100;
						2'b11: data_be = 4'b1000;
					endcase
				else
					data_be = 4'b0001;
			2'b10, 2'b11:
				case (data_addr_int[1:0])
					2'b00: data_be = 4'b0001;
					2'b01: data_be = 4'b0010;
					2'b10: data_be = 4'b0100;
					2'b11: data_be = 4'b1000;
				endcase
		endcase
	end
	assign wdata_offset = data_addr_int[1:0] - data_reg_offset_ex_i[1:0];
	always @(*) begin
		if (_sv2v_0)
			;
		case (wdata_offset)
			2'b00: data_wdata = data_wdata_ex_i[31:0];
			2'b01: data_wdata = {data_wdata_ex_i[23:0], data_wdata_ex_i[31:24]};
			2'b10: data_wdata = {data_wdata_ex_i[15:0], data_wdata_ex_i[31:16]};
			2'b11: data_wdata = {data_wdata_ex_i[7:0], data_wdata_ex_i[31:8]};
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			data_type_q <= 1'sb0;
			rdata_offset_q <= 1'sb0;
			data_sign_ext_q <= 1'sb0;
			data_we_q <= 1'b0;
			data_load_event_q <= 1'b0;
		end
		else if (ctrl_update) begin
			data_type_q <= data_type_ex_i;
			rdata_offset_q <= data_addr_int[1:0];
			data_sign_ext_q <= data_sign_ext_ex_i;
			data_we_q <= data_we_ex_i;
			data_load_event_q <= data_load_event_ex_i;
		end
	assign p_elw_start_o = data_load_event_ex_i && data_req_o;
	assign p_elw_finish_o = (data_load_event_q && data_rvalid_i) && !data_misaligned_ex_i;
	reg [31:0] data_rdata_ext;
	reg [31:0] rdata_w_ext;
	reg [31:0] rdata_h_ext;
	reg [31:0] rdata_b_ext;
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00: rdata_w_ext = resp_rdata[31:0];
			2'b01: rdata_w_ext = {resp_rdata[7:0], rdata_q[31:8]};
			2'b10: rdata_w_ext = {resp_rdata[15:0], rdata_q[31:16]};
			2'b11: rdata_w_ext = {resp_rdata[23:0], rdata_q[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, resp_rdata[15:0]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, resp_rdata[15:0]};
				else
					rdata_h_ext = {{16 {resp_rdata[15]}}, resp_rdata[15:0]};
			2'b01:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, resp_rdata[23:8]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, resp_rdata[23:8]};
				else
					rdata_h_ext = {{16 {resp_rdata[23]}}, resp_rdata[23:8]};
			2'b10:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, resp_rdata[31:16]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, resp_rdata[31:16]};
				else
					rdata_h_ext = {{16 {resp_rdata[31]}}, resp_rdata[31:16]};
			2'b11:
				if (data_sign_ext_q == 2'b00)
					rdata_h_ext = {16'h0000, resp_rdata[7:0], rdata_q[31:24]};
				else if (data_sign_ext_q == 2'b10)
					rdata_h_ext = {16'hffff, resp_rdata[7:0], rdata_q[31:24]};
				else
					rdata_h_ext = {{16 {resp_rdata[7]}}, resp_rdata[7:0], rdata_q[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (rdata_offset_q)
			2'b00:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, resp_rdata[7:0]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, resp_rdata[7:0]};
				else
					rdata_b_ext = {{24 {resp_rdata[7]}}, resp_rdata[7:0]};
			2'b01:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, resp_rdata[15:8]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, resp_rdata[15:8]};
				else
					rdata_b_ext = {{24 {resp_rdata[15]}}, resp_rdata[15:8]};
			2'b10:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, resp_rdata[23:16]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, resp_rdata[23:16]};
				else
					rdata_b_ext = {{24 {resp_rdata[23]}}, resp_rdata[23:16]};
			2'b11:
				if (data_sign_ext_q == 2'b00)
					rdata_b_ext = {24'h000000, resp_rdata[31:24]};
				else if (data_sign_ext_q == 2'b10)
					rdata_b_ext = {24'hffffff, resp_rdata[31:24]};
				else
					rdata_b_ext = {{24 {resp_rdata[31]}}, resp_rdata[31:24]};
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		case (data_type_q)
			2'b00: data_rdata_ext = rdata_w_ext;
			2'b01: data_rdata_ext = rdata_h_ext;
			2'b10, 2'b11: data_rdata_ext = rdata_b_ext;
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			rdata_q <= 1'sb0;
		else if (resp_valid && ~data_we_q) begin
			if ((data_misaligned_ex_i == 1'b1) || (data_misaligned_o == 1'b1))
				rdata_q <= resp_rdata;
			else
				rdata_q <= data_rdata_ext;
		end
	assign data_rdata_ex_o = (resp_valid == 1'b1 ? data_rdata_ext : rdata_q);
	assign misaligned_st = data_misaligned_ex_i;
	assign load_err_o = (data_gnt_i && data_err_pmp_i) && ~data_we_o;
	assign store_err_o = (data_gnt_i && data_err_pmp_i) && data_we_o;
	always @(*) begin
		if (_sv2v_0)
			;
		data_misaligned_o = 1'b0;
		if ((data_req_ex_i == 1'b1) && (data_misaligned_ex_i == 1'b0))
			case (data_type_ex_i)
				2'b00:
					if (data_addr_int[1:0] != 2'b00)
						data_misaligned_o = 1'b1;
				2'b01:
					if (data_addr_int[1:0] == 2'b11)
						data_misaligned_o = 1'b1;
			endcase
	end
	assign data_addr_int = (addr_useincr_ex_i ? operand_a_ex_i + operand_b_ex_i : operand_a_ex_i);
	assign busy_o = (cnt_q != 2'b00) || trans_valid;
	assign trans_addr = (data_misaligned_ex_i ? {data_addr_int[31:2], 2'b00} : data_addr_int);
	assign trans_we = data_we_ex_i;
	assign trans_be = data_be;
	assign trans_wdata = data_wdata;
	assign trans_atop = data_atop_ex_i;
	generate
		if (PULP_OBI == 0) begin : gen_no_pulp_obi
			assign trans_valid = data_req_ex_i && (cnt_q < DEPTH);
		end
		else begin : gen_pulp_obi
			assign trans_valid = (cnt_q == 2'b00 ? data_req_ex_i && (cnt_q < DEPTH) : (data_req_ex_i && (cnt_q < DEPTH)) && resp_valid);
		end
	endgenerate
	assign lsu_ready_wb_o = (cnt_q == 2'b00 ? 1'b1 : resp_valid);
	assign lsu_ready_ex_o = (data_req_ex_i == 1'b0 ? 1'b1 : (cnt_q == 2'b00 ? trans_valid && trans_ready : (cnt_q == 2'b01 ? (resp_valid && trans_valid) && trans_ready : resp_valid)));
	assign ctrl_update = lsu_ready_ex_o && data_req_ex_i;
	assign count_up = trans_valid && trans_ready;
	assign count_down = resp_valid;
	always @(*) begin
		if (_sv2v_0)
			;
		case ({count_up, count_down})
			2'b00: next_cnt = cnt_q;
			2'b01: next_cnt = cnt_q - 1'b1;
			2'b10: next_cnt = cnt_q + 1'b1;
			2'b11: next_cnt = cnt_q;
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			cnt_q <= 1'sb0;
		else
			cnt_q <= next_cnt;
	cv32e40p_obi_interface #(.TRANS_STABLE(1)) data_obi_i(
		.clk(clk),
		.rst_n(rst_n),
		.trans_valid_i(trans_valid),
		.trans_ready_o(trans_ready),
		.trans_addr_i(trans_addr),
		.trans_we_i(trans_we),
		.trans_be_i(trans_be),
		.trans_wdata_i(trans_wdata),
		.trans_atop_i(trans_atop),
		.resp_valid_o(resp_valid),
		.resp_rdata_o(resp_rdata),
		.resp_err_o(resp_err),
		.obi_req_o(data_req_o),
		.obi_gnt_i(data_gnt_i),
		.obi_addr_o(data_addr_o),
		.obi_we_o(data_we_o),
		.obi_be_o(data_be_o),
		.obi_wdata_o(data_wdata_o),
		.obi_atop_o(data_atop_o),
		.obi_rdata_i(data_rdata_i),
		.obi_rvalid_i(data_rvalid_i),
		.obi_err_i(data_err_i)
	);
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_id_stage (
	clk,
	clk_ungated_i,
	rst_n,
	scan_cg_en_i,
	fetch_enable_i,
	ctrl_busy_o,
	is_decoding_o,
	instr_valid_i,
	instr_rdata_i,
	instr_req_o,
	is_compressed_i,
	illegal_c_insn_i,
	branch_in_ex_o,
	branch_decision_i,
	jump_target_o,
	ctrl_transfer_insn_in_dec_o,
	clear_instr_valid_o,
	pc_set_o,
	pc_mux_o,
	exc_pc_mux_o,
	trap_addr_mux_o,
	is_fetch_failed_i,
	pc_id_i,
	halt_if_o,
	id_ready_o,
	ex_ready_i,
	wb_ready_i,
	id_valid_o,
	ex_valid_i,
	pc_ex_o,
	alu_operand_a_ex_o,
	alu_operand_b_ex_o,
	alu_operand_c_ex_o,
	bmask_a_ex_o,
	bmask_b_ex_o,
	imm_vec_ext_ex_o,
	alu_vec_mode_ex_o,
	regfile_waddr_ex_o,
	regfile_we_ex_o,
	regfile_alu_waddr_ex_o,
	regfile_alu_we_ex_o,
	alu_en_ex_o,
	alu_operator_ex_o,
	alu_is_clpx_ex_o,
	alu_is_subrot_ex_o,
	alu_clpx_shift_ex_o,
	mult_operator_ex_o,
	mult_operand_a_ex_o,
	mult_operand_b_ex_o,
	mult_operand_c_ex_o,
	mult_en_ex_o,
	mult_sel_subword_ex_o,
	mult_signed_mode_ex_o,
	mult_imm_ex_o,
	mult_dot_op_a_ex_o,
	mult_dot_op_b_ex_o,
	mult_dot_op_c_ex_o,
	mult_dot_signed_ex_o,
	mult_is_clpx_ex_o,
	mult_clpx_shift_ex_o,
	mult_clpx_img_ex_o,
	apu_en_ex_o,
	apu_op_ex_o,
	apu_lat_ex_o,
	apu_operands_ex_o,
	apu_flags_ex_o,
	apu_waddr_ex_o,
	apu_read_regs_o,
	apu_read_regs_valid_o,
	apu_read_dep_i,
	apu_read_dep_for_jalr_i,
	apu_write_regs_o,
	apu_write_regs_valid_o,
	apu_write_dep_i,
	apu_perf_dep_o,
	apu_busy_i,
	fs_off_i,
	frm_i,
	csr_access_ex_o,
	csr_op_ex_o,
	current_priv_lvl_i,
	csr_irq_sec_o,
	csr_cause_o,
	csr_save_if_o,
	csr_save_id_o,
	csr_save_ex_o,
	csr_restore_mret_id_o,
	csr_restore_uret_id_o,
	csr_restore_dret_id_o,
	csr_save_cause_o,
	hwlp_start_o,
	hwlp_end_o,
	hwlp_cnt_o,
	hwlp_jump_o,
	hwlp_target_o,
	data_req_ex_o,
	data_we_ex_o,
	data_type_ex_o,
	data_sign_ext_ex_o,
	data_reg_offset_ex_o,
	data_load_event_ex_o,
	data_misaligned_ex_o,
	prepost_useincr_ex_o,
	data_misaligned_i,
	data_err_i,
	data_err_ack_o,
	atop_ex_o,
	irq_i,
	irq_sec_i,
	mie_bypass_i,
	mip_o,
	m_irq_enable_i,
	u_irq_enable_i,
	irq_ack_o,
	irq_id_o,
	exc_cause_o,
	debug_mode_o,
	debug_cause_o,
	debug_csr_save_o,
	debug_req_i,
	debug_single_step_i,
	debug_ebreakm_i,
	debug_ebreaku_i,
	trigger_match_i,
	debug_p_elw_no_sleep_o,
	debug_havereset_o,
	debug_running_o,
	debug_halted_o,
	wake_from_sleep_o,
	regfile_waddr_wb_i,
	regfile_we_wb_i,
	regfile_we_wb_power_i,
	regfile_wdata_wb_i,
	regfile_alu_waddr_fw_i,
	regfile_alu_we_fw_i,
	regfile_alu_we_fw_power_i,
	regfile_alu_wdata_fw_i,
	mult_multicycle_i,
	mhpmevent_minstret_o,
	mhpmevent_load_o,
	mhpmevent_store_o,
	mhpmevent_jump_o,
	mhpmevent_branch_o,
	mhpmevent_branch_taken_o,
	mhpmevent_compressed_o,
	mhpmevent_jr_stall_o,
	mhpmevent_imiss_o,
	mhpmevent_ld_stall_o,
	mhpmevent_pipe_stall_o,
	perf_imiss_i,
	mcounteren_i
);
	reg _sv2v_0;
	parameter COREV_PULP = 1;
	parameter COREV_CLUSTER = 0;
	parameter N_HWLP = 2;
	parameter N_HWLP_BITS = $clog2(N_HWLP);
	parameter PULP_SECURE = 0;
	parameter USE_PMP = 0;
	parameter A_EXTENSION = 0;
	parameter APU = 0;
	parameter FPU = 0;
	parameter FPU_ADDMUL_LAT = 0;
	parameter FPU_OTHERS_LAT = 0;
	parameter ZFINX = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	parameter DEBUG_TRIGGER_EN = 1;
	input wire clk;
	input wire clk_ungated_i;
	input wire rst_n;
	input wire scan_cg_en_i;
	input wire fetch_enable_i;
	output wire ctrl_busy_o;
	output wire is_decoding_o;
	input wire instr_valid_i;
	input wire [31:0] instr_rdata_i;
	output wire instr_req_o;
	input wire is_compressed_i;
	input wire illegal_c_insn_i;
	output reg branch_in_ex_o;
	input wire branch_decision_i;
	output wire [31:0] jump_target_o;
	output wire [1:0] ctrl_transfer_insn_in_dec_o;
	output wire clear_instr_valid_o;
	output wire pc_set_o;
	output wire [3:0] pc_mux_o;
	output wire [2:0] exc_pc_mux_o;
	output wire [1:0] trap_addr_mux_o;
	input wire is_fetch_failed_i;
	input wire [31:0] pc_id_i;
	output wire halt_if_o;
	output wire id_ready_o;
	input wire ex_ready_i;
	input wire wb_ready_i;
	output wire id_valid_o;
	input wire ex_valid_i;
	output reg [31:0] pc_ex_o;
	output reg [31:0] alu_operand_a_ex_o;
	output reg [31:0] alu_operand_b_ex_o;
	output reg [31:0] alu_operand_c_ex_o;
	output reg [4:0] bmask_a_ex_o;
	output reg [4:0] bmask_b_ex_o;
	output reg [1:0] imm_vec_ext_ex_o;
	output reg [1:0] alu_vec_mode_ex_o;
	output reg [5:0] regfile_waddr_ex_o;
	output reg regfile_we_ex_o;
	output reg [5:0] regfile_alu_waddr_ex_o;
	output reg regfile_alu_we_ex_o;
	output reg alu_en_ex_o;
	localparam cv32e40p_pkg_ALU_OP_WIDTH = 7;
	output reg [6:0] alu_operator_ex_o;
	output reg alu_is_clpx_ex_o;
	output reg alu_is_subrot_ex_o;
	output reg [1:0] alu_clpx_shift_ex_o;
	localparam cv32e40p_pkg_MUL_OP_WIDTH = 3;
	output reg [2:0] mult_operator_ex_o;
	output reg [31:0] mult_operand_a_ex_o;
	output reg [31:0] mult_operand_b_ex_o;
	output reg [31:0] mult_operand_c_ex_o;
	output reg mult_en_ex_o;
	output reg mult_sel_subword_ex_o;
	output reg [1:0] mult_signed_mode_ex_o;
	output reg [4:0] mult_imm_ex_o;
	output reg [31:0] mult_dot_op_a_ex_o;
	output reg [31:0] mult_dot_op_b_ex_o;
	output reg [31:0] mult_dot_op_c_ex_o;
	output reg [1:0] mult_dot_signed_ex_o;
	output reg mult_is_clpx_ex_o;
	output reg [1:0] mult_clpx_shift_ex_o;
	output reg mult_clpx_img_ex_o;
	output reg apu_en_ex_o;
	output reg [APU_WOP_CPU - 1:0] apu_op_ex_o;
	output reg [1:0] apu_lat_ex_o;
	output reg [(APU_NARGS_CPU * 32) - 1:0] apu_operands_ex_o;
	output reg [APU_NDSFLAGS_CPU - 1:0] apu_flags_ex_o;
	output reg [5:0] apu_waddr_ex_o;
	output wire [17:0] apu_read_regs_o;
	output wire [2:0] apu_read_regs_valid_o;
	input wire apu_read_dep_i;
	input wire apu_read_dep_for_jalr_i;
	output wire [11:0] apu_write_regs_o;
	output wire [1:0] apu_write_regs_valid_o;
	input wire apu_write_dep_i;
	output wire apu_perf_dep_o;
	input wire apu_busy_i;
	input wire fs_off_i;
	localparam cv32e40p_pkg_C_RM = 3;
	input wire [2:0] frm_i;
	output reg csr_access_ex_o;
	localparam cv32e40p_pkg_CSR_OP_WIDTH = 2;
	output reg [1:0] csr_op_ex_o;
	input wire [1:0] current_priv_lvl_i;
	output wire csr_irq_sec_o;
	output wire [5:0] csr_cause_o;
	output wire csr_save_if_o;
	output wire csr_save_id_o;
	output wire csr_save_ex_o;
	output wire csr_restore_mret_id_o;
	output wire csr_restore_uret_id_o;
	output wire csr_restore_dret_id_o;
	output wire csr_save_cause_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_start_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_end_o;
	output wire [(N_HWLP * 32) - 1:0] hwlp_cnt_o;
	output wire hwlp_jump_o;
	output wire [31:0] hwlp_target_o;
	output reg data_req_ex_o;
	output reg data_we_ex_o;
	output reg [1:0] data_type_ex_o;
	output reg [1:0] data_sign_ext_ex_o;
	output reg [1:0] data_reg_offset_ex_o;
	output reg data_load_event_ex_o;
	output reg data_misaligned_ex_o;
	output reg prepost_useincr_ex_o;
	input wire data_misaligned_i;
	input wire data_err_i;
	output wire data_err_ack_o;
	output reg [5:0] atop_ex_o;
	input wire [31:0] irq_i;
	input wire irq_sec_i;
	input wire [31:0] mie_bypass_i;
	output wire [31:0] mip_o;
	input wire m_irq_enable_i;
	input wire u_irq_enable_i;
	output wire irq_ack_o;
	output wire [4:0] irq_id_o;
	output wire [4:0] exc_cause_o;
	output wire debug_mode_o;
	output wire [2:0] debug_cause_o;
	output wire debug_csr_save_o;
	input wire debug_req_i;
	input wire debug_single_step_i;
	input wire debug_ebreakm_i;
	input wire debug_ebreaku_i;
	input wire trigger_match_i;
	output wire debug_p_elw_no_sleep_o;
	output wire debug_havereset_o;
	output wire debug_running_o;
	output wire debug_halted_o;
	output wire wake_from_sleep_o;
	input wire [5:0] regfile_waddr_wb_i;
	input wire regfile_we_wb_i;
	input wire regfile_we_wb_power_i;
	input wire [31:0] regfile_wdata_wb_i;
	input wire [5:0] regfile_alu_waddr_fw_i;
	input wire regfile_alu_we_fw_i;
	input wire regfile_alu_we_fw_power_i;
	input wire [31:0] regfile_alu_wdata_fw_i;
	input wire mult_multicycle_i;
	output reg mhpmevent_minstret_o;
	output reg mhpmevent_load_o;
	output reg mhpmevent_store_o;
	output reg mhpmevent_jump_o;
	output reg mhpmevent_branch_o;
	output reg mhpmevent_branch_taken_o;
	output reg mhpmevent_compressed_o;
	output reg mhpmevent_jr_stall_o;
	output reg mhpmevent_imiss_o;
	output reg mhpmevent_ld_stall_o;
	output reg mhpmevent_pipe_stall_o;
	input wire perf_imiss_i;
	input wire [31:0] mcounteren_i;
	localparam REG_S1_MSB = 19;
	localparam REG_S1_LSB = 15;
	localparam REG_S2_MSB = 24;
	localparam REG_S2_LSB = 20;
	localparam REG_S4_MSB = 31;
	localparam REG_S4_LSB = 27;
	localparam REG_D_MSB = 11;
	localparam REG_D_LSB = 7;
	wire [31:0] instr;
	wire deassert_we;
	wire illegal_insn_dec;
	wire ebrk_insn_dec;
	wire mret_insn_dec;
	wire uret_insn_dec;
	wire dret_insn_dec;
	wire ecall_insn_dec;
	wire wfi_insn_dec;
	wire fencei_insn_dec;
	wire rega_used_dec;
	wire regb_used_dec;
	wire regc_used_dec;
	wire branch_taken_ex;
	wire [1:0] ctrl_transfer_insn_in_id;
	wire [1:0] ctrl_transfer_insn_in_dec;
	wire misaligned_stall;
	wire jr_stall;
	wire load_stall;
	wire csr_apu_stall;
	wire hwlp_mask;
	wire halt_id;
	wire halt_if;
	wire debug_wfi_no_sleep;
	wire [31:0] imm_i_type;
	wire [31:0] imm_iz_type;
	wire [31:0] imm_s_type;
	wire [31:0] imm_sb_type;
	wire [31:0] imm_u_type;
	wire [31:0] imm_uj_type;
	wire [31:0] imm_z_type;
	wire [31:0] imm_s2_type;
	wire [31:0] imm_bi_type;
	wire [31:0] imm_s3_type;
	wire [31:0] imm_vs_type;
	wire [31:0] imm_vu_type;
	wire [31:0] imm_shuffleb_type;
	wire [31:0] imm_shuffleh_type;
	reg [31:0] imm_shuffle_type;
	wire [31:0] imm_clip_type;
	reg [31:0] imm_a;
	reg [31:0] imm_b;
	reg [31:0] jump_target;
	wire irq_req_ctrl;
	wire irq_sec_ctrl;
	wire irq_wu_ctrl;
	wire [4:0] irq_id_ctrl;
	wire [5:0] regfile_addr_ra_id;
	wire [5:0] regfile_addr_rb_id;
	reg [5:0] regfile_addr_rc_id;
	wire regfile_fp_a;
	wire regfile_fp_b;
	wire regfile_fp_c;
	wire regfile_fp_d;
	wire [5:0] regfile_waddr_id;
	wire [5:0] regfile_alu_waddr_id;
	wire regfile_alu_we_id;
	wire regfile_alu_we_dec_id;
	wire [31:0] regfile_data_ra_id;
	wire [31:0] regfile_data_rb_id;
	wire [31:0] regfile_data_rc_id;
	wire alu_en;
	wire [6:0] alu_operator;
	wire [2:0] alu_op_a_mux_sel;
	wire [2:0] alu_op_b_mux_sel;
	wire [1:0] alu_op_c_mux_sel;
	wire [1:0] regc_mux;
	wire [0:0] imm_a_mux_sel;
	wire [3:0] imm_b_mux_sel;
	wire [1:0] ctrl_transfer_target_mux_sel;
	wire [2:0] mult_operator;
	wire mult_en;
	wire mult_int_en;
	wire mult_sel_subword;
	wire [1:0] mult_signed_mode;
	wire mult_dot_en;
	wire [1:0] mult_dot_signed;
	localparam [31:0] cv32e40p_fpu_pkg_NUM_FP_FORMATS = 5;
	localparam [31:0] cv32e40p_fpu_pkg_FP_FORMAT_BITS = 3;
	wire [2:0] fpu_src_fmt;
	wire [2:0] fpu_dst_fmt;
	localparam [31:0] cv32e40p_fpu_pkg_NUM_INT_FORMATS = 4;
	localparam [31:0] cv32e40p_fpu_pkg_INT_FORMAT_BITS = 2;
	wire [1:0] fpu_int_fmt;
	wire apu_en;
	wire [APU_WOP_CPU - 1:0] apu_op;
	wire [1:0] apu_lat;
	wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands;
	wire [APU_NDSFLAGS_CPU - 1:0] apu_flags;
	wire [5:0] apu_waddr;
	reg [17:0] apu_read_regs;
	reg [2:0] apu_read_regs_valid;
	wire [11:0] apu_write_regs;
	wire [1:0] apu_write_regs_valid;
	wire apu_stall;
	wire [2:0] fp_rnd_mode;
	wire regfile_we_id;
	wire regfile_alu_waddr_mux_sel;
	wire data_we_id;
	wire [1:0] data_type_id;
	wire [1:0] data_sign_ext_id;
	wire [1:0] data_reg_offset_id;
	wire data_req_id;
	wire data_load_event_id;
	wire [5:0] atop_id;
	wire [N_HWLP_BITS - 1:0] hwlp_regid;
	wire [2:0] hwlp_we;
	wire [2:0] hwlp_we_masked;
	wire [1:0] hwlp_target_mux_sel;
	wire [1:0] hwlp_start_mux_sel;
	wire hwlp_cnt_mux_sel;
	reg [31:0] hwlp_start;
	reg [31:0] hwlp_end;
	reg [31:0] hwlp_cnt;
	wire [N_HWLP - 1:0] hwlp_dec_cnt;
	wire hwlp_valid;
	wire csr_access;
	wire [1:0] csr_op;
	wire csr_status;
	wire prepost_useincr;
	wire [1:0] operand_a_fw_mux_sel;
	wire [1:0] operand_b_fw_mux_sel;
	wire [1:0] operand_c_fw_mux_sel;
	reg [31:0] operand_a_fw_id;
	reg [31:0] operand_b_fw_id;
	reg [31:0] operand_c_fw_id;
	reg [31:0] operand_b;
	reg [31:0] operand_b_vec;
	reg [31:0] operand_c;
	reg [31:0] operand_c_vec;
	reg [31:0] alu_operand_a;
	wire [31:0] alu_operand_b;
	wire [31:0] alu_operand_c;
	wire [0:0] bmask_a_mux;
	wire [1:0] bmask_b_mux;
	wire alu_bmask_a_mux_sel;
	wire alu_bmask_b_mux_sel;
	wire [0:0] mult_imm_mux;
	reg [4:0] bmask_a_id_imm;
	reg [4:0] bmask_b_id_imm;
	reg [4:0] bmask_a_id;
	reg [4:0] bmask_b_id;
	wire [1:0] imm_vec_ext_id;
	reg [4:0] mult_imm_id;
	wire alu_vec;
	wire [1:0] alu_vec_mode;
	wire scalar_replication;
	wire scalar_replication_c;
	wire reg_d_ex_is_reg_a_id;
	wire reg_d_ex_is_reg_b_id;
	wire reg_d_ex_is_reg_c_id;
	wire reg_d_wb_is_reg_a_id;
	wire reg_d_wb_is_reg_b_id;
	wire reg_d_wb_is_reg_c_id;
	wire reg_d_alu_is_reg_a_id;
	wire reg_d_alu_is_reg_b_id;
	wire reg_d_alu_is_reg_c_id;
	wire is_clpx;
	wire is_subrot;
	wire mret_dec;
	wire uret_dec;
	wire dret_dec;
	reg id_valid_q;
	wire minstret;
	wire perf_pipeline_stall;
	assign instr = instr_rdata_i;
	assign imm_i_type = {{20 {instr[31]}}, instr[31:20]};
	assign imm_iz_type = {20'b00000000000000000000, instr[31:20]};
	assign imm_s_type = {{20 {instr[31]}}, instr[31:25], instr[11:7]};
	assign imm_sb_type = {{19 {instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
	assign imm_u_type = {instr[31:12], 12'b000000000000};
	assign imm_uj_type = {{12 {instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
	assign imm_z_type = {27'b000000000000000000000000000, instr[REG_S1_MSB:REG_S1_LSB]};
	assign imm_s2_type = {27'b000000000000000000000000000, instr[24:20]};
	assign imm_bi_type = {{27 {instr[24]}}, instr[24:20]};
	assign imm_s3_type = {27'b000000000000000000000000000, instr[29:25]};
	assign imm_vs_type = {{26 {instr[24]}}, instr[24:20], instr[25]};
	assign imm_vu_type = {26'b00000000000000000000000000, instr[24:20], instr[25]};
	assign imm_shuffleb_type = {6'b000000, instr[28:27], 6'b000000, instr[24:23], 6'b000000, instr[22:21], 6'b000000, instr[20], instr[25]};
	assign imm_shuffleh_type = {15'h0000, instr[20], 15'h0000, instr[25]};
	assign imm_clip_type = (32'h00000001 << instr[24:20]) - 1;
	assign regfile_addr_ra_id = {regfile_fp_a, instr[REG_S1_MSB:REG_S1_LSB]};
	assign regfile_addr_rb_id = {regfile_fp_b, instr[REG_S2_MSB:REG_S2_LSB]};
	localparam cv32e40p_pkg_REGC_RD = 2'b01;
	localparam cv32e40p_pkg_REGC_S1 = 2'b10;
	localparam cv32e40p_pkg_REGC_S4 = 2'b00;
	localparam cv32e40p_pkg_REGC_ZERO = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		case (regc_mux)
			cv32e40p_pkg_REGC_ZERO: regfile_addr_rc_id = 1'sb0;
			cv32e40p_pkg_REGC_RD: regfile_addr_rc_id = {regfile_fp_c, instr[REG_D_MSB:REG_D_LSB]};
			cv32e40p_pkg_REGC_S1: regfile_addr_rc_id = {regfile_fp_c, instr[REG_S1_MSB:REG_S1_LSB]};
			cv32e40p_pkg_REGC_S4: regfile_addr_rc_id = {regfile_fp_c, instr[REG_S4_MSB:REG_S4_LSB]};
		endcase
	end
	assign regfile_waddr_id = {regfile_fp_d, instr[REG_D_MSB:REG_D_LSB]};
	assign regfile_alu_waddr_id = (regfile_alu_waddr_mux_sel ? regfile_waddr_id : regfile_addr_ra_id);
	assign reg_d_ex_is_reg_a_id = ((regfile_waddr_ex_o == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_ex_is_reg_b_id = ((regfile_waddr_ex_o == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_ex_is_reg_c_id = ((regfile_waddr_ex_o == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_a_id = ((regfile_waddr_wb_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_b_id = ((regfile_waddr_wb_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_wb_is_reg_c_id = ((regfile_waddr_wb_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_a_id = ((regfile_alu_waddr_fw_i == regfile_addr_ra_id) && (rega_used_dec == 1'b1)) && (regfile_addr_ra_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_b_id = ((regfile_alu_waddr_fw_i == regfile_addr_rb_id) && (regb_used_dec == 1'b1)) && (regfile_addr_rb_id != {6 {1'sb0}});
	assign reg_d_alu_is_reg_c_id = ((regfile_alu_waddr_fw_i == regfile_addr_rc_id) && (regc_used_dec == 1'b1)) && (regfile_addr_rc_id != {6 {1'sb0}});
	assign clear_instr_valid_o = (id_ready_o | halt_id) | branch_taken_ex;
	assign branch_taken_ex = branch_in_ex_o && branch_decision_i;
	assign mult_en = mult_int_en | mult_dot_en;
	localparam cv32e40p_pkg_JT_COND = 2'b11;
	localparam cv32e40p_pkg_JT_JAL = 2'b01;
	localparam cv32e40p_pkg_JT_JALR = 2'b10;
	always @(*) begin : jump_target_mux
		if (_sv2v_0)
			;
		case (ctrl_transfer_target_mux_sel)
			cv32e40p_pkg_JT_JAL: jump_target = pc_id_i + imm_uj_type;
			cv32e40p_pkg_JT_COND: jump_target = pc_id_i + imm_sb_type;
			cv32e40p_pkg_JT_JALR: jump_target = regfile_data_ra_id + imm_i_type;
			default: jump_target = regfile_data_ra_id + imm_i_type;
		endcase
	end
	assign jump_target_o = jump_target;
	localparam cv32e40p_pkg_OP_A_CURRPC = 3'b001;
	localparam cv32e40p_pkg_OP_A_IMM = 3'b010;
	localparam cv32e40p_pkg_OP_A_REGA_OR_FWD = 3'b000;
	localparam cv32e40p_pkg_OP_A_REGB_OR_FWD = 3'b011;
	localparam cv32e40p_pkg_OP_A_REGC_OR_FWD = 3'b100;
	always @(*) begin : alu_operand_a_mux
		if (_sv2v_0)
			;
		case (alu_op_a_mux_sel)
			cv32e40p_pkg_OP_A_REGA_OR_FWD: alu_operand_a = operand_a_fw_id;
			cv32e40p_pkg_OP_A_REGB_OR_FWD: alu_operand_a = operand_b_fw_id;
			cv32e40p_pkg_OP_A_REGC_OR_FWD: alu_operand_a = operand_c_fw_id;
			cv32e40p_pkg_OP_A_CURRPC: alu_operand_a = pc_id_i;
			cv32e40p_pkg_OP_A_IMM: alu_operand_a = imm_a;
			default: alu_operand_a = operand_a_fw_id;
		endcase
	end
	localparam cv32e40p_pkg_IMMA_Z = 1'b0;
	localparam cv32e40p_pkg_IMMA_ZERO = 1'b1;
	always @(*) begin : immediate_a_mux
		if (_sv2v_0)
			;
		case (imm_a_mux_sel)
			cv32e40p_pkg_IMMA_Z: imm_a = imm_z_type;
			cv32e40p_pkg_IMMA_ZERO: imm_a = 1'sb0;
		endcase
	end
	localparam cv32e40p_pkg_SEL_FW_EX = 2'b01;
	localparam cv32e40p_pkg_SEL_FW_WB = 2'b10;
	localparam cv32e40p_pkg_SEL_REGFILE = 2'b00;
	always @(*) begin : operand_a_fw_mux
		if (_sv2v_0)
			;
		case (operand_a_fw_mux_sel)
			cv32e40p_pkg_SEL_FW_EX: operand_a_fw_id = regfile_alu_wdata_fw_i;
			cv32e40p_pkg_SEL_FW_WB: operand_a_fw_id = regfile_wdata_wb_i;
			cv32e40p_pkg_SEL_REGFILE: operand_a_fw_id = regfile_data_ra_id;
			default: operand_a_fw_id = regfile_data_ra_id;
		endcase
	end
	localparam cv32e40p_pkg_IMMB_BI = 4'b1011;
	localparam cv32e40p_pkg_IMMB_CLIP = 4'b1001;
	localparam cv32e40p_pkg_IMMB_I = 4'b0000;
	localparam cv32e40p_pkg_IMMB_PCINCR = 4'b0011;
	localparam cv32e40p_pkg_IMMB_S = 4'b0001;
	localparam cv32e40p_pkg_IMMB_S2 = 4'b0100;
	localparam cv32e40p_pkg_IMMB_S3 = 4'b0101;
	localparam cv32e40p_pkg_IMMB_SHUF = 4'b1000;
	localparam cv32e40p_pkg_IMMB_U = 4'b0010;
	localparam cv32e40p_pkg_IMMB_VS = 4'b0110;
	localparam cv32e40p_pkg_IMMB_VU = 4'b0111;
	always @(*) begin : immediate_b_mux
		if (_sv2v_0)
			;
		case (imm_b_mux_sel)
			cv32e40p_pkg_IMMB_I: imm_b = imm_i_type;
			cv32e40p_pkg_IMMB_S: imm_b = imm_s_type;
			cv32e40p_pkg_IMMB_U: imm_b = imm_u_type;
			cv32e40p_pkg_IMMB_PCINCR: imm_b = (is_compressed_i ? 32'h00000002 : 32'h00000004);
			cv32e40p_pkg_IMMB_S2: imm_b = imm_s2_type;
			cv32e40p_pkg_IMMB_BI: imm_b = imm_bi_type;
			cv32e40p_pkg_IMMB_S3: imm_b = imm_s3_type;
			cv32e40p_pkg_IMMB_VS: imm_b = imm_vs_type;
			cv32e40p_pkg_IMMB_VU: imm_b = imm_vu_type;
			cv32e40p_pkg_IMMB_SHUF: imm_b = imm_shuffle_type;
			cv32e40p_pkg_IMMB_CLIP: imm_b = {1'b0, imm_clip_type[31:1]};
			default: imm_b = imm_i_type;
		endcase
	end
	localparam cv32e40p_pkg_OP_B_BMASK = 3'b100;
	localparam cv32e40p_pkg_OP_B_IMM = 3'b010;
	localparam cv32e40p_pkg_OP_B_REGA_OR_FWD = 3'b011;
	localparam cv32e40p_pkg_OP_B_REGB_OR_FWD = 3'b000;
	localparam cv32e40p_pkg_OP_B_REGC_OR_FWD = 3'b001;
	always @(*) begin : alu_operand_b_mux
		if (_sv2v_0)
			;
		case (alu_op_b_mux_sel)
			cv32e40p_pkg_OP_B_REGA_OR_FWD: operand_b = operand_a_fw_id;
			cv32e40p_pkg_OP_B_REGB_OR_FWD: operand_b = operand_b_fw_id;
			cv32e40p_pkg_OP_B_REGC_OR_FWD: operand_b = operand_c_fw_id;
			cv32e40p_pkg_OP_B_IMM: operand_b = imm_b;
			cv32e40p_pkg_OP_B_BMASK: operand_b = $unsigned(operand_b_fw_id[4:0]);
			default: operand_b = operand_b_fw_id;
		endcase
	end
	localparam cv32e40p_pkg_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		if (alu_vec_mode == cv32e40p_pkg_VEC_MODE8) begin
			operand_b_vec = {4 {operand_b[7:0]}};
			imm_shuffle_type = imm_shuffleb_type;
		end
		else begin
			operand_b_vec = {2 {operand_b[15:0]}};
			imm_shuffle_type = imm_shuffleh_type;
		end
	end
	assign alu_operand_b = (scalar_replication == 1'b1 ? operand_b_vec : operand_b);
	always @(*) begin : operand_b_fw_mux
		if (_sv2v_0)
			;
		case (operand_b_fw_mux_sel)
			cv32e40p_pkg_SEL_FW_EX: operand_b_fw_id = regfile_alu_wdata_fw_i;
			cv32e40p_pkg_SEL_FW_WB: operand_b_fw_id = regfile_wdata_wb_i;
			cv32e40p_pkg_SEL_REGFILE: operand_b_fw_id = regfile_data_rb_id;
			default: operand_b_fw_id = regfile_data_rb_id;
		endcase
	end
	localparam cv32e40p_pkg_OP_C_JT = 2'b10;
	localparam cv32e40p_pkg_OP_C_REGB_OR_FWD = 2'b01;
	localparam cv32e40p_pkg_OP_C_REGC_OR_FWD = 2'b00;
	always @(*) begin : alu_operand_c_mux
		if (_sv2v_0)
			;
		case (alu_op_c_mux_sel)
			cv32e40p_pkg_OP_C_REGC_OR_FWD: operand_c = operand_c_fw_id;
			cv32e40p_pkg_OP_C_REGB_OR_FWD: operand_c = operand_b_fw_id;
			cv32e40p_pkg_OP_C_JT: operand_c = jump_target;
			default: operand_c = operand_c_fw_id;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		if (alu_vec_mode == cv32e40p_pkg_VEC_MODE8)
			operand_c_vec = {4 {operand_c[7:0]}};
		else
			operand_c_vec = {2 {operand_c[15:0]}};
	end
	assign alu_operand_c = (scalar_replication_c == 1'b1 ? operand_c_vec : operand_c);
	always @(*) begin : operand_c_fw_mux
		if (_sv2v_0)
			;
		case (operand_c_fw_mux_sel)
			cv32e40p_pkg_SEL_FW_EX: operand_c_fw_id = regfile_alu_wdata_fw_i;
			cv32e40p_pkg_SEL_FW_WB: operand_c_fw_id = regfile_wdata_wb_i;
			cv32e40p_pkg_SEL_REGFILE: operand_c_fw_id = regfile_data_rc_id;
			default: operand_c_fw_id = regfile_data_rc_id;
		endcase
	end
	localparam cv32e40p_pkg_BMASK_A_S3 = 1'b1;
	localparam cv32e40p_pkg_BMASK_A_ZERO = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (bmask_a_mux)
			cv32e40p_pkg_BMASK_A_ZERO: bmask_a_id_imm = 1'sb0;
			cv32e40p_pkg_BMASK_A_S3: bmask_a_id_imm = imm_s3_type[4:0];
		endcase
	end
	localparam cv32e40p_pkg_BMASK_B_ONE = 2'b11;
	localparam cv32e40p_pkg_BMASK_B_S2 = 2'b00;
	localparam cv32e40p_pkg_BMASK_B_S3 = 2'b01;
	localparam cv32e40p_pkg_BMASK_B_ZERO = 2'b10;
	always @(*) begin
		if (_sv2v_0)
			;
		case (bmask_b_mux)
			cv32e40p_pkg_BMASK_B_ZERO: bmask_b_id_imm = 1'sb0;
			cv32e40p_pkg_BMASK_B_ONE: bmask_b_id_imm = 5'd1;
			cv32e40p_pkg_BMASK_B_S2: bmask_b_id_imm = imm_s2_type[4:0];
			cv32e40p_pkg_BMASK_B_S3: bmask_b_id_imm = imm_s3_type[4:0];
		endcase
	end
	localparam cv32e40p_pkg_BMASK_A_IMM = 1'b1;
	localparam cv32e40p_pkg_BMASK_A_REG = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (alu_bmask_a_mux_sel)
			cv32e40p_pkg_BMASK_A_IMM: bmask_a_id = bmask_a_id_imm;
			cv32e40p_pkg_BMASK_A_REG: bmask_a_id = operand_b_fw_id[9:5];
		endcase
	end
	localparam cv32e40p_pkg_BMASK_B_IMM = 1'b1;
	localparam cv32e40p_pkg_BMASK_B_REG = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (alu_bmask_b_mux_sel)
			cv32e40p_pkg_BMASK_B_IMM: bmask_b_id = bmask_b_id_imm;
			cv32e40p_pkg_BMASK_B_REG: bmask_b_id = operand_b_fw_id[4:0];
		endcase
	end
	generate
		if (!COREV_PULP) begin : genblk1
			assign imm_vec_ext_id = imm_vu_type[1:0];
		end
		else begin : genblk1
			assign imm_vec_ext_id = (alu_vec ? imm_vu_type[1:0] : 2'b00);
		end
	endgenerate
	localparam cv32e40p_pkg_MIMM_S3 = 1'b1;
	localparam cv32e40p_pkg_MIMM_ZERO = 1'b0;
	always @(*) begin
		if (_sv2v_0)
			;
		case (mult_imm_mux)
			cv32e40p_pkg_MIMM_ZERO: mult_imm_id = 1'sb0;
			cv32e40p_pkg_MIMM_S3: mult_imm_id = imm_s3_type[4:0];
		endcase
	end
	generate
		if (APU == 1) begin : gen_apu
			if (APU_NARGS_CPU >= 1) begin : genblk1
				assign apu_operands[0+:32] = alu_operand_a;
			end
			if (APU_NARGS_CPU >= 2) begin : genblk2
				assign apu_operands[32+:32] = alu_operand_b;
			end
			if (APU_NARGS_CPU >= 3) begin : genblk3
				assign apu_operands[64+:32] = alu_operand_c;
			end
			assign apu_waddr = regfile_alu_waddr_id;
			assign apu_flags = (FPU == 1 ? {fpu_int_fmt, fpu_src_fmt, fpu_dst_fmt, fp_rnd_mode} : {APU_NDSFLAGS_CPU {1'sb0}});
			always @(*) begin
				if (_sv2v_0)
					;
				case (alu_op_a_mux_sel)
					cv32e40p_pkg_OP_A_CURRPC:
						if (ctrl_transfer_target_mux_sel == cv32e40p_pkg_JT_JALR) begin
							apu_read_regs[0+:6] = regfile_addr_ra_id;
							apu_read_regs_valid[0] = 1'b1;
						end
						else begin
							apu_read_regs[0+:6] = regfile_addr_ra_id;
							apu_read_regs_valid[0] = 1'b0;
						end
					cv32e40p_pkg_OP_A_REGA_OR_FWD: begin
						apu_read_regs[0+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[0] = 1'b1;
					end
					cv32e40p_pkg_OP_A_REGB_OR_FWD, cv32e40p_pkg_OP_A_REGC_OR_FWD: begin
						apu_read_regs[0+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[0] = 1'b1;
					end
					default: begin
						apu_read_regs[0+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[0] = 1'b0;
					end
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				case (alu_op_b_mux_sel)
					cv32e40p_pkg_OP_B_REGA_OR_FWD: begin
						apu_read_regs[6+:6] = regfile_addr_ra_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					cv32e40p_pkg_OP_B_REGB_OR_FWD, cv32e40p_pkg_OP_B_BMASK: begin
						apu_read_regs[6+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					cv32e40p_pkg_OP_B_REGC_OR_FWD: begin
						apu_read_regs[6+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[1] = 1'b1;
					end
					cv32e40p_pkg_OP_B_IMM:
						if (alu_bmask_b_mux_sel == cv32e40p_pkg_BMASK_B_REG) begin
							apu_read_regs[6+:6] = regfile_addr_rb_id;
							apu_read_regs_valid[1] = 1'b1;
						end
						else begin
							apu_read_regs[6+:6] = regfile_addr_rb_id;
							apu_read_regs_valid[1] = 1'b0;
						end
					default: begin
						apu_read_regs[6+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[1] = 1'b0;
					end
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				case (alu_op_c_mux_sel)
					cv32e40p_pkg_OP_C_REGB_OR_FWD: begin
						apu_read_regs[12+:6] = regfile_addr_rb_id;
						apu_read_regs_valid[2] = 1'b1;
					end
					cv32e40p_pkg_OP_C_REGC_OR_FWD:
						if ((((alu_op_a_mux_sel != cv32e40p_pkg_OP_A_REGC_OR_FWD) && (ctrl_transfer_target_mux_sel != cv32e40p_pkg_JT_JALR)) && !((alu_op_b_mux_sel == cv32e40p_pkg_OP_B_IMM) && (alu_bmask_b_mux_sel == cv32e40p_pkg_BMASK_B_REG))) && (alu_op_b_mux_sel != cv32e40p_pkg_OP_B_BMASK)) begin
							apu_read_regs[12+:6] = regfile_addr_rc_id;
							apu_read_regs_valid[2] = 1'b1;
						end
						else begin
							apu_read_regs[12+:6] = regfile_addr_rc_id;
							apu_read_regs_valid[2] = 1'b0;
						end
					default: begin
						apu_read_regs[12+:6] = regfile_addr_rc_id;
						apu_read_regs_valid[2] = 1'b0;
					end
				endcase
			end
			assign apu_write_regs[0+:6] = regfile_alu_waddr_id;
			assign apu_write_regs_valid[0] = regfile_alu_we_id;
			assign apu_write_regs[6+:6] = regfile_waddr_id;
			assign apu_write_regs_valid[1] = regfile_we_id;
			assign apu_read_regs_o = apu_read_regs;
			assign apu_read_regs_valid_o = apu_read_regs_valid;
			assign apu_write_regs_o = apu_write_regs;
			assign apu_write_regs_valid_o = apu_write_regs_valid;
		end
		else begin : gen_no_apu
			genvar _gv_i_2;
			for (_gv_i_2 = 0; _gv_i_2 < APU_NARGS_CPU; _gv_i_2 = _gv_i_2 + 1) begin : gen_apu_tie_off
				localparam i = _gv_i_2;
				assign apu_operands[i * 32+:32] = 1'sb0;
			end
			wire [18:1] sv2v_tmp_C6C4C;
			assign sv2v_tmp_C6C4C = 1'sb0;
			always @(*) apu_read_regs = sv2v_tmp_C6C4C;
			wire [3:1] sv2v_tmp_C13AD;
			assign sv2v_tmp_C13AD = 1'sb0;
			always @(*) apu_read_regs_valid = sv2v_tmp_C13AD;
			assign apu_write_regs = 1'sb0;
			assign apu_write_regs_valid = 1'sb0;
			assign apu_waddr = 1'sb0;
			assign apu_flags = 1'sb0;
			assign apu_write_regs_o = 1'sb0;
			assign apu_read_regs_o = 1'sb0;
			assign apu_write_regs_valid_o = 1'sb0;
			assign apu_read_regs_valid_o = 1'sb0;
		end
	endgenerate
	assign apu_perf_dep_o = apu_stall;
	assign csr_apu_stall = csr_access & ((apu_en_ex_o & (apu_lat_ex_o[1] == 1'b1)) | apu_busy_i);
	cv32e40p_register_file #(
		.ADDR_WIDTH(6),
		.DATA_WIDTH(32),
		.FPU(FPU),
		.ZFINX(ZFINX)
	) register_file_i(
		.clk(clk),
		.rst_n(rst_n),
		.scan_cg_en_i(scan_cg_en_i),
		.raddr_a_i(regfile_addr_ra_id),
		.rdata_a_o(regfile_data_ra_id),
		.raddr_b_i(regfile_addr_rb_id),
		.rdata_b_o(regfile_data_rb_id),
		.raddr_c_i(regfile_addr_rc_id),
		.rdata_c_o(regfile_data_rc_id),
		.waddr_a_i(regfile_waddr_wb_i),
		.wdata_a_i(regfile_wdata_wb_i),
		.we_a_i(regfile_we_wb_power_i),
		.waddr_b_i(regfile_alu_waddr_fw_i),
		.wdata_b_i(regfile_alu_wdata_fw_i),
		.we_b_i(regfile_alu_we_fw_power_i)
	);
	cv32e40p_decoder #(
		.COREV_PULP(COREV_PULP),
		.COREV_CLUSTER(COREV_CLUSTER),
		.A_EXTENSION(A_EXTENSION),
		.FPU(FPU),
		.FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
		.FPU_OTHERS_LAT(FPU_OTHERS_LAT),
		.ZFINX(ZFINX),
		.PULP_SECURE(PULP_SECURE),
		.USE_PMP(USE_PMP),
		.APU_WOP_CPU(APU_WOP_CPU),
		.DEBUG_TRIGGER_EN(DEBUG_TRIGGER_EN)
	) decoder_i(
		.deassert_we_i(deassert_we),
		.illegal_insn_o(illegal_insn_dec),
		.ebrk_insn_o(ebrk_insn_dec),
		.mret_insn_o(mret_insn_dec),
		.uret_insn_o(uret_insn_dec),
		.dret_insn_o(dret_insn_dec),
		.mret_dec_o(mret_dec),
		.uret_dec_o(uret_dec),
		.dret_dec_o(dret_dec),
		.ecall_insn_o(ecall_insn_dec),
		.wfi_o(wfi_insn_dec),
		.fencei_insn_o(fencei_insn_dec),
		.rega_used_o(rega_used_dec),
		.regb_used_o(regb_used_dec),
		.regc_used_o(regc_used_dec),
		.reg_fp_a_o(regfile_fp_a),
		.reg_fp_b_o(regfile_fp_b),
		.reg_fp_c_o(regfile_fp_c),
		.reg_fp_d_o(regfile_fp_d),
		.bmask_a_mux_o(bmask_a_mux),
		.bmask_b_mux_o(bmask_b_mux),
		.alu_bmask_a_mux_sel_o(alu_bmask_a_mux_sel),
		.alu_bmask_b_mux_sel_o(alu_bmask_b_mux_sel),
		.instr_rdata_i(instr),
		.illegal_c_insn_i(illegal_c_insn_i),
		.alu_en_o(alu_en),
		.alu_operator_o(alu_operator),
		.alu_op_a_mux_sel_o(alu_op_a_mux_sel),
		.alu_op_b_mux_sel_o(alu_op_b_mux_sel),
		.alu_op_c_mux_sel_o(alu_op_c_mux_sel),
		.alu_vec_o(alu_vec),
		.alu_vec_mode_o(alu_vec_mode),
		.scalar_replication_o(scalar_replication),
		.scalar_replication_c_o(scalar_replication_c),
		.imm_a_mux_sel_o(imm_a_mux_sel),
		.imm_b_mux_sel_o(imm_b_mux_sel),
		.regc_mux_o(regc_mux),
		.is_clpx_o(is_clpx),
		.is_subrot_o(is_subrot),
		.mult_operator_o(mult_operator),
		.mult_int_en_o(mult_int_en),
		.mult_sel_subword_o(mult_sel_subword),
		.mult_signed_mode_o(mult_signed_mode),
		.mult_imm_mux_o(mult_imm_mux),
		.mult_dot_en_o(mult_dot_en),
		.mult_dot_signed_o(mult_dot_signed),
		.fs_off_i(fs_off_i),
		.frm_i(frm_i),
		.fpu_src_fmt_o(fpu_src_fmt),
		.fpu_dst_fmt_o(fpu_dst_fmt),
		.fpu_int_fmt_o(fpu_int_fmt),
		.apu_en_o(apu_en),
		.apu_op_o(apu_op),
		.apu_lat_o(apu_lat),
		.fp_rnd_mode_o(fp_rnd_mode),
		.regfile_mem_we_o(regfile_we_id),
		.regfile_alu_we_o(regfile_alu_we_id),
		.regfile_alu_we_dec_o(regfile_alu_we_dec_id),
		.regfile_alu_waddr_sel_o(regfile_alu_waddr_mux_sel),
		.csr_access_o(csr_access),
		.csr_status_o(csr_status),
		.csr_op_o(csr_op),
		.current_priv_lvl_i(current_priv_lvl_i),
		.data_req_o(data_req_id),
		.data_we_o(data_we_id),
		.prepost_useincr_o(prepost_useincr),
		.data_type_o(data_type_id),
		.data_sign_extension_o(data_sign_ext_id),
		.data_reg_offset_o(data_reg_offset_id),
		.data_load_event_o(data_load_event_id),
		.atop_o(atop_id),
		.hwlp_we_o(hwlp_we),
		.hwlp_target_mux_sel_o(hwlp_target_mux_sel),
		.hwlp_start_mux_sel_o(hwlp_start_mux_sel),
		.hwlp_cnt_mux_sel_o(hwlp_cnt_mux_sel),
		.debug_mode_i(debug_mode_o),
		.debug_wfi_no_sleep_i(debug_wfi_no_sleep),
		.ctrl_transfer_insn_in_dec_o(ctrl_transfer_insn_in_dec_o),
		.ctrl_transfer_insn_in_id_o(ctrl_transfer_insn_in_id),
		.ctrl_transfer_target_mux_sel_o(ctrl_transfer_target_mux_sel),
		.mcounteren_i(mcounteren_i)
	);
	cv32e40p_controller #(
		.COREV_CLUSTER(COREV_CLUSTER),
		.COREV_PULP(COREV_PULP),
		.FPU(FPU)
	) controller_i(
		.clk(clk),
		.clk_ungated_i(clk_ungated_i),
		.rst_n(rst_n),
		.fetch_enable_i(fetch_enable_i),
		.ctrl_busy_o(ctrl_busy_o),
		.is_decoding_o(is_decoding_o),
		.is_fetch_failed_i(is_fetch_failed_i),
		.deassert_we_o(deassert_we),
		.illegal_insn_i(illegal_insn_dec),
		.ecall_insn_i(ecall_insn_dec),
		.mret_insn_i(mret_insn_dec),
		.uret_insn_i(uret_insn_dec),
		.dret_insn_i(dret_insn_dec),
		.mret_dec_i(mret_dec),
		.uret_dec_i(uret_dec),
		.dret_dec_i(dret_dec),
		.wfi_i(wfi_insn_dec),
		.ebrk_insn_i(ebrk_insn_dec),
		.fencei_insn_i(fencei_insn_dec),
		.csr_status_i(csr_status),
		.hwlp_mask_o(hwlp_mask),
		.instr_valid_i(instr_valid_i),
		.instr_req_o(instr_req_o),
		.pc_set_o(pc_set_o),
		.pc_mux_o(pc_mux_o),
		.exc_pc_mux_o(exc_pc_mux_o),
		.exc_cause_o(exc_cause_o),
		.trap_addr_mux_o(trap_addr_mux_o),
		.pc_id_i(pc_id_i),
		.hwlp_start_addr_i(hwlp_start_o),
		.hwlp_end_addr_i(hwlp_end_o),
		.hwlp_counter_i(hwlp_cnt_o),
		.hwlp_dec_cnt_o(hwlp_dec_cnt),
		.hwlp_jump_o(hwlp_jump_o),
		.hwlp_targ_addr_o(hwlp_target_o),
		.data_req_ex_i(data_req_ex_o),
		.data_we_ex_i(data_we_ex_o),
		.data_misaligned_i(data_misaligned_i),
		.data_load_event_i(data_load_event_id),
		.data_err_i(data_err_i),
		.data_err_ack_o(data_err_ack_o),
		.mult_multicycle_i(mult_multicycle_i),
		.apu_en_i(apu_en),
		.apu_read_dep_i(apu_read_dep_i),
		.apu_read_dep_for_jalr_i(apu_read_dep_for_jalr_i),
		.apu_write_dep_i(apu_write_dep_i),
		.apu_stall_o(apu_stall),
		.branch_taken_ex_i(branch_taken_ex),
		.ctrl_transfer_insn_in_id_i(ctrl_transfer_insn_in_id),
		.ctrl_transfer_insn_in_dec_i(ctrl_transfer_insn_in_dec_o),
		.irq_wu_ctrl_i(irq_wu_ctrl),
		.irq_req_ctrl_i(irq_req_ctrl),
		.irq_sec_ctrl_i(irq_sec_ctrl),
		.irq_id_ctrl_i(irq_id_ctrl),
		.current_priv_lvl_i(current_priv_lvl_i),
		.irq_ack_o(irq_ack_o),
		.irq_id_o(irq_id_o),
		.debug_mode_o(debug_mode_o),
		.debug_cause_o(debug_cause_o),
		.debug_csr_save_o(debug_csr_save_o),
		.debug_req_i(debug_req_i),
		.debug_single_step_i(debug_single_step_i),
		.debug_ebreakm_i(debug_ebreakm_i),
		.debug_ebreaku_i(debug_ebreaku_i),
		.trigger_match_i(trigger_match_i),
		.debug_p_elw_no_sleep_o(debug_p_elw_no_sleep_o),
		.debug_wfi_no_sleep_o(debug_wfi_no_sleep),
		.debug_havereset_o(debug_havereset_o),
		.debug_running_o(debug_running_o),
		.debug_halted_o(debug_halted_o),
		.wake_from_sleep_o(wake_from_sleep_o),
		.csr_save_cause_o(csr_save_cause_o),
		.csr_cause_o(csr_cause_o),
		.csr_save_if_o(csr_save_if_o),
		.csr_save_id_o(csr_save_id_o),
		.csr_save_ex_o(csr_save_ex_o),
		.csr_restore_mret_id_o(csr_restore_mret_id_o),
		.csr_restore_uret_id_o(csr_restore_uret_id_o),
		.csr_restore_dret_id_o(csr_restore_dret_id_o),
		.csr_irq_sec_o(csr_irq_sec_o),
		.regfile_we_id_i(regfile_alu_we_dec_id),
		.regfile_alu_waddr_id_i(regfile_alu_waddr_id),
		.regfile_we_ex_i(regfile_we_ex_o),
		.regfile_waddr_ex_i(regfile_waddr_ex_o),
		.regfile_we_wb_i(regfile_we_wb_i),
		.regfile_alu_we_fw_i(regfile_alu_we_fw_i),
		.reg_d_ex_is_reg_a_i(reg_d_ex_is_reg_a_id),
		.reg_d_ex_is_reg_b_i(reg_d_ex_is_reg_b_id),
		.reg_d_ex_is_reg_c_i(reg_d_ex_is_reg_c_id),
		.reg_d_wb_is_reg_a_i(reg_d_wb_is_reg_a_id),
		.reg_d_wb_is_reg_b_i(reg_d_wb_is_reg_b_id),
		.reg_d_wb_is_reg_c_i(reg_d_wb_is_reg_c_id),
		.reg_d_alu_is_reg_a_i(reg_d_alu_is_reg_a_id),
		.reg_d_alu_is_reg_b_i(reg_d_alu_is_reg_b_id),
		.reg_d_alu_is_reg_c_i(reg_d_alu_is_reg_c_id),
		.operand_a_fw_mux_sel_o(operand_a_fw_mux_sel),
		.operand_b_fw_mux_sel_o(operand_b_fw_mux_sel),
		.operand_c_fw_mux_sel_o(operand_c_fw_mux_sel),
		.halt_if_o(halt_if),
		.halt_id_o(halt_id),
		.misaligned_stall_o(misaligned_stall),
		.jr_stall_o(jr_stall),
		.load_stall_o(load_stall),
		.id_ready_i(id_ready_o),
		.id_valid_i(id_valid_o),
		.ex_valid_i(ex_valid_i),
		.wb_ready_i(wb_ready_i),
		.perf_pipeline_stall_o(perf_pipeline_stall)
	);
	cv32e40p_int_controller #(.PULP_SECURE(PULP_SECURE)) int_controller_i(
		.clk(clk),
		.rst_n(rst_n),
		.irq_i(irq_i),
		.irq_sec_i(irq_sec_i),
		.irq_req_ctrl_o(irq_req_ctrl),
		.irq_sec_ctrl_o(irq_sec_ctrl),
		.irq_id_ctrl_o(irq_id_ctrl),
		.irq_wu_ctrl_o(irq_wu_ctrl),
		.mie_bypass_i(mie_bypass_i),
		.mip_o(mip_o),
		.m_ie_i(m_irq_enable_i),
		.u_ie_i(u_irq_enable_i),
		.current_priv_lvl_i(current_priv_lvl_i)
	);
	generate
		if (COREV_PULP) begin : gen_hwloop_regs
			cv32e40p_hwloop_regs #(.N_REGS(N_HWLP)) hwloop_regs_i(
				.clk(clk),
				.rst_n(rst_n),
				.hwlp_start_data_i(hwlp_start),
				.hwlp_end_data_i(hwlp_end),
				.hwlp_cnt_data_i(hwlp_cnt),
				.hwlp_we_i(hwlp_we_masked),
				.hwlp_regid_i(hwlp_regid),
				.valid_i(hwlp_valid),
				.hwlp_start_addr_o(hwlp_start_o),
				.hwlp_end_addr_o(hwlp_end_o),
				.hwlp_counter_o(hwlp_cnt_o),
				.hwlp_dec_cnt_i(hwlp_dec_cnt)
			);
			assign hwlp_valid = instr_valid_i & clear_instr_valid_o;
			assign hwlp_regid = instr[7];
			always @(*) begin
				if (_sv2v_0)
					;
				case (hwlp_target_mux_sel)
					2'b00: hwlp_end = pc_id_i + {imm_iz_type[29:0], 2'b00};
					2'b01: hwlp_end = pc_id_i + {imm_z_type[29:0], 2'b00};
					2'b10: hwlp_end = operand_a_fw_id;
					default: hwlp_end = operand_a_fw_id;
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				case (hwlp_start_mux_sel)
					2'b00: hwlp_start = hwlp_end;
					2'b01: hwlp_start = pc_id_i + 4;
					2'b10: hwlp_start = operand_a_fw_id;
					default: hwlp_start = operand_a_fw_id;
				endcase
			end
			always @(*) begin : hwlp_cnt_mux
				if (_sv2v_0)
					;
				case (hwlp_cnt_mux_sel)
					1'b0: hwlp_cnt = imm_iz_type;
					1'b1: hwlp_cnt = operand_a_fw_id;
				endcase
			end
			assign hwlp_we_masked = (hwlp_we & ~{3 {hwlp_mask}}) & {3 {id_ready_o}};
		end
		else begin : gen_no_hwloop_regs
			assign hwlp_start_o = 'b0;
			assign hwlp_end_o = 'b0;
			assign hwlp_cnt_o = 'b0;
			assign hwlp_valid = 'b0;
			assign hwlp_we_masked = 'b0;
			wire [32:1] sv2v_tmp_46680;
			assign sv2v_tmp_46680 = 'b0;
			always @(*) hwlp_start = sv2v_tmp_46680;
			wire [32:1] sv2v_tmp_A8F69;
			assign sv2v_tmp_A8F69 = 'b0;
			always @(*) hwlp_end = sv2v_tmp_A8F69;
			wire [32:1] sv2v_tmp_79FCB;
			assign sv2v_tmp_79FCB = 'b0;
			always @(*) hwlp_cnt = sv2v_tmp_79FCB;
			assign hwlp_regid = 'b0;
		end
	endgenerate
	localparam cv32e40p_pkg_BRANCH_COND = 2'b11;
	function automatic [6:0] sv2v_cast_576C1;
		input reg [6:0] inp;
		sv2v_cast_576C1 = inp;
	endfunction
	function automatic [2:0] sv2v_cast_9D1C7;
		input reg [2:0] inp;
		sv2v_cast_9D1C7 = inp;
	endfunction
	function automatic [1:0] sv2v_cast_315CD;
		input reg [1:0] inp;
		sv2v_cast_315CD = inp;
	endfunction
	always @(posedge clk or negedge rst_n) begin : ID_EX_PIPE_REGISTERS
		if (rst_n == 1'b0) begin
			alu_en_ex_o <= 1'sb0;
			alu_operator_ex_o <= sv2v_cast_576C1(7'b0000011);
			alu_operand_a_ex_o <= 1'sb0;
			alu_operand_b_ex_o <= 1'sb0;
			alu_operand_c_ex_o <= 1'sb0;
			bmask_a_ex_o <= 1'sb0;
			bmask_b_ex_o <= 1'sb0;
			imm_vec_ext_ex_o <= 1'sb0;
			alu_vec_mode_ex_o <= 1'sb0;
			alu_clpx_shift_ex_o <= 2'b00;
			alu_is_clpx_ex_o <= 1'b0;
			alu_is_subrot_ex_o <= 1'b0;
			mult_operator_ex_o <= sv2v_cast_9D1C7(3'b000);
			mult_operand_a_ex_o <= 1'sb0;
			mult_operand_b_ex_o <= 1'sb0;
			mult_operand_c_ex_o <= 1'sb0;
			mult_en_ex_o <= 1'b0;
			mult_sel_subword_ex_o <= 1'b0;
			mult_signed_mode_ex_o <= 2'b00;
			mult_imm_ex_o <= 1'sb0;
			mult_dot_op_a_ex_o <= 1'sb0;
			mult_dot_op_b_ex_o <= 1'sb0;
			mult_dot_op_c_ex_o <= 1'sb0;
			mult_dot_signed_ex_o <= 1'sb0;
			mult_is_clpx_ex_o <= 1'b0;
			mult_clpx_shift_ex_o <= 2'b00;
			mult_clpx_img_ex_o <= 1'b0;
			apu_en_ex_o <= 1'sb0;
			apu_op_ex_o <= 1'sb0;
			apu_lat_ex_o <= 1'sb0;
			apu_operands_ex_o[0+:32] <= 1'sb0;
			apu_operands_ex_o[32+:32] <= 1'sb0;
			apu_operands_ex_o[64+:32] <= 1'sb0;
			apu_flags_ex_o <= 1'sb0;
			apu_waddr_ex_o <= 1'sb0;
			regfile_waddr_ex_o <= 6'b000000;
			regfile_we_ex_o <= 1'b0;
			regfile_alu_waddr_ex_o <= 6'b000000;
			regfile_alu_we_ex_o <= 1'b0;
			prepost_useincr_ex_o <= 1'b0;
			csr_access_ex_o <= 1'b0;
			csr_op_ex_o <= sv2v_cast_315CD(2'b00);
			data_we_ex_o <= 1'b0;
			data_type_ex_o <= 2'b00;
			data_sign_ext_ex_o <= 2'b00;
			data_reg_offset_ex_o <= 2'b00;
			data_req_ex_o <= 1'b0;
			data_load_event_ex_o <= 1'b0;
			atop_ex_o <= 5'b00000;
			data_misaligned_ex_o <= 1'b0;
			pc_ex_o <= 1'sb0;
			branch_in_ex_o <= 1'b0;
		end
		else if (data_misaligned_i) begin
			if (ex_ready_i) begin
				if (prepost_useincr_ex_o == 1'b1)
					alu_operand_a_ex_o <= operand_a_fw_id;
				alu_operand_b_ex_o <= 32'h00000004;
				regfile_alu_we_ex_o <= 1'b0;
				prepost_useincr_ex_o <= 1'b1;
				data_misaligned_ex_o <= 1'b1;
			end
		end
		else if (mult_multicycle_i)
			mult_operand_c_ex_o <= operand_c_fw_id;
		else if (id_valid_o) begin
			alu_en_ex_o <= alu_en;
			if (alu_en) begin
				alu_operator_ex_o <= alu_operator;
				alu_operand_a_ex_o <= alu_operand_a;
				alu_operand_b_ex_o <= alu_operand_b;
				alu_operand_c_ex_o <= alu_operand_c;
				bmask_a_ex_o <= bmask_a_id;
				bmask_b_ex_o <= bmask_b_id;
				imm_vec_ext_ex_o <= imm_vec_ext_id;
				alu_vec_mode_ex_o <= alu_vec_mode;
				alu_is_clpx_ex_o <= is_clpx;
				alu_clpx_shift_ex_o <= instr[14:13];
				alu_is_subrot_ex_o <= is_subrot;
			end
			mult_en_ex_o <= mult_en;
			if (mult_int_en) begin
				mult_operator_ex_o <= mult_operator;
				mult_sel_subword_ex_o <= mult_sel_subword;
				mult_signed_mode_ex_o <= mult_signed_mode;
				mult_operand_a_ex_o <= alu_operand_a;
				mult_operand_b_ex_o <= alu_operand_b;
				mult_operand_c_ex_o <= alu_operand_c;
				mult_imm_ex_o <= mult_imm_id;
			end
			if (mult_dot_en) begin
				mult_operator_ex_o <= mult_operator;
				mult_dot_signed_ex_o <= mult_dot_signed;
				mult_dot_op_a_ex_o <= alu_operand_a;
				mult_dot_op_b_ex_o <= alu_operand_b;
				mult_dot_op_c_ex_o <= alu_operand_c;
				mult_is_clpx_ex_o <= is_clpx;
				mult_clpx_shift_ex_o <= instr[14:13];
				mult_clpx_img_ex_o <= instr[25];
			end
			apu_en_ex_o <= apu_en;
			if (apu_en) begin
				apu_op_ex_o <= apu_op;
				apu_lat_ex_o <= apu_lat;
				apu_operands_ex_o <= apu_operands;
				apu_flags_ex_o <= apu_flags;
				apu_waddr_ex_o <= apu_waddr;
			end
			regfile_we_ex_o <= regfile_we_id;
			if (regfile_we_id)
				regfile_waddr_ex_o <= regfile_waddr_id;
			regfile_alu_we_ex_o <= regfile_alu_we_id;
			if (regfile_alu_we_id)
				regfile_alu_waddr_ex_o <= regfile_alu_waddr_id;
			prepost_useincr_ex_o <= prepost_useincr;
			csr_access_ex_o <= csr_access;
			csr_op_ex_o <= csr_op;
			data_req_ex_o <= data_req_id;
			if (data_req_id) begin
				data_we_ex_o <= data_we_id;
				data_type_ex_o <= data_type_id;
				data_sign_ext_ex_o <= data_sign_ext_id;
				data_reg_offset_ex_o <= data_reg_offset_id;
				data_load_event_ex_o <= data_load_event_id;
				atop_ex_o <= atop_id;
			end
			else
				data_load_event_ex_o <= 1'b0;
			data_misaligned_ex_o <= 1'b0;
			if ((ctrl_transfer_insn_in_id == cv32e40p_pkg_BRANCH_COND) || data_req_id)
				pc_ex_o <= pc_id_i;
			branch_in_ex_o <= ctrl_transfer_insn_in_id == cv32e40p_pkg_BRANCH_COND;
		end
		else if (ex_ready_i) begin
			regfile_we_ex_o <= 1'b0;
			regfile_alu_we_ex_o <= 1'b0;
			csr_op_ex_o <= sv2v_cast_315CD(2'b00);
			data_req_ex_o <= 1'b0;
			data_load_event_ex_o <= 1'b0;
			data_misaligned_ex_o <= 1'b0;
			branch_in_ex_o <= 1'b0;
			apu_en_ex_o <= 1'b0;
			alu_operator_ex_o <= sv2v_cast_576C1(7'b0000011);
			mult_en_ex_o <= 1'b0;
			alu_en_ex_o <= 1'b1;
		end
		else if (csr_access_ex_o)
			regfile_alu_we_ex_o <= 1'b0;
	end
	assign minstret = (id_valid_o && is_decoding_o) && !((illegal_insn_dec || ebrk_insn_dec) || ecall_insn_dec);
	localparam cv32e40p_pkg_BRANCH_JAL = 2'b01;
	localparam cv32e40p_pkg_BRANCH_JALR = 2'b10;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			id_valid_q <= 1'b0;
			mhpmevent_minstret_o <= 1'b0;
			mhpmevent_load_o <= 1'b0;
			mhpmevent_store_o <= 1'b0;
			mhpmevent_jump_o <= 1'b0;
			mhpmevent_branch_o <= 1'b0;
			mhpmevent_compressed_o <= 1'b0;
			mhpmevent_branch_taken_o <= 1'b0;
			mhpmevent_jr_stall_o <= 1'b0;
			mhpmevent_imiss_o <= 1'b0;
			mhpmevent_ld_stall_o <= 1'b0;
			mhpmevent_pipe_stall_o <= 1'b0;
		end
		else begin
			id_valid_q <= id_valid_o;
			mhpmevent_minstret_o <= minstret;
			mhpmevent_load_o <= (minstret && data_req_id) && !data_we_id;
			mhpmevent_store_o <= (minstret && data_req_id) && data_we_id;
			mhpmevent_jump_o <= minstret && ((ctrl_transfer_insn_in_id == cv32e40p_pkg_BRANCH_JAL) || (ctrl_transfer_insn_in_id == cv32e40p_pkg_BRANCH_JALR));
			mhpmevent_branch_o <= minstret && (ctrl_transfer_insn_in_id == cv32e40p_pkg_BRANCH_COND);
			mhpmevent_compressed_o <= minstret && is_compressed_i;
			mhpmevent_branch_taken_o <= mhpmevent_branch_o && branch_decision_i;
			mhpmevent_imiss_o <= perf_imiss_i;
			mhpmevent_jr_stall_o <= (jr_stall && !halt_id) && id_valid_q;
			mhpmevent_ld_stall_o <= (load_stall && !halt_id) && id_valid_q;
			mhpmevent_pipe_stall_o <= perf_pipeline_stall;
		end
	assign id_ready_o = ((((~misaligned_stall & ~jr_stall) & ~load_stall) & ~apu_stall) & ~csr_apu_stall) & ex_ready_i;
	assign id_valid_o = ~halt_id & id_ready_o;
	assign halt_if_o = halt_if;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_aligner (
	clk,
	rst_n,
	fetch_valid_i,
	aligner_ready_o,
	if_valid_i,
	fetch_rdata_i,
	instr_aligned_o,
	instr_valid_o,
	branch_addr_i,
	branch_i,
	hwlp_addr_i,
	hwlp_update_pc_i,
	pc_o
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire fetch_valid_i;
	output reg aligner_ready_o;
	input wire if_valid_i;
	input wire [31:0] fetch_rdata_i;
	output reg [31:0] instr_aligned_o;
	output reg instr_valid_o;
	input wire [31:0] branch_addr_i;
	input wire branch_i;
	input wire [31:0] hwlp_addr_i;
	input wire hwlp_update_pc_i;
	output wire [31:0] pc_o;
	reg [2:0] state;
	reg [2:0] next_state;
	reg [15:0] r_instr_h;
	reg [31:0] hwlp_addr_q;
	reg [31:0] pc_q;
	reg [31:0] pc_n;
	reg update_state;
	wire [31:0] pc_plus4;
	wire [31:0] pc_plus2;
	reg aligner_ready_q;
	reg hwlp_update_pc_q;
	assign pc_o = pc_q;
	assign pc_plus2 = pc_q + 2;
	assign pc_plus4 = pc_q + 4;
	always @(posedge clk or negedge rst_n) begin : proc_SEQ_FSM
		if (~rst_n) begin
			state <= 3'd0;
			r_instr_h <= 1'sb0;
			hwlp_addr_q <= 1'sb0;
			pc_q <= 1'sb0;
			aligner_ready_q <= 1'b0;
			hwlp_update_pc_q <= 1'b0;
		end
		else if (update_state) begin
			pc_q <= pc_n;
			state <= next_state;
			r_instr_h <= fetch_rdata_i[31:16];
			aligner_ready_q <= aligner_ready_o;
			hwlp_update_pc_q <= 1'b0;
		end
		else if (hwlp_update_pc_i) begin
			hwlp_addr_q <= hwlp_addr_i;
			hwlp_update_pc_q <= 1'b1;
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		pc_n = pc_q;
		instr_valid_o = fetch_valid_i;
		instr_aligned_o = fetch_rdata_i;
		aligner_ready_o = 1'b1;
		update_state = 1'b0;
		next_state = state;
		case (state)
			3'd0:
				if (fetch_rdata_i[1:0] == 2'b11) begin
					next_state = 3'd0;
					pc_n = pc_plus4;
					instr_aligned_o = fetch_rdata_i;
					update_state = fetch_valid_i & if_valid_i;
					if (hwlp_update_pc_i || hwlp_update_pc_q)
						pc_n = (hwlp_update_pc_i ? hwlp_addr_i : hwlp_addr_q);
				end
				else begin
					next_state = 3'd1;
					pc_n = pc_plus2;
					instr_aligned_o = fetch_rdata_i;
					update_state = fetch_valid_i & if_valid_i;
				end
			3'd1:
				if (r_instr_h[1:0] == 2'b11) begin
					next_state = 3'd1;
					pc_n = pc_plus4;
					instr_aligned_o = {fetch_rdata_i[15:0], r_instr_h[15:0]};
					update_state = fetch_valid_i & if_valid_i;
				end
				else begin
					instr_aligned_o = {fetch_rdata_i[31:16], r_instr_h[15:0]};
					next_state = 3'd2;
					instr_valid_o = 1'b1;
					pc_n = pc_plus2;
					aligner_ready_o = !fetch_valid_i;
					update_state = if_valid_i;
				end
			3'd2: begin
				instr_valid_o = !aligner_ready_q || fetch_valid_i;
				if (fetch_rdata_i[1:0] == 2'b11) begin
					next_state = 3'd0;
					pc_n = pc_plus4;
					instr_aligned_o = fetch_rdata_i;
					update_state = (!aligner_ready_q | fetch_valid_i) & if_valid_i;
				end
				else begin
					next_state = 3'd1;
					pc_n = pc_plus2;
					instr_aligned_o = fetch_rdata_i;
					update_state = (!aligner_ready_q | fetch_valid_i) & if_valid_i;
				end
			end
			3'd3:
				if (fetch_rdata_i[17:16] == 2'b11) begin
					next_state = 3'd1;
					instr_valid_o = 1'b0;
					pc_n = pc_q;
					instr_aligned_o = fetch_rdata_i;
					update_state = fetch_valid_i & if_valid_i;
				end
				else begin
					next_state = 3'd0;
					pc_n = pc_plus2;
					instr_aligned_o = {fetch_rdata_i[31:16], fetch_rdata_i[31:16]};
					update_state = fetch_valid_i & if_valid_i;
				end
		endcase
		if (branch_i) begin
			update_state = 1'b1;
			pc_n = branch_addr_i;
			next_state = (branch_addr_i[1] ? 3'd3 : 3'd0);
		end
	end
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_decoder (
	deassert_we_i,
	illegal_insn_o,
	ebrk_insn_o,
	mret_insn_o,
	uret_insn_o,
	dret_insn_o,
	mret_dec_o,
	uret_dec_o,
	dret_dec_o,
	ecall_insn_o,
	wfi_o,
	fencei_insn_o,
	rega_used_o,
	regb_used_o,
	regc_used_o,
	reg_fp_a_o,
	reg_fp_b_o,
	reg_fp_c_o,
	reg_fp_d_o,
	bmask_a_mux_o,
	bmask_b_mux_o,
	alu_bmask_a_mux_sel_o,
	alu_bmask_b_mux_sel_o,
	instr_rdata_i,
	illegal_c_insn_i,
	alu_en_o,
	alu_operator_o,
	alu_op_a_mux_sel_o,
	alu_op_b_mux_sel_o,
	alu_op_c_mux_sel_o,
	alu_vec_o,
	alu_vec_mode_o,
	scalar_replication_o,
	scalar_replication_c_o,
	imm_a_mux_sel_o,
	imm_b_mux_sel_o,
	regc_mux_o,
	is_clpx_o,
	is_subrot_o,
	mult_operator_o,
	mult_int_en_o,
	mult_dot_en_o,
	mult_imm_mux_o,
	mult_sel_subword_o,
	mult_signed_mode_o,
	mult_dot_signed_o,
	fs_off_i,
	frm_i,
	fpu_dst_fmt_o,
	fpu_src_fmt_o,
	fpu_int_fmt_o,
	apu_en_o,
	apu_op_o,
	apu_lat_o,
	fp_rnd_mode_o,
	regfile_mem_we_o,
	regfile_alu_we_o,
	regfile_alu_we_dec_o,
	regfile_alu_waddr_sel_o,
	csr_access_o,
	csr_status_o,
	csr_op_o,
	current_priv_lvl_i,
	data_req_o,
	data_we_o,
	prepost_useincr_o,
	data_type_o,
	data_sign_extension_o,
	data_reg_offset_o,
	data_load_event_o,
	atop_o,
	hwlp_we_o,
	hwlp_target_mux_sel_o,
	hwlp_start_mux_sel_o,
	hwlp_cnt_mux_sel_o,
	debug_mode_i,
	debug_wfi_no_sleep_i,
	ctrl_transfer_insn_in_dec_o,
	ctrl_transfer_insn_in_id_o,
	ctrl_transfer_target_mux_sel_o,
	mcounteren_i
);
	reg _sv2v_0;
	parameter COREV_PULP = 1;
	parameter COREV_CLUSTER = 0;
	parameter A_EXTENSION = 0;
	parameter FPU = 0;
	parameter FPU_ADDMUL_LAT = 0;
	parameter FPU_OTHERS_LAT = 0;
	parameter ZFINX = 0;
	parameter PULP_SECURE = 0;
	parameter USE_PMP = 0;
	parameter APU_WOP_CPU = 6;
	parameter DEBUG_TRIGGER_EN = 1;
	input wire deassert_we_i;
	output reg illegal_insn_o;
	output reg ebrk_insn_o;
	output reg mret_insn_o;
	output reg uret_insn_o;
	output reg dret_insn_o;
	output reg mret_dec_o;
	output reg uret_dec_o;
	output reg dret_dec_o;
	output reg ecall_insn_o;
	output reg wfi_o;
	output reg fencei_insn_o;
	output reg rega_used_o;
	output reg regb_used_o;
	output reg regc_used_o;
	output reg reg_fp_a_o;
	output reg reg_fp_b_o;
	output reg reg_fp_c_o;
	output reg reg_fp_d_o;
	output reg [0:0] bmask_a_mux_o;
	output reg [1:0] bmask_b_mux_o;
	output reg alu_bmask_a_mux_sel_o;
	output reg alu_bmask_b_mux_sel_o;
	input wire [31:0] instr_rdata_i;
	input wire illegal_c_insn_i;
	output wire alu_en_o;
	localparam cv32e40p_pkg_ALU_OP_WIDTH = 7;
	output reg [6:0] alu_operator_o;
	output reg [2:0] alu_op_a_mux_sel_o;
	output reg [2:0] alu_op_b_mux_sel_o;
	output reg [1:0] alu_op_c_mux_sel_o;
	output reg alu_vec_o;
	output reg [1:0] alu_vec_mode_o;
	output reg scalar_replication_o;
	output reg scalar_replication_c_o;
	output reg [0:0] imm_a_mux_sel_o;
	output reg [3:0] imm_b_mux_sel_o;
	output reg [1:0] regc_mux_o;
	output reg is_clpx_o;
	output reg is_subrot_o;
	localparam cv32e40p_pkg_MUL_OP_WIDTH = 3;
	output reg [2:0] mult_operator_o;
	output wire mult_int_en_o;
	output wire mult_dot_en_o;
	output reg [0:0] mult_imm_mux_o;
	output reg mult_sel_subword_o;
	output reg [1:0] mult_signed_mode_o;
	output reg [1:0] mult_dot_signed_o;
	input wire fs_off_i;
	localparam cv32e40p_pkg_C_RM = 3;
	input wire [2:0] frm_i;
	localparam [31:0] cv32e40p_fpu_pkg_NUM_FP_FORMATS = 5;
	localparam [31:0] cv32e40p_fpu_pkg_FP_FORMAT_BITS = 3;
	output reg [2:0] fpu_dst_fmt_o;
	output reg [2:0] fpu_src_fmt_o;
	localparam [31:0] cv32e40p_fpu_pkg_NUM_INT_FORMATS = 4;
	localparam [31:0] cv32e40p_fpu_pkg_INT_FORMAT_BITS = 2;
	output reg [1:0] fpu_int_fmt_o;
	output wire apu_en_o;
	output reg [APU_WOP_CPU - 1:0] apu_op_o;
	output reg [1:0] apu_lat_o;
	output reg [2:0] fp_rnd_mode_o;
	output wire regfile_mem_we_o;
	output wire regfile_alu_we_o;
	output wire regfile_alu_we_dec_o;
	output reg regfile_alu_waddr_sel_o;
	output reg csr_access_o;
	output reg csr_status_o;
	localparam cv32e40p_pkg_CSR_OP_WIDTH = 2;
	output wire [1:0] csr_op_o;
	input wire [1:0] current_priv_lvl_i;
	output wire data_req_o;
	output reg data_we_o;
	output reg prepost_useincr_o;
	output reg [1:0] data_type_o;
	output reg [1:0] data_sign_extension_o;
	output reg [1:0] data_reg_offset_o;
	output reg data_load_event_o;
	output reg [5:0] atop_o;
	output wire [2:0] hwlp_we_o;
	output reg [1:0] hwlp_target_mux_sel_o;
	output reg [1:0] hwlp_start_mux_sel_o;
	output reg hwlp_cnt_mux_sel_o;
	input wire debug_mode_i;
	input wire debug_wfi_no_sleep_i;
	output wire [1:0] ctrl_transfer_insn_in_dec_o;
	output wire [1:0] ctrl_transfer_insn_in_id_o;
	output reg [1:0] ctrl_transfer_target_mux_sel_o;
	input wire [31:0] mcounteren_i;
	reg regfile_mem_we;
	reg regfile_alu_we;
	reg data_req;
	reg [2:0] hwlp_we;
	reg csr_illegal;
	reg [1:0] ctrl_transfer_insn;
	reg [1:0] csr_op;
	reg alu_en;
	reg mult_int_en;
	reg mult_dot_en;
	reg apu_en;
	reg check_fprm;
	localparam [31:0] cv32e40p_fpu_pkg_OP_BITS = 4;
	reg [3:0] fpu_op;
	reg fpu_op_mod;
	reg fpu_vec_op;
	reg [1:0] fp_op_group;
	localparam cv32e40p_pkg_AMO_ADD = 5'b00000;
	localparam cv32e40p_pkg_AMO_AND = 5'b01100;
	localparam cv32e40p_pkg_AMO_LR = 5'b00010;
	localparam cv32e40p_pkg_AMO_MAX = 5'b10100;
	localparam cv32e40p_pkg_AMO_MAXU = 5'b11100;
	localparam cv32e40p_pkg_AMO_MIN = 5'b10000;
	localparam cv32e40p_pkg_AMO_MINU = 5'b11000;
	localparam cv32e40p_pkg_AMO_OR = 5'b01000;
	localparam cv32e40p_pkg_AMO_SC = 5'b00011;
	localparam cv32e40p_pkg_AMO_SWAP = 5'b00001;
	localparam cv32e40p_pkg_AMO_XOR = 5'b00100;
	localparam cv32e40p_pkg_BMASK_A_IMM = 1'b1;
	localparam cv32e40p_pkg_BMASK_A_REG = 1'b0;
	localparam cv32e40p_pkg_BMASK_A_S3 = 1'b1;
	localparam cv32e40p_pkg_BMASK_A_ZERO = 1'b0;
	localparam cv32e40p_pkg_BMASK_B_IMM = 1'b1;
	localparam cv32e40p_pkg_BMASK_B_ONE = 2'b11;
	localparam cv32e40p_pkg_BMASK_B_REG = 1'b0;
	localparam cv32e40p_pkg_BMASK_B_S2 = 2'b00;
	localparam cv32e40p_pkg_BMASK_B_S3 = 2'b01;
	localparam cv32e40p_pkg_BMASK_B_ZERO = 2'b10;
	localparam cv32e40p_pkg_BRANCH_COND = 2'b11;
	localparam cv32e40p_pkg_BRANCH_JAL = 2'b01;
	localparam cv32e40p_pkg_BRANCH_JALR = 2'b10;
	localparam cv32e40p_pkg_BRANCH_NONE = 2'b00;
	localparam [31:0] cv32e40p_pkg_C_LAT_FP16 = 'd0;
	localparam [31:0] cv32e40p_pkg_C_LAT_FP16ALT = 'd0;
	localparam [31:0] cv32e40p_pkg_C_LAT_FP64 = 'd0;
	localparam [31:0] cv32e40p_pkg_C_LAT_FP8 = 'd0;
	localparam [0:0] cv32e40p_pkg_C_RVD = 1'b0;
	localparam [0:0] cv32e40p_pkg_C_RVF = 1'b1;
	localparam [0:0] cv32e40p_pkg_C_XF16 = 1'b0;
	localparam [0:0] cv32e40p_pkg_C_XF16ALT = 1'b0;
	localparam [0:0] cv32e40p_pkg_C_XF8 = 1'b0;
	localparam [0:0] cv32e40p_pkg_C_XFVEC = 1'b0;
	localparam cv32e40p_pkg_IMMA_Z = 1'b0;
	localparam cv32e40p_pkg_IMMA_ZERO = 1'b1;
	localparam cv32e40p_pkg_IMMB_BI = 4'b1011;
	localparam cv32e40p_pkg_IMMB_CLIP = 4'b1001;
	localparam cv32e40p_pkg_IMMB_I = 4'b0000;
	localparam cv32e40p_pkg_IMMB_PCINCR = 4'b0011;
	localparam cv32e40p_pkg_IMMB_S = 4'b0001;
	localparam cv32e40p_pkg_IMMB_S2 = 4'b0100;
	localparam cv32e40p_pkg_IMMB_SHUF = 4'b1000;
	localparam cv32e40p_pkg_IMMB_U = 4'b0010;
	localparam cv32e40p_pkg_IMMB_VS = 4'b0110;
	localparam cv32e40p_pkg_IMMB_VU = 4'b0111;
	localparam cv32e40p_pkg_JT_COND = 2'b11;
	localparam cv32e40p_pkg_JT_JAL = 2'b01;
	localparam cv32e40p_pkg_JT_JALR = 2'b10;
	localparam cv32e40p_pkg_MIMM_S3 = 1'b1;
	localparam cv32e40p_pkg_MIMM_ZERO = 1'b0;
	localparam cv32e40p_pkg_OPCODE_AMO = 7'h2f;
	localparam cv32e40p_pkg_OPCODE_AUIPC = 7'h17;
	localparam cv32e40p_pkg_OPCODE_BRANCH = 7'h63;
	localparam cv32e40p_pkg_OPCODE_CUSTOM_0 = 7'h0b;
	localparam cv32e40p_pkg_OPCODE_CUSTOM_1 = 7'h2b;
	localparam cv32e40p_pkg_OPCODE_CUSTOM_2 = 7'h5b;
	localparam cv32e40p_pkg_OPCODE_CUSTOM_3 = 7'h7b;
	localparam cv32e40p_pkg_OPCODE_FENCE = 7'h0f;
	localparam cv32e40p_pkg_OPCODE_JAL = 7'h6f;
	localparam cv32e40p_pkg_OPCODE_JALR = 7'h67;
	localparam cv32e40p_pkg_OPCODE_LOAD = 7'h03;
	localparam cv32e40p_pkg_OPCODE_LOAD_FP = 7'h07;
	localparam cv32e40p_pkg_OPCODE_LUI = 7'h37;
	localparam cv32e40p_pkg_OPCODE_OP = 7'h33;
	localparam cv32e40p_pkg_OPCODE_OPIMM = 7'h13;
	localparam cv32e40p_pkg_OPCODE_OP_FMADD = 7'h43;
	localparam cv32e40p_pkg_OPCODE_OP_FMSUB = 7'h47;
	localparam cv32e40p_pkg_OPCODE_OP_FNMADD = 7'h4f;
	localparam cv32e40p_pkg_OPCODE_OP_FNMSUB = 7'h4b;
	localparam cv32e40p_pkg_OPCODE_OP_FP = 7'h53;
	localparam cv32e40p_pkg_OPCODE_STORE = 7'h23;
	localparam cv32e40p_pkg_OPCODE_STORE_FP = 7'h27;
	localparam cv32e40p_pkg_OPCODE_SYSTEM = 7'h73;
	localparam cv32e40p_pkg_OP_A_CURRPC = 3'b001;
	localparam cv32e40p_pkg_OP_A_IMM = 3'b010;
	localparam cv32e40p_pkg_OP_A_REGA_OR_FWD = 3'b000;
	localparam cv32e40p_pkg_OP_A_REGB_OR_FWD = 3'b011;
	localparam cv32e40p_pkg_OP_A_REGC_OR_FWD = 3'b100;
	localparam cv32e40p_pkg_OP_B_BMASK = 3'b100;
	localparam cv32e40p_pkg_OP_B_IMM = 3'b010;
	localparam cv32e40p_pkg_OP_B_REGA_OR_FWD = 3'b011;
	localparam cv32e40p_pkg_OP_B_REGB_OR_FWD = 3'b000;
	localparam cv32e40p_pkg_OP_B_REGC_OR_FWD = 3'b001;
	localparam cv32e40p_pkg_OP_C_JT = 2'b10;
	localparam cv32e40p_pkg_OP_C_REGB_OR_FWD = 2'b01;
	localparam cv32e40p_pkg_OP_C_REGC_OR_FWD = 2'b00;
	localparam cv32e40p_pkg_REGC_RD = 2'b01;
	localparam cv32e40p_pkg_REGC_S4 = 2'b00;
	localparam cv32e40p_pkg_REGC_ZERO = 2'b11;
	localparam cv32e40p_pkg_VEC_MODE16 = 2'b10;
	localparam cv32e40p_pkg_VEC_MODE32 = 2'b00;
	localparam cv32e40p_pkg_VEC_MODE8 = 2'b11;
	function automatic [6:0] sv2v_cast_576C1;
		input reg [6:0] inp;
		sv2v_cast_576C1 = inp;
	endfunction
	function automatic [2:0] sv2v_cast_9D1C7;
		input reg [2:0] inp;
		sv2v_cast_9D1C7 = inp;
	endfunction
	function automatic [3:0] sv2v_cast_EC9D9;
		input reg [3:0] inp;
		sv2v_cast_EC9D9 = inp;
	endfunction
	function automatic [2:0] sv2v_cast_59BD5;
		input reg [2:0] inp;
		sv2v_cast_59BD5 = inp;
	endfunction
	function automatic [1:0] sv2v_cast_3C5CB;
		input reg [1:0] inp;
		sv2v_cast_3C5CB = inp;
	endfunction
	function automatic [1:0] sv2v_cast_315CD;
		input reg [1:0] inp;
		sv2v_cast_315CD = inp;
	endfunction
	always @(*) begin : instruction_decoder
		if (_sv2v_0)
			;
		ctrl_transfer_insn = cv32e40p_pkg_BRANCH_NONE;
		ctrl_transfer_target_mux_sel_o = cv32e40p_pkg_JT_JAL;
		alu_en = 1'b1;
		alu_operator_o = sv2v_cast_576C1(7'b0000011);
		alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGA_OR_FWD;
		alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGB_OR_FWD;
		alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGC_OR_FWD;
		alu_vec_o = 1'b0;
		alu_vec_mode_o = cv32e40p_pkg_VEC_MODE32;
		scalar_replication_o = 1'b0;
		scalar_replication_c_o = 1'b0;
		regc_mux_o = cv32e40p_pkg_REGC_ZERO;
		imm_a_mux_sel_o = cv32e40p_pkg_IMMA_ZERO;
		imm_b_mux_sel_o = cv32e40p_pkg_IMMB_I;
		mult_int_en = 1'b0;
		mult_dot_en = 1'b0;
		mult_operator_o = sv2v_cast_9D1C7(3'b010);
		mult_imm_mux_o = cv32e40p_pkg_MIMM_ZERO;
		mult_signed_mode_o = 2'b00;
		mult_sel_subword_o = 1'b0;
		mult_dot_signed_o = 2'b00;
		apu_en = 1'b0;
		apu_op_o = 1'sb0;
		apu_lat_o = 1'sb0;
		fp_rnd_mode_o = 1'sb0;
		fpu_op = sv2v_cast_EC9D9(6);
		fpu_op_mod = 1'b0;
		fpu_vec_op = 1'b0;
		fpu_dst_fmt_o = sv2v_cast_59BD5('d0);
		fpu_src_fmt_o = sv2v_cast_59BD5('d0);
		fpu_int_fmt_o = sv2v_cast_3C5CB(2);
		check_fprm = 1'b0;
		fp_op_group = 2'd0;
		regfile_mem_we = 1'b0;
		regfile_alu_we = 1'b0;
		regfile_alu_waddr_sel_o = 1'b1;
		prepost_useincr_o = 1'b1;
		hwlp_we = 3'b000;
		hwlp_target_mux_sel_o = 2'b00;
		hwlp_start_mux_sel_o = 2'b00;
		hwlp_cnt_mux_sel_o = 1'b0;
		csr_access_o = 1'b0;
		csr_status_o = 1'b0;
		csr_illegal = 1'b0;
		csr_op = sv2v_cast_315CD(2'b00);
		mret_insn_o = 1'b0;
		uret_insn_o = 1'b0;
		dret_insn_o = 1'b0;
		data_we_o = 1'b0;
		data_type_o = 2'b00;
		data_sign_extension_o = 2'b00;
		data_reg_offset_o = 2'b00;
		data_req = 1'b0;
		data_load_event_o = 1'b0;
		atop_o = 6'b000000;
		illegal_insn_o = 1'b0;
		ebrk_insn_o = 1'b0;
		ecall_insn_o = 1'b0;
		wfi_o = 1'b0;
		fencei_insn_o = 1'b0;
		rega_used_o = 1'b0;
		regb_used_o = 1'b0;
		regc_used_o = 1'b0;
		reg_fp_a_o = 1'b0;
		reg_fp_b_o = 1'b0;
		reg_fp_c_o = 1'b0;
		reg_fp_d_o = 1'b0;
		bmask_a_mux_o = cv32e40p_pkg_BMASK_A_ZERO;
		bmask_b_mux_o = cv32e40p_pkg_BMASK_B_ZERO;
		alu_bmask_a_mux_sel_o = cv32e40p_pkg_BMASK_A_IMM;
		alu_bmask_b_mux_sel_o = cv32e40p_pkg_BMASK_B_IMM;
		is_clpx_o = 1'b0;
		is_subrot_o = 1'b0;
		mret_dec_o = 1'b0;
		uret_dec_o = 1'b0;
		dret_dec_o = 1'b0;
		case (instr_rdata_i[6:0])
			cv32e40p_pkg_OPCODE_JAL: begin
				ctrl_transfer_target_mux_sel_o = cv32e40p_pkg_JT_JAL;
				ctrl_transfer_insn = cv32e40p_pkg_BRANCH_JAL;
				alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_CURRPC;
				alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
				imm_b_mux_sel_o = cv32e40p_pkg_IMMB_PCINCR;
				alu_operator_o = sv2v_cast_576C1(7'b0011000);
				regfile_alu_we = 1'b1;
			end
			cv32e40p_pkg_OPCODE_JALR: begin
				ctrl_transfer_target_mux_sel_o = cv32e40p_pkg_JT_JALR;
				ctrl_transfer_insn = cv32e40p_pkg_BRANCH_JALR;
				alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_CURRPC;
				alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
				imm_b_mux_sel_o = cv32e40p_pkg_IMMB_PCINCR;
				alu_operator_o = sv2v_cast_576C1(7'b0011000);
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				if (instr_rdata_i[14:12] != 3'b000) begin
					ctrl_transfer_insn = cv32e40p_pkg_BRANCH_NONE;
					regfile_alu_we = 1'b0;
					illegal_insn_o = 1'b1;
				end
			end
			cv32e40p_pkg_OPCODE_BRANCH: begin
				ctrl_transfer_target_mux_sel_o = cv32e40p_pkg_JT_COND;
				ctrl_transfer_insn = cv32e40p_pkg_BRANCH_COND;
				alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_JT;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				case (instr_rdata_i[14:12])
					3'b000: alu_operator_o = sv2v_cast_576C1(7'b0001100);
					3'b001: alu_operator_o = sv2v_cast_576C1(7'b0001101);
					3'b100: alu_operator_o = sv2v_cast_576C1(7'b0000000);
					3'b101: alu_operator_o = sv2v_cast_576C1(7'b0001010);
					3'b110: alu_operator_o = sv2v_cast_576C1(7'b0000001);
					3'b111: alu_operator_o = sv2v_cast_576C1(7'b0001011);
					default: illegal_insn_o = 1'b1;
				endcase
			end
			cv32e40p_pkg_OPCODE_STORE: begin
				data_req = 1'b1;
				data_we_o = 1'b1;
				rega_used_o = 1'b1;
				regb_used_o = 1'b1;
				alu_operator_o = sv2v_cast_576C1(7'b0011000);
				alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
				imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S;
				alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
				case (instr_rdata_i[14:12])
					3'b000: data_type_o = 2'b10;
					3'b001: data_type_o = 2'b01;
					3'b010: data_type_o = 2'b00;
					default: begin
						illegal_insn_o = 1'b1;
						data_req = 1'b0;
						data_we_o = 1'b0;
					end
				endcase
			end
			cv32e40p_pkg_OPCODE_LOAD: begin
				data_req = 1'b1;
				regfile_mem_we = 1'b1;
				rega_used_o = 1'b1;
				alu_operator_o = sv2v_cast_576C1(7'b0011000);
				alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
				imm_b_mux_sel_o = cv32e40p_pkg_IMMB_I;
				data_sign_extension_o = {1'b0, ~instr_rdata_i[14]};
				case (instr_rdata_i[14:12])
					3'b000, 3'b100: data_type_o = 2'b10;
					3'b001, 3'b101: data_type_o = 2'b01;
					3'b010: data_type_o = 2'b00;
					default: illegal_insn_o = 1'b1;
				endcase
			end
			cv32e40p_pkg_OPCODE_AMO:
				if (A_EXTENSION) begin : decode_amo
					if (instr_rdata_i[14:12] == 3'b010) begin
						data_req = 1'b1;
						data_type_o = 2'b00;
						rega_used_o = 1'b1;
						regb_used_o = 1'b1;
						regfile_mem_we = 1'b1;
						prepost_useincr_o = 1'b0;
						alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGA_OR_FWD;
						data_sign_extension_o = 1'b1;
						atop_o = {1'b1, instr_rdata_i[31:27]};
						case (instr_rdata_i[31:27])
							cv32e40p_pkg_AMO_LR: data_we_o = 1'b0;
							cv32e40p_pkg_AMO_SC, cv32e40p_pkg_AMO_SWAP, cv32e40p_pkg_AMO_ADD, cv32e40p_pkg_AMO_XOR, cv32e40p_pkg_AMO_AND, cv32e40p_pkg_AMO_OR, cv32e40p_pkg_AMO_MIN, cv32e40p_pkg_AMO_MAX, cv32e40p_pkg_AMO_MINU, cv32e40p_pkg_AMO_MAXU: begin
								data_we_o = 1'b1;
								alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
							end
							default: illegal_insn_o = 1'b1;
						endcase
					end
					else
						illegal_insn_o = 1'b1;
				end
				else begin : no_decode_amo
					illegal_insn_o = 1'b1;
				end
			cv32e40p_pkg_OPCODE_LUI: begin
				alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_IMM;
				alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
				imm_a_mux_sel_o = cv32e40p_pkg_IMMA_ZERO;
				imm_b_mux_sel_o = cv32e40p_pkg_IMMB_U;
				alu_operator_o = sv2v_cast_576C1(7'b0011000);
				regfile_alu_we = 1'b1;
			end
			cv32e40p_pkg_OPCODE_AUIPC: begin
				alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_CURRPC;
				alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
				imm_b_mux_sel_o = cv32e40p_pkg_IMMB_U;
				alu_operator_o = sv2v_cast_576C1(7'b0011000);
				regfile_alu_we = 1'b1;
			end
			cv32e40p_pkg_OPCODE_OPIMM: begin
				alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
				imm_b_mux_sel_o = cv32e40p_pkg_IMMB_I;
				regfile_alu_we = 1'b1;
				rega_used_o = 1'b1;
				case (instr_rdata_i[14:12])
					3'b000: alu_operator_o = sv2v_cast_576C1(7'b0011000);
					3'b010: alu_operator_o = sv2v_cast_576C1(7'b0000010);
					3'b011: alu_operator_o = sv2v_cast_576C1(7'b0000011);
					3'b100: alu_operator_o = sv2v_cast_576C1(7'b0101111);
					3'b110: alu_operator_o = sv2v_cast_576C1(7'b0101110);
					3'b111: alu_operator_o = sv2v_cast_576C1(7'b0010101);
					3'b001: begin
						alu_operator_o = sv2v_cast_576C1(7'b0100111);
						if (instr_rdata_i[31:25] != 7'b0000000)
							illegal_insn_o = 1'b1;
					end
					3'b101:
						if (instr_rdata_i[31:25] == 7'b0000000)
							alu_operator_o = sv2v_cast_576C1(7'b0100101);
						else if (instr_rdata_i[31:25] == 7'b0100000)
							alu_operator_o = sv2v_cast_576C1(7'b0100100);
						else
							illegal_insn_o = 1'b1;
				endcase
			end
			cv32e40p_pkg_OPCODE_OP:
				if (instr_rdata_i[31:30] == 2'b11)
					illegal_insn_o = 1'b1;
				else if (instr_rdata_i[31:30] == 2'b10) begin
					if (instr_rdata_i[29:25] == 5'b00000)
						illegal_insn_o = 1'b1;
					else if ((FPU == 1) && 1'd0) begin
						alu_en = 1'b0;
						apu_en = 1'b1;
						rega_used_o = 1'b1;
						regb_used_o = 1'b1;
						if (ZFINX == 0) begin
							reg_fp_a_o = 1'b1;
							reg_fp_b_o = 1'b1;
							reg_fp_d_o = 1'b1;
						end
						else begin
							reg_fp_a_o = 1'b0;
							reg_fp_b_o = 1'b0;
							reg_fp_d_o = 1'b0;
						end
						fpu_vec_op = 1'b1;
						scalar_replication_o = instr_rdata_i[14];
						check_fprm = 1'b1;
						fp_rnd_mode_o = frm_i;
						case (instr_rdata_i[13:12])
							2'b00: begin
								fpu_dst_fmt_o = sv2v_cast_59BD5('d0);
								alu_vec_mode_o = cv32e40p_pkg_VEC_MODE32;
							end
							2'b01: begin
								fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
								alu_vec_mode_o = cv32e40p_pkg_VEC_MODE16;
							end
							2'b10: begin
								fpu_dst_fmt_o = sv2v_cast_59BD5('d2);
								alu_vec_mode_o = cv32e40p_pkg_VEC_MODE16;
							end
							2'b11: begin
								fpu_dst_fmt_o = sv2v_cast_59BD5('d3);
								alu_vec_mode_o = cv32e40p_pkg_VEC_MODE8;
							end
						endcase
						fpu_src_fmt_o = fpu_dst_fmt_o;
						if (instr_rdata_i[29:25] == 5'b00001) begin
							fpu_op = sv2v_cast_EC9D9(2);
							fp_op_group = 2'd0;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
							scalar_replication_o = 1'b0;
							scalar_replication_c_o = instr_rdata_i[14];
						end
						else if (instr_rdata_i[29:25] == 5'b00010) begin
							fpu_op = sv2v_cast_EC9D9(2);
							fpu_op_mod = 1'b1;
							fp_op_group = 2'd0;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
							scalar_replication_o = 1'b0;
							scalar_replication_c_o = instr_rdata_i[14];
						end
						else if (instr_rdata_i[29:25] == 5'b00011) begin
							fpu_op = sv2v_cast_EC9D9(3);
							fp_op_group = 2'd0;
						end
						else if (instr_rdata_i[29:25] == 5'b00100) begin
							fpu_op = sv2v_cast_EC9D9(4);
							fp_op_group = 2'd1;
						end
						else if (instr_rdata_i[29:25] == 5'b00101) begin
							fpu_op = sv2v_cast_EC9D9(7);
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b00110) begin
							fpu_op = sv2v_cast_EC9D9(7);
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b00111) begin
							regb_used_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(5);
							fp_op_group = 2'd1;
							if ((instr_rdata_i[24:20] != 5'b00000) || instr_rdata_i[14])
								illegal_insn_o = 1'b1;
						end
						else if (instr_rdata_i[29:25] == 5'b01000) begin
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							if (ZFINX == 0)
								reg_fp_c_o = 1'b1;
							else
								reg_fp_c_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(0);
							fp_op_group = 2'd0;
						end
						else if (instr_rdata_i[29:25] == 5'b01001) begin
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							if (ZFINX == 0)
								reg_fp_c_o = 1'b1;
							else
								reg_fp_c_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(0);
							fpu_op_mod = 1'b1;
							fp_op_group = 2'd0;
						end
						else if (instr_rdata_i[29:25] == 5'b01100) begin
							regb_used_o = 1'b0;
							scalar_replication_o = 1'b0;
							if (instr_rdata_i[24:20] == 5'b00000) begin
								alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
								fpu_op = sv2v_cast_EC9D9(6);
								fp_rnd_mode_o = 3'b011;
								fp_op_group = 2'd2;
								check_fprm = 1'b0;
								if (instr_rdata_i[14]) begin
									reg_fp_a_o = 1'b0;
									fpu_op_mod = 1'b0;
								end
								else begin
									reg_fp_d_o = 1'b0;
									fpu_op_mod = 1'b1;
								end
							end
							else if (instr_rdata_i[24:20] == 5'b00001) begin
								reg_fp_d_o = 1'b0;
								fpu_op = sv2v_cast_EC9D9(9);
								fp_rnd_mode_o = 3'b000;
								fp_op_group = 2'd2;
								check_fprm = 1'b0;
								if (instr_rdata_i[14])
									illegal_insn_o = 1'b1;
							end
							else if ((instr_rdata_i[24:20] | 5'b00001) == 5'b00011) begin
								fp_op_group = 2'd3;
								fpu_op_mod = instr_rdata_i[14];
								case (instr_rdata_i[13:12])
									2'b00: fpu_int_fmt_o = sv2v_cast_3C5CB(2);
									2'b01, 2'b10: fpu_int_fmt_o = sv2v_cast_3C5CB(1);
									2'b11: fpu_int_fmt_o = sv2v_cast_3C5CB(0);
								endcase
								if (instr_rdata_i[20]) begin
									reg_fp_a_o = 1'b0;
									fpu_op = sv2v_cast_EC9D9(12);
								end
								else begin
									reg_fp_d_o = 1'b0;
									fpu_op = sv2v_cast_EC9D9(11);
								end
							end
							else if ((instr_rdata_i[24:20] | 5'b00011) == 5'b00111) begin
								fpu_op = sv2v_cast_EC9D9(10);
								fp_op_group = 2'd3;
								case (instr_rdata_i[21:20])
									2'b00: begin
										fpu_src_fmt_o = sv2v_cast_59BD5('d0);
										if (~cv32e40p_pkg_C_RVF)
											illegal_insn_o = 1'b1;
									end
									2'b01: begin
										fpu_src_fmt_o = sv2v_cast_59BD5('d4);
										if (~cv32e40p_pkg_C_XF16ALT)
											illegal_insn_o = 1'b1;
									end
									2'b10: begin
										fpu_src_fmt_o = sv2v_cast_59BD5('d2);
										if (~cv32e40p_pkg_C_XF16)
											illegal_insn_o = 1'b1;
									end
									2'b11: begin
										fpu_src_fmt_o = sv2v_cast_59BD5('d3);
										if (~cv32e40p_pkg_C_XF8)
											illegal_insn_o = 1'b1;
									end
								endcase
								if (instr_rdata_i[14])
									illegal_insn_o = 1'b1;
							end
							else
								illegal_insn_o = 1'b1;
						end
						else if (instr_rdata_i[29:25] == 5'b01101) begin
							fpu_op = sv2v_cast_EC9D9(6);
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b01110) begin
							fpu_op = sv2v_cast_EC9D9(6);
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b01111) begin
							fpu_op = sv2v_cast_EC9D9(6);
							fp_rnd_mode_o = 3'b010;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10000) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(8);
							fp_rnd_mode_o = 3'b010;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10001) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(8);
							fpu_op_mod = 1'b1;
							fp_rnd_mode_o = 3'b010;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10010) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(8);
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10011) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(8);
							fpu_op_mod = 1'b1;
							fp_rnd_mode_o = 3'b001;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10100) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(8);
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if (instr_rdata_i[29:25] == 5'b10101) begin
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(8);
							fpu_op_mod = 1'b1;
							fp_rnd_mode_o = 3'b000;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
						end
						else if ((instr_rdata_i[29:25] | 5'b00011) == 5'b11011) begin
							fpu_op_mod = instr_rdata_i[14];
							fp_op_group = 2'd3;
							scalar_replication_o = 1'b0;
							if (instr_rdata_i[25])
								fpu_op = sv2v_cast_EC9D9(14);
							else
								fpu_op = sv2v_cast_EC9D9(13);
							if (instr_rdata_i[26]) begin
								fpu_src_fmt_o = sv2v_cast_59BD5('d1);
								if (~cv32e40p_pkg_C_RVD)
									illegal_insn_o = 1'b1;
							end
							else begin
								fpu_src_fmt_o = sv2v_cast_59BD5('d0);
								if (~cv32e40p_pkg_C_RVF)
									illegal_insn_o = 1'b1;
							end
							if (fpu_op == sv2v_cast_EC9D9(14)) begin
								if (~cv32e40p_pkg_C_XF8 || ~cv32e40p_pkg_C_RVD)
									illegal_insn_o = 1'b1;
							end
							else if (instr_rdata_i[14]) begin
								if (fpu_dst_fmt_o == sv2v_cast_59BD5('d0))
									illegal_insn_o = 1'b1;
								if (~cv32e40p_pkg_C_RVD && (fpu_dst_fmt_o != sv2v_cast_59BD5('d3)))
									illegal_insn_o = 1'b1;
							end
						end
						else
							illegal_insn_o = 1'b1;
						if ((~cv32e40p_pkg_C_RVF || ~cv32e40p_pkg_C_RVD) && (fpu_dst_fmt_o == sv2v_cast_59BD5('d0)))
							illegal_insn_o = 1'b1;
						if ((~cv32e40p_pkg_C_XF16 || ~cv32e40p_pkg_C_RVF) && (fpu_dst_fmt_o == sv2v_cast_59BD5('d2)))
							illegal_insn_o = 1'b1;
						if ((~cv32e40p_pkg_C_XF16ALT || ~cv32e40p_pkg_C_RVF) && (fpu_dst_fmt_o == sv2v_cast_59BD5('d4)))
							illegal_insn_o = 1'b1;
						if ((~cv32e40p_pkg_C_XF8 || (~cv32e40p_pkg_C_XF16 && ~cv32e40p_pkg_C_XF16ALT)) && (fpu_dst_fmt_o == sv2v_cast_59BD5('d3)))
							illegal_insn_o = 1'b1;
						if (check_fprm) begin
							if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
								;
							else
								illegal_insn_o = 1'b1;
						end
						case (fp_op_group)
							2'd0:
								case (fpu_dst_fmt_o)
									sv2v_cast_59BD5('d0): apu_lat_o = (FPU_ADDMUL_LAT < 2 ? FPU_ADDMUL_LAT + 1 : 2'h3);
									sv2v_cast_59BD5('d2): apu_lat_o = 1;
									sv2v_cast_59BD5('d4): apu_lat_o = 1;
									sv2v_cast_59BD5('d3): apu_lat_o = 1;
									default:
										;
								endcase
							2'd1: apu_lat_o = 2'h3;
							2'd2: apu_lat_o = (FPU_OTHERS_LAT < 2 ? FPU_OTHERS_LAT + 1 : 2'h3);
							2'd3: apu_lat_o = (FPU_OTHERS_LAT < 2 ? FPU_OTHERS_LAT + 1 : 2'h3);
						endcase
						apu_op_o = {fpu_vec_op, fpu_op_mod, fpu_op};
					end
					else
						illegal_insn_o = 1'b1;
				end
				else begin
					regfile_alu_we = 1'b1;
					rega_used_o = 1'b1;
					if (~instr_rdata_i[28])
						regb_used_o = 1'b1;
					case ({instr_rdata_i[30:25], instr_rdata_i[14:12]})
						9'b000000000: alu_operator_o = sv2v_cast_576C1(7'b0011000);
						9'b100000000: alu_operator_o = sv2v_cast_576C1(7'b0011001);
						9'b000000010: alu_operator_o = sv2v_cast_576C1(7'b0000010);
						9'b000000011: alu_operator_o = sv2v_cast_576C1(7'b0000011);
						9'b000000100: alu_operator_o = sv2v_cast_576C1(7'b0101111);
						9'b000000110: alu_operator_o = sv2v_cast_576C1(7'b0101110);
						9'b000000111: alu_operator_o = sv2v_cast_576C1(7'b0010101);
						9'b000000001: alu_operator_o = sv2v_cast_576C1(7'b0100111);
						9'b000000101: alu_operator_o = sv2v_cast_576C1(7'b0100101);
						9'b100000101: alu_operator_o = sv2v_cast_576C1(7'b0100100);
						9'b000001000: begin
							alu_en = 1'b0;
							mult_int_en = 1'b1;
							mult_operator_o = sv2v_cast_9D1C7(3'b000);
							regc_mux_o = cv32e40p_pkg_REGC_ZERO;
						end
						9'b000001001: begin
							alu_en = 1'b0;
							mult_int_en = 1'b1;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_ZERO;
							mult_signed_mode_o = 2'b11;
							mult_operator_o = sv2v_cast_9D1C7(3'b110);
						end
						9'b000001010: begin
							alu_en = 1'b0;
							mult_int_en = 1'b1;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_ZERO;
							mult_signed_mode_o = 2'b01;
							mult_operator_o = sv2v_cast_9D1C7(3'b110);
						end
						9'b000001011: begin
							alu_en = 1'b0;
							mult_int_en = 1'b1;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_ZERO;
							mult_signed_mode_o = 2'b00;
							mult_operator_o = sv2v_cast_9D1C7(3'b110);
						end
						9'b000001100: begin
							alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							regb_used_o = 1'b1;
							alu_operator_o = sv2v_cast_576C1(7'b0110001);
						end
						9'b000001101: begin
							alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							regb_used_o = 1'b1;
							alu_operator_o = sv2v_cast_576C1(7'b0110000);
						end
						9'b000001110: begin
							alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							regb_used_o = 1'b1;
							alu_operator_o = sv2v_cast_576C1(7'b0110011);
						end
						9'b000001111: begin
							alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGB_OR_FWD;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							regb_used_o = 1'b1;
							alu_operator_o = sv2v_cast_576C1(7'b0110010);
						end
						default: illegal_insn_o = 1'b1;
					endcase
				end
			cv32e40p_pkg_OPCODE_OP_FP:
				if ((FPU == 1) && ((ZFINX == 1) || (fs_off_i == 1'b0))) begin
					alu_en = 1'b0;
					apu_en = 1'b1;
					rega_used_o = 1'b1;
					regb_used_o = 1'b1;
					if (ZFINX == 0) begin
						reg_fp_a_o = 1'b1;
						reg_fp_b_o = 1'b1;
						reg_fp_d_o = 1'b1;
					end
					else begin
						reg_fp_a_o = 1'b0;
						reg_fp_b_o = 1'b0;
						reg_fp_d_o = 1'b0;
					end
					check_fprm = 1'b1;
					fp_rnd_mode_o = instr_rdata_i[14:12];
					case (instr_rdata_i[26:25])
						2'b00: fpu_dst_fmt_o = sv2v_cast_59BD5('d0);
						2'b01: fpu_dst_fmt_o = sv2v_cast_59BD5('d1);
						2'b10:
							if (instr_rdata_i[14:12] == 3'b101)
								fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
							else
								fpu_dst_fmt_o = sv2v_cast_59BD5('d2);
						2'b11: fpu_dst_fmt_o = sv2v_cast_59BD5('d3);
					endcase
					fpu_src_fmt_o = fpu_dst_fmt_o;
					case (instr_rdata_i[31:27])
						5'b00000: begin
							fpu_op = sv2v_cast_EC9D9(2);
							fp_op_group = 2'd0;
							apu_op_o = 2'b00;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
						end
						5'b00001: begin
							fpu_op = sv2v_cast_EC9D9(2);
							fpu_op_mod = 1'b1;
							fp_op_group = 2'd0;
							apu_op_o = 2'b01;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
						end
						5'b00010: begin
							fpu_op = sv2v_cast_EC9D9(3);
							fp_op_group = 2'd0;
						end
						5'b00011: begin
							fpu_op = sv2v_cast_EC9D9(4);
							fp_op_group = 2'd1;
						end
						5'b01011: begin
							regb_used_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(5);
							fp_op_group = 2'd1;
							apu_op_o = 1'b1;
							if (instr_rdata_i[24:20] != 5'b00000)
								illegal_insn_o = 1'b1;
						end
						5'b00100: begin
							fpu_op = sv2v_cast_EC9D9(6);
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
							if (cv32e40p_pkg_C_XF16ALT) begin
								if (!(|{(3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12]), (3'b100 <= instr_rdata_i[14:12]) && (3'b110 >= instr_rdata_i[14:12])}))
									illegal_insn_o = 1'b1;
								if (instr_rdata_i[14]) begin
									fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
									fpu_src_fmt_o = sv2v_cast_59BD5('d4);
								end
								else
									fp_rnd_mode_o = {1'b0, instr_rdata_i[13:12]};
							end
							else if (!((3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12])))
								illegal_insn_o = 1'b1;
						end
						5'b00101: begin
							fpu_op = sv2v_cast_EC9D9(7);
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
							if (cv32e40p_pkg_C_XF16ALT) begin
								if (!(|{(3'b000 <= instr_rdata_i[14:12]) && (3'b001 >= instr_rdata_i[14:12]), (3'b100 <= instr_rdata_i[14:12]) && (3'b101 >= instr_rdata_i[14:12])}))
									illegal_insn_o = 1'b1;
								if (instr_rdata_i[14]) begin
									fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
									fpu_src_fmt_o = sv2v_cast_59BD5('d4);
								end
								else
									fp_rnd_mode_o = {1'b0, instr_rdata_i[13:12]};
							end
							else if (!((3'b000 <= instr_rdata_i[14:12]) && (3'b001 >= instr_rdata_i[14:12])))
								illegal_insn_o = 1'b1;
						end
						5'b01000: begin
							regb_used_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(10);
							fp_op_group = 2'd3;
							if (instr_rdata_i[24:23])
								illegal_insn_o = 1'b1;
							case (instr_rdata_i[22:20])
								3'b000: begin
									illegal_insn_o = 1'b1;
									fpu_src_fmt_o = sv2v_cast_59BD5('d0);
								end
								3'b001: begin
									if (~cv32e40p_pkg_C_RVD)
										illegal_insn_o = 1'b1;
									fpu_src_fmt_o = sv2v_cast_59BD5('d1);
								end
								3'b010: begin
									if (~cv32e40p_pkg_C_XF16)
										illegal_insn_o = 1'b1;
									fpu_src_fmt_o = sv2v_cast_59BD5('d2);
								end
								3'b110: begin
									if (~cv32e40p_pkg_C_XF16ALT)
										illegal_insn_o = 1'b1;
									fpu_src_fmt_o = sv2v_cast_59BD5('d4);
								end
								3'b011: begin
									if (~cv32e40p_pkg_C_XF8)
										illegal_insn_o = 1'b1;
									fpu_src_fmt_o = sv2v_cast_59BD5('d3);
								end
								default: illegal_insn_o = 1'b1;
							endcase
						end
						5'b01001: begin
							if ((~cv32e40p_pkg_C_XF16 && ~cv32e40p_pkg_C_XF16ALT) && ~cv32e40p_pkg_C_XF8)
								illegal_insn_o = 1;
							fpu_op = sv2v_cast_EC9D9(3);
							fp_op_group = 2'd0;
							fpu_dst_fmt_o = sv2v_cast_59BD5('d0);
						end
						5'b01010: begin
							if ((~cv32e40p_pkg_C_XF16 && ~cv32e40p_pkg_C_XF16ALT) && ~cv32e40p_pkg_C_XF8)
								illegal_insn_o = 1;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							if (ZFINX == 0)
								reg_fp_c_o = 1'b1;
							else
								reg_fp_c_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(0);
							fp_op_group = 2'd0;
							fpu_dst_fmt_o = sv2v_cast_59BD5('d0);
						end
						5'b10100: begin
							fpu_op = sv2v_cast_EC9D9(8);
							fp_op_group = 2'd2;
							reg_fp_d_o = 1'b0;
							check_fprm = 1'b0;
							if (cv32e40p_pkg_C_XF16ALT) begin
								if (!(|{(3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12]), (3'b100 <= instr_rdata_i[14:12]) && (3'b110 >= instr_rdata_i[14:12])}))
									illegal_insn_o = 1'b1;
								if (instr_rdata_i[14]) begin
									fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
									fpu_src_fmt_o = sv2v_cast_59BD5('d4);
								end
								else
									fp_rnd_mode_o = {1'b0, instr_rdata_i[13:12]};
							end
							else if (!((3'b000 <= instr_rdata_i[14:12]) && (3'b010 >= instr_rdata_i[14:12])))
								illegal_insn_o = 1'b1;
						end
						5'b11000: begin
							regb_used_o = 1'b0;
							reg_fp_d_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(11);
							fp_op_group = 2'd3;
							fpu_op_mod = instr_rdata_i[20];
							apu_op_o = 2'b01;
							case (instr_rdata_i[26:25])
								2'b00:
									if (~cv32e40p_pkg_C_RVF)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_59BD5('d0);
								2'b01:
									if (~cv32e40p_pkg_C_RVD)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_59BD5('d1);
								2'b10:
									if (instr_rdata_i[14:12] == 3'b101) begin
										if (~cv32e40p_pkg_C_XF16ALT)
											illegal_insn_o = 1;
										else
											fpu_src_fmt_o = sv2v_cast_59BD5('d4);
									end
									else if (~cv32e40p_pkg_C_XF16)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_59BD5('d2);
								2'b11:
									if (~cv32e40p_pkg_C_XF8)
										illegal_insn_o = 1;
									else
										fpu_src_fmt_o = sv2v_cast_59BD5('d3);
							endcase
							if (instr_rdata_i[24:21])
								illegal_insn_o = 1'b1;
						end
						5'b11010: begin
							regb_used_o = 1'b0;
							reg_fp_a_o = 1'b0;
							fpu_op = sv2v_cast_EC9D9(12);
							fp_op_group = 2'd3;
							fpu_op_mod = instr_rdata_i[20];
							apu_op_o = 2'b00;
							if (instr_rdata_i[24:21])
								illegal_insn_o = 1'b1;
						end
						5'b11100: begin
							regb_used_o = 1'b0;
							reg_fp_d_o = 1'b0;
							fp_op_group = 2'd2;
							check_fprm = 1'b0;
							if (((ZFINX == 0) && (instr_rdata_i[14:12] == 3'b000)) || 1'd0) begin
								alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
								fpu_op = sv2v_cast_EC9D9(6);
								fpu_op_mod = 1'b1;
								fp_rnd_mode_o = 3'b011;
								if (instr_rdata_i[14]) begin
									fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
									fpu_src_fmt_o = sv2v_cast_59BD5('d4);
								end
							end
							else if ((instr_rdata_i[14:12] == 3'b001) || 1'd0) begin
								fpu_op = sv2v_cast_EC9D9(9);
								fp_rnd_mode_o = 3'b000;
								if (instr_rdata_i[14]) begin
									fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
									fpu_src_fmt_o = sv2v_cast_59BD5('d4);
								end
							end
							else
								illegal_insn_o = 1'b1;
							if (instr_rdata_i[24:20])
								illegal_insn_o = 1'b1;
						end
						5'b11110: begin
							regb_used_o = 1'b0;
							reg_fp_a_o = 1'b0;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
							fpu_op = sv2v_cast_EC9D9(6);
							fpu_op_mod = 1'b0;
							fp_op_group = 2'd2;
							fp_rnd_mode_o = 3'b011;
							check_fprm = 1'b0;
							if (((ZFINX == 0) && (instr_rdata_i[14:12] == 3'b000)) || 1'd0) begin
								if (instr_rdata_i[14]) begin
									fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
									fpu_src_fmt_o = sv2v_cast_59BD5('d4);
								end
							end
							else
								illegal_insn_o = 1'b1;
							if (instr_rdata_i[24:20] != 5'b00000)
								illegal_insn_o = 1'b1;
						end
						default: illegal_insn_o = 1'b1;
					endcase
					if (~cv32e40p_pkg_C_RVF && (fpu_dst_fmt_o == sv2v_cast_59BD5('d0)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_RVD && (fpu_dst_fmt_o == sv2v_cast_59BD5('d1)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_XF16 && (fpu_dst_fmt_o == sv2v_cast_59BD5('d2)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_XF16ALT && (fpu_dst_fmt_o == sv2v_cast_59BD5('d4)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_XF8 && (fpu_dst_fmt_o == sv2v_cast_59BD5('d3)))
						illegal_insn_o = 1'b1;
					if (check_fprm) begin
						if ((3'b000 <= instr_rdata_i[14:12]) && (3'b100 >= instr_rdata_i[14:12]))
							;
						else if (instr_rdata_i[14:12] == 3'b101) begin
							if (~cv32e40p_pkg_C_XF16ALT || (fpu_dst_fmt_o != sv2v_cast_59BD5('d4)))
								illegal_insn_o = 1'b1;
							if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
								fp_rnd_mode_o = frm_i;
							else
								illegal_insn_o = 1'b1;
						end
						else if (instr_rdata_i[14:12] == 3'b111) begin
							if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
								fp_rnd_mode_o = frm_i;
							else
								illegal_insn_o = 1'b1;
						end
						else
							illegal_insn_o = 1'b1;
					end
					case (fp_op_group)
						2'd0:
							case (fpu_dst_fmt_o)
								sv2v_cast_59BD5('d0): apu_lat_o = (FPU_ADDMUL_LAT < 2 ? FPU_ADDMUL_LAT + 1 : 2'h3);
								sv2v_cast_59BD5('d1): apu_lat_o = 1;
								sv2v_cast_59BD5('d2): apu_lat_o = 1;
								sv2v_cast_59BD5('d4): apu_lat_o = 1;
								sv2v_cast_59BD5('d3): apu_lat_o = 1;
								default:
									;
							endcase
						2'd1: apu_lat_o = 2'h3;
						2'd2: apu_lat_o = (FPU_OTHERS_LAT < 2 ? FPU_OTHERS_LAT + 1 : 2'h3);
						2'd3: apu_lat_o = (FPU_OTHERS_LAT < 2 ? FPU_OTHERS_LAT + 1 : 2'h3);
					endcase
					apu_op_o = {fpu_vec_op, fpu_op_mod, fpu_op};
				end
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_OP_FMADD, cv32e40p_pkg_OPCODE_OP_FMSUB, cv32e40p_pkg_OPCODE_OP_FNMSUB, cv32e40p_pkg_OPCODE_OP_FNMADD:
				if ((FPU == 1) && ((ZFINX == 1) || (fs_off_i == 1'b0))) begin
					alu_en = 1'b0;
					apu_en = 1'b1;
					rega_used_o = 1'b1;
					regb_used_o = 1'b1;
					regc_used_o = 1'b1;
					regc_mux_o = cv32e40p_pkg_REGC_S4;
					if (ZFINX == 0) begin
						reg_fp_a_o = 1'b1;
						reg_fp_b_o = 1'b1;
						reg_fp_c_o = 1'b1;
						reg_fp_d_o = 1'b1;
					end
					else begin
						reg_fp_a_o = 1'b0;
						reg_fp_b_o = 1'b0;
						reg_fp_c_o = 1'b0;
						reg_fp_d_o = 1'b0;
					end
					fp_rnd_mode_o = instr_rdata_i[14:12];
					case (instr_rdata_i[26:25])
						2'b00: fpu_dst_fmt_o = sv2v_cast_59BD5('d0);
						2'b01: fpu_dst_fmt_o = sv2v_cast_59BD5('d1);
						2'b10:
							if (instr_rdata_i[14:12] == 3'b101)
								fpu_dst_fmt_o = sv2v_cast_59BD5('d4);
							else
								fpu_dst_fmt_o = sv2v_cast_59BD5('d2);
						2'b11: fpu_dst_fmt_o = sv2v_cast_59BD5('d3);
					endcase
					fpu_src_fmt_o = fpu_dst_fmt_o;
					case (instr_rdata_i[6:0])
						cv32e40p_pkg_OPCODE_OP_FMADD: begin
							fpu_op = sv2v_cast_EC9D9(0);
							apu_op_o = 2'b00;
						end
						cv32e40p_pkg_OPCODE_OP_FMSUB: begin
							fpu_op = sv2v_cast_EC9D9(0);
							fpu_op_mod = 1'b1;
							apu_op_o = 2'b01;
						end
						cv32e40p_pkg_OPCODE_OP_FNMSUB: begin
							fpu_op = sv2v_cast_EC9D9(1);
							apu_op_o = 2'b10;
						end
						cv32e40p_pkg_OPCODE_OP_FNMADD: begin
							fpu_op = sv2v_cast_EC9D9(1);
							fpu_op_mod = 1'b1;
							apu_op_o = 2'b11;
						end
						default:
							;
					endcase
					if (~cv32e40p_pkg_C_RVF && (fpu_dst_fmt_o == sv2v_cast_59BD5('d0)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_RVD && (fpu_dst_fmt_o == sv2v_cast_59BD5('d1)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_XF16 && (fpu_dst_fmt_o == sv2v_cast_59BD5('d2)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_XF16ALT && (fpu_dst_fmt_o == sv2v_cast_59BD5('d4)))
						illegal_insn_o = 1'b1;
					if (~cv32e40p_pkg_C_XF8 && (fpu_dst_fmt_o == sv2v_cast_59BD5('d3)))
						illegal_insn_o = 1'b1;
					if ((3'b000 <= instr_rdata_i[14:12]) && (3'b100 >= instr_rdata_i[14:12]))
						;
					else if (instr_rdata_i[14:12] == 3'b101) begin
						if (~cv32e40p_pkg_C_XF16ALT || (fpu_dst_fmt_o != sv2v_cast_59BD5('d4)))
							illegal_insn_o = 1'b1;
						if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
							fp_rnd_mode_o = frm_i;
						else
							illegal_insn_o = 1'b1;
					end
					else if (instr_rdata_i[14:12] == 3'b111) begin
						if ((3'b000 <= frm_i) && (3'b100 >= frm_i))
							fp_rnd_mode_o = frm_i;
						else
							illegal_insn_o = 1'b1;
					end
					else
						illegal_insn_o = 1'b1;
					case (fpu_dst_fmt_o)
						sv2v_cast_59BD5('d0): apu_lat_o = (FPU_ADDMUL_LAT < 2 ? FPU_ADDMUL_LAT + 1 : 2'h3);
						sv2v_cast_59BD5('d1): apu_lat_o = 1;
						sv2v_cast_59BD5('d2): apu_lat_o = 1;
						sv2v_cast_59BD5('d4): apu_lat_o = 1;
						sv2v_cast_59BD5('d3): apu_lat_o = 1;
						default:
							;
					endcase
					apu_op_o = {fpu_vec_op, fpu_op_mod, fpu_op};
				end
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_STORE_FP:
				if (((FPU == 1) && (ZFINX == 0)) && (fs_off_i == 1'b0)) begin
					data_req = 1'b1;
					data_we_o = 1'b1;
					rega_used_o = 1'b1;
					regb_used_o = 1'b1;
					alu_operator_o = sv2v_cast_576C1(7'b0011000);
					reg_fp_b_o = 1'b1;
					imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S;
					alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
					alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
					case (instr_rdata_i[14:12])
						3'b000:
							if (cv32e40p_pkg_C_XF8)
								data_type_o = 2'b10;
							else
								illegal_insn_o = 1'b1;
						3'b001:
							if (cv32e40p_pkg_C_XF16 | cv32e40p_pkg_C_XF16ALT)
								data_type_o = 2'b01;
							else
								illegal_insn_o = 1'b1;
						3'b010:
							if (cv32e40p_pkg_C_RVF)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						3'b011:
							if (cv32e40p_pkg_C_RVD)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						default: illegal_insn_o = 1'b1;
					endcase
					if (illegal_insn_o) begin
						data_req = 1'b0;
						data_we_o = 1'b0;
					end
				end
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_LOAD_FP:
				if (((FPU == 1) && (ZFINX == 0)) && (fs_off_i == 1'b0)) begin
					data_req = 1'b1;
					regfile_mem_we = 1'b1;
					reg_fp_d_o = 1'b1;
					rega_used_o = 1'b1;
					alu_operator_o = sv2v_cast_576C1(7'b0011000);
					imm_b_mux_sel_o = cv32e40p_pkg_IMMB_I;
					alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
					data_sign_extension_o = 2'b10;
					case (instr_rdata_i[14:12])
						3'b000:
							if (cv32e40p_pkg_C_XF8)
								data_type_o = 2'b10;
							else
								illegal_insn_o = 1'b1;
						3'b001:
							if (cv32e40p_pkg_C_XF16 | cv32e40p_pkg_C_XF16ALT)
								data_type_o = 2'b01;
							else
								illegal_insn_o = 1'b1;
						3'b010:
							if (cv32e40p_pkg_C_RVF)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						3'b011:
							if (cv32e40p_pkg_C_RVD)
								data_type_o = 2'b00;
							else
								illegal_insn_o = 1'b1;
						default: illegal_insn_o = 1'b1;
					endcase
				end
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_CUSTOM_0:
				if (COREV_PULP && (instr_rdata_i[14:13] != 2'b11)) begin
					data_req = 1'b1;
					regfile_mem_we = 1'b1;
					rega_used_o = 1'b1;
					alu_operator_o = sv2v_cast_576C1(7'b0011000);
					alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
					imm_b_mux_sel_o = cv32e40p_pkg_IMMB_I;
					if (instr_rdata_i[13:12] != 2'b11) begin
						prepost_useincr_o = 1'b0;
						regfile_alu_waddr_sel_o = 1'b0;
						regfile_alu_we = 1'b1;
					end
					data_sign_extension_o = {1'b0, ~instr_rdata_i[14]};
					case (instr_rdata_i[13:12])
						2'b00: data_type_o = 2'b10;
						2'b01: data_type_o = 2'b01;
						default: data_type_o = 2'b00;
					endcase
					if (instr_rdata_i[13:12] == 2'b11) begin
						if (COREV_CLUSTER)
							data_load_event_o = 1'b1;
						else
							illegal_insn_o = 1'b1;
					end
				end
				else if (COREV_PULP) begin
					ctrl_transfer_target_mux_sel_o = cv32e40p_pkg_JT_COND;
					ctrl_transfer_insn = cv32e40p_pkg_BRANCH_COND;
					alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_JT;
					rega_used_o = 1'b1;
					alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
					imm_b_mux_sel_o = cv32e40p_pkg_IMMB_BI;
					if (instr_rdata_i[12] == 1'b0)
						alu_operator_o = sv2v_cast_576C1(7'b0001100);
					else
						alu_operator_o = sv2v_cast_576C1(7'b0001101);
				end
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_CUSTOM_1:
				if (COREV_PULP)
					case (instr_rdata_i[14:12])
						3'b000, 3'b001, 3'b010: begin
							data_req = 1'b1;
							data_we_o = 1'b1;
							rega_used_o = 1'b1;
							regb_used_o = 1'b1;
							alu_operator_o = sv2v_cast_576C1(7'b0011000);
							alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
							prepost_useincr_o = 1'b0;
							regfile_alu_waddr_sel_o = 1'b0;
							regfile_alu_we = 1'b1;
							case (instr_rdata_i[13:12])
								2'b00: data_type_o = 2'b10;
								2'b01: data_type_o = 2'b01;
								default: data_type_o = 2'b00;
							endcase
						end
						3'b011:
							case (instr_rdata_i[31:25])
								7'b0000000, 7'b0000001, 7'b0000010, 7'b0000011, 7'b0000100, 7'b0000101, 7'b0000110, 7'b0000111, 7'b0001000, 7'b0001001, 7'b0001010, 7'b0001011, 7'b0001100, 7'b0001101, 7'b0001110, 7'b0001111: begin
									data_req = 1'b1;
									regfile_mem_we = 1'b1;
									rega_used_o = 1'b1;
									alu_operator_o = sv2v_cast_576C1(7'b0011000);
									regb_used_o = 1'b1;
									alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGB_OR_FWD;
									if (instr_rdata_i[27] == 1'b0) begin
										prepost_useincr_o = 1'b0;
										regfile_alu_waddr_sel_o = 1'b0;
										regfile_alu_we = 1'b1;
									end
									data_sign_extension_o = {1'b0, ~instr_rdata_i[28]};
									case ({instr_rdata_i[28], instr_rdata_i[26:25]})
										3'b000: data_type_o = 2'b10;
										3'b001: data_type_o = 2'b01;
										3'b010: data_type_o = 2'b00;
										3'b100: data_type_o = 2'b10;
										3'b101: data_type_o = 2'b01;
										default: begin
											illegal_insn_o = 1'b1;
											data_req = 1'b0;
											regfile_mem_we = 1'b0;
											regfile_alu_we = 1'b0;
										end
									endcase
								end
								7'b0010000, 7'b0010001, 7'b0010010, 7'b0010011, 7'b0010100, 7'b0010101, 7'b0010110, 7'b0010111: begin
									data_req = 1'b1;
									data_we_o = 1'b1;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									alu_operator_o = sv2v_cast_576C1(7'b0011000);
									alu_op_c_mux_sel_o = cv32e40p_pkg_OP_C_REGB_OR_FWD;
									regc_used_o = 1'b1;
									alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGC_OR_FWD;
									regc_mux_o = cv32e40p_pkg_REGC_RD;
									if (instr_rdata_i[27] == 1'b0) begin
										prepost_useincr_o = 1'b0;
										regfile_alu_waddr_sel_o = 1'b0;
										regfile_alu_we = 1'b1;
									end
									case (instr_rdata_i[26:25])
										2'b00: data_type_o = 2'b10;
										2'b01: data_type_o = 2'b01;
										2'b10: data_type_o = 2'b00;
										default: begin
											illegal_insn_o = 1'b1;
											data_req = 1'b0;
											data_we_o = 1'b0;
											data_type_o = 2'b00;
										end
									endcase
								end
								7'b0011000, 7'b0011001, 7'b0011010, 7'b0011011, 7'b0011100, 7'b0011101, 7'b0011110, 7'b0011111: begin
									regfile_alu_we = 1'b1;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									bmask_a_mux_o = cv32e40p_pkg_BMASK_A_S3;
									bmask_b_mux_o = cv32e40p_pkg_BMASK_B_S2;
									alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
									alu_bmask_a_mux_sel_o = cv32e40p_pkg_BMASK_A_REG;
									case (instr_rdata_i[27:25])
										3'b000: begin
											alu_operator_o = sv2v_cast_576C1(7'b0101000);
											imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S2;
											bmask_b_mux_o = cv32e40p_pkg_BMASK_B_ZERO;
											alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_BMASK;
										end
										3'b001: begin
											alu_operator_o = sv2v_cast_576C1(7'b0101001);
											imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S2;
											bmask_b_mux_o = cv32e40p_pkg_BMASK_B_ZERO;
											alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_BMASK;
										end
										3'b010: begin
											alu_operator_o = sv2v_cast_576C1(7'b0101010);
											imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S2;
											regc_used_o = 1'b1;
											regc_mux_o = cv32e40p_pkg_REGC_RD;
											alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_BMASK;
											alu_bmask_b_mux_sel_o = cv32e40p_pkg_BMASK_B_REG;
										end
										3'b100: begin
											alu_operator_o = sv2v_cast_576C1(7'b0101011);
											alu_bmask_b_mux_sel_o = cv32e40p_pkg_BMASK_B_REG;
										end
										3'b101: begin
											alu_operator_o = sv2v_cast_576C1(7'b0101100);
											alu_bmask_b_mux_sel_o = cv32e40p_pkg_BMASK_B_REG;
										end
										default: illegal_insn_o = 1'b1;
									endcase
								end
								7'b0100000, 7'b0100001, 7'b0100010, 7'b0100011, 7'b0100100, 7'b0100101, 7'b0100110, 7'b0100111, 7'b0101000, 7'b0101001, 7'b0101010, 7'b0101011, 7'b0101100, 7'b0101101, 7'b0101110, 7'b0101111, 7'b0110000, 7'b0110001, 7'b0110010, 7'b0110011, 7'b0110100, 7'b0110101, 7'b0110110, 7'b0110111, 7'b0111000, 7'b0111001, 7'b0111010, 7'b0111011, 7'b0111100, 7'b0111101, 7'b0111110, 7'b0111111: begin
									regfile_alu_we = 1'b1;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									case (instr_rdata_i[29:25])
										5'b00000: alu_operator_o = sv2v_cast_576C1(7'b0100110);
										5'b00001: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0110110);
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b00010: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0110111);
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b00011: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0110101);
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b00100: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0110100);
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b01000: begin
											alu_operator_o = sv2v_cast_576C1(7'b0010100);
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b01001: alu_operator_o = sv2v_cast_576C1(7'b0000110);
										5'b01010: alu_operator_o = sv2v_cast_576C1(7'b0000111);
										5'b01011: alu_operator_o = sv2v_cast_576C1(7'b0010000);
										5'b01100: alu_operator_o = sv2v_cast_576C1(7'b0010001);
										5'b01101: alu_operator_o = sv2v_cast_576C1(7'b0010010);
										5'b01110: alu_operator_o = sv2v_cast_576C1(7'b0010011);
										5'b10000: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0111110);
											alu_vec_mode_o = cv32e40p_pkg_VEC_MODE16;
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b10001: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0111111);
											alu_vec_mode_o = cv32e40p_pkg_VEC_MODE16;
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b10010: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0111110);
											alu_vec_mode_o = cv32e40p_pkg_VEC_MODE8;
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b10011: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0111111);
											alu_vec_mode_o = cv32e40p_pkg_VEC_MODE8;
											if (instr_rdata_i[24:20] != 5'b00000)
												illegal_insn_o = 1'b1;
										end
										5'b11000: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0010110);
											alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
											imm_b_mux_sel_o = cv32e40p_pkg_IMMB_CLIP;
										end
										5'b11001: begin
											regb_used_o = 1'b0;
											alu_operator_o = sv2v_cast_576C1(7'b0010111);
											alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
											imm_b_mux_sel_o = cv32e40p_pkg_IMMB_CLIP;
										end
										5'b11010: alu_operator_o = sv2v_cast_576C1(7'b0010110);
										5'b11011: alu_operator_o = sv2v_cast_576C1(7'b0010111);
										default: illegal_insn_o = 1'b1;
									endcase
								end
								7'b1000000, 7'b1000001, 7'b1000010, 7'b1000011, 7'b1000100, 7'b1000101, 7'b1000110, 7'b1000111: begin
									regfile_alu_we = 1'b1;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									regc_used_o = 1'b1;
									regc_mux_o = cv32e40p_pkg_REGC_RD;
									bmask_a_mux_o = cv32e40p_pkg_BMASK_A_ZERO;
									bmask_b_mux_o = cv32e40p_pkg_BMASK_B_S3;
									alu_bmask_b_mux_sel_o = cv32e40p_pkg_BMASK_B_REG;
									alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGC_OR_FWD;
									alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGA_OR_FWD;
									case (instr_rdata_i[27:25])
										3'b000: alu_operator_o = sv2v_cast_576C1(7'b0011000);
										3'b001: alu_operator_o = sv2v_cast_576C1(7'b0011010);
										3'b010: alu_operator_o = sv2v_cast_576C1(7'b0011100);
										3'b011: alu_operator_o = sv2v_cast_576C1(7'b0011110);
										3'b100: alu_operator_o = sv2v_cast_576C1(7'b0011001);
										3'b101: alu_operator_o = sv2v_cast_576C1(7'b0011011);
										3'b110: alu_operator_o = sv2v_cast_576C1(7'b0011101);
										3'b111: alu_operator_o = sv2v_cast_576C1(7'b0011111);
										default: alu_operator_o = sv2v_cast_576C1(7'b0011000);
									endcase
								end
								7'b1001000, 7'b1001001: begin
									alu_en = 1'b0;
									mult_int_en = 1'b1;
									regfile_alu_we = 1'b1;
									rega_used_o = 1'b1;
									regb_used_o = 1'b1;
									regc_used_o = 1'b1;
									regc_mux_o = cv32e40p_pkg_REGC_RD;
									if (instr_rdata_i[25] == 1'b0)
										mult_operator_o = sv2v_cast_9D1C7(3'b000);
									else
										mult_operator_o = sv2v_cast_9D1C7(3'b001);
								end
								default: illegal_insn_o = 1'b1;
							endcase
						3'b100: begin
							hwlp_target_mux_sel_o = 2'b00;
							case (instr_rdata_i[11:8])
								4'b0000: begin
									hwlp_we[0] = 1'b1;
									hwlp_start_mux_sel_o = 2'b00;
									if (instr_rdata_i[19:15] != 5'b00000)
										illegal_insn_o = 1'b1;
								end
								4'b0001: begin
									hwlp_we[0] = 1'b1;
									hwlp_start_mux_sel_o = 2'b10;
									rega_used_o = 1'b1;
									if (instr_rdata_i[31:20] != 12'b000000000000)
										illegal_insn_o = 1'b1;
								end
								4'b0010: begin
									hwlp_we[1] = 1'b1;
									if (instr_rdata_i[19:15] != 5'b00000)
										illegal_insn_o = 1'b1;
								end
								4'b0011: begin
									hwlp_we[1] = 1'b1;
									hwlp_target_mux_sel_o = 2'b10;
									rega_used_o = 1'b1;
									if (instr_rdata_i[31:20] != 12'b000000000000)
										illegal_insn_o = 1'b1;
								end
								4'b0100: begin
									hwlp_we[2] = 1'b1;
									hwlp_cnt_mux_sel_o = 1'b0;
									if (instr_rdata_i[19:15] != 5'b00000)
										illegal_insn_o = 1'b1;
								end
								4'b0101: begin
									hwlp_we[2] = 1'b1;
									hwlp_cnt_mux_sel_o = 1'b1;
									rega_used_o = 1'b1;
									if (instr_rdata_i[31:20] != 12'b000000000000)
										illegal_insn_o = 1'b1;
								end
								4'b0110: begin
									hwlp_we = 3'b111;
									hwlp_target_mux_sel_o = 2'b01;
									hwlp_start_mux_sel_o = 2'b01;
									hwlp_cnt_mux_sel_o = 1'b0;
								end
								4'b0111: begin
									hwlp_we = 3'b111;
									hwlp_start_mux_sel_o = 2'b01;
									hwlp_cnt_mux_sel_o = 1'b1;
									rega_used_o = 1'b1;
								end
								default: illegal_insn_o = 1'b1;
							endcase
						end
						default: illegal_insn_o = 1'b1;
					endcase
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_CUSTOM_2:
				if (COREV_PULP) begin
					regfile_alu_we = 1'b1;
					rega_used_o = 1'b1;
					regb_used_o = 1'b1;
					case (instr_rdata_i[14:13])
						2'b00: begin
							regb_used_o = 1'b0;
							bmask_a_mux_o = cv32e40p_pkg_BMASK_A_S3;
							bmask_b_mux_o = cv32e40p_pkg_BMASK_B_S2;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
							case ({instr_rdata_i[31:30], instr_rdata_i[12]})
								3'b000: begin
									alu_operator_o = sv2v_cast_576C1(7'b0101000);
									imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S2;
									bmask_b_mux_o = cv32e40p_pkg_BMASK_B_ZERO;
								end
								3'b010: begin
									alu_operator_o = sv2v_cast_576C1(7'b0101001);
									imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S2;
									bmask_b_mux_o = cv32e40p_pkg_BMASK_B_ZERO;
								end
								3'b100: begin
									alu_operator_o = sv2v_cast_576C1(7'b0101010);
									imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S2;
									regc_used_o = 1'b1;
									regc_mux_o = cv32e40p_pkg_REGC_RD;
								end
								3'b001: alu_operator_o = sv2v_cast_576C1(7'b0101011);
								3'b011: alu_operator_o = sv2v_cast_576C1(7'b0101100);
								3'b111: begin
									alu_operator_o = sv2v_cast_576C1(7'b1001001);
									regc_used_o = 1'b1;
									regc_mux_o = cv32e40p_pkg_REGC_RD;
									imm_b_mux_sel_o = cv32e40p_pkg_IMMB_S2;
									alu_bmask_a_mux_sel_o = cv32e40p_pkg_BMASK_A_IMM;
									if (instr_rdata_i[29:27] != 3'b000)
										illegal_insn_o = 1'b1;
								end
								default: illegal_insn_o = 1'b1;
							endcase
						end
						2'b01: begin
							bmask_a_mux_o = cv32e40p_pkg_BMASK_A_ZERO;
							bmask_b_mux_o = cv32e40p_pkg_BMASK_B_S3;
							case ({instr_rdata_i[31:30], instr_rdata_i[12]})
								3'b000: alu_operator_o = sv2v_cast_576C1(7'b0011000);
								3'b010: alu_operator_o = sv2v_cast_576C1(7'b0011010);
								3'b100: alu_operator_o = sv2v_cast_576C1(7'b0011100);
								3'b110: alu_operator_o = sv2v_cast_576C1(7'b0011110);
								3'b001: alu_operator_o = sv2v_cast_576C1(7'b0011001);
								3'b011: alu_operator_o = sv2v_cast_576C1(7'b0011011);
								3'b101: alu_operator_o = sv2v_cast_576C1(7'b0011101);
								3'b111: alu_operator_o = sv2v_cast_576C1(7'b0011111);
								default: alu_operator_o = sv2v_cast_576C1(7'b0011000);
							endcase
						end
						2'b10, 2'b11: begin
							alu_en = 1'b0;
							mult_int_en = 1'b1;
							mult_imm_mux_o = cv32e40p_pkg_MIMM_S3;
							mult_sel_subword_o = instr_rdata_i[30];
							mult_signed_mode_o = {2 {~instr_rdata_i[12]}};
							if (instr_rdata_i[13]) begin
								regc_used_o = 1'b1;
								regc_mux_o = cv32e40p_pkg_REGC_RD;
							end
							else
								regc_mux_o = cv32e40p_pkg_REGC_ZERO;
							if (instr_rdata_i[31])
								mult_operator_o = sv2v_cast_9D1C7(3'b011);
							else
								mult_operator_o = sv2v_cast_9D1C7(3'b010);
						end
						default: illegal_insn_o = 1'b1;
					endcase
				end
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_CUSTOM_3:
				if (COREV_PULP) begin
					regfile_alu_we = 1'b1;
					rega_used_o = 1'b1;
					imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
					alu_vec_o = 1'b1;
					if (instr_rdata_i[12]) begin
						alu_vec_mode_o = cv32e40p_pkg_VEC_MODE8;
						mult_operator_o = sv2v_cast_9D1C7(3'b100);
					end
					else begin
						alu_vec_mode_o = cv32e40p_pkg_VEC_MODE16;
						mult_operator_o = sv2v_cast_9D1C7(3'b101);
					end
					if (instr_rdata_i[14]) begin
						scalar_replication_o = 1'b1;
						if (instr_rdata_i[13])
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
						else
							regb_used_o = 1'b1;
					end
					else
						regb_used_o = 1'b1;
					case (instr_rdata_i[31:26])
						6'b000000: begin
							alu_operator_o = sv2v_cast_576C1(7'b0011000);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b000010: begin
							alu_operator_o = sv2v_cast_576C1(7'b0011001);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b000100: begin
							alu_operator_o = sv2v_cast_576C1(7'b0011000);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							bmask_b_mux_o = cv32e40p_pkg_BMASK_B_ONE;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b000110: begin
							alu_operator_o = sv2v_cast_576C1(7'b0011010);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							bmask_b_mux_o = cv32e40p_pkg_BMASK_B_ONE;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001000: begin
							alu_operator_o = sv2v_cast_576C1(7'b0010000);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001010: begin
							alu_operator_o = sv2v_cast_576C1(7'b0010001);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001100: begin
							alu_operator_o = sv2v_cast_576C1(7'b0010010);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001110: begin
							alu_operator_o = sv2v_cast_576C1(7'b0010011);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b010000: begin
							alu_operator_o = sv2v_cast_576C1(7'b0100101);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b010010: begin
							alu_operator_o = sv2v_cast_576C1(7'b0100100);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b010100: begin
							alu_operator_o = sv2v_cast_576C1(7'b0100111);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b010110: begin
							alu_operator_o = sv2v_cast_576C1(7'b0101110);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b011000: begin
							alu_operator_o = sv2v_cast_576C1(7'b0101111);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b011010: begin
							alu_operator_o = sv2v_cast_576C1(7'b0010101);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b011100: begin
							alu_operator_o = sv2v_cast_576C1(7'b0010100);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] != 3'b000) && (instr_rdata_i[14:12] != 3'b001))
								illegal_insn_o = 1'b1;
							if (instr_rdata_i[25:20] != 6'b000000)
								illegal_insn_o = 1'b1;
						end
						6'b100000: begin
							alu_en = 1'b0;
							mult_dot_en = 1'b1;
							mult_dot_signed_o = 2'b00;
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b100010: begin
							alu_en = 1'b0;
							mult_dot_en = 1'b1;
							mult_dot_signed_o = 2'b01;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b100100: begin
							alu_en = 1'b0;
							mult_dot_en = 1'b1;
							mult_dot_signed_o = 2'b11;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b100110: begin
							alu_en = 1'b0;
							mult_dot_en = 1'b1;
							mult_dot_signed_o = 2'b00;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b101000: begin
							alu_en = 1'b0;
							mult_dot_en = 1'b1;
							mult_dot_signed_o = 2'b01;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b101010: begin
							alu_en = 1'b0;
							mult_dot_en = 1'b1;
							mult_dot_signed_o = 2'b11;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b101110:
							case (instr_rdata_i[14:13])
								2'b00: alu_operator_o = sv2v_cast_576C1(7'b0111110);
								2'b01: alu_operator_o = sv2v_cast_576C1(7'b0111111);
								2'b10: begin
									alu_operator_o = sv2v_cast_576C1(7'b0101101);
									regc_used_o = 1'b1;
									regc_mux_o = cv32e40p_pkg_REGC_RD;
									alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGC_OR_FWD;
								end
								default: illegal_insn_o = 1'b1;
							endcase
						6'b110000: begin
							alu_operator_o = sv2v_cast_576C1(7'b0111010);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_SHUF;
							regb_used_o = 1'b1;
							scalar_replication_o = 1'b0;
							if ((((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011)) || (instr_rdata_i[14:12] == 3'b100)) || (instr_rdata_i[14:12] == 3'b101))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b110010, 6'b110100, 6'b110110: begin
							alu_operator_o = sv2v_cast_576C1(7'b0111010);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_SHUF;
							regb_used_o = 1'b1;
							scalar_replication_o = 1'b0;
							if (instr_rdata_i[14:12] != 3'b111)
								illegal_insn_o = 1'b1;
						end
						6'b111000: begin
							alu_operator_o = sv2v_cast_576C1(7'b0111011);
							regb_used_o = 1'b1;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							scalar_replication_o = 1'b0;
							if ((instr_rdata_i[14:12] != 3'b000) && (instr_rdata_i[14:12] != 3'b001))
								illegal_insn_o = 1'b1;
							if (instr_rdata_i[25] != 1'b0)
								illegal_insn_o = 1'b1;
						end
						6'b111100: begin
							alu_operator_o = (instr_rdata_i[25] ? sv2v_cast_576C1(7'b0111001) : sv2v_cast_576C1(7'b0111000));
							regb_used_o = 1'b1;
							if (instr_rdata_i[14:12] != 3'b000)
								illegal_insn_o = 1'b1;
						end
						6'b111110: begin
							alu_operator_o = (instr_rdata_i[25] ? sv2v_cast_576C1(7'b0111001) : sv2v_cast_576C1(7'b0111000));
							regb_used_o = 1'b1;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							if (instr_rdata_i[14:12] != 3'b001)
								illegal_insn_o = 1'b1;
						end
						6'b000001: begin
							alu_operator_o = sv2v_cast_576C1(7'b0001100);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b000011: begin
							alu_operator_o = sv2v_cast_576C1(7'b0001101);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b000101: begin
							alu_operator_o = sv2v_cast_576C1(7'b0001000);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b000111: begin
							alu_operator_o = sv2v_cast_576C1(7'b0001010);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001001: begin
							alu_operator_o = sv2v_cast_576C1(7'b0000000);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001011: begin
							alu_operator_o = sv2v_cast_576C1(7'b0000100);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VS;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001101: begin
							alu_operator_o = sv2v_cast_576C1(7'b0001001);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b001111: begin
							alu_operator_o = sv2v_cast_576C1(7'b0001011);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b010001: begin
							alu_operator_o = sv2v_cast_576C1(7'b0000001);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b010011: begin
							alu_operator_o = sv2v_cast_576C1(7'b0000101);
							imm_b_mux_sel_o = cv32e40p_pkg_IMMB_VU;
							if ((instr_rdata_i[14:12] == 3'b010) || (instr_rdata_i[14:12] == 3'b011))
								illegal_insn_o = 1'b1;
							if (((instr_rdata_i[14:12] != 3'b110) && (instr_rdata_i[14:12] != 3'b111)) && (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b010101: begin
							alu_en = 1'b0;
							mult_dot_en = 1'b1;
							mult_dot_signed_o = 2'b11;
							is_clpx_o = 1'b1;
							regc_used_o = 1'b1;
							regc_mux_o = cv32e40p_pkg_REGC_RD;
							scalar_replication_o = 1'b0;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGB_OR_FWD;
							regb_used_o = 1'b1;
							illegal_insn_o = instr_rdata_i[12];
						end
						6'b010111: begin
							alu_operator_o = sv2v_cast_576C1(7'b0010100);
							is_clpx_o = 1'b1;
							scalar_replication_o = 1'b0;
							regb_used_o = 1'b0;
							if ((instr_rdata_i[14:12] != 3'b000) || (instr_rdata_i[25:20] != 6'b000000))
								illegal_insn_o = 1'b1;
						end
						6'b011001: begin
							alu_operator_o = sv2v_cast_576C1(7'b0011001);
							is_clpx_o = 1'b1;
							scalar_replication_o = 1'b0;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGB_OR_FWD;
							regb_used_o = 1'b1;
							is_subrot_o = 1'b1;
							if ((instr_rdata_i[12] != 1'b0) || (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b011011: begin
							alu_operator_o = sv2v_cast_576C1(7'b0011000);
							is_clpx_o = 1'b1;
							scalar_replication_o = 1'b0;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGB_OR_FWD;
							regb_used_o = 1'b1;
							if (((instr_rdata_i[12] != 1'b0) || (instr_rdata_i[14:12] == 3'b000)) || (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						6'b011101: begin
							alu_operator_o = sv2v_cast_576C1(7'b0011001);
							is_clpx_o = 1'b1;
							scalar_replication_o = 1'b0;
							alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_REGB_OR_FWD;
							regb_used_o = 1'b1;
							if (((instr_rdata_i[12] != 1'b0) || (instr_rdata_i[14:12] == 3'b000)) || (instr_rdata_i[25] != 1'b0))
								illegal_insn_o = 1'b1;
						end
						default: illegal_insn_o = 1'b1;
					endcase
				end
				else
					illegal_insn_o = 1'b1;
			cv32e40p_pkg_OPCODE_FENCE:
				case (instr_rdata_i[14:12])
					3'b000: fencei_insn_o = 1'b1;
					3'b001: fencei_insn_o = 1'b1;
					default: illegal_insn_o = 1'b1;
				endcase
			cv32e40p_pkg_OPCODE_SYSTEM:
				if (instr_rdata_i[14:12] == 3'b000) begin
					if ({instr_rdata_i[19:15], instr_rdata_i[11:7]} == {10 {1'sb0}})
						case (instr_rdata_i[31:20])
							12'h000: ecall_insn_o = 1'b1;
							12'h001: ebrk_insn_o = 1'b1;
							12'h302: begin
								illegal_insn_o = (PULP_SECURE ? current_priv_lvl_i != 2'b11 : 1'b0);
								mret_insn_o = ~illegal_insn_o;
								mret_dec_o = 1'b1;
							end
							12'h002: begin
								illegal_insn_o = (PULP_SECURE ? 1'b0 : 1'b1);
								uret_insn_o = ~illegal_insn_o;
								uret_dec_o = 1'b1;
							end
							12'h7b2: begin
								illegal_insn_o = !debug_mode_i;
								dret_insn_o = debug_mode_i;
								dret_dec_o = 1'b1;
							end
							12'h105: begin
								wfi_o = 1'b1;
								if (debug_wfi_no_sleep_i) begin
									alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
									imm_b_mux_sel_o = cv32e40p_pkg_IMMB_I;
									alu_operator_o = sv2v_cast_576C1(7'b0011000);
								end
							end
							default: illegal_insn_o = 1'b1;
						endcase
					else
						illegal_insn_o = 1'b1;
				end
				else begin
					csr_access_o = 1'b1;
					regfile_alu_we = 1'b1;
					alu_op_b_mux_sel_o = cv32e40p_pkg_OP_B_IMM;
					imm_a_mux_sel_o = cv32e40p_pkg_IMMA_Z;
					imm_b_mux_sel_o = cv32e40p_pkg_IMMB_I;
					if (instr_rdata_i[14] == 1'b1)
						alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_IMM;
					else begin
						rega_used_o = 1'b1;
						alu_op_a_mux_sel_o = cv32e40p_pkg_OP_A_REGA_OR_FWD;
					end
					case (instr_rdata_i[13:12])
						2'b01: csr_op = sv2v_cast_315CD(2'b01);
						2'b10: csr_op = (instr_rdata_i[19:15] == 5'b00000 ? sv2v_cast_315CD(2'b00) : sv2v_cast_315CD(2'b10));
						2'b11: csr_op = (instr_rdata_i[19:15] == 5'b00000 ? sv2v_cast_315CD(2'b00) : sv2v_cast_315CD(2'b11));
						default: csr_illegal = 1'b1;
					endcase
					if (instr_rdata_i[29:28] > current_priv_lvl_i)
						csr_illegal = 1'b1;
					case (instr_rdata_i[31:20])
						12'h001:
							if ((FPU == 0) || (fs_off_i == 1'b1))
								csr_illegal = 1'b1;
						12'h002, 12'h003:
							if ((FPU == 0) || (fs_off_i == 1'b1))
								csr_illegal = 1'b1;
							else if (csr_op != sv2v_cast_315CD(2'b00))
								csr_status_o = 1'b1;
						12'hf11, 12'hf12, 12'hf13, 12'hf14:
							if (csr_op != sv2v_cast_315CD(2'b00))
								csr_illegal = 1'b1;
						12'h300, 12'h341, 12'h305, 12'h342: csr_status_o = 1'b1;
						12'h301, 12'h304, 12'h340, 12'h343, 12'h344:
							;
						12'hb00, 12'hb02, 12'hb03, 12'hb04, 12'hb05, 12'hb06, 12'hb07, 12'hb08, 12'hb09, 12'hb0a, 12'hb0b, 12'hb0c, 12'hb0d, 12'hb0e, 12'hb0f, 12'hb10, 12'hb11, 12'hb12, 12'hb13, 12'hb14, 12'hb15, 12'hb16, 12'hb17, 12'hb18, 12'hb19, 12'hb1a, 12'hb1b, 12'hb1c, 12'hb1d, 12'hb1e, 12'hb1f, 12'hb80, 12'hb82, 12'hb83, 12'hb84, 12'hb85, 12'hb86, 12'hb87, 12'hb88, 12'hb89, 12'hb8a, 12'hb8b, 12'hb8c, 12'hb8d, 12'hb8e, 12'hb8f, 12'hb90, 12'hb91, 12'hb92, 12'hb93, 12'hb94, 12'hb95, 12'hb96, 12'hb97, 12'hb98, 12'hb99, 12'hb9a, 12'hb9b, 12'hb9c, 12'hb9d, 12'hb9e, 12'hb9f, 12'h320, 12'h323, 12'h324, 12'h325, 12'h326, 12'h327, 12'h328, 12'h329, 12'h32a, 12'h32b, 12'h32c, 12'h32d, 12'h32e, 12'h32f, 12'h330, 12'h331, 12'h332, 12'h333, 12'h334, 12'h335, 12'h336, 12'h337, 12'h338, 12'h339, 12'h33a, 12'h33b, 12'h33c, 12'h33d, 12'h33e, 12'h33f: csr_status_o = 1'b1;
						12'hc00, 12'hc02, 12'hc03, 12'hc04, 12'hc05, 12'hc06, 12'hc07, 12'hc08, 12'hc09, 12'hc0a, 12'hc0b, 12'hc0c, 12'hc0d, 12'hc0e, 12'hc0f, 12'hc10, 12'hc11, 12'hc12, 12'hc13, 12'hc14, 12'hc15, 12'hc16, 12'hc17, 12'hc18, 12'hc19, 12'hc1a, 12'hc1b, 12'hc1c, 12'hc1d, 12'hc1e, 12'hc1f, 12'hc80, 12'hc82, 12'hc83, 12'hc84, 12'hc85, 12'hc86, 12'hc87, 12'hc88, 12'hc89, 12'hc8a, 12'hc8b, 12'hc8c, 12'hc8d, 12'hc8e, 12'hc8f, 12'hc90, 12'hc91, 12'hc92, 12'hc93, 12'hc94, 12'hc95, 12'hc96, 12'hc97, 12'hc98, 12'hc99, 12'hc9a, 12'hc9b, 12'hc9c, 12'hc9d, 12'hc9e, 12'hc9f:
							if ((csr_op != sv2v_cast_315CD(2'b00)) || ((PULP_SECURE && (current_priv_lvl_i != 2'b11)) && !mcounteren_i[instr_rdata_i[24:20]]))
								csr_illegal = 1'b1;
							else
								csr_status_o = 1'b1;
						12'h306:
							if (!PULP_SECURE)
								csr_illegal = 1'b1;
							else
								csr_status_o = 1'b1;
						12'h7b0, 12'h7b1, 12'h7b2, 12'h7b3:
							if (!debug_mode_i)
								csr_illegal = 1'b1;
							else
								csr_status_o = 1'b1;
						12'h7a0, 12'h7a1, 12'h7a2, 12'h7a3, 12'h7a4, 12'h7a8, 12'h7aa:
							if (DEBUG_TRIGGER_EN != 1)
								csr_illegal = 1'b1;
						12'hcc0, 12'hcc1, 12'hcc2, 12'hcc4, 12'hcc5, 12'hcc6:
							if (!COREV_PULP || (csr_op != sv2v_cast_315CD(2'b00)))
								csr_illegal = 1'b1;
						12'hcd0:
							if (!COREV_PULP || (csr_op != sv2v_cast_315CD(2'b00)))
								csr_illegal = 1'b1;
						12'hcd1:
							if (!COREV_PULP || (csr_op != sv2v_cast_315CD(2'b00)))
								csr_illegal = 1'b1;
							else
								csr_status_o = 1'b1;
						12'hcd2:
							if ((!COREV_PULP || (FPU && !ZFINX)) || (csr_op != sv2v_cast_315CD(2'b00)))
								csr_illegal = 1'b1;
						12'h3a0, 12'h3a1, 12'h3a2, 12'h3a3, 12'h3b0, 12'h3b1, 12'h3b2, 12'h3b3, 12'h3b4, 12'h3b5, 12'h3b6, 12'h3b7, 12'h3b8, 12'h3b9, 12'h3ba, 12'h3bb, 12'h3bc, 12'h3bd, 12'h3be, 12'h3bf:
							if (!USE_PMP)
								csr_illegal = 1'b1;
						12'h000, 12'h041, 12'h005, 12'h042:
							if (!PULP_SECURE)
								csr_illegal = 1'b1;
							else
								csr_status_o = 1'b1;
						default: csr_illegal = 1'b1;
					endcase
					illegal_insn_o = csr_illegal;
				end
			default: illegal_insn_o = 1'b1;
		endcase
		if (illegal_c_insn_i)
			illegal_insn_o = 1'b1;
	end
	assign alu_en_o = (deassert_we_i ? 1'b0 : alu_en);
	assign mult_int_en_o = (deassert_we_i ? 1'b0 : mult_int_en);
	assign mult_dot_en_o = (deassert_we_i ? 1'b0 : mult_dot_en);
	assign apu_en_o = (deassert_we_i ? 1'b0 : apu_en);
	assign regfile_mem_we_o = (deassert_we_i ? 1'b0 : regfile_mem_we);
	assign regfile_alu_we_o = (deassert_we_i ? 1'b0 : regfile_alu_we);
	assign data_req_o = (deassert_we_i ? 1'b0 : data_req);
	assign hwlp_we_o = (deassert_we_i ? 3'b000 : hwlp_we);
	assign csr_op_o = (deassert_we_i ? sv2v_cast_315CD(2'b00) : csr_op);
	assign ctrl_transfer_insn_in_id_o = (deassert_we_i ? cv32e40p_pkg_BRANCH_NONE : ctrl_transfer_insn);
	assign ctrl_transfer_insn_in_dec_o = ctrl_transfer_insn;
	assign regfile_alu_we_dec_o = regfile_alu_we;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_compressed_decoder (
	instr_i,
	instr_o,
	is_compressed_o,
	illegal_instr_o
);
	reg _sv2v_0;
	parameter FPU = 0;
	parameter ZFINX = 0;
	input wire [31:0] instr_i;
	output reg [31:0] instr_o;
	output wire is_compressed_o;
	output reg illegal_instr_o;
	localparam cv32e40p_pkg_OPCODE_BRANCH = 7'h63;
	localparam cv32e40p_pkg_OPCODE_JAL = 7'h6f;
	localparam cv32e40p_pkg_OPCODE_JALR = 7'h67;
	localparam cv32e40p_pkg_OPCODE_LOAD = 7'h03;
	localparam cv32e40p_pkg_OPCODE_LOAD_FP = 7'h07;
	localparam cv32e40p_pkg_OPCODE_LUI = 7'h37;
	localparam cv32e40p_pkg_OPCODE_OP = 7'h33;
	localparam cv32e40p_pkg_OPCODE_OPIMM = 7'h13;
	localparam cv32e40p_pkg_OPCODE_STORE = 7'h23;
	localparam cv32e40p_pkg_OPCODE_STORE_FP = 7'h27;
	always @(*) begin
		if (_sv2v_0)
			;
		illegal_instr_o = 1'b0;
		instr_o = 1'sb0;
		case (instr_i[1:0])
			2'b00:
				case (instr_i[15:13])
					3'b000: begin
						instr_o = {2'b00, instr_i[10:7], instr_i[12:11], instr_i[5], instr_i[6], 12'h041, instr_i[4:2], cv32e40p_pkg_OPCODE_OPIMM};
						if (instr_i[12:5] == 8'b00000000)
							illegal_instr_o = 1'b1;
					end
					3'b001:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {4'b0000, instr_i[6:5], instr_i[12:10], 5'b00001, instr_i[9:7], 5'b01101, instr_i[4:2], cv32e40p_pkg_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b010: instr_o = {5'b00000, instr_i[5], instr_i[12:10], instr_i[6], 4'b0001, instr_i[9:7], 5'b01001, instr_i[4:2], cv32e40p_pkg_OPCODE_LOAD};
					3'b011:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {5'b00000, instr_i[5], instr_i[12:10], instr_i[6], 4'b0001, instr_i[9:7], 5'b01001, instr_i[4:2], cv32e40p_pkg_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b101:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {4'b0000, instr_i[6:5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b011, instr_i[11:10], 3'b000, cv32e40p_pkg_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					3'b110: instr_o = {5'b00000, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, cv32e40p_pkg_OPCODE_STORE};
					3'b111:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {5'b00000, instr_i[5], instr_i[12], 2'b01, instr_i[4:2], 2'b01, instr_i[9:7], 3'b010, instr_i[11:10], instr_i[6], 2'b00, cv32e40p_pkg_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					default: illegal_instr_o = 1'b1;
				endcase
			2'b01:
				case (instr_i[15:13])
					3'b000: instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], instr_i[11:7], 3'b000, instr_i[11:7], cv32e40p_pkg_OPCODE_OPIMM};
					3'b001, 3'b101: instr_o = {instr_i[12], instr_i[8], instr_i[10:9], instr_i[6], instr_i[7], instr_i[2], instr_i[11], instr_i[5:3], {9 {instr_i[12]}}, 4'b0000, ~instr_i[15], cv32e40p_pkg_OPCODE_JAL};
					3'b010:
						if (instr_i[11:7] == 5'b00000)
							instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 8'b00000000, instr_i[11:7], cv32e40p_pkg_OPCODE_OPIMM};
						else
							instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 8'b00000000, instr_i[11:7], cv32e40p_pkg_OPCODE_OPIMM};
					3'b011:
						if ({instr_i[12], instr_i[6:2]} == 6'b000000)
							illegal_instr_o = 1'b1;
						else if (instr_i[11:7] == 5'h02)
							instr_o = {{3 {instr_i[12]}}, instr_i[4:3], instr_i[5], instr_i[2], instr_i[6], 17'h00202, cv32e40p_pkg_OPCODE_OPIMM};
						else if (instr_i[11:7] == 5'b00000)
							instr_o = {{15 {instr_i[12]}}, instr_i[6:2], instr_i[11:7], cv32e40p_pkg_OPCODE_LUI};
						else
							instr_o = {{15 {instr_i[12]}}, instr_i[6:2], instr_i[11:7], cv32e40p_pkg_OPCODE_LUI};
					3'b100:
						case (instr_i[11:10])
							2'b00, 2'b01:
								if (instr_i[12] == 1'b1) begin
									instr_o = {1'b0, instr_i[10], 5'b00000, instr_i[6:2], 2'b01, instr_i[9:7], 5'b10101, instr_i[9:7], cv32e40p_pkg_OPCODE_OPIMM};
									illegal_instr_o = 1'b1;
								end
								else if (instr_i[6:2] == 5'b00000)
									instr_o = {1'b0, instr_i[10], 5'b00000, instr_i[6:2], 2'b01, instr_i[9:7], 5'b10101, instr_i[9:7], cv32e40p_pkg_OPCODE_OPIMM};
								else
									instr_o = {1'b0, instr_i[10], 5'b00000, instr_i[6:2], 2'b01, instr_i[9:7], 5'b10101, instr_i[9:7], cv32e40p_pkg_OPCODE_OPIMM};
							2'b10: instr_o = {{6 {instr_i[12]}}, instr_i[12], instr_i[6:2], 2'b01, instr_i[9:7], 5'b11101, instr_i[9:7], cv32e40p_pkg_OPCODE_OPIMM};
							2'b11:
								case ({instr_i[12], instr_i[6:5]})
									3'b000: instr_o = {9'b010000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b00001, instr_i[9:7], cv32e40p_pkg_OPCODE_OP};
									3'b001: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b10001, instr_i[9:7], cv32e40p_pkg_OPCODE_OP};
									3'b010: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b11001, instr_i[9:7], cv32e40p_pkg_OPCODE_OP};
									3'b011: instr_o = {9'b000000001, instr_i[4:2], 2'b01, instr_i[9:7], 5'b11101, instr_i[9:7], cv32e40p_pkg_OPCODE_OP};
									3'b100, 3'b101, 3'b110, 3'b111: illegal_instr_o = 1'b1;
								endcase
						endcase
					3'b110, 3'b111: instr_o = {{4 {instr_i[12]}}, instr_i[6:5], instr_i[2], 7'b0000001, instr_i[9:7], 2'b00, instr_i[13], instr_i[11:10], instr_i[4:3], instr_i[12], cv32e40p_pkg_OPCODE_BRANCH};
				endcase
			2'b10:
				case (instr_i[15:13])
					3'b000:
						if (instr_i[12] == 1'b1) begin
							instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], cv32e40p_pkg_OPCODE_OPIMM};
							illegal_instr_o = 1'b1;
						end
						else if ((instr_i[6:2] == 5'b00000) || (instr_i[11:7] == 5'b00000))
							instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], cv32e40p_pkg_OPCODE_OPIMM};
						else
							instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b001, instr_i[11:7], cv32e40p_pkg_OPCODE_OPIMM};
					3'b001:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {3'b000, instr_i[4:2], instr_i[12], instr_i[6:5], 11'h013, instr_i[11:7], cv32e40p_pkg_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b010: begin
						instr_o = {4'b0000, instr_i[3:2], instr_i[12], instr_i[6:4], 10'h012, instr_i[11:7], cv32e40p_pkg_OPCODE_LOAD};
						if (instr_i[11:7] == 5'b00000)
							illegal_instr_o = 1'b1;
					end
					3'b011:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {4'b0000, instr_i[3:2], instr_i[12], instr_i[6:4], 10'h012, instr_i[11:7], cv32e40p_pkg_OPCODE_LOAD_FP};
						else
							illegal_instr_o = 1'b1;
					3'b100:
						if (instr_i[12] == 1'b0) begin
							if (instr_i[6:2] == 5'b00000) begin
								instr_o = {12'b000000000000, instr_i[11:7], 8'b00000000, cv32e40p_pkg_OPCODE_JALR};
								if (instr_i[11:7] == 5'b00000)
									illegal_instr_o = 1'b1;
							end
							else if (instr_i[11:7] == 5'b00000)
								instr_o = {7'b0000000, instr_i[6:2], 8'b00000000, instr_i[11:7], cv32e40p_pkg_OPCODE_OP};
							else
								instr_o = {7'b0000000, instr_i[6:2], 8'b00000000, instr_i[11:7], cv32e40p_pkg_OPCODE_OP};
						end
						else if (instr_i[6:2] == 5'b00000) begin
							if (instr_i[11:7] == 5'b00000)
								instr_o = 32'h00100073;
							else
								instr_o = {12'b000000000000, instr_i[11:7], 8'b00000001, cv32e40p_pkg_OPCODE_JALR};
						end
						else if (instr_i[11:7] == 5'b00000)
							instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b000, instr_i[11:7], cv32e40p_pkg_OPCODE_OP};
						else
							instr_o = {7'b0000000, instr_i[6:2], instr_i[11:7], 3'b000, instr_i[11:7], cv32e40p_pkg_OPCODE_OP};
					3'b101:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {3'b000, instr_i[9:7], instr_i[12], instr_i[6:2], 8'h13, instr_i[11:10], 3'b000, cv32e40p_pkg_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
					3'b110: instr_o = {4'b0000, instr_i[8:7], instr_i[12], instr_i[6:2], 8'h12, instr_i[11:9], 2'b00, cv32e40p_pkg_OPCODE_STORE};
					3'b111:
						if ((FPU == 1) && (ZFINX == 0))
							instr_o = {4'b0000, instr_i[8:7], instr_i[12], instr_i[6:2], 8'h12, instr_i[11:9], 2'b00, cv32e40p_pkg_OPCODE_STORE_FP};
						else
							illegal_instr_o = 1'b1;
				endcase
			default: instr_o = instr_i;
		endcase
	end
	assign is_compressed_o = instr_i[1:0] != 2'b11;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_fifo (
	clk_i,
	rst_ni,
	flush_i,
	flush_but_first_i,
	testmode_i,
	full_o,
	empty_o,
	cnt_o,
	data_i,
	push_i,
	data_o,
	pop_i
);
	reg _sv2v_0;
	parameter [0:0] FALL_THROUGH = 1'b0;
	parameter [31:0] DATA_WIDTH = 32;
	parameter [31:0] DEPTH = 8;
	parameter [31:0] ADDR_DEPTH = (DEPTH > 1 ? $clog2(DEPTH) : 1);
	input wire clk_i;
	input wire rst_ni;
	input wire flush_i;
	input wire flush_but_first_i;
	input wire testmode_i;
	output wire full_o;
	output wire empty_o;
	output wire [ADDR_DEPTH:0] cnt_o;
	input wire [DATA_WIDTH - 1:0] data_i;
	input wire push_i;
	output reg [DATA_WIDTH - 1:0] data_o;
	input wire pop_i;
	localparam [31:0] FIFO_DEPTH = (DEPTH > 0 ? DEPTH : 1);
	reg gate_clock;
	reg [ADDR_DEPTH - 1:0] read_pointer_n;
	reg [ADDR_DEPTH - 1:0] read_pointer_q;
	reg [ADDR_DEPTH - 1:0] write_pointer_n;
	reg [ADDR_DEPTH - 1:0] write_pointer_q;
	reg [ADDR_DEPTH:0] status_cnt_n;
	reg [ADDR_DEPTH:0] status_cnt_q;
	reg [(FIFO_DEPTH * DATA_WIDTH) - 1:0] mem_n;
	reg [(FIFO_DEPTH * DATA_WIDTH) - 1:0] mem_q;
	assign cnt_o = status_cnt_q;
	generate
		if (DEPTH == 0) begin : gen_zero_depth
			assign empty_o = ~push_i;
			assign full_o = ~pop_i;
		end
		else begin : gen_non_zero_depth
			assign full_o = status_cnt_q == FIFO_DEPTH[ADDR_DEPTH:0];
			assign empty_o = (status_cnt_q == 0) & ~(FALL_THROUGH & push_i);
		end
	endgenerate
	always @(*) begin : read_write_comb
		if (_sv2v_0)
			;
		read_pointer_n = read_pointer_q;
		write_pointer_n = write_pointer_q;
		status_cnt_n = status_cnt_q;
		data_o = (DEPTH == 0 ? data_i : mem_q[read_pointer_q * DATA_WIDTH+:DATA_WIDTH]);
		mem_n = mem_q;
		gate_clock = 1'b1;
		if (push_i && ~full_o) begin
			mem_n[write_pointer_q * DATA_WIDTH+:DATA_WIDTH] = data_i;
			gate_clock = 1'b0;
			if (write_pointer_q == (FIFO_DEPTH[ADDR_DEPTH - 1:0] - 1))
				write_pointer_n = 1'sb0;
			else
				write_pointer_n = write_pointer_q + 1;
			status_cnt_n = status_cnt_q + 1;
		end
		if (pop_i && ~empty_o) begin
			if (read_pointer_n == (FIFO_DEPTH[ADDR_DEPTH - 1:0] - 1))
				read_pointer_n = 1'sb0;
			else
				read_pointer_n = read_pointer_q + 1;
			status_cnt_n = status_cnt_q - 1;
		end
		if (((push_i && pop_i) && ~full_o) && ~empty_o)
			status_cnt_n = status_cnt_q;
		if ((FALL_THROUGH && (status_cnt_q == 0)) && push_i) begin
			data_o = data_i;
			if (pop_i) begin
				status_cnt_n = status_cnt_q;
				read_pointer_n = read_pointer_q;
				write_pointer_n = write_pointer_q;
			end
		end
	end
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni) begin
			read_pointer_q <= 1'sb0;
			write_pointer_q <= 1'sb0;
			status_cnt_q <= 1'sb0;
		end
		else
			case (1'b1)
				flush_i: begin
					read_pointer_q <= 1'sb0;
					write_pointer_q <= 1'sb0;
					status_cnt_q <= 1'sb0;
				end
				flush_but_first_i: begin
					read_pointer_q <= (status_cnt_q > 0 ? read_pointer_q : {ADDR_DEPTH {1'sb0}});
					write_pointer_q <= (status_cnt_q > 0 ? read_pointer_q + 1 : {ADDR_DEPTH {1'sb0}});
					status_cnt_q <= (status_cnt_q > 0 ? 1'b1 : {(ADDR_DEPTH >= 0 ? ADDR_DEPTH + 1 : 1 - ADDR_DEPTH) {1'sb0}});
				end
				default: begin
					read_pointer_q <= read_pointer_n;
					write_pointer_q <= write_pointer_n;
					status_cnt_q <= status_cnt_n;
				end
			endcase
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni)
			mem_q <= 1'sb0;
		else if (!gate_clock)
			mem_q <= mem_n;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_prefetch_buffer (
	clk,
	rst_n,
	req_i,
	branch_i,
	branch_addr_i,
	hwlp_jump_i,
	hwlp_target_i,
	fetch_ready_i,
	fetch_valid_o,
	fetch_rdata_o,
	instr_req_o,
	instr_gnt_i,
	instr_addr_o,
	instr_rdata_i,
	instr_rvalid_i,
	instr_err_i,
	instr_err_pmp_i,
	busy_o
);
	parameter PULP_OBI = 0;
	parameter COREV_PULP = 1;
	input wire clk;
	input wire rst_n;
	input wire req_i;
	input wire branch_i;
	input wire [31:0] branch_addr_i;
	input wire hwlp_jump_i;
	input wire [31:0] hwlp_target_i;
	input wire fetch_ready_i;
	output wire fetch_valid_o;
	output wire [31:0] fetch_rdata_o;
	output wire instr_req_o;
	input wire instr_gnt_i;
	output wire [31:0] instr_addr_o;
	input wire [31:0] instr_rdata_i;
	input wire instr_rvalid_i;
	input wire instr_err_i;
	input wire instr_err_pmp_i;
	output wire busy_o;
	localparam FIFO_DEPTH = 2;
	localparam [31:0] FIFO_ADDR_DEPTH = 1;
	wire trans_valid;
	wire trans_ready;
	wire [31:0] trans_addr;
	wire fifo_flush;
	wire fifo_flush_but_first;
	wire [FIFO_ADDR_DEPTH:0] fifo_cnt;
	wire [31:0] fifo_rdata;
	wire fifo_push;
	wire fifo_pop;
	wire fifo_empty;
	wire resp_valid;
	wire [31:0] resp_rdata;
	wire resp_err;
	cv32e40p_prefetch_controller #(
		.DEPTH(FIFO_DEPTH),
		.PULP_OBI(PULP_OBI),
		.COREV_PULP(COREV_PULP)
	) prefetch_controller_i(
		.clk(clk),
		.rst_n(rst_n),
		.req_i(req_i),
		.branch_i(branch_i),
		.branch_addr_i(branch_addr_i),
		.busy_o(busy_o),
		.hwlp_jump_i(hwlp_jump_i),
		.hwlp_target_i(hwlp_target_i),
		.trans_valid_o(trans_valid),
		.trans_ready_i(trans_ready),
		.trans_addr_o(trans_addr),
		.resp_valid_i(resp_valid),
		.fetch_ready_i(fetch_ready_i),
		.fetch_valid_o(fetch_valid_o),
		.fifo_push_o(fifo_push),
		.fifo_pop_o(fifo_pop),
		.fifo_flush_o(fifo_flush),
		.fifo_flush_but_first_o(fifo_flush_but_first),
		.fifo_cnt_i(fifo_cnt),
		.fifo_empty_i(fifo_empty)
	);
	cv32e40p_fifo #(
		.FALL_THROUGH(1'b0),
		.DATA_WIDTH(32),
		.DEPTH(FIFO_DEPTH)
	) fifo_i(
		.clk_i(clk),
		.rst_ni(rst_n),
		.flush_i(fifo_flush),
		.flush_but_first_i(fifo_flush_but_first),
		.testmode_i(1'b0),
		.full_o(),
		.empty_o(fifo_empty),
		.cnt_o(fifo_cnt),
		.data_i(resp_rdata),
		.push_i(fifo_push),
		.data_o(fifo_rdata),
		.pop_i(fifo_pop)
	);
	assign fetch_rdata_o = (fifo_empty ? resp_rdata : fifo_rdata);
	cv32e40p_obi_interface #(.TRANS_STABLE(0)) instruction_obi_i(
		.clk(clk),
		.rst_n(rst_n),
		.trans_valid_i(trans_valid),
		.trans_ready_o(trans_ready),
		.trans_addr_i({trans_addr[31:2], 2'b00}),
		.trans_we_i(1'b0),
		.trans_be_i(4'b1111),
		.trans_wdata_i(32'b00000000000000000000000000000000),
		.trans_atop_i(6'b000000),
		.resp_valid_o(resp_valid),
		.resp_rdata_o(resp_rdata),
		.resp_err_o(resp_err),
		.obi_req_o(instr_req_o),
		.obi_gnt_i(instr_gnt_i),
		.obi_addr_o(instr_addr_o),
		.obi_we_o(),
		.obi_be_o(),
		.obi_wdata_o(),
		.obi_atop_o(),
		.obi_rdata_i(instr_rdata_i),
		.obi_rvalid_i(instr_rvalid_i),
		.obi_err_i(instr_err_i)
	);
endmodule
module cv32e40p_hwloop_regs (
	clk,
	rst_n,
	hwlp_start_data_i,
	hwlp_end_data_i,
	hwlp_cnt_data_i,
	hwlp_we_i,
	hwlp_regid_i,
	valid_i,
	hwlp_dec_cnt_i,
	hwlp_start_addr_o,
	hwlp_end_addr_o,
	hwlp_counter_o
);
	parameter N_REGS = 2;
	parameter N_REG_BITS = $clog2(N_REGS);
	input wire clk;
	input wire rst_n;
	input wire [31:0] hwlp_start_data_i;
	input wire [31:0] hwlp_end_data_i;
	input wire [31:0] hwlp_cnt_data_i;
	input wire [2:0] hwlp_we_i;
	input wire [N_REG_BITS - 1:0] hwlp_regid_i;
	input wire valid_i;
	input wire [N_REGS - 1:0] hwlp_dec_cnt_i;
	output wire [(N_REGS * 32) - 1:0] hwlp_start_addr_o;
	output wire [(N_REGS * 32) - 1:0] hwlp_end_addr_o;
	output wire [(N_REGS * 32) - 1:0] hwlp_counter_o;
	reg [(N_REGS * 32) - 1:0] hwlp_start_q;
	reg [(N_REGS * 32) - 1:0] hwlp_end_q;
	reg [(N_REGS * 32) - 1:0] hwlp_counter_q;
	wire [(N_REGS * 32) - 1:0] hwlp_counter_n;
	reg [31:0] i;
	assign hwlp_start_addr_o = hwlp_start_q;
	assign hwlp_end_addr_o = hwlp_end_q;
	assign hwlp_counter_o = hwlp_counter_q;
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_START
		if (rst_n == 1'b0)
			hwlp_start_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else if (hwlp_we_i[0] == 1'b1)
			hwlp_start_q[hwlp_regid_i * 32+:32] <= {hwlp_start_data_i[31:2], 2'b00};
	end
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_END
		if (rst_n == 1'b0)
			hwlp_end_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else if (hwlp_we_i[1] == 1'b1)
			hwlp_end_q[hwlp_regid_i * 32+:32] <= {hwlp_end_data_i[31:2], 2'b00};
	end
	genvar _gv_k_1;
	generate
		for (_gv_k_1 = 0; _gv_k_1 < N_REGS; _gv_k_1 = _gv_k_1 + 1) begin : genblk1
			localparam k = _gv_k_1;
			assign hwlp_counter_n[k * 32+:32] = hwlp_counter_q[k * 32+:32] - 1;
		end
	endgenerate
	always @(posedge clk or negedge rst_n) begin : HWLOOP_REGS_COUNTER
		if (rst_n == 1'b0)
			hwlp_counter_q <= {N_REGS {32'b00000000000000000000000000000000}};
		else
			for (i = 0; i < N_REGS; i = i + 1)
				if ((hwlp_we_i[2] == 1'b1) && (i == hwlp_regid_i))
					hwlp_counter_q[i * 32+:32] <= hwlp_cnt_data_i;
				else if (hwlp_dec_cnt_i[i] && valid_i)
					hwlp_counter_q[i * 32+:32] <= hwlp_counter_n[i * 32+:32];
	end
endmodule
module cv32e40p_mult (
	clk,
	rst_n,
	enable_i,
	operator_i,
	short_subword_i,
	short_signed_i,
	op_a_i,
	op_b_i,
	op_c_i,
	imm_i,
	dot_signed_i,
	dot_op_a_i,
	dot_op_b_i,
	dot_op_c_i,
	is_clpx_i,
	clpx_shift_i,
	clpx_img_i,
	result_o,
	multicycle_o,
	mulh_active_o,
	ready_o,
	ex_ready_i
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire enable_i;
	localparam cv32e40p_pkg_MUL_OP_WIDTH = 3;
	input wire [2:0] operator_i;
	input wire short_subword_i;
	input wire [1:0] short_signed_i;
	input wire [31:0] op_a_i;
	input wire [31:0] op_b_i;
	input wire [31:0] op_c_i;
	input wire [4:0] imm_i;
	input wire [1:0] dot_signed_i;
	input wire [31:0] dot_op_a_i;
	input wire [31:0] dot_op_b_i;
	input wire [31:0] dot_op_c_i;
	input wire is_clpx_i;
	input wire [1:0] clpx_shift_i;
	input wire clpx_img_i;
	output reg [31:0] result_o;
	output reg multicycle_o;
	output reg mulh_active_o;
	output wire ready_o;
	input wire ex_ready_i;
	wire [16:0] short_op_a;
	wire [16:0] short_op_b;
	wire [32:0] short_op_c;
	wire [33:0] short_mul;
	wire [33:0] short_mac;
	wire [31:0] short_round;
	wire [31:0] short_round_tmp;
	wire [33:0] short_result;
	wire short_mac_msb1;
	wire short_mac_msb0;
	wire [4:0] short_imm;
	wire [1:0] short_subword;
	wire [1:0] short_signed;
	wire short_shift_arith;
	reg [4:0] mulh_imm;
	reg [1:0] mulh_subword;
	reg [1:0] mulh_signed;
	reg mulh_shift_arith;
	reg mulh_carry_q;
	reg mulh_save;
	reg mulh_clearcarry;
	reg mulh_ready;
	reg [2:0] mulh_CS;
	reg [2:0] mulh_NS;
	assign short_round_tmp = 32'h00000001 << imm_i;
	function automatic [2:0] sv2v_cast_9D1C7;
		input reg [2:0] inp;
		sv2v_cast_9D1C7 = inp;
	endfunction
	assign short_round = (operator_i == sv2v_cast_9D1C7(3'b011) ? {1'b0, short_round_tmp[31:1]} : {32 {1'sb0}});
	assign short_op_a[15:0] = (short_subword[0] ? op_a_i[31:16] : op_a_i[15:0]);
	assign short_op_b[15:0] = (short_subword[1] ? op_b_i[31:16] : op_b_i[15:0]);
	assign short_op_a[16] = short_signed[0] & short_op_a[15];
	assign short_op_b[16] = short_signed[1] & short_op_b[15];
	assign short_op_c = (mulh_active_o ? $signed({mulh_carry_q, op_c_i}) : $signed(op_c_i));
	assign short_mul = $signed(short_op_a) * $signed(short_op_b);
	assign short_mac = ($signed(short_op_c) + $signed(short_mul)) + $signed(short_round);
	assign short_result = $signed({short_shift_arith & short_mac_msb1, short_shift_arith & short_mac_msb0, short_mac[31:0]}) >>> short_imm;
	assign short_imm = (mulh_active_o ? mulh_imm : imm_i);
	assign short_subword = (mulh_active_o ? mulh_subword : {2 {short_subword_i}});
	assign short_signed = (mulh_active_o ? mulh_signed : short_signed_i);
	assign short_shift_arith = (mulh_active_o ? mulh_shift_arith : short_signed_i[0]);
	assign short_mac_msb1 = (mulh_active_o ? short_mac[33] : short_mac[31]);
	assign short_mac_msb0 = (mulh_active_o ? short_mac[32] : short_mac[31]);
	always @(*) begin
		if (_sv2v_0)
			;
		mulh_NS = mulh_CS;
		mulh_imm = 5'd0;
		mulh_subword = 2'b00;
		mulh_signed = 2'b00;
		mulh_shift_arith = 1'b0;
		mulh_ready = 1'b0;
		mulh_active_o = 1'b1;
		mulh_save = 1'b0;
		mulh_clearcarry = 1'b0;
		multicycle_o = 1'b0;
		case (mulh_CS)
			3'd0: begin
				mulh_active_o = 1'b0;
				mulh_ready = 1'b1;
				mulh_save = 1'b0;
				if ((operator_i == sv2v_cast_9D1C7(3'b110)) && enable_i) begin
					mulh_ready = 1'b0;
					mulh_NS = 3'd1;
				end
			end
			3'd1: begin
				multicycle_o = 1'b1;
				mulh_imm = 5'd16;
				mulh_active_o = 1'b1;
				mulh_save = 1'b0;
				mulh_NS = 3'd2;
			end
			3'd2: begin
				multicycle_o = 1'b1;
				mulh_signed = {short_signed_i[1], 1'b0};
				mulh_subword = 2'b10;
				mulh_save = 1'b1;
				mulh_shift_arith = 1'b1;
				mulh_NS = 3'd3;
			end
			3'd3: begin
				multicycle_o = 1'b1;
				mulh_signed = {1'b0, short_signed_i[0]};
				mulh_subword = 2'b01;
				mulh_imm = 5'd16;
				mulh_save = 1'b1;
				mulh_clearcarry = 1'b1;
				mulh_shift_arith = 1'b1;
				mulh_NS = 3'd4;
			end
			3'd4: begin
				mulh_signed = short_signed_i;
				mulh_subword = 2'b11;
				mulh_ready = 1'b1;
				if (ex_ready_i)
					mulh_NS = 3'd0;
			end
		endcase
	end
	always @(posedge clk or negedge rst_n)
		if (~rst_n) begin
			mulh_CS <= 3'd0;
			mulh_carry_q <= 1'b0;
		end
		else begin
			mulh_CS <= mulh_NS;
			if (mulh_save)
				mulh_carry_q <= ~mulh_clearcarry & short_mac[32];
			else if (ex_ready_i)
				mulh_carry_q <= 1'b0;
		end
	wire [31:0] int_op_a_msu;
	wire [31:0] int_op_b_msu;
	wire [31:0] int_result;
	wire int_is_msu;
	assign int_is_msu = operator_i == sv2v_cast_9D1C7(3'b001);
	assign int_op_a_msu = op_a_i ^ {32 {int_is_msu}};
	assign int_op_b_msu = op_b_i & {32 {int_is_msu}};
	assign int_result = ($signed(op_c_i) + $signed(int_op_b_msu)) + ($signed(int_op_a_msu) * $signed(op_b_i));
	wire [31:0] dot_char_result;
	wire [32:0] dot_short_result;
	wire [31:0] accumulator;
	wire [15:0] clpx_shift_result;
	wire [35:0] dot_char_op_a;
	wire [35:0] dot_char_op_b;
	wire [71:0] dot_char_mul;
	wire [33:0] dot_short_op_a;
	wire [33:0] dot_short_op_b;
	wire [67:0] dot_short_mul;
	wire [16:0] dot_short_op_a_1_neg;
	wire [31:0] dot_short_op_b_ext;
	assign dot_char_op_a[0+:9] = {dot_signed_i[1] & dot_op_a_i[7], dot_op_a_i[7:0]};
	assign dot_char_op_a[9+:9] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15:8]};
	assign dot_char_op_a[18+:9] = {dot_signed_i[1] & dot_op_a_i[23], dot_op_a_i[23:16]};
	assign dot_char_op_a[27+:9] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:24]};
	assign dot_char_op_b[0+:9] = {dot_signed_i[0] & dot_op_b_i[7], dot_op_b_i[7:0]};
	assign dot_char_op_b[9+:9] = {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:8]};
	assign dot_char_op_b[18+:9] = {dot_signed_i[0] & dot_op_b_i[23], dot_op_b_i[23:16]};
	assign dot_char_op_b[27+:9] = {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:24]};
	assign dot_char_mul[0+:18] = $signed(dot_char_op_a[0+:9]) * $signed(dot_char_op_b[0+:9]);
	assign dot_char_mul[18+:18] = $signed(dot_char_op_a[9+:9]) * $signed(dot_char_op_b[9+:9]);
	assign dot_char_mul[36+:18] = $signed(dot_char_op_a[18+:9]) * $signed(dot_char_op_b[18+:9]);
	assign dot_char_mul[54+:18] = $signed(dot_char_op_a[27+:9]) * $signed(dot_char_op_b[27+:9]);
	assign dot_char_result = ((($signed(dot_char_mul[0+:18]) + $signed(dot_char_mul[18+:18])) + $signed(dot_char_mul[36+:18])) + $signed(dot_char_mul[54+:18])) + $signed(dot_op_c_i);
	assign dot_short_op_a[0+:17] = {dot_signed_i[1] & dot_op_a_i[15], dot_op_a_i[15:0]};
	assign dot_short_op_a[17+:17] = {dot_signed_i[1] & dot_op_a_i[31], dot_op_a_i[31:16]};
	assign dot_short_op_a_1_neg = dot_short_op_a[17+:17] ^ {17 {is_clpx_i & ~clpx_img_i}};
	assign dot_short_op_b[0+:17] = (is_clpx_i & clpx_img_i ? {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:16]} : {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:0]});
	assign dot_short_op_b[17+:17] = (is_clpx_i & clpx_img_i ? {dot_signed_i[0] & dot_op_b_i[15], dot_op_b_i[15:0]} : {dot_signed_i[0] & dot_op_b_i[31], dot_op_b_i[31:16]});
	assign dot_short_mul[0+:34] = $signed(dot_short_op_a[0+:17]) * $signed(dot_short_op_b[0+:17]);
	assign dot_short_mul[34+:34] = $signed(dot_short_op_a_1_neg) * $signed(dot_short_op_b[17+:17]);
	assign dot_short_op_b_ext = $signed(dot_short_op_b[17+:17]);
	assign accumulator = (is_clpx_i ? dot_short_op_b_ext & {32 {~clpx_img_i}} : $signed(dot_op_c_i));
	assign dot_short_result = ($signed(dot_short_mul[31-:32]) + $signed(dot_short_mul[65-:32])) + $signed(accumulator);
	assign clpx_shift_result = $signed(dot_short_result[31:15]) >>> clpx_shift_i;
	always @(*) begin
		if (_sv2v_0)
			;
		result_o = 1'sb0;
		case (operator_i)
			sv2v_cast_9D1C7(3'b000), sv2v_cast_9D1C7(3'b001): result_o = int_result[31:0];
			sv2v_cast_9D1C7(3'b010), sv2v_cast_9D1C7(3'b011), sv2v_cast_9D1C7(3'b110): result_o = short_result[31:0];
			sv2v_cast_9D1C7(3'b100): result_o = dot_char_result[31:0];
			sv2v_cast_9D1C7(3'b101):
				if (is_clpx_i) begin
					if (clpx_img_i) begin
						result_o[31:16] = clpx_shift_result;
						result_o[15:0] = dot_op_c_i[15:0];
					end
					else begin
						result_o[15:0] = clpx_shift_result;
						result_o[31:16] = dot_op_c_i[31:16];
					end
				end
				else
					result_o = dot_short_result[31:0];
			default:
				;
		endcase
	end
	assign ready_o = mulh_ready;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_int_controller (
	clk,
	rst_n,
	irq_i,
	irq_sec_i,
	irq_req_ctrl_o,
	irq_sec_ctrl_o,
	irq_id_ctrl_o,
	irq_wu_ctrl_o,
	mie_bypass_i,
	mip_o,
	m_ie_i,
	u_ie_i,
	current_priv_lvl_i
);
	reg _sv2v_0;
	parameter PULP_SECURE = 0;
	input wire clk;
	input wire rst_n;
	input wire [31:0] irq_i;
	input wire irq_sec_i;
	output wire irq_req_ctrl_o;
	output wire irq_sec_ctrl_o;
	output reg [4:0] irq_id_ctrl_o;
	output wire irq_wu_ctrl_o;
	input wire [31:0] mie_bypass_i;
	output wire [31:0] mip_o;
	input wire m_ie_i;
	input wire u_ie_i;
	input wire [1:0] current_priv_lvl_i;
	wire global_irq_enable;
	wire [31:0] irq_local_qual;
	reg [31:0] irq_q;
	reg irq_sec_q;
	localparam cv32e40p_pkg_IRQ_MASK = 32'hffff0888;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			irq_q <= 1'sb0;
			irq_sec_q <= 1'b0;
		end
		else begin
			irq_q <= irq_i & cv32e40p_pkg_IRQ_MASK;
			irq_sec_q <= irq_sec_i;
		end
	assign mip_o = irq_q;
	assign irq_local_qual = irq_q & mie_bypass_i;
	assign irq_wu_ctrl_o = |(irq_i & mie_bypass_i);
	generate
		if (PULP_SECURE) begin : gen_pulp_secure
			assign global_irq_enable = ((u_ie_i || irq_sec_i) && (current_priv_lvl_i == 2'b00)) || (m_ie_i && (current_priv_lvl_i == 2'b11));
		end
		else begin : gen_no_pulp_secure
			assign global_irq_enable = m_ie_i;
		end
	endgenerate
	assign irq_req_ctrl_o = |irq_local_qual && global_irq_enable;
	localparam [31:0] cv32e40p_pkg_CSR_MEIX_BIT = 11;
	localparam [31:0] cv32e40p_pkg_CSR_MSIX_BIT = 3;
	localparam [31:0] cv32e40p_pkg_CSR_MTIX_BIT = 7;
	always @(*) begin
		if (_sv2v_0)
			;
		if (irq_local_qual[31])
			irq_id_ctrl_o = 5'd31;
		else if (irq_local_qual[30])
			irq_id_ctrl_o = 5'd30;
		else if (irq_local_qual[29])
			irq_id_ctrl_o = 5'd29;
		else if (irq_local_qual[28])
			irq_id_ctrl_o = 5'd28;
		else if (irq_local_qual[27])
			irq_id_ctrl_o = 5'd27;
		else if (irq_local_qual[26])
			irq_id_ctrl_o = 5'd26;
		else if (irq_local_qual[25])
			irq_id_ctrl_o = 5'd25;
		else if (irq_local_qual[24])
			irq_id_ctrl_o = 5'd24;
		else if (irq_local_qual[23])
			irq_id_ctrl_o = 5'd23;
		else if (irq_local_qual[22])
			irq_id_ctrl_o = 5'd22;
		else if (irq_local_qual[21])
			irq_id_ctrl_o = 5'd21;
		else if (irq_local_qual[20])
			irq_id_ctrl_o = 5'd20;
		else if (irq_local_qual[19])
			irq_id_ctrl_o = 5'd19;
		else if (irq_local_qual[18])
			irq_id_ctrl_o = 5'd18;
		else if (irq_local_qual[17])
			irq_id_ctrl_o = 5'd17;
		else if (irq_local_qual[16])
			irq_id_ctrl_o = 5'd16;
		else if (irq_local_qual[15])
			irq_id_ctrl_o = 5'd15;
		else if (irq_local_qual[14])
			irq_id_ctrl_o = 5'd14;
		else if (irq_local_qual[13])
			irq_id_ctrl_o = 5'd13;
		else if (irq_local_qual[12])
			irq_id_ctrl_o = 5'd12;
		else if (irq_local_qual[cv32e40p_pkg_CSR_MEIX_BIT])
			irq_id_ctrl_o = cv32e40p_pkg_CSR_MEIX_BIT;
		else if (irq_local_qual[cv32e40p_pkg_CSR_MSIX_BIT])
			irq_id_ctrl_o = cv32e40p_pkg_CSR_MSIX_BIT;
		else if (irq_local_qual[cv32e40p_pkg_CSR_MTIX_BIT])
			irq_id_ctrl_o = cv32e40p_pkg_CSR_MTIX_BIT;
		else if (irq_local_qual[10])
			irq_id_ctrl_o = 5'd10;
		else if (irq_local_qual[2])
			irq_id_ctrl_o = 5'd2;
		else if (irq_local_qual[6])
			irq_id_ctrl_o = 5'd6;
		else if (irq_local_qual[9])
			irq_id_ctrl_o = 5'd9;
		else if (irq_local_qual[1])
			irq_id_ctrl_o = 5'd1;
		else if (irq_local_qual[5])
			irq_id_ctrl_o = 5'd5;
		else if (irq_local_qual[8])
			irq_id_ctrl_o = 5'd8;
		else if (irq_local_qual[0])
			irq_id_ctrl_o = 5'd0;
		else if (irq_local_qual[4])
			irq_id_ctrl_o = 5'd4;
		else
			irq_id_ctrl_o = cv32e40p_pkg_CSR_MTIX_BIT;
	end
	assign irq_sec_ctrl_o = irq_sec_q;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_ex_stage (
	clk,
	rst_n,
	alu_operator_i,
	alu_operand_a_i,
	alu_operand_b_i,
	alu_operand_c_i,
	alu_en_i,
	bmask_a_i,
	bmask_b_i,
	imm_vec_ext_i,
	alu_vec_mode_i,
	alu_is_clpx_i,
	alu_is_subrot_i,
	alu_clpx_shift_i,
	mult_operator_i,
	mult_operand_a_i,
	mult_operand_b_i,
	mult_operand_c_i,
	mult_en_i,
	mult_sel_subword_i,
	mult_signed_mode_i,
	mult_imm_i,
	mult_dot_op_a_i,
	mult_dot_op_b_i,
	mult_dot_op_c_i,
	mult_dot_signed_i,
	mult_is_clpx_i,
	mult_clpx_shift_i,
	mult_clpx_img_i,
	mult_multicycle_o,
	data_req_i,
	data_rvalid_i,
	data_misaligned_ex_i,
	data_misaligned_i,
	ctrl_transfer_insn_in_dec_i,
	fpu_fflags_we_o,
	fpu_fflags_o,
	apu_en_i,
	apu_op_i,
	apu_lat_i,
	apu_operands_i,
	apu_waddr_i,
	apu_flags_i,
	apu_read_regs_i,
	apu_read_regs_valid_i,
	apu_read_dep_o,
	apu_read_dep_for_jalr_o,
	apu_write_regs_i,
	apu_write_regs_valid_i,
	apu_write_dep_o,
	apu_perf_type_o,
	apu_perf_cont_o,
	apu_perf_wb_o,
	apu_busy_o,
	apu_ready_wb_o,
	apu_req_o,
	apu_gnt_i,
	apu_operands_o,
	apu_op_o,
	apu_rvalid_i,
	apu_result_i,
	lsu_en_i,
	lsu_rdata_i,
	branch_in_ex_i,
	regfile_alu_waddr_i,
	regfile_alu_we_i,
	regfile_we_i,
	regfile_waddr_i,
	csr_access_i,
	csr_rdata_i,
	regfile_waddr_wb_o,
	regfile_we_wb_o,
	regfile_we_wb_power_o,
	regfile_wdata_wb_o,
	regfile_alu_waddr_fw_o,
	regfile_alu_we_fw_o,
	regfile_alu_we_fw_power_o,
	regfile_alu_wdata_fw_o,
	jump_target_o,
	branch_decision_o,
	is_decoding_i,
	lsu_ready_ex_i,
	lsu_err_i,
	ex_ready_o,
	ex_valid_o,
	wb_ready_i
);
	reg _sv2v_0;
	parameter COREV_PULP = 0;
	parameter FPU = 0;
	parameter APU_NARGS_CPU = 3;
	parameter APU_WOP_CPU = 6;
	parameter APU_NDSFLAGS_CPU = 15;
	parameter APU_NUSFLAGS_CPU = 5;
	input wire clk;
	input wire rst_n;
	localparam cv32e40p_pkg_ALU_OP_WIDTH = 7;
	input wire [6:0] alu_operator_i;
	input wire [31:0] alu_operand_a_i;
	input wire [31:0] alu_operand_b_i;
	input wire [31:0] alu_operand_c_i;
	input wire alu_en_i;
	input wire [4:0] bmask_a_i;
	input wire [4:0] bmask_b_i;
	input wire [1:0] imm_vec_ext_i;
	input wire [1:0] alu_vec_mode_i;
	input wire alu_is_clpx_i;
	input wire alu_is_subrot_i;
	input wire [1:0] alu_clpx_shift_i;
	localparam cv32e40p_pkg_MUL_OP_WIDTH = 3;
	input wire [2:0] mult_operator_i;
	input wire [31:0] mult_operand_a_i;
	input wire [31:0] mult_operand_b_i;
	input wire [31:0] mult_operand_c_i;
	input wire mult_en_i;
	input wire mult_sel_subword_i;
	input wire [1:0] mult_signed_mode_i;
	input wire [4:0] mult_imm_i;
	input wire [31:0] mult_dot_op_a_i;
	input wire [31:0] mult_dot_op_b_i;
	input wire [31:0] mult_dot_op_c_i;
	input wire [1:0] mult_dot_signed_i;
	input wire mult_is_clpx_i;
	input wire [1:0] mult_clpx_shift_i;
	input wire mult_clpx_img_i;
	output wire mult_multicycle_o;
	input wire data_req_i;
	input wire data_rvalid_i;
	input wire data_misaligned_ex_i;
	input wire data_misaligned_i;
	input wire [1:0] ctrl_transfer_insn_in_dec_i;
	output wire fpu_fflags_we_o;
	output wire [APU_NUSFLAGS_CPU - 1:0] fpu_fflags_o;
	input wire apu_en_i;
	input wire [APU_WOP_CPU - 1:0] apu_op_i;
	input wire [1:0] apu_lat_i;
	input wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands_i;
	input wire [5:0] apu_waddr_i;
	input wire [APU_NUSFLAGS_CPU - 1:0] apu_flags_i;
	input wire [17:0] apu_read_regs_i;
	input wire [2:0] apu_read_regs_valid_i;
	output wire apu_read_dep_o;
	output wire apu_read_dep_for_jalr_o;
	input wire [11:0] apu_write_regs_i;
	input wire [1:0] apu_write_regs_valid_i;
	output wire apu_write_dep_o;
	output wire apu_perf_type_o;
	output wire apu_perf_cont_o;
	output wire apu_perf_wb_o;
	output wire apu_busy_o;
	output wire apu_ready_wb_o;
	output wire apu_req_o;
	input wire apu_gnt_i;
	output wire [(APU_NARGS_CPU * 32) - 1:0] apu_operands_o;
	output wire [APU_WOP_CPU - 1:0] apu_op_o;
	input wire apu_rvalid_i;
	input wire [31:0] apu_result_i;
	input wire lsu_en_i;
	input wire [31:0] lsu_rdata_i;
	input wire branch_in_ex_i;
	input wire [5:0] regfile_alu_waddr_i;
	input wire regfile_alu_we_i;
	input wire regfile_we_i;
	input wire [5:0] regfile_waddr_i;
	input wire csr_access_i;
	input wire [31:0] csr_rdata_i;
	output reg [5:0] regfile_waddr_wb_o;
	output reg regfile_we_wb_o;
	output reg regfile_we_wb_power_o;
	output reg [31:0] regfile_wdata_wb_o;
	output reg [5:0] regfile_alu_waddr_fw_o;
	output reg regfile_alu_we_fw_o;
	output reg regfile_alu_we_fw_power_o;
	output reg [31:0] regfile_alu_wdata_fw_o;
	output wire [31:0] jump_target_o;
	output wire branch_decision_o;
	input wire is_decoding_i;
	input wire lsu_ready_ex_i;
	input wire lsu_err_i;
	output wire ex_ready_o;
	output wire ex_valid_o;
	input wire wb_ready_i;
	wire [31:0] alu_result;
	wire [31:0] mult_result;
	wire alu_cmp_result;
	reg regfile_we_lsu;
	reg [5:0] regfile_waddr_lsu;
	reg wb_contention;
	reg wb_contention_lsu;
	wire alu_ready;
	wire mulh_active;
	wire mult_ready;
	wire apu_valid;
	wire [5:0] apu_waddr;
	wire [31:0] apu_result;
	wire apu_stall;
	wire apu_active;
	wire apu_singlecycle;
	wire apu_multicycle;
	wire apu_req;
	wire apu_gnt;
	reg apu_rvalid_q;
	reg [31:0] apu_result_q;
	reg [APU_NUSFLAGS_CPU - 1:0] apu_flags_q;
	always @(*) begin
		if (_sv2v_0)
			;
		regfile_alu_wdata_fw_o = 1'sb0;
		regfile_alu_waddr_fw_o = 1'sb0;
		regfile_alu_we_fw_o = 1'b0;
		regfile_alu_we_fw_power_o = 1'b0;
		wb_contention = 1'b0;
		if (apu_valid & (apu_singlecycle | apu_multicycle)) begin
			regfile_alu_we_fw_o = 1'b1;
			regfile_alu_we_fw_power_o = 1'b1;
			regfile_alu_waddr_fw_o = apu_waddr;
			regfile_alu_wdata_fw_o = apu_result;
			if (regfile_alu_we_i & ~apu_en_i)
				wb_contention = 1'b1;
		end
		else begin
			regfile_alu_we_fw_o = regfile_alu_we_i & ~apu_en_i;
			regfile_alu_we_fw_power_o = (!COREV_PULP ? regfile_alu_we_i & ~apu_en_i : (((regfile_alu_we_i & ~apu_en_i) & mult_ready) & alu_ready) & lsu_ready_ex_i);
			regfile_alu_waddr_fw_o = regfile_alu_waddr_i;
			if (alu_en_i)
				regfile_alu_wdata_fw_o = alu_result;
			if (mult_en_i)
				regfile_alu_wdata_fw_o = mult_result;
			if (csr_access_i)
				regfile_alu_wdata_fw_o = csr_rdata_i;
		end
	end
	always @(*) begin
		if (_sv2v_0)
			;
		regfile_we_wb_o = 1'b0;
		regfile_we_wb_power_o = 1'b0;
		regfile_waddr_wb_o = regfile_waddr_lsu;
		regfile_wdata_wb_o = lsu_rdata_i;
		wb_contention_lsu = 1'b0;
		if (regfile_we_lsu) begin
			regfile_we_wb_o = 1'b1;
			regfile_we_wb_power_o = (!COREV_PULP ? 1'b1 : ~data_misaligned_ex_i & wb_ready_i);
			if (apu_valid & (!apu_singlecycle & !apu_multicycle))
				wb_contention_lsu = 1'b1;
		end
		else if (apu_valid & (!apu_singlecycle & !apu_multicycle)) begin
			regfile_we_wb_o = 1'b1;
			regfile_we_wb_power_o = 1'b1;
			regfile_waddr_wb_o = apu_waddr;
			regfile_wdata_wb_o = apu_result;
		end
	end
	assign branch_decision_o = alu_cmp_result;
	assign jump_target_o = alu_operand_c_i;
	cv32e40p_alu alu_i(
		.clk(clk),
		.rst_n(rst_n),
		.enable_i(alu_en_i),
		.operator_i(alu_operator_i),
		.operand_a_i(alu_operand_a_i),
		.operand_b_i(alu_operand_b_i),
		.operand_c_i(alu_operand_c_i),
		.vector_mode_i(alu_vec_mode_i),
		.bmask_a_i(bmask_a_i),
		.bmask_b_i(bmask_b_i),
		.imm_vec_ext_i(imm_vec_ext_i),
		.is_clpx_i(alu_is_clpx_i),
		.clpx_shift_i(alu_clpx_shift_i),
		.is_subrot_i(alu_is_subrot_i),
		.result_o(alu_result),
		.comparison_result_o(alu_cmp_result),
		.ready_o(alu_ready),
		.ex_ready_i(ex_ready_o)
	);
	cv32e40p_mult mult_i(
		.clk(clk),
		.rst_n(rst_n),
		.enable_i(mult_en_i),
		.operator_i(mult_operator_i),
		.short_subword_i(mult_sel_subword_i),
		.short_signed_i(mult_signed_mode_i),
		.op_a_i(mult_operand_a_i),
		.op_b_i(mult_operand_b_i),
		.op_c_i(mult_operand_c_i),
		.imm_i(mult_imm_i),
		.dot_op_a_i(mult_dot_op_a_i),
		.dot_op_b_i(mult_dot_op_b_i),
		.dot_op_c_i(mult_dot_op_c_i),
		.dot_signed_i(mult_dot_signed_i),
		.is_clpx_i(mult_is_clpx_i),
		.clpx_shift_i(mult_clpx_shift_i),
		.clpx_img_i(mult_clpx_img_i),
		.result_o(mult_result),
		.multicycle_o(mult_multicycle_o),
		.mulh_active_o(mulh_active),
		.ready_o(mult_ready),
		.ex_ready_i(ex_ready_o)
	);
	localparam cv32e40p_pkg_BRANCH_JALR = 2'b10;
	function automatic [2:0] sv2v_cast_9D1C7;
		input reg [2:0] inp;
		sv2v_cast_9D1C7 = inp;
	endfunction
	generate
		if (FPU == 1) begin : gen_apu
			cv32e40p_apu_disp apu_disp_i(
				.clk_i(clk),
				.rst_ni(rst_n),
				.enable_i(apu_en_i),
				.apu_lat_i(apu_lat_i),
				.apu_waddr_i(apu_waddr_i),
				.apu_waddr_o(apu_waddr),
				.apu_multicycle_o(apu_multicycle),
				.apu_singlecycle_o(apu_singlecycle),
				.active_o(apu_active),
				.stall_o(apu_stall),
				.is_decoding_i(is_decoding_i),
				.read_regs_i(apu_read_regs_i),
				.read_regs_valid_i(apu_read_regs_valid_i),
				.read_dep_o(apu_read_dep_o),
				.read_dep_for_jalr_o(apu_read_dep_for_jalr_o),
				.write_regs_i(apu_write_regs_i),
				.write_regs_valid_i(apu_write_regs_valid_i),
				.write_dep_o(apu_write_dep_o),
				.perf_type_o(apu_perf_type_o),
				.perf_cont_o(apu_perf_cont_o),
				.apu_req_o(apu_req),
				.apu_gnt_i(apu_gnt),
				.apu_rvalid_i(apu_valid)
			);
			assign apu_perf_wb_o = wb_contention | wb_contention_lsu;
			assign apu_ready_wb_o = ~((apu_active | apu_en_i) | apu_stall) | apu_valid;
			always @(posedge clk or negedge rst_n) begin : APU_Result_Memorization
				if (~rst_n) begin
					apu_rvalid_q <= 1'b0;
					apu_result_q <= 'b0;
					apu_flags_q <= 'b0;
				end
				else if ((apu_rvalid_i && apu_multicycle) && ((((data_misaligned_i || data_misaligned_ex_i) || ((data_req_i || data_rvalid_i) && regfile_alu_we_i)) || (mulh_active && (mult_operator_i == sv2v_cast_9D1C7(3'b110)))) || (((ctrl_transfer_insn_in_dec_i == cv32e40p_pkg_BRANCH_JALR) && regfile_alu_we_i) && ~apu_read_dep_for_jalr_o))) begin
					apu_rvalid_q <= 1'b1;
					apu_result_q <= apu_result_i;
					apu_flags_q <= apu_flags_i;
				end
				else if (apu_rvalid_q && !((((data_misaligned_i || data_misaligned_ex_i) || ((data_req_i || data_rvalid_i) && regfile_alu_we_i)) || (mulh_active && (mult_operator_i == sv2v_cast_9D1C7(3'b110)))) || (((ctrl_transfer_insn_in_dec_i == cv32e40p_pkg_BRANCH_JALR) && regfile_alu_we_i) && ~apu_read_dep_for_jalr_o)))
					apu_rvalid_q <= 1'b0;
			end
			assign apu_req_o = apu_req;
			assign apu_gnt = apu_gnt_i;
			assign apu_valid = (apu_multicycle && ((((data_misaligned_i || data_misaligned_ex_i) || ((data_req_i || data_rvalid_i) && regfile_alu_we_i)) || (mulh_active && (mult_operator_i == sv2v_cast_9D1C7(3'b110)))) || (((ctrl_transfer_insn_in_dec_i == cv32e40p_pkg_BRANCH_JALR) && regfile_alu_we_i) && ~apu_read_dep_for_jalr_o)) ? 1'b0 : apu_rvalid_i || apu_rvalid_q);
			assign apu_operands_o = apu_operands_i;
			assign apu_op_o = apu_op_i;
			assign apu_result = (apu_rvalid_q ? apu_result_q : apu_result_i);
			assign fpu_fflags_we_o = apu_valid;
			assign fpu_fflags_o = (apu_rvalid_q ? apu_flags_q : apu_flags_i);
		end
		else begin : gen_no_apu
			assign apu_req_o = 1'sb0;
			assign apu_operands_o[0+:32] = 1'sb0;
			assign apu_operands_o[32+:32] = 1'sb0;
			assign apu_operands_o[64+:32] = 1'sb0;
			assign apu_op_o = 1'sb0;
			assign apu_req = 1'b0;
			assign apu_gnt = 1'b0;
			assign apu_result = 32'b00000000000000000000000000000000;
			assign apu_valid = 1'b0;
			assign apu_waddr = 6'b000000;
			assign apu_stall = 1'b0;
			assign apu_active = 1'b0;
			assign apu_ready_wb_o = 1'b1;
			assign apu_perf_wb_o = 1'b0;
			assign apu_perf_cont_o = 1'b0;
			assign apu_perf_type_o = 1'b0;
			assign apu_singlecycle = 1'b0;
			assign apu_multicycle = 1'b0;
			assign apu_read_dep_o = 1'b0;
			assign apu_read_dep_for_jalr_o = 1'b0;
			assign apu_write_dep_o = 1'b0;
			assign fpu_fflags_o = 1'sb0;
			assign fpu_fflags_we_o = 1'sb0;
		end
	endgenerate
	assign apu_busy_o = apu_active;
	always @(posedge clk or negedge rst_n) begin : EX_WB_Pipeline_Register
		if (~rst_n) begin
			regfile_waddr_lsu <= 1'sb0;
			regfile_we_lsu <= 1'b0;
		end
		else if (ex_valid_o) begin
			regfile_we_lsu <= regfile_we_i & ~lsu_err_i;
			if (regfile_we_i & ~lsu_err_i)
				regfile_waddr_lsu <= regfile_waddr_i;
		end
		else if (wb_ready_i)
			regfile_we_lsu <= 1'b0;
	end
	assign ex_ready_o = (((((~apu_stall & alu_ready) & mult_ready) & lsu_ready_ex_i) & wb_ready_i) & ~wb_contention) | branch_in_ex_i;
	assign ex_valid_o = ((((apu_valid | alu_en_i) | mult_en_i) | csr_access_i) | lsu_en_i) & (((alu_ready & mult_ready) & lsu_ready_ex_i) & wb_ready_i);
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_alu_div (
	Clk_CI,
	Rst_RBI,
	OpA_DI,
	OpB_DI,
	OpBShift_DI,
	OpBIsZero_SI,
	OpBSign_SI,
	OpCode_SI,
	InVld_SI,
	OutRdy_SI,
	OutVld_SO,
	Res_DO
);
	reg _sv2v_0;
	parameter C_WIDTH = 32;
	parameter C_LOG_WIDTH = 6;
	input wire Clk_CI;
	input wire Rst_RBI;
	input wire [C_WIDTH - 1:0] OpA_DI;
	input wire [C_WIDTH - 1:0] OpB_DI;
	input wire [C_LOG_WIDTH - 1:0] OpBShift_DI;
	input wire OpBIsZero_SI;
	input wire OpBSign_SI;
	input wire [1:0] OpCode_SI;
	input wire InVld_SI;
	input wire OutRdy_SI;
	output reg OutVld_SO;
	output wire [C_WIDTH - 1:0] Res_DO;
	reg [C_WIDTH - 1:0] ResReg_DP;
	wire [C_WIDTH - 1:0] ResReg_DN;
	wire [C_WIDTH - 1:0] ResReg_DP_rev;
	reg [C_WIDTH - 1:0] AReg_DP;
	wire [C_WIDTH - 1:0] AReg_DN;
	reg [C_WIDTH - 1:0] BReg_DP;
	wire [C_WIDTH - 1:0] BReg_DN;
	wire RemSel_SN;
	reg RemSel_SP;
	wire CompInv_SN;
	reg CompInv_SP;
	wire ResInv_SN;
	reg ResInv_SP;
	wire [C_WIDTH - 1:0] AddMux_D;
	wire [C_WIDTH - 1:0] AddOut_D;
	wire [C_WIDTH - 1:0] AddTmp_D;
	wire [C_WIDTH - 1:0] BMux_D;
	wire [C_WIDTH - 1:0] OutMux_D;
	reg [C_LOG_WIDTH - 1:0] Cnt_DP;
	wire [C_LOG_WIDTH - 1:0] Cnt_DN;
	wire CntZero_S;
	reg ARegEn_S;
	reg BRegEn_S;
	reg ResRegEn_S;
	wire ABComp_S;
	wire PmSel_S;
	reg LoadEn_S;
	reg [1:0] State_SN;
	reg [1:0] State_SP;
	assign PmSel_S = LoadEn_S & ~(OpCode_SI[0] & (OpA_DI[C_WIDTH - 1] ^ OpBSign_SI));
	assign AddMux_D = (LoadEn_S ? OpA_DI : BReg_DP);
	assign BMux_D = (LoadEn_S ? OpB_DI : {CompInv_SP, BReg_DP[C_WIDTH - 1:1]});
	genvar _gv_index_1;
	generate
		for (_gv_index_1 = 0; _gv_index_1 < C_WIDTH; _gv_index_1 = _gv_index_1 + 1) begin : gen_bit_swapping
			localparam index = _gv_index_1;
			assign ResReg_DP_rev[index] = ResReg_DP[(C_WIDTH - 1) - index];
		end
	endgenerate
	assign OutMux_D = (RemSel_SP ? AReg_DP : ResReg_DP_rev);
	assign Res_DO = (ResInv_SP ? -$signed(OutMux_D) : OutMux_D);
	assign ABComp_S = ((AReg_DP == BReg_DP) | ((AReg_DP > BReg_DP) ^ CompInv_SP)) & (|AReg_DP | OpBIsZero_SI);
	assign AddTmp_D = (LoadEn_S ? 0 : AReg_DP);
	assign AddOut_D = (PmSel_S ? AddTmp_D + AddMux_D : AddTmp_D - $signed(AddMux_D));
	assign Cnt_DN = (LoadEn_S ? OpBShift_DI : (~CntZero_S ? Cnt_DP - 1 : Cnt_DP));
	assign CntZero_S = ~(|Cnt_DP);
	always @(*) begin : p_fsm
		if (_sv2v_0)
			;
		State_SN = State_SP;
		OutVld_SO = 1'b0;
		LoadEn_S = 1'b0;
		ARegEn_S = 1'b0;
		BRegEn_S = 1'b0;
		ResRegEn_S = 1'b0;
		case (State_SP)
			2'd0: begin
				OutVld_SO = 1'b1;
				if (InVld_SI) begin
					OutVld_SO = 1'b0;
					ARegEn_S = 1'b1;
					BRegEn_S = 1'b1;
					LoadEn_S = 1'b1;
					State_SN = 2'd1;
				end
			end
			2'd1: begin
				ARegEn_S = ABComp_S;
				BRegEn_S = 1'b1;
				ResRegEn_S = 1'b1;
				if (CntZero_S)
					State_SN = 2'd2;
			end
			2'd2: begin
				OutVld_SO = 1'b1;
				if (OutRdy_SI)
					State_SN = 2'd0;
			end
			default:
				;
		endcase
	end
	assign RemSel_SN = (LoadEn_S ? OpCode_SI[1] : RemSel_SP);
	assign CompInv_SN = (LoadEn_S ? OpBSign_SI : CompInv_SP);
	assign ResInv_SN = (LoadEn_S ? ((~OpBIsZero_SI | OpCode_SI[1]) & OpCode_SI[0]) & (OpA_DI[C_WIDTH - 1] ^ OpBSign_SI) : ResInv_SP);
	assign AReg_DN = (ARegEn_S ? AddOut_D : AReg_DP);
	assign BReg_DN = (BRegEn_S ? BMux_D : BReg_DP);
	assign ResReg_DN = (LoadEn_S ? {C_WIDTH {1'sb0}} : (ResRegEn_S ? {ABComp_S, ResReg_DP[C_WIDTH - 1:1]} : ResReg_DP));
	always @(posedge Clk_CI or negedge Rst_RBI) begin : p_regs
		if (~Rst_RBI) begin
			State_SP <= 2'd0;
			AReg_DP <= 1'sb0;
			BReg_DP <= 1'sb0;
			ResReg_DP <= 1'sb0;
			Cnt_DP <= 1'sb0;
			RemSel_SP <= 1'b0;
			CompInv_SP <= 1'b0;
			ResInv_SP <= 1'b0;
		end
		else begin
			State_SP <= State_SN;
			AReg_DP <= AReg_DN;
			BReg_DP <= BReg_DN;
			ResReg_DP <= ResReg_DN;
			Cnt_DP <= Cnt_DN;
			RemSel_SP <= RemSel_SN;
			CompInv_SP <= CompInv_SN;
			ResInv_SP <= ResInv_SN;
		end
	end
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_alu (
	clk,
	rst_n,
	enable_i,
	operator_i,
	operand_a_i,
	operand_b_i,
	operand_c_i,
	vector_mode_i,
	bmask_a_i,
	bmask_b_i,
	imm_vec_ext_i,
	is_clpx_i,
	is_subrot_i,
	clpx_shift_i,
	result_o,
	comparison_result_o,
	ready_o,
	ex_ready_i
);
	reg _sv2v_0;
	input wire clk;
	input wire rst_n;
	input wire enable_i;
	localparam cv32e40p_pkg_ALU_OP_WIDTH = 7;
	input wire [6:0] operator_i;
	input wire [31:0] operand_a_i;
	input wire [31:0] operand_b_i;
	input wire [31:0] operand_c_i;
	input wire [1:0] vector_mode_i;
	input wire [4:0] bmask_a_i;
	input wire [4:0] bmask_b_i;
	input wire [1:0] imm_vec_ext_i;
	input wire is_clpx_i;
	input wire is_subrot_i;
	input wire [1:0] clpx_shift_i;
	output reg [31:0] result_o;
	output wire comparison_result_o;
	output wire ready_o;
	input wire ex_ready_i;
	wire [31:0] operand_a_rev;
	wire [31:0] operand_a_neg;
	wire [31:0] operand_a_neg_rev;
	assign operand_a_neg = ~operand_a_i;
	genvar _gv_k_2;
	generate
		for (_gv_k_2 = 0; _gv_k_2 < 32; _gv_k_2 = _gv_k_2 + 1) begin : gen_operand_a_rev
			localparam k = _gv_k_2;
			assign operand_a_rev[k] = operand_a_i[31 - k];
		end
	endgenerate
	genvar _gv_m_1;
	generate
		for (_gv_m_1 = 0; _gv_m_1 < 32; _gv_m_1 = _gv_m_1 + 1) begin : gen_operand_a_neg_rev
			localparam m = _gv_m_1;
			assign operand_a_neg_rev[m] = operand_a_neg[31 - m];
		end
	endgenerate
	wire [31:0] operand_b_neg;
	assign operand_b_neg = ~operand_b_i;
	wire [5:0] div_shift;
	wire div_valid;
	wire [31:0] bmask;
	wire adder_op_b_negate;
	wire [31:0] adder_op_a;
	wire [31:0] adder_op_b;
	reg [35:0] adder_in_a;
	reg [35:0] adder_in_b;
	wire [31:0] adder_result;
	wire [36:0] adder_result_expanded;
	function automatic [6:0] sv2v_cast_576C1;
		input reg [6:0] inp;
		sv2v_cast_576C1 = inp;
	endfunction
	assign adder_op_b_negate = ((((operator_i == sv2v_cast_576C1(7'b0011001)) || (operator_i == sv2v_cast_576C1(7'b0011101))) || (operator_i == sv2v_cast_576C1(7'b0011011))) || (operator_i == sv2v_cast_576C1(7'b0011111))) || is_subrot_i;
	assign adder_op_a = (operator_i == sv2v_cast_576C1(7'b0010100) ? operand_a_neg : (is_subrot_i ? {operand_b_i[15:0], operand_a_i[31:16]} : operand_a_i));
	assign adder_op_b = (adder_op_b_negate ? (is_subrot_i ? ~{operand_a_i[15:0], operand_b_i[31:16]} : operand_b_neg) : operand_b_i);
	localparam cv32e40p_pkg_VEC_MODE16 = 2'b10;
	localparam cv32e40p_pkg_VEC_MODE8 = 2'b11;
	always @(*) begin
		if (_sv2v_0)
			;
		adder_in_a[0] = 1'b1;
		adder_in_a[8:1] = adder_op_a[7:0];
		adder_in_a[9] = 1'b1;
		adder_in_a[17:10] = adder_op_a[15:8];
		adder_in_a[18] = 1'b1;
		adder_in_a[26:19] = adder_op_a[23:16];
		adder_in_a[27] = 1'b1;
		adder_in_a[35:28] = adder_op_a[31:24];
		adder_in_b[0] = 1'b0;
		adder_in_b[8:1] = adder_op_b[7:0];
		adder_in_b[9] = 1'b0;
		adder_in_b[17:10] = adder_op_b[15:8];
		adder_in_b[18] = 1'b0;
		adder_in_b[26:19] = adder_op_b[23:16];
		adder_in_b[27] = 1'b0;
		adder_in_b[35:28] = adder_op_b[31:24];
		if (adder_op_b_negate || ((operator_i == sv2v_cast_576C1(7'b0010100)) || (operator_i == sv2v_cast_576C1(7'b0010110)))) begin
			adder_in_b[0] = 1'b1;
			case (vector_mode_i)
				cv32e40p_pkg_VEC_MODE16: adder_in_b[18] = 1'b1;
				cv32e40p_pkg_VEC_MODE8: begin
					adder_in_b[9] = 1'b1;
					adder_in_b[18] = 1'b1;
					adder_in_b[27] = 1'b1;
				end
			endcase
		end
		else
			case (vector_mode_i)
				cv32e40p_pkg_VEC_MODE16: adder_in_a[18] = 1'b0;
				cv32e40p_pkg_VEC_MODE8: begin
					adder_in_a[9] = 1'b0;
					adder_in_a[18] = 1'b0;
					adder_in_a[27] = 1'b0;
				end
			endcase
	end
	assign adder_result_expanded = $signed(adder_in_a) + $signed(adder_in_b);
	assign adder_result = {adder_result_expanded[35:28], adder_result_expanded[26:19], adder_result_expanded[17:10], adder_result_expanded[8:1]};
	wire [31:0] adder_round_value;
	wire [31:0] adder_round_result;
	assign adder_round_value = ((((operator_i == sv2v_cast_576C1(7'b0011100)) || (operator_i == sv2v_cast_576C1(7'b0011101))) || (operator_i == sv2v_cast_576C1(7'b0011110))) || (operator_i == sv2v_cast_576C1(7'b0011111)) ? {1'b0, bmask[31:1]} : {32 {1'sb0}});
	assign adder_round_result = adder_result + adder_round_value;
	wire shift_left;
	wire shift_use_round;
	wire shift_arithmetic;
	reg [31:0] shift_amt_left;
	wire [31:0] shift_amt;
	wire [31:0] shift_amt_int;
	wire [31:0] shift_amt_norm;
	wire [31:0] shift_op_a;
	wire [31:0] shift_result;
	reg [31:0] shift_right_result;
	wire [31:0] shift_left_result;
	wire [15:0] clpx_shift_ex;
	assign shift_amt = (div_valid ? div_shift : operand_b_i);
	always @(*) begin
		if (_sv2v_0)
			;
		case (vector_mode_i)
			cv32e40p_pkg_VEC_MODE16: begin
				shift_amt_left[15:0] = shift_amt[31:16];
				shift_amt_left[31:16] = shift_amt[15:0];
			end
			cv32e40p_pkg_VEC_MODE8: begin
				shift_amt_left[7:0] = shift_amt[31:24];
				shift_amt_left[15:8] = shift_amt[23:16];
				shift_amt_left[23:16] = shift_amt[15:8];
				shift_amt_left[31:24] = shift_amt[7:0];
			end
			default: shift_amt_left[31:0] = shift_amt[31:0];
		endcase
	end
	assign shift_left = ((((((((operator_i == sv2v_cast_576C1(7'b0100111)) || (operator_i == sv2v_cast_576C1(7'b0101010))) || (operator_i == sv2v_cast_576C1(7'b0110111))) || (operator_i == sv2v_cast_576C1(7'b0110101))) || (operator_i == sv2v_cast_576C1(7'b0110001))) || (operator_i == sv2v_cast_576C1(7'b0110000))) || (operator_i == sv2v_cast_576C1(7'b0110011))) || (operator_i == sv2v_cast_576C1(7'b0110010))) || (operator_i == sv2v_cast_576C1(7'b1001001));
	assign shift_use_round = (((((((operator_i == sv2v_cast_576C1(7'b0011000)) || (operator_i == sv2v_cast_576C1(7'b0011001))) || (operator_i == sv2v_cast_576C1(7'b0011100))) || (operator_i == sv2v_cast_576C1(7'b0011101))) || (operator_i == sv2v_cast_576C1(7'b0011010))) || (operator_i == sv2v_cast_576C1(7'b0011011))) || (operator_i == sv2v_cast_576C1(7'b0011110))) || (operator_i == sv2v_cast_576C1(7'b0011111));
	assign shift_arithmetic = (((((operator_i == sv2v_cast_576C1(7'b0100100)) || (operator_i == sv2v_cast_576C1(7'b0101000))) || (operator_i == sv2v_cast_576C1(7'b0011000))) || (operator_i == sv2v_cast_576C1(7'b0011001))) || (operator_i == sv2v_cast_576C1(7'b0011100))) || (operator_i == sv2v_cast_576C1(7'b0011101));
	assign shift_op_a = (shift_left ? operand_a_rev : (shift_use_round ? adder_round_result : operand_a_i));
	assign shift_amt_int = (shift_use_round ? shift_amt_norm : (shift_left ? shift_amt_left : shift_amt));
	assign shift_amt_norm = (is_clpx_i ? {clpx_shift_ex, clpx_shift_ex} : {4 {3'b000, bmask_b_i}});
	assign clpx_shift_ex = $unsigned(clpx_shift_i);
	wire [63:0] shift_op_a_32;
	assign shift_op_a_32 = (operator_i == sv2v_cast_576C1(7'b0100110) ? {shift_op_a, shift_op_a} : $signed({{32 {shift_arithmetic & shift_op_a[31]}}, shift_op_a}));
	always @(*) begin
		if (_sv2v_0)
			;
		case (vector_mode_i)
			cv32e40p_pkg_VEC_MODE16: begin
				shift_right_result[31:16] = $signed({shift_arithmetic & shift_op_a[31], shift_op_a[31:16]}) >>> shift_amt_int[19:16];
				shift_right_result[15:0] = $signed({shift_arithmetic & shift_op_a[15], shift_op_a[15:0]}) >>> shift_amt_int[3:0];
			end
			cv32e40p_pkg_VEC_MODE8: begin
				shift_right_result[31:24] = $signed({shift_arithmetic & shift_op_a[31], shift_op_a[31:24]}) >>> shift_amt_int[26:24];
				shift_right_result[23:16] = $signed({shift_arithmetic & shift_op_a[23], shift_op_a[23:16]}) >>> shift_amt_int[18:16];
				shift_right_result[15:8] = $signed({shift_arithmetic & shift_op_a[15], shift_op_a[15:8]}) >>> shift_amt_int[10:8];
				shift_right_result[7:0] = $signed({shift_arithmetic & shift_op_a[7], shift_op_a[7:0]}) >>> shift_amt_int[2:0];
			end
			default: shift_right_result = shift_op_a_32 >> shift_amt_int[4:0];
		endcase
	end
	genvar _gv_j_2;
	generate
		for (_gv_j_2 = 0; _gv_j_2 < 32; _gv_j_2 = _gv_j_2 + 1) begin : gen_shift_left_result
			localparam j = _gv_j_2;
			assign shift_left_result[j] = shift_right_result[31 - j];
		end
	endgenerate
	assign shift_result = (shift_left ? shift_left_result : shift_right_result);
	reg [3:0] is_equal;
	reg [3:0] is_greater;
	reg [3:0] cmp_signed;
	wire [3:0] is_equal_vec;
	wire [3:0] is_greater_vec;
	reg [31:0] operand_b_eq;
	wire is_equal_clip;
	always @(*) begin
		if (_sv2v_0)
			;
		operand_b_eq = operand_b_neg;
		if (operator_i == sv2v_cast_576C1(7'b0010111))
			operand_b_eq = 1'sb0;
		else
			operand_b_eq = operand_b_neg;
	end
	assign is_equal_clip = operand_a_i == operand_b_eq;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_signed = 4'b0000;
		case (operator_i)
			sv2v_cast_576C1(7'b0001000), sv2v_cast_576C1(7'b0001010), sv2v_cast_576C1(7'b0000000), sv2v_cast_576C1(7'b0000100), sv2v_cast_576C1(7'b0000010), sv2v_cast_576C1(7'b0000110), sv2v_cast_576C1(7'b0010000), sv2v_cast_576C1(7'b0010010), sv2v_cast_576C1(7'b0010100), sv2v_cast_576C1(7'b0010110), sv2v_cast_576C1(7'b0010111):
				case (vector_mode_i)
					cv32e40p_pkg_VEC_MODE8: cmp_signed[3:0] = 4'b1111;
					cv32e40p_pkg_VEC_MODE16: cmp_signed[3:0] = 4'b1010;
					default: cmp_signed[3:0] = 4'b1000;
				endcase
			default:
				;
		endcase
	end
	genvar _gv_i_3;
	generate
		for (_gv_i_3 = 0; _gv_i_3 < 4; _gv_i_3 = _gv_i_3 + 1) begin : gen_is_vec
			localparam i = _gv_i_3;
			assign is_equal_vec[i] = operand_a_i[(8 * i) + 7:8 * i] == operand_b_i[(8 * i) + 7:i * 8];
			assign is_greater_vec[i] = $signed({operand_a_i[(8 * i) + 7] & cmp_signed[i], operand_a_i[(8 * i) + 7:8 * i]}) > $signed({operand_b_i[(8 * i) + 7] & cmp_signed[i], operand_b_i[(8 * i) + 7:i * 8]});
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		is_equal[3:0] = {4 {((is_equal_vec[3] & is_equal_vec[2]) & is_equal_vec[1]) & is_equal_vec[0]}};
		is_greater[3:0] = {4 {is_greater_vec[3] | (is_equal_vec[3] & (is_greater_vec[2] | (is_equal_vec[2] & (is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])))))}};
		case (vector_mode_i)
			cv32e40p_pkg_VEC_MODE16: begin
				is_equal[1:0] = {2 {is_equal_vec[0] & is_equal_vec[1]}};
				is_equal[3:2] = {2 {is_equal_vec[2] & is_equal_vec[3]}};
				is_greater[1:0] = {2 {is_greater_vec[1] | (is_equal_vec[1] & is_greater_vec[0])}};
				is_greater[3:2] = {2 {is_greater_vec[3] | (is_equal_vec[3] & is_greater_vec[2])}};
			end
			cv32e40p_pkg_VEC_MODE8: begin
				is_equal[3:0] = is_equal_vec[3:0];
				is_greater[3:0] = is_greater_vec[3:0];
			end
			default:
				;
		endcase
	end
	reg [3:0] cmp_result;
	always @(*) begin
		if (_sv2v_0)
			;
		cmp_result = is_equal;
		case (operator_i)
			sv2v_cast_576C1(7'b0001100): cmp_result = is_equal;
			sv2v_cast_576C1(7'b0001101): cmp_result = ~is_equal;
			sv2v_cast_576C1(7'b0001000), sv2v_cast_576C1(7'b0001001): cmp_result = is_greater;
			sv2v_cast_576C1(7'b0001010), sv2v_cast_576C1(7'b0001011): cmp_result = is_greater | is_equal;
			sv2v_cast_576C1(7'b0000000), sv2v_cast_576C1(7'b0000010), sv2v_cast_576C1(7'b0000001), sv2v_cast_576C1(7'b0000011): cmp_result = ~(is_greater | is_equal);
			sv2v_cast_576C1(7'b0000110), sv2v_cast_576C1(7'b0000111), sv2v_cast_576C1(7'b0000100), sv2v_cast_576C1(7'b0000101): cmp_result = ~is_greater;
			default:
				;
		endcase
	end
	assign comparison_result_o = cmp_result[3];
	wire [31:0] result_minmax;
	wire [3:0] sel_minmax;
	wire do_min;
	wire [31:0] minmax_b;
	assign minmax_b = (operator_i == sv2v_cast_576C1(7'b0010100) ? adder_result : operand_b_i);
	assign do_min = (((operator_i == sv2v_cast_576C1(7'b0010000)) || (operator_i == sv2v_cast_576C1(7'b0010001))) || (operator_i == sv2v_cast_576C1(7'b0010110))) || (operator_i == sv2v_cast_576C1(7'b0010111));
	assign sel_minmax[3:0] = is_greater ^ {4 {do_min}};
	assign result_minmax[31:24] = (sel_minmax[3] == 1'b1 ? operand_a_i[31:24] : minmax_b[31:24]);
	assign result_minmax[23:16] = (sel_minmax[2] == 1'b1 ? operand_a_i[23:16] : minmax_b[23:16]);
	assign result_minmax[15:8] = (sel_minmax[1] == 1'b1 ? operand_a_i[15:8] : minmax_b[15:8]);
	assign result_minmax[7:0] = (sel_minmax[0] == 1'b1 ? operand_a_i[7:0] : minmax_b[7:0]);
	reg [31:0] clip_result;
	always @(*) begin
		if (_sv2v_0)
			;
		clip_result = result_minmax;
		if (operator_i == sv2v_cast_576C1(7'b0010111)) begin
			if (operand_a_i[31] || is_equal_clip)
				clip_result = 1'sb0;
			else
				clip_result = result_minmax;
		end
		else if (adder_result_expanded[36] || is_equal_clip)
			clip_result = operand_b_neg;
		else
			clip_result = result_minmax;
	end
	reg [7:0] shuffle_byte_sel;
	reg [3:0] shuffle_reg_sel;
	reg [1:0] shuffle_reg1_sel;
	reg [1:0] shuffle_reg0_sel;
	reg [3:0] shuffle_through;
	wire [31:0] shuffle_r1;
	wire [31:0] shuffle_r0;
	wire [31:0] shuffle_r1_in;
	wire [31:0] shuffle_r0_in;
	wire [31:0] shuffle_result;
	wire [31:0] pack_result;
	always @(*) begin
		if (_sv2v_0)
			;
		shuffle_reg_sel = 1'sb0;
		shuffle_reg1_sel = 2'b01;
		shuffle_reg0_sel = 2'b10;
		shuffle_through = 1'sb1;
		case (operator_i)
			sv2v_cast_576C1(7'b0111111), sv2v_cast_576C1(7'b0111110): begin
				if (operator_i == sv2v_cast_576C1(7'b0111110))
					shuffle_reg1_sel = 2'b11;
				if (vector_mode_i == cv32e40p_pkg_VEC_MODE8) begin
					shuffle_reg_sel[3:1] = 3'b111;
					shuffle_reg_sel[0] = 1'b0;
				end
				else begin
					shuffle_reg_sel[3:2] = 2'b11;
					shuffle_reg_sel[1:0] = 2'b00;
				end
			end
			sv2v_cast_576C1(7'b0111000): begin
				shuffle_reg1_sel = 2'b00;
				if (vector_mode_i == cv32e40p_pkg_VEC_MODE8) begin
					shuffle_through = 4'b0011;
					shuffle_reg_sel = 4'b0001;
				end
				else
					shuffle_reg_sel = 4'b0011;
			end
			sv2v_cast_576C1(7'b0111001): begin
				shuffle_reg1_sel = 2'b00;
				if (vector_mode_i == cv32e40p_pkg_VEC_MODE8) begin
					shuffle_through = 4'b1100;
					shuffle_reg_sel = 4'b0100;
				end
				else
					shuffle_reg_sel = 4'b0011;
			end
			sv2v_cast_576C1(7'b0111011):
				case (vector_mode_i)
					cv32e40p_pkg_VEC_MODE8: begin
						shuffle_reg_sel[3] = ~operand_b_i[26];
						shuffle_reg_sel[2] = ~operand_b_i[18];
						shuffle_reg_sel[1] = ~operand_b_i[10];
						shuffle_reg_sel[0] = ~operand_b_i[2];
					end
					cv32e40p_pkg_VEC_MODE16: begin
						shuffle_reg_sel[3] = ~operand_b_i[17];
						shuffle_reg_sel[2] = ~operand_b_i[17];
						shuffle_reg_sel[1] = ~operand_b_i[1];
						shuffle_reg_sel[0] = ~operand_b_i[1];
					end
					default:
						;
				endcase
			sv2v_cast_576C1(7'b0101101):
				case (vector_mode_i)
					cv32e40p_pkg_VEC_MODE8: begin
						shuffle_reg0_sel = 2'b00;
						case (imm_vec_ext_i)
							2'b00: shuffle_reg_sel[3:0] = 4'b1110;
							2'b01: shuffle_reg_sel[3:0] = 4'b1101;
							2'b10: shuffle_reg_sel[3:0] = 4'b1011;
							2'b11: shuffle_reg_sel[3:0] = 4'b0111;
						endcase
					end
					cv32e40p_pkg_VEC_MODE16: begin
						shuffle_reg0_sel = 2'b01;
						shuffle_reg_sel[3] = ~imm_vec_ext_i[0];
						shuffle_reg_sel[2] = ~imm_vec_ext_i[0];
						shuffle_reg_sel[1] = imm_vec_ext_i[0];
						shuffle_reg_sel[0] = imm_vec_ext_i[0];
					end
					default:
						;
				endcase
			default:
				;
		endcase
	end
	always @(*) begin
		if (_sv2v_0)
			;
		shuffle_byte_sel = 1'sb0;
		case (operator_i)
			sv2v_cast_576C1(7'b0111110), sv2v_cast_576C1(7'b0111111):
				case (vector_mode_i)
					cv32e40p_pkg_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[4+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[2+:2] = imm_vec_ext_i[1:0];
						shuffle_byte_sel[0+:2] = imm_vec_ext_i[1:0];
					end
					cv32e40p_pkg_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[4+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[2+:2] = {imm_vec_ext_i[0], 1'b1};
						shuffle_byte_sel[0+:2] = {imm_vec_ext_i[0], 1'b0};
					end
					default:
						;
				endcase
			sv2v_cast_576C1(7'b0111000):
				case (vector_mode_i)
					cv32e40p_pkg_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = 2'b00;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b00;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					cv32e40p_pkg_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = 2'b01;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b01;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					default:
						;
				endcase
			sv2v_cast_576C1(7'b0111001):
				case (vector_mode_i)
					cv32e40p_pkg_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = 2'b00;
						shuffle_byte_sel[4+:2] = 2'b00;
						shuffle_byte_sel[2+:2] = 2'b00;
						shuffle_byte_sel[0+:2] = 2'b00;
					end
					cv32e40p_pkg_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = 2'b11;
						shuffle_byte_sel[4+:2] = 2'b10;
						shuffle_byte_sel[2+:2] = 2'b11;
						shuffle_byte_sel[0+:2] = 2'b10;
					end
					default:
						;
				endcase
			sv2v_cast_576C1(7'b0111011), sv2v_cast_576C1(7'b0111010):
				case (vector_mode_i)
					cv32e40p_pkg_VEC_MODE8: begin
						shuffle_byte_sel[6+:2] = operand_b_i[25:24];
						shuffle_byte_sel[4+:2] = operand_b_i[17:16];
						shuffle_byte_sel[2+:2] = operand_b_i[9:8];
						shuffle_byte_sel[0+:2] = operand_b_i[1:0];
					end
					cv32e40p_pkg_VEC_MODE16: begin
						shuffle_byte_sel[6+:2] = {operand_b_i[16], 1'b1};
						shuffle_byte_sel[4+:2] = {operand_b_i[16], 1'b0};
						shuffle_byte_sel[2+:2] = {operand_b_i[0], 1'b1};
						shuffle_byte_sel[0+:2] = {operand_b_i[0], 1'b0};
					end
					default:
						;
				endcase
			sv2v_cast_576C1(7'b0101101): begin
				shuffle_byte_sel[6+:2] = 2'b11;
				shuffle_byte_sel[4+:2] = 2'b10;
				shuffle_byte_sel[2+:2] = 2'b01;
				shuffle_byte_sel[0+:2] = 2'b00;
			end
			default:
				;
		endcase
	end
	assign shuffle_r0_in = (shuffle_reg0_sel[1] ? operand_a_i : (shuffle_reg0_sel[0] ? {2 {operand_a_i[15:0]}} : {4 {operand_a_i[7:0]}}));
	assign shuffle_r1_in = (shuffle_reg1_sel[1] ? {{8 {operand_a_i[31]}}, {8 {operand_a_i[23]}}, {8 {operand_a_i[15]}}, {8 {operand_a_i[7]}}} : (shuffle_reg1_sel[0] ? operand_c_i : operand_b_i));
	assign shuffle_r0[31:24] = (shuffle_byte_sel[7] ? (shuffle_byte_sel[6] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[6] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[23:16] = (shuffle_byte_sel[5] ? (shuffle_byte_sel[4] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[4] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[15:8] = (shuffle_byte_sel[3] ? (shuffle_byte_sel[2] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[2] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r0[7:0] = (shuffle_byte_sel[1] ? (shuffle_byte_sel[0] ? shuffle_r0_in[31:24] : shuffle_r0_in[23:16]) : (shuffle_byte_sel[0] ? shuffle_r0_in[15:8] : shuffle_r0_in[7:0]));
	assign shuffle_r1[31:24] = (shuffle_byte_sel[7] ? (shuffle_byte_sel[6] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[6] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[23:16] = (shuffle_byte_sel[5] ? (shuffle_byte_sel[4] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[4] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[15:8] = (shuffle_byte_sel[3] ? (shuffle_byte_sel[2] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[2] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_r1[7:0] = (shuffle_byte_sel[1] ? (shuffle_byte_sel[0] ? shuffle_r1_in[31:24] : shuffle_r1_in[23:16]) : (shuffle_byte_sel[0] ? shuffle_r1_in[15:8] : shuffle_r1_in[7:0]));
	assign shuffle_result[31:24] = (shuffle_reg_sel[3] ? shuffle_r1[31:24] : shuffle_r0[31:24]);
	assign shuffle_result[23:16] = (shuffle_reg_sel[2] ? shuffle_r1[23:16] : shuffle_r0[23:16]);
	assign shuffle_result[15:8] = (shuffle_reg_sel[1] ? shuffle_r1[15:8] : shuffle_r0[15:8]);
	assign shuffle_result[7:0] = (shuffle_reg_sel[0] ? shuffle_r1[7:0] : shuffle_r0[7:0]);
	assign pack_result[31:24] = (shuffle_through[3] ? shuffle_result[31:24] : operand_c_i[31:24]);
	assign pack_result[23:16] = (shuffle_through[2] ? shuffle_result[23:16] : operand_c_i[23:16]);
	assign pack_result[15:8] = (shuffle_through[1] ? shuffle_result[15:8] : operand_c_i[15:8]);
	assign pack_result[7:0] = (shuffle_through[0] ? shuffle_result[7:0] : operand_c_i[7:0]);
	reg [31:0] ff_input;
	wire [5:0] cnt_result;
	wire [5:0] clb_result;
	wire [4:0] ff1_result;
	wire ff_no_one;
	wire [4:0] fl1_result;
	reg [5:0] bitop_result;
	cv32e40p_popcnt popcnt_i(
		.in_i(operand_a_i),
		.result_o(cnt_result)
	);
	always @(*) begin
		if (_sv2v_0)
			;
		ff_input = 1'sb0;
		case (operator_i)
			sv2v_cast_576C1(7'b0110110): ff_input = operand_a_i;
			sv2v_cast_576C1(7'b0110000), sv2v_cast_576C1(7'b0110010), sv2v_cast_576C1(7'b0110111): ff_input = operand_a_rev;
			sv2v_cast_576C1(7'b0110001), sv2v_cast_576C1(7'b0110011), sv2v_cast_576C1(7'b0110101):
				if (operand_a_i[31])
					ff_input = operand_a_neg_rev;
				else
					ff_input = operand_a_rev;
		endcase
	end
	cv32e40p_ff_one ff_one_i(
		.in_i(ff_input),
		.first_one_o(ff1_result),
		.no_ones_o(ff_no_one)
	);
	assign fl1_result = 5'd31 - ff1_result;
	assign clb_result = ff1_result - 5'd1;
	always @(*) begin
		if (_sv2v_0)
			;
		bitop_result = 1'sb0;
		case (operator_i)
			sv2v_cast_576C1(7'b0110110): bitop_result = (ff_no_one ? 6'd32 : {1'b0, ff1_result});
			sv2v_cast_576C1(7'b0110111): bitop_result = (ff_no_one ? 6'd32 : {1'b0, fl1_result});
			sv2v_cast_576C1(7'b0110100): bitop_result = cnt_result;
			sv2v_cast_576C1(7'b0110101):
				if (ff_no_one) begin
					if (operand_a_i[31])
						bitop_result = 6'd31;
					else
						bitop_result = 1'sb0;
				end
				else
					bitop_result = clb_result;
			default:
				;
		endcase
	end
	wire extract_is_signed;
	wire extract_sign;
	wire [31:0] bmask_first;
	wire [31:0] bmask_inv;
	wire [31:0] bextins_and;
	wire [31:0] bextins_result;
	wire [31:0] bclr_result;
	wire [31:0] bset_result;
	assign bmask_first = 32'hfffffffe << bmask_a_i;
	assign bmask = ~bmask_first << bmask_b_i;
	assign bmask_inv = ~bmask;
	assign bextins_and = (operator_i == sv2v_cast_576C1(7'b0101010) ? operand_c_i : {32 {extract_sign}});
	assign extract_is_signed = operator_i == sv2v_cast_576C1(7'b0101000);
	assign extract_sign = extract_is_signed & shift_result[bmask_a_i];
	assign bextins_result = (bmask & shift_result) | (bextins_and & bmask_inv);
	assign bclr_result = operand_a_i & bmask_inv;
	assign bset_result = operand_a_i | bmask;
	wire [31:0] radix_2_rev;
	wire [31:0] radix_4_rev;
	wire [31:0] radix_8_rev;
	reg [31:0] reverse_result;
	wire [1:0] radix_mux_sel;
	assign radix_mux_sel = bmask_a_i[1:0];
	generate
		for (_gv_j_2 = 0; _gv_j_2 < 32; _gv_j_2 = _gv_j_2 + 1) begin : gen_radix_2_rev
			localparam j = _gv_j_2;
			assign radix_2_rev[j] = shift_result[31 - j];
		end
		for (_gv_j_2 = 0; _gv_j_2 < 16; _gv_j_2 = _gv_j_2 + 1) begin : gen_radix_4_rev
			localparam j = _gv_j_2;
			assign radix_4_rev[(2 * j) + 1:2 * j] = shift_result[31 - (j * 2):(31 - (j * 2)) - 1];
		end
		for (_gv_j_2 = 0; _gv_j_2 < 10; _gv_j_2 = _gv_j_2 + 1) begin : gen_radix_8_rev
			localparam j = _gv_j_2;
			assign radix_8_rev[(3 * j) + 2:3 * j] = shift_result[31 - (j * 3):(31 - (j * 3)) - 2];
		end
	endgenerate
	assign radix_8_rev[31:30] = 2'b00;
	always @(*) begin
		if (_sv2v_0)
			;
		reverse_result = 1'sb0;
		case (radix_mux_sel)
			2'b00: reverse_result = radix_2_rev;
			2'b01: reverse_result = radix_4_rev;
			2'b10: reverse_result = radix_8_rev;
			default: reverse_result = radix_2_rev;
		endcase
	end
	wire [31:0] result_div;
	wire div_ready;
	wire div_signed;
	wire div_op_a_signed;
	wire [5:0] div_shift_int;
	assign div_signed = operator_i[0];
	assign div_op_a_signed = operand_a_i[31] & div_signed;
	assign div_shift_int = (ff_no_one ? 6'd31 : clb_result);
	assign div_shift = div_shift_int + (div_op_a_signed ? 6'd0 : 6'd1);
	assign div_valid = enable_i & ((((operator_i == sv2v_cast_576C1(7'b0110001)) || (operator_i == sv2v_cast_576C1(7'b0110000))) || (operator_i == sv2v_cast_576C1(7'b0110011))) || (operator_i == sv2v_cast_576C1(7'b0110010)));
	cv32e40p_alu_div alu_div_i(
		.Clk_CI(clk),
		.Rst_RBI(rst_n),
		.OpA_DI(operand_b_i),
		.OpB_DI(shift_left_result),
		.OpBShift_DI(div_shift),
		.OpBIsZero_SI(cnt_result == 0),
		.OpBSign_SI(div_op_a_signed),
		.OpCode_SI(operator_i[1:0]),
		.Res_DO(result_div),
		.InVld_SI(div_valid),
		.OutRdy_SI(ex_ready_i),
		.OutVld_SO(div_ready)
	);
	always @(*) begin
		if (_sv2v_0)
			;
		result_o = 1'sb0;
		case (operator_i)
			sv2v_cast_576C1(7'b0010101): result_o = operand_a_i & operand_b_i;
			sv2v_cast_576C1(7'b0101110): result_o = operand_a_i | operand_b_i;
			sv2v_cast_576C1(7'b0101111): result_o = operand_a_i ^ operand_b_i;
			sv2v_cast_576C1(7'b0011000), sv2v_cast_576C1(7'b0011100), sv2v_cast_576C1(7'b0011010), sv2v_cast_576C1(7'b0011110), sv2v_cast_576C1(7'b0011001), sv2v_cast_576C1(7'b0011101), sv2v_cast_576C1(7'b0011011), sv2v_cast_576C1(7'b0011111), sv2v_cast_576C1(7'b0100111), sv2v_cast_576C1(7'b0100101), sv2v_cast_576C1(7'b0100100), sv2v_cast_576C1(7'b0100110): result_o = shift_result;
			sv2v_cast_576C1(7'b0101010), sv2v_cast_576C1(7'b0101000), sv2v_cast_576C1(7'b0101001): result_o = bextins_result;
			sv2v_cast_576C1(7'b0101011): result_o = bclr_result;
			sv2v_cast_576C1(7'b0101100): result_o = bset_result;
			sv2v_cast_576C1(7'b1001001): result_o = reverse_result;
			sv2v_cast_576C1(7'b0111010), sv2v_cast_576C1(7'b0111011), sv2v_cast_576C1(7'b0111000), sv2v_cast_576C1(7'b0111001), sv2v_cast_576C1(7'b0111111), sv2v_cast_576C1(7'b0111110), sv2v_cast_576C1(7'b0101101): result_o = pack_result;
			sv2v_cast_576C1(7'b0010000), sv2v_cast_576C1(7'b0010001), sv2v_cast_576C1(7'b0010010), sv2v_cast_576C1(7'b0010011): result_o = result_minmax;
			sv2v_cast_576C1(7'b0010100): result_o = (is_clpx_i ? {adder_result[31:16], operand_a_i[15:0]} : result_minmax);
			sv2v_cast_576C1(7'b0010110), sv2v_cast_576C1(7'b0010111): result_o = clip_result;
			sv2v_cast_576C1(7'b0001100), sv2v_cast_576C1(7'b0001101), sv2v_cast_576C1(7'b0001001), sv2v_cast_576C1(7'b0001011), sv2v_cast_576C1(7'b0000001), sv2v_cast_576C1(7'b0000101), sv2v_cast_576C1(7'b0001000), sv2v_cast_576C1(7'b0001010), sv2v_cast_576C1(7'b0000000), sv2v_cast_576C1(7'b0000100): begin
				result_o[31:24] = {8 {cmp_result[3]}};
				result_o[23:16] = {8 {cmp_result[2]}};
				result_o[15:8] = {8 {cmp_result[1]}};
				result_o[7:0] = {8 {cmp_result[0]}};
			end
			sv2v_cast_576C1(7'b0000010), sv2v_cast_576C1(7'b0000011), sv2v_cast_576C1(7'b0000110), sv2v_cast_576C1(7'b0000111): result_o = {31'b0000000000000000000000000000000, comparison_result_o};
			sv2v_cast_576C1(7'b0110110), sv2v_cast_576C1(7'b0110111), sv2v_cast_576C1(7'b0110101), sv2v_cast_576C1(7'b0110100): result_o = {26'h0000000, bitop_result[5:0]};
			sv2v_cast_576C1(7'b0110001), sv2v_cast_576C1(7'b0110000), sv2v_cast_576C1(7'b0110011), sv2v_cast_576C1(7'b0110010): result_o = result_div;
			default:
				;
		endcase
	end
	assign ready_o = div_ready;
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_ff_one (
	in_i,
	first_one_o,
	no_ones_o
);
	parameter LEN = 32;
	input wire [LEN - 1:0] in_i;
	output wire [$clog2(LEN) - 1:0] first_one_o;
	output wire no_ones_o;
	localparam NUM_LEVELS = $clog2(LEN);
	wire [(LEN * NUM_LEVELS) - 1:0] index_lut;
	wire [(2 ** NUM_LEVELS) - 1:0] sel_nodes;
	wire [((2 ** NUM_LEVELS) * NUM_LEVELS) - 1:0] index_nodes;
	genvar _gv_j_3;
	generate
		for (_gv_j_3 = 0; _gv_j_3 < LEN; _gv_j_3 = _gv_j_3 + 1) begin : gen_index_lut
			localparam j = _gv_j_3;
			assign index_lut[j * NUM_LEVELS+:NUM_LEVELS] = $unsigned(j);
		end
	endgenerate
	genvar _gv_k_3;
	genvar _gv_l_2;
	genvar _gv_level_1;
	assign sel_nodes[(2 ** NUM_LEVELS) - 1] = 1'b0;
	generate
		for (_gv_level_1 = 0; _gv_level_1 < NUM_LEVELS; _gv_level_1 = _gv_level_1 + 1) begin : gen_tree
			localparam level = _gv_level_1;
			if (level < (NUM_LEVELS - 1)) begin : gen_non_root_level
				for (_gv_l_2 = 0; _gv_l_2 < (2 ** level); _gv_l_2 = _gv_l_2 + 1) begin : gen_node
					localparam l = _gv_l_2;
					assign sel_nodes[((2 ** level) - 1) + l] = sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] | sel_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) + 1];
					assign index_nodes[(((2 ** level) - 1) + l) * NUM_LEVELS+:NUM_LEVELS] = (sel_nodes[((2 ** (level + 1)) - 1) + (l * 2)] == 1'b1 ? index_nodes[(((2 ** (level + 1)) - 1) + (l * 2)) * NUM_LEVELS+:NUM_LEVELS] : index_nodes[((((2 ** (level + 1)) - 1) + (l * 2)) + 1) * NUM_LEVELS+:NUM_LEVELS]);
				end
			end
			if (level == (NUM_LEVELS - 1)) begin : gen_root_level
				for (_gv_k_3 = 0; _gv_k_3 < (2 ** level); _gv_k_3 = _gv_k_3 + 1) begin : gen_node
					localparam k = _gv_k_3;
					if ((k * 2) < (LEN - 1)) begin : gen_two
						assign sel_nodes[((2 ** level) - 1) + k] = in_i[k * 2] | in_i[(k * 2) + 1];
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = (in_i[k * 2] == 1'b1 ? index_lut[(k * 2) * NUM_LEVELS+:NUM_LEVELS] : index_lut[((k * 2) + 1) * NUM_LEVELS+:NUM_LEVELS]);
					end
					if ((k * 2) == (LEN - 1)) begin : gen_one
						assign sel_nodes[((2 ** level) - 1) + k] = in_i[k * 2];
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = index_lut[(k * 2) * NUM_LEVELS+:NUM_LEVELS];
					end
					if ((k * 2) > (LEN - 1)) begin : gen_out_of_range
						assign sel_nodes[((2 ** level) - 1) + k] = 1'b0;
						assign index_nodes[(((2 ** level) - 1) + k) * NUM_LEVELS+:NUM_LEVELS] = 1'sb0;
					end
				end
			end
		end
	endgenerate
	assign first_one_o = index_nodes[0+:NUM_LEVELS];
	assign no_ones_o = ~sel_nodes[0];
endmodule
module cv32e40p_popcnt (
	in_i,
	result_o
);
	input wire [31:0] in_i;
	output wire [5:0] result_o;
	wire [31:0] cnt_l1;
	wire [23:0] cnt_l2;
	wire [15:0] cnt_l3;
	wire [9:0] cnt_l4;
	genvar _gv_l_3;
	genvar _gv_m_2;
	genvar _gv_n_1;
	genvar _gv_p_1;
	generate
		for (_gv_l_3 = 0; _gv_l_3 < 16; _gv_l_3 = _gv_l_3 + 1) begin : gen_cnt_l1
			localparam l = _gv_l_3;
			assign cnt_l1[l * 2+:2] = {1'b0, in_i[2 * l]} + {1'b0, in_i[(2 * l) + 1]};
		end
		for (_gv_m_2 = 0; _gv_m_2 < 8; _gv_m_2 = _gv_m_2 + 1) begin : gen_cnt_l2
			localparam m = _gv_m_2;
			assign cnt_l2[m * 3+:3] = {1'b0, cnt_l1[(2 * m) * 2+:2]} + {1'b0, cnt_l1[((2 * m) + 1) * 2+:2]};
		end
		for (_gv_n_1 = 0; _gv_n_1 < 4; _gv_n_1 = _gv_n_1 + 1) begin : gen_cnt_l3
			localparam n = _gv_n_1;
			assign cnt_l3[n * 4+:4] = {1'b0, cnt_l2[(2 * n) * 3+:3]} + {1'b0, cnt_l2[((2 * n) + 1) * 3+:3]};
		end
		for (_gv_p_1 = 0; _gv_p_1 < 2; _gv_p_1 = _gv_p_1 + 1) begin : gen_cnt_l4
			localparam p = _gv_p_1;
			assign cnt_l4[p * 5+:5] = {1'b0, cnt_l3[(2 * p) * 4+:4]} + {1'b0, cnt_l3[((2 * p) + 1) * 4+:4]};
		end
	endgenerate
	assign result_o = {1'b0, cnt_l4[0+:5]} + {1'b0, cnt_l4[5+:5]};
endmodule
module cv32e40p_apu_disp (
	clk_i,
	rst_ni,
	enable_i,
	apu_lat_i,
	apu_waddr_i,
	apu_waddr_o,
	apu_multicycle_o,
	apu_singlecycle_o,
	active_o,
	stall_o,
	is_decoding_i,
	read_regs_i,
	read_regs_valid_i,
	read_dep_o,
	read_dep_for_jalr_o,
	write_regs_i,
	write_regs_valid_i,
	write_dep_o,
	perf_type_o,
	perf_cont_o,
	apu_req_o,
	apu_gnt_i,
	apu_rvalid_i
);
	reg _sv2v_0;
	input wire clk_i;
	input wire rst_ni;
	input wire enable_i;
	input wire [1:0] apu_lat_i;
	input wire [5:0] apu_waddr_i;
	output reg [5:0] apu_waddr_o;
	output wire apu_multicycle_o;
	output wire apu_singlecycle_o;
	output wire active_o;
	output wire stall_o;
	input wire is_decoding_i;
	input wire [17:0] read_regs_i;
	input wire [2:0] read_regs_valid_i;
	output wire read_dep_o;
	output wire read_dep_for_jalr_o;
	input wire [11:0] write_regs_i;
	input wire [1:0] write_regs_valid_i;
	output wire write_dep_o;
	output wire perf_type_o;
	output wire perf_cont_o;
	output wire apu_req_o;
	input wire apu_gnt_i;
	input wire apu_rvalid_i;
	wire [5:0] addr_req;
	reg [5:0] addr_inflight;
	reg [5:0] addr_waiting;
	reg [5:0] addr_inflight_dn;
	reg [5:0] addr_waiting_dn;
	wire valid_req;
	reg valid_inflight;
	reg valid_waiting;
	reg valid_inflight_dn;
	reg valid_waiting_dn;
	wire returned_req;
	wire returned_inflight;
	wire returned_waiting;
	wire req_accepted;
	wire active;
	reg [1:0] apu_lat;
	wire [2:0] read_deps_req;
	wire [2:0] read_deps_inflight;
	wire [2:0] read_deps_waiting;
	wire [1:0] write_deps_req;
	wire [1:0] write_deps_inflight;
	wire [1:0] write_deps_waiting;
	wire read_dep_req;
	wire read_dep_inflight;
	wire read_dep_waiting;
	wire write_dep_req;
	wire write_dep_inflight;
	wire write_dep_waiting;
	wire stall_full;
	wire stall_type;
	wire stall_nack;
	assign valid_req = enable_i & !(stall_full | stall_type);
	assign addr_req = apu_waddr_i;
	assign req_accepted = valid_req & apu_gnt_i;
	assign returned_req = ((valid_req & apu_rvalid_i) & !valid_inflight) & !valid_waiting;
	assign returned_inflight = (valid_inflight & apu_rvalid_i) & !valid_waiting;
	assign returned_waiting = valid_waiting & apu_rvalid_i;
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni) begin
			valid_inflight <= 1'b0;
			valid_waiting <= 1'b0;
			addr_inflight <= 1'sb0;
			addr_waiting <= 1'sb0;
		end
		else begin
			valid_inflight <= valid_inflight_dn;
			valid_waiting <= valid_waiting_dn;
			addr_inflight <= addr_inflight_dn;
			addr_waiting <= addr_waiting_dn;
		end
	always @(*) begin
		if (_sv2v_0)
			;
		valid_inflight_dn = valid_inflight;
		valid_waiting_dn = valid_waiting;
		addr_inflight_dn = addr_inflight;
		addr_waiting_dn = addr_waiting;
		if (req_accepted & !returned_req) begin
			valid_inflight_dn = 1'b1;
			addr_inflight_dn = addr_req;
			if (valid_inflight & !returned_inflight) begin
				valid_waiting_dn = 1'b1;
				addr_waiting_dn = addr_inflight;
			end
			if (returned_waiting) begin
				valid_waiting_dn = 1'b1;
				addr_waiting_dn = addr_inflight;
			end
		end
		else if (returned_inflight) begin
			valid_inflight_dn = 1'sb0;
			valid_waiting_dn = 1'sb0;
			addr_inflight_dn = 1'sb0;
			addr_waiting_dn = 1'sb0;
		end
		else if (returned_waiting) begin
			valid_waiting_dn = 1'sb0;
			addr_waiting_dn = 1'sb0;
		end
	end
	assign active = valid_inflight | valid_waiting;
	always @(posedge clk_i or negedge rst_ni)
		if (~rst_ni)
			apu_lat <= 1'sb0;
		else if (valid_req)
			apu_lat <= apu_lat_i;
	genvar _gv_i_4;
	generate
		for (_gv_i_4 = 0; _gv_i_4 < 3; _gv_i_4 = _gv_i_4 + 1) begin : gen_read_deps
			localparam i = _gv_i_4;
			assign read_deps_req[i] = (read_regs_i[i * 6+:6] == addr_req) & read_regs_valid_i[i];
			assign read_deps_inflight[i] = (read_regs_i[i * 6+:6] == addr_inflight) & read_regs_valid_i[i];
			assign read_deps_waiting[i] = (read_regs_i[i * 6+:6] == addr_waiting) & read_regs_valid_i[i];
		end
	endgenerate
	genvar _gv_i_5;
	generate
		for (_gv_i_5 = 0; _gv_i_5 < 2; _gv_i_5 = _gv_i_5 + 1) begin : gen_write_deps
			localparam i = _gv_i_5;
			assign write_deps_req[i] = (write_regs_i[i * 6+:6] == addr_req) & write_regs_valid_i[i];
			assign write_deps_inflight[i] = (write_regs_i[i * 6+:6] == addr_inflight) & write_regs_valid_i[i];
			assign write_deps_waiting[i] = (write_regs_i[i * 6+:6] == addr_waiting) & write_regs_valid_i[i];
		end
	endgenerate
	assign read_dep_req = (|read_deps_req & valid_req) & !returned_req;
	assign read_dep_inflight = (|read_deps_inflight & valid_inflight) & !returned_inflight;
	assign read_dep_waiting = (|read_deps_waiting & valid_waiting) & !returned_waiting;
	assign write_dep_req = (|write_deps_req & valid_req) & !returned_req;
	assign write_dep_inflight = (|write_deps_inflight & valid_inflight) & !returned_inflight;
	assign write_dep_waiting = (|write_deps_waiting & valid_waiting) & !returned_waiting;
	assign read_dep_o = ((read_dep_req | read_dep_inflight) | read_dep_waiting) & is_decoding_i;
	assign write_dep_o = ((write_dep_req | write_dep_inflight) | write_dep_waiting) & is_decoding_i;
	assign read_dep_for_jalr_o = is_decoding_i & (((|read_deps_req & enable_i) | (|read_deps_inflight & valid_inflight)) | (|read_deps_waiting & valid_waiting));
	assign stall_full = valid_inflight & valid_waiting;
	assign stall_type = (enable_i & active) & (((apu_lat_i == 2'h1) | ((apu_lat_i == 2'h2) & (apu_lat == 2'h3))) | (apu_lat_i == 2'h3));
	assign stall_nack = valid_req & !apu_gnt_i;
	assign stall_o = (stall_full | stall_type) | stall_nack;
	assign apu_req_o = valid_req;
	always @(*) begin
		if (_sv2v_0)
			;
		apu_waddr_o = 1'sb0;
		if (returned_req)
			apu_waddr_o = addr_req;
		if (returned_inflight)
			apu_waddr_o = addr_inflight;
		if (returned_waiting)
			apu_waddr_o = addr_waiting;
	end
	assign active_o = active;
	assign perf_type_o = stall_type;
	assign perf_cont_o = stall_nack;
	assign apu_multicycle_o = apu_lat == 2'h3;
	assign apu_singlecycle_o = ~(valid_inflight | valid_waiting);
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_controller (
	clk,
	clk_ungated_i,
	rst_n,
	fetch_enable_i,
	ctrl_busy_o,
	is_decoding_o,
	is_fetch_failed_i,
	deassert_we_o,
	illegal_insn_i,
	ecall_insn_i,
	mret_insn_i,
	uret_insn_i,
	dret_insn_i,
	mret_dec_i,
	uret_dec_i,
	dret_dec_i,
	wfi_i,
	ebrk_insn_i,
	fencei_insn_i,
	csr_status_i,
	hwlp_mask_o,
	instr_valid_i,
	instr_req_o,
	pc_set_o,
	pc_mux_o,
	exc_pc_mux_o,
	trap_addr_mux_o,
	pc_id_i,
	hwlp_start_addr_i,
	hwlp_end_addr_i,
	hwlp_counter_i,
	hwlp_dec_cnt_o,
	hwlp_jump_o,
	hwlp_targ_addr_o,
	data_req_ex_i,
	data_we_ex_i,
	data_misaligned_i,
	data_load_event_i,
	data_err_i,
	data_err_ack_o,
	mult_multicycle_i,
	apu_en_i,
	apu_read_dep_i,
	apu_read_dep_for_jalr_i,
	apu_write_dep_i,
	apu_stall_o,
	branch_taken_ex_i,
	ctrl_transfer_insn_in_id_i,
	ctrl_transfer_insn_in_dec_i,
	irq_req_ctrl_i,
	irq_sec_ctrl_i,
	irq_id_ctrl_i,
	irq_wu_ctrl_i,
	current_priv_lvl_i,
	irq_ack_o,
	irq_id_o,
	exc_cause_o,
	debug_mode_o,
	debug_cause_o,
	debug_csr_save_o,
	debug_req_i,
	debug_single_step_i,
	debug_ebreakm_i,
	debug_ebreaku_i,
	trigger_match_i,
	debug_p_elw_no_sleep_o,
	debug_wfi_no_sleep_o,
	debug_havereset_o,
	debug_running_o,
	debug_halted_o,
	wake_from_sleep_o,
	csr_save_if_o,
	csr_save_id_o,
	csr_save_ex_o,
	csr_cause_o,
	csr_irq_sec_o,
	csr_restore_mret_id_o,
	csr_restore_uret_id_o,
	csr_restore_dret_id_o,
	csr_save_cause_o,
	regfile_we_id_i,
	regfile_alu_waddr_id_i,
	regfile_we_ex_i,
	regfile_waddr_ex_i,
	regfile_we_wb_i,
	regfile_alu_we_fw_i,
	operand_a_fw_mux_sel_o,
	operand_b_fw_mux_sel_o,
	operand_c_fw_mux_sel_o,
	reg_d_ex_is_reg_a_i,
	reg_d_ex_is_reg_b_i,
	reg_d_ex_is_reg_c_i,
	reg_d_wb_is_reg_a_i,
	reg_d_wb_is_reg_b_i,
	reg_d_wb_is_reg_c_i,
	reg_d_alu_is_reg_a_i,
	reg_d_alu_is_reg_b_i,
	reg_d_alu_is_reg_c_i,
	halt_if_o,
	halt_id_o,
	misaligned_stall_o,
	jr_stall_o,
	load_stall_o,
	id_ready_i,
	id_valid_i,
	ex_valid_i,
	wb_ready_i,
	perf_pipeline_stall_o
);
	reg _sv2v_0;
	parameter COREV_CLUSTER = 0;
	parameter COREV_PULP = 0;
	parameter FPU = 0;
	input wire clk;
	input wire clk_ungated_i;
	input wire rst_n;
	input wire fetch_enable_i;
	output reg ctrl_busy_o;
	output reg is_decoding_o;
	input wire is_fetch_failed_i;
	output reg deassert_we_o;
	input wire illegal_insn_i;
	input wire ecall_insn_i;
	input wire mret_insn_i;
	input wire uret_insn_i;
	input wire dret_insn_i;
	input wire mret_dec_i;
	input wire uret_dec_i;
	input wire dret_dec_i;
	input wire wfi_i;
	input wire ebrk_insn_i;
	input wire fencei_insn_i;
	input wire csr_status_i;
	output reg hwlp_mask_o;
	input wire instr_valid_i;
	output reg instr_req_o;
	output reg pc_set_o;
	output reg [3:0] pc_mux_o;
	output reg [2:0] exc_pc_mux_o;
	output reg [1:0] trap_addr_mux_o;
	input wire [31:0] pc_id_i;
	input wire [63:0] hwlp_start_addr_i;
	input wire [63:0] hwlp_end_addr_i;
	input wire [63:0] hwlp_counter_i;
	output reg [1:0] hwlp_dec_cnt_o;
	output wire hwlp_jump_o;
	output reg [31:0] hwlp_targ_addr_o;
	input wire data_req_ex_i;
	input wire data_we_ex_i;
	input wire data_misaligned_i;
	input wire data_load_event_i;
	input wire data_err_i;
	output reg data_err_ack_o;
	input wire mult_multicycle_i;
	input wire apu_en_i;
	input wire apu_read_dep_i;
	input wire apu_read_dep_for_jalr_i;
	input wire apu_write_dep_i;
	output wire apu_stall_o;
	input wire branch_taken_ex_i;
	input wire [1:0] ctrl_transfer_insn_in_id_i;
	input wire [1:0] ctrl_transfer_insn_in_dec_i;
	input wire irq_req_ctrl_i;
	input wire irq_sec_ctrl_i;
	input wire [4:0] irq_id_ctrl_i;
	input wire irq_wu_ctrl_i;
	input wire [1:0] current_priv_lvl_i;
	output reg irq_ack_o;
	output reg [4:0] irq_id_o;
	output reg [4:0] exc_cause_o;
	output wire debug_mode_o;
	output reg [2:0] debug_cause_o;
	output reg debug_csr_save_o;
	input wire debug_req_i;
	input wire debug_single_step_i;
	input wire debug_ebreakm_i;
	input wire debug_ebreaku_i;
	input wire trigger_match_i;
	output wire debug_p_elw_no_sleep_o;
	output wire debug_wfi_no_sleep_o;
	output wire debug_havereset_o;
	output wire debug_running_o;
	output wire debug_halted_o;
	output wire wake_from_sleep_o;
	output reg csr_save_if_o;
	output reg csr_save_id_o;
	output reg csr_save_ex_o;
	output reg [5:0] csr_cause_o;
	output reg csr_irq_sec_o;
	output reg csr_restore_mret_id_o;
	output reg csr_restore_uret_id_o;
	output reg csr_restore_dret_id_o;
	output reg csr_save_cause_o;
	input wire regfile_we_id_i;
	input wire [5:0] regfile_alu_waddr_id_i;
	input wire regfile_we_ex_i;
	input wire [5:0] regfile_waddr_ex_i;
	input wire regfile_we_wb_i;
	input wire regfile_alu_we_fw_i;
	output reg [1:0] operand_a_fw_mux_sel_o;
	output reg [1:0] operand_b_fw_mux_sel_o;
	output reg [1:0] operand_c_fw_mux_sel_o;
	input wire reg_d_ex_is_reg_a_i;
	input wire reg_d_ex_is_reg_b_i;
	input wire reg_d_ex_is_reg_c_i;
	input wire reg_d_wb_is_reg_a_i;
	input wire reg_d_wb_is_reg_b_i;
	input wire reg_d_wb_is_reg_c_i;
	input wire reg_d_alu_is_reg_a_i;
	input wire reg_d_alu_is_reg_b_i;
	input wire reg_d_alu_is_reg_c_i;
	output reg halt_if_o;
	output reg halt_id_o;
	output wire misaligned_stall_o;
	output reg jr_stall_o;
	output reg load_stall_o;
	input wire id_ready_i;
	input wire id_valid_i;
	input wire ex_valid_i;
	input wire wb_ready_i;
	output reg perf_pipeline_stall_o;
	reg [4:0] ctrl_fsm_cs;
	reg [4:0] ctrl_fsm_ns;
	reg [2:0] debug_fsm_cs;
	reg [2:0] debug_fsm_ns;
	reg jump_done;
	reg jump_done_q;
	reg jump_in_dec;
	reg branch_in_id;
	reg data_err_q;
	reg debug_mode_q;
	reg debug_mode_n;
	reg ebrk_force_debug_mode;
	wire is_hwlp_body;
	reg illegal_insn_q;
	reg illegal_insn_n;
	reg debug_req_entry_q;
	reg debug_req_entry_n;
	reg debug_force_wakeup_q;
	reg debug_force_wakeup_n;
	wire hwlp_end0_eq_pc;
	wire hwlp_end1_eq_pc;
	wire hwlp_counter0_gt_1;
	wire hwlp_counter1_gt_1;
	wire hwlp_counter0_eq_1;
	wire hwlp_counter1_eq_1;
	wire hwlp_counter0_eq_0;
	wire hwlp_counter1_eq_0;
	wire hwlp_end0_eq_pc_plus4;
	wire hwlp_end1_eq_pc_plus4;
	wire hwlp_start0_leq_pc;
	wire hwlp_start1_leq_pc;
	wire hwlp_end0_geq_pc;
	wire hwlp_end1_geq_pc;
	reg hwlp_end_4_id_d;
	reg hwlp_end_4_id_q;
	reg debug_req_q;
	wire debug_req_pending;
	wire wfi_active;
	localparam cv32e40p_pkg_BRANCH_COND = 2'b11;
	localparam cv32e40p_pkg_BRANCH_JAL = 2'b01;
	localparam cv32e40p_pkg_BRANCH_JALR = 2'b10;
	localparam cv32e40p_pkg_DBG_CAUSE_EBREAK = 3'h1;
	localparam cv32e40p_pkg_DBG_CAUSE_HALTREQ = 3'h3;
	localparam cv32e40p_pkg_DBG_CAUSE_STEP = 3'h4;
	localparam cv32e40p_pkg_DBG_CAUSE_TRIGGER = 3'h2;
	localparam cv32e40p_pkg_EXC_CAUSE_BREAKPOINT = 5'h03;
	localparam cv32e40p_pkg_EXC_CAUSE_ECALL_MMODE = 5'h0b;
	localparam cv32e40p_pkg_EXC_CAUSE_ECALL_UMODE = 5'h08;
	localparam cv32e40p_pkg_EXC_CAUSE_ILLEGAL_INSN = 5'h02;
	localparam cv32e40p_pkg_EXC_CAUSE_INSTR_FAULT = 5'h01;
	localparam cv32e40p_pkg_EXC_CAUSE_LOAD_FAULT = 5'h05;
	localparam cv32e40p_pkg_EXC_CAUSE_STORE_FAULT = 5'h07;
	localparam cv32e40p_pkg_EXC_PC_DBD = 3'b010;
	localparam cv32e40p_pkg_EXC_PC_DBE = 3'b011;
	localparam cv32e40p_pkg_EXC_PC_EXCEPTION = 3'b000;
	localparam cv32e40p_pkg_EXC_PC_IRQ = 3'b001;
	localparam cv32e40p_pkg_PC_BOOT = 4'b0000;
	localparam cv32e40p_pkg_PC_BRANCH = 4'b0011;
	localparam cv32e40p_pkg_PC_DRET = 4'b0111;
	localparam cv32e40p_pkg_PC_EXCEPTION = 4'b0100;
	localparam cv32e40p_pkg_PC_FENCEI = 4'b0001;
	localparam cv32e40p_pkg_PC_HWLOOP = 4'b1000;
	localparam cv32e40p_pkg_PC_JUMP = 4'b0010;
	localparam cv32e40p_pkg_PC_MRET = 4'b0101;
	localparam cv32e40p_pkg_PC_URET = 4'b0110;
	localparam cv32e40p_pkg_TRAP_MACHINE = 2'b00;
	localparam cv32e40p_pkg_TRAP_USER = 2'b01;
	always @(*) begin
		if (_sv2v_0)
			;
		instr_req_o = 1'b1;
		data_err_ack_o = 1'b0;
		csr_save_if_o = 1'b0;
		csr_save_id_o = 1'b0;
		csr_save_ex_o = 1'b0;
		csr_restore_mret_id_o = 1'b0;
		csr_restore_uret_id_o = 1'b0;
		csr_restore_dret_id_o = 1'b0;
		csr_save_cause_o = 1'b0;
		exc_cause_o = 1'sb0;
		exc_pc_mux_o = cv32e40p_pkg_EXC_PC_IRQ;
		trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
		csr_cause_o = 1'sb0;
		csr_irq_sec_o = 1'b0;
		pc_mux_o = cv32e40p_pkg_PC_BOOT;
		pc_set_o = 1'b0;
		jump_done = jump_done_q;
		ctrl_fsm_ns = ctrl_fsm_cs;
		ctrl_busy_o = 1'b1;
		halt_if_o = 1'b0;
		halt_id_o = 1'b0;
		is_decoding_o = 1'b0;
		irq_ack_o = 1'b0;
		irq_id_o = 5'b00000;
		jump_in_dec = (ctrl_transfer_insn_in_dec_i == cv32e40p_pkg_BRANCH_JALR) || (ctrl_transfer_insn_in_dec_i == cv32e40p_pkg_BRANCH_JAL);
		branch_in_id = ctrl_transfer_insn_in_id_i == cv32e40p_pkg_BRANCH_COND;
		ebrk_force_debug_mode = (debug_ebreakm_i && (current_priv_lvl_i == 2'b11)) || (debug_ebreaku_i && (current_priv_lvl_i == 2'b00));
		debug_csr_save_o = 1'b0;
		debug_cause_o = cv32e40p_pkg_DBG_CAUSE_EBREAK;
		debug_mode_n = debug_mode_q;
		illegal_insn_n = illegal_insn_q;
		debug_req_entry_n = debug_req_entry_q;
		debug_force_wakeup_n = debug_force_wakeup_q;
		perf_pipeline_stall_o = 1'b0;
		hwlp_mask_o = 1'b0;
		hwlp_dec_cnt_o = 1'sb0;
		hwlp_end_4_id_d = 1'b0;
		hwlp_targ_addr_o = ((hwlp_start1_leq_pc && hwlp_end1_geq_pc) && !(hwlp_start0_leq_pc && hwlp_end0_geq_pc) ? hwlp_start_addr_i[32+:32] : hwlp_start_addr_i[0+:32]);
		case (ctrl_fsm_cs)
			5'd0: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b0;
				if (fetch_enable_i == 1'b1)
					ctrl_fsm_ns = 5'd1;
			end
			5'd1: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b1;
				pc_mux_o = cv32e40p_pkg_PC_BOOT;
				pc_set_o = 1'b1;
				if (debug_req_pending) begin
					ctrl_fsm_ns = 5'd12;
					debug_force_wakeup_n = 1'b1;
				end
				else
					ctrl_fsm_ns = 5'd4;
			end
			5'd3: begin
				is_decoding_o = 1'b0;
				ctrl_busy_o = 1'b0;
				instr_req_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				ctrl_fsm_ns = 5'd2;
			end
			5'd2: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (wake_from_sleep_o) begin
					if (debug_req_pending) begin
						ctrl_fsm_ns = 5'd12;
						debug_force_wakeup_n = 1'b1;
					end
					else
						ctrl_fsm_ns = 5'd4;
				end
				else
					ctrl_busy_o = 1'b0;
			end
			5'd4: begin
				is_decoding_o = 1'b0;
				ctrl_fsm_ns = 5'd5;
				if (irq_req_ctrl_i && ~(debug_req_pending || debug_mode_q)) begin
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
					pc_set_o = 1'b1;
					pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
					exc_pc_mux_o = cv32e40p_pkg_EXC_PC_IRQ;
					exc_cause_o = irq_id_ctrl_i;
					csr_irq_sec_o = irq_sec_ctrl_i;
					irq_ack_o = 1'b1;
					irq_id_o = irq_id_ctrl_i;
					if (irq_sec_ctrl_i)
						trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
					else
						trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? cv32e40p_pkg_TRAP_USER : cv32e40p_pkg_TRAP_MACHINE);
					csr_save_cause_o = 1'b1;
					csr_cause_o = {1'b1, irq_id_ctrl_i};
					csr_save_if_o = 1'b1;
				end
			end
			5'd5:
				if (branch_taken_ex_i) begin
					is_decoding_o = 1'b0;
					pc_mux_o = cv32e40p_pkg_PC_BRANCH;
					pc_set_o = 1'b1;
				end
				else if (data_err_i) begin
					is_decoding_o = 1'b0;
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
					csr_save_ex_o = 1'b1;
					csr_save_cause_o = 1'b1;
					data_err_ack_o = 1'b1;
					csr_cause_o = {1'b0, (data_we_ex_i ? cv32e40p_pkg_EXC_CAUSE_STORE_FAULT : cv32e40p_pkg_EXC_CAUSE_LOAD_FAULT)};
					ctrl_fsm_ns = 5'd9;
				end
				else if (is_fetch_failed_i) begin
					is_decoding_o = 1'b0;
					halt_id_o = 1'b1;
					halt_if_o = 1'b1;
					csr_save_if_o = 1'b1;
					csr_save_cause_o = !debug_mode_q;
					csr_cause_o = {1'b0, cv32e40p_pkg_EXC_CAUSE_INSTR_FAULT};
					ctrl_fsm_ns = 5'd9;
				end
				else if (instr_valid_i) begin : blk_decode_level1
					is_decoding_o = 1'b1;
					illegal_insn_n = 1'b0;
					if ((debug_req_pending || trigger_match_i) & ~debug_mode_q) begin
						is_decoding_o = (COREV_PULP ? 1'b0 : 1'b1);
						halt_if_o = 1'b1;
						halt_id_o = 1'b1;
						ctrl_fsm_ns = 5'd13;
						debug_req_entry_n = 1'b1;
					end
					else if (irq_req_ctrl_i && ~debug_mode_q) begin
						hwlp_mask_o = (COREV_PULP ? 1'b1 : 1'b0);
						is_decoding_o = 1'b0;
						halt_if_o = 1'b1;
						halt_id_o = 1'b1;
						pc_set_o = 1'b1;
						pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
						exc_pc_mux_o = cv32e40p_pkg_EXC_PC_IRQ;
						exc_cause_o = irq_id_ctrl_i;
						csr_irq_sec_o = irq_sec_ctrl_i;
						irq_ack_o = 1'b1;
						irq_id_o = irq_id_ctrl_i;
						if (irq_sec_ctrl_i)
							trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
						else
							trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? cv32e40p_pkg_TRAP_USER : cv32e40p_pkg_TRAP_MACHINE);
						csr_save_cause_o = 1'b1;
						csr_cause_o = {1'b1, irq_id_ctrl_i};
						csr_save_id_o = 1'b1;
					end
					else begin
						if (illegal_insn_i) begin
							halt_if_o = 1'b1;
							halt_id_o = 1'b0;
							ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd5);
							illegal_insn_n = 1'b1;
						end
						else
							case (1'b1)
								jump_in_dec: begin
									pc_mux_o = cv32e40p_pkg_PC_JUMP;
									if (~jr_stall_o && ~jump_done_q) begin
										pc_set_o = 1'b1;
										jump_done = 1'b1;
									end
								end
								ebrk_insn_i: begin
									halt_if_o = 1'b1;
									halt_id_o = 1'b0;
									if (debug_mode_q)
										ctrl_fsm_ns = 5'd13;
									else if (ebrk_force_debug_mode)
										ctrl_fsm_ns = 5'd13;
									else
										ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd5);
								end
								wfi_active: begin
									halt_if_o = 1'b1;
									halt_id_o = 1'b0;
									ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd5);
								end
								ecall_insn_i: begin
									halt_if_o = 1'b1;
									halt_id_o = 1'b0;
									ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd5);
								end
								fencei_insn_i: begin
									halt_if_o = 1'b1;
									halt_id_o = 1'b0;
									ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd5);
								end
								(mret_insn_i | uret_insn_i) | dret_insn_i: begin
									halt_if_o = 1'b1;
									halt_id_o = 1'b0;
									ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd5);
								end
								csr_status_i: begin
									halt_if_o = 1'b1;
									ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd5);
								end
								data_load_event_i: begin
									ctrl_fsm_ns = (id_ready_i ? 5'd7 : 5'd5);
									halt_if_o = 1'b1;
								end
								default: begin
									if (is_hwlp_body) begin
										ctrl_fsm_ns = (hwlp_end0_eq_pc_plus4 || hwlp_end1_eq_pc_plus4 ? 5'd5 : 5'd15);
										if (hwlp_end0_eq_pc && hwlp_counter0_gt_1) begin
											pc_mux_o = cv32e40p_pkg_PC_HWLOOP;
											if (~jump_done_q) begin
												pc_set_o = 1'b1;
												jump_done = 1'b1;
												hwlp_dec_cnt_o[0] = 1'b1;
											end
										end
										if (hwlp_end1_eq_pc && hwlp_counter1_gt_1) begin
											pc_mux_o = cv32e40p_pkg_PC_HWLOOP;
											if (~jump_done_q) begin
												pc_set_o = 1'b1;
												jump_done = 1'b1;
												hwlp_dec_cnt_o[1] = 1'b1;
											end
										end
									end
									if (hwlp_end0_eq_pc && hwlp_counter0_eq_1)
										hwlp_dec_cnt_o[0] = 1'b1;
									if (hwlp_end1_eq_pc && hwlp_counter1_eq_1)
										hwlp_dec_cnt_o[1] = 1'b1;
								end
							endcase
						if (debug_single_step_i & ~debug_mode_q) begin
							halt_if_o = 1'b1;
							if (id_ready_i)
								case (1'b1)
									illegal_insn_i | ecall_insn_i: ctrl_fsm_ns = 5'd8;
									~ebrk_force_debug_mode & ebrk_insn_i: ctrl_fsm_ns = 5'd8;
									mret_insn_i | uret_insn_i: ctrl_fsm_ns = 5'd8;
									branch_in_id: ctrl_fsm_ns = 5'd14;
									default: ctrl_fsm_ns = 5'd13;
								endcase
						end
					end
				end
				else begin
					is_decoding_o = 1'b0;
					perf_pipeline_stall_o = data_load_event_i;
				end
			5'd15:
				if (COREV_PULP) begin
					if (instr_valid_i) begin
						is_decoding_o = 1'b1;
						if ((debug_req_pending || trigger_match_i) & ~debug_mode_q) begin
							is_decoding_o = (COREV_PULP ? 1'b0 : 1'b1);
							halt_if_o = 1'b1;
							halt_id_o = 1'b1;
							ctrl_fsm_ns = 5'd13;
							debug_req_entry_n = 1'b1;
						end
						else if (irq_req_ctrl_i && ~debug_mode_q) begin
							hwlp_mask_o = (COREV_PULP ? 1'b1 : 1'b0);
							is_decoding_o = 1'b0;
							halt_if_o = 1'b1;
							halt_id_o = 1'b1;
							pc_set_o = 1'b1;
							pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
							exc_pc_mux_o = cv32e40p_pkg_EXC_PC_IRQ;
							exc_cause_o = irq_id_ctrl_i;
							csr_irq_sec_o = irq_sec_ctrl_i;
							irq_ack_o = 1'b1;
							irq_id_o = irq_id_ctrl_i;
							if (irq_sec_ctrl_i)
								trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
							else
								trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? cv32e40p_pkg_TRAP_USER : cv32e40p_pkg_TRAP_MACHINE);
							csr_save_cause_o = 1'b1;
							csr_cause_o = {1'b1, irq_id_ctrl_i};
							csr_save_id_o = 1'b1;
							ctrl_fsm_ns = 5'd5;
						end
						else begin
							if (illegal_insn_i) begin
								halt_if_o = 1'b1;
								halt_id_o = 1'b1;
								ctrl_fsm_ns = 5'd8;
								illegal_insn_n = 1'b1;
							end
							else
								case (1'b1)
									ebrk_insn_i: begin
										halt_if_o = 1'b1;
										halt_id_o = 1'b0;
										if (debug_mode_q)
											ctrl_fsm_ns = 5'd13;
										else if (ebrk_force_debug_mode)
											ctrl_fsm_ns = 5'd13;
										else
											ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd15);
									end
									ecall_insn_i: begin
										halt_if_o = 1'b1;
										halt_id_o = 1'b0;
										ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd15);
									end
									csr_status_i: begin
										halt_if_o = 1'b1;
										ctrl_fsm_ns = (id_ready_i ? 5'd8 : 5'd15);
									end
									data_load_event_i: begin
										ctrl_fsm_ns = (id_ready_i ? 5'd7 : 5'd15);
										halt_if_o = 1'b1;
									end
									default: begin
										if (hwlp_end1_eq_pc_plus4) begin
											if (hwlp_counter1_gt_1) begin
												hwlp_end_4_id_d = 1'b1;
												hwlp_targ_addr_o = hwlp_start_addr_i[32+:32];
												ctrl_fsm_ns = 5'd15;
											end
											else
												ctrl_fsm_ns = (is_hwlp_body ? 5'd15 : 5'd5);
										end
										if (hwlp_end0_eq_pc_plus4) begin
											if (hwlp_counter0_gt_1) begin
												hwlp_end_4_id_d = 1'b1;
												hwlp_targ_addr_o = hwlp_start_addr_i[0+:32];
												ctrl_fsm_ns = 5'd15;
											end
											else
												ctrl_fsm_ns = (is_hwlp_body ? 5'd15 : 5'd5);
										end
										hwlp_dec_cnt_o[0] = hwlp_end0_eq_pc && !hwlp_counter0_eq_0;
										hwlp_dec_cnt_o[1] = hwlp_end1_eq_pc && !hwlp_counter1_eq_0;
									end
								endcase
							if (debug_single_step_i & ~debug_mode_q) begin
								halt_if_o = 1'b1;
								if (id_ready_i)
									case (1'b1)
										illegal_insn_i | ecall_insn_i: ctrl_fsm_ns = 5'd8;
										~ebrk_force_debug_mode & ebrk_insn_i: ctrl_fsm_ns = 5'd8;
										mret_insn_i | uret_insn_i: ctrl_fsm_ns = 5'd8;
										branch_in_id: ctrl_fsm_ns = 5'd14;
										default: ctrl_fsm_ns = 5'd13;
									endcase
							end
						end
					end
					else begin
						is_decoding_o = 1'b0;
						perf_pipeline_stall_o = data_load_event_i;
					end
				end
			5'd8: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				if (data_err_i) begin
					csr_save_ex_o = 1'b1;
					csr_save_cause_o = 1'b1;
					data_err_ack_o = 1'b1;
					csr_cause_o = {1'b0, (data_we_ex_i ? cv32e40p_pkg_EXC_CAUSE_STORE_FAULT : cv32e40p_pkg_EXC_CAUSE_LOAD_FAULT)};
					ctrl_fsm_ns = 5'd9;
					illegal_insn_n = 1'b0;
				end
				else if (ex_valid_i) begin
					ctrl_fsm_ns = 5'd9;
					if (illegal_insn_q) begin
						csr_save_id_o = 1'b1;
						csr_save_cause_o = !debug_mode_q;
						csr_cause_o = {1'b0, cv32e40p_pkg_EXC_CAUSE_ILLEGAL_INSN};
					end
					else
						case (1'b1)
							ebrk_insn_i: begin
								csr_save_id_o = 1'b1;
								csr_save_cause_o = 1'b1;
								csr_cause_o = {1'b0, cv32e40p_pkg_EXC_CAUSE_BREAKPOINT};
							end
							ecall_insn_i: begin
								csr_save_id_o = 1'b1;
								csr_save_cause_o = !debug_mode_q;
								csr_cause_o = {1'b0, (current_priv_lvl_i == 2'b00 ? cv32e40p_pkg_EXC_CAUSE_ECALL_UMODE : cv32e40p_pkg_EXC_CAUSE_ECALL_MMODE)};
							end
							default:
								;
						endcase
				end
			end
			5'd6:
				if (COREV_CLUSTER == 1'b1) begin
					is_decoding_o = 1'b0;
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
					ctrl_fsm_ns = 5'd5;
					perf_pipeline_stall_o = data_load_event_i;
					if (irq_req_ctrl_i && ~(debug_req_pending || debug_mode_q)) begin
						is_decoding_o = 1'b0;
						halt_if_o = 1'b1;
						halt_id_o = 1'b1;
						pc_set_o = 1'b1;
						pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
						exc_pc_mux_o = cv32e40p_pkg_EXC_PC_IRQ;
						exc_cause_o = irq_id_ctrl_i;
						csr_irq_sec_o = irq_sec_ctrl_i;
						irq_ack_o = 1'b1;
						irq_id_o = irq_id_ctrl_i;
						if (irq_sec_ctrl_i)
							trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
						else
							trap_addr_mux_o = (current_priv_lvl_i == 2'b00 ? cv32e40p_pkg_TRAP_USER : cv32e40p_pkg_TRAP_MACHINE);
						csr_save_cause_o = 1'b1;
						csr_cause_o = {1'b1, irq_id_ctrl_i};
						csr_save_id_o = 1'b1;
					end
				end
			5'd7:
				if (COREV_CLUSTER == 1'b1) begin
					is_decoding_o = 1'b0;
					halt_if_o = 1'b1;
					halt_id_o = 1'b1;
					if (id_ready_i)
						ctrl_fsm_ns = ((debug_req_pending || trigger_match_i) & ~debug_mode_q ? 5'd13 : 5'd6);
					else
						ctrl_fsm_ns = 5'd7;
					perf_pipeline_stall_o = data_load_event_i;
				end
			5'd9: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
				if (data_err_q) begin
					pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
					pc_set_o = 1'b1;
					trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
					exc_pc_mux_o = cv32e40p_pkg_EXC_PC_EXCEPTION;
					exc_cause_o = (data_we_ex_i ? cv32e40p_pkg_EXC_CAUSE_LOAD_FAULT : cv32e40p_pkg_EXC_CAUSE_STORE_FAULT);
				end
				else if (is_fetch_failed_i) begin
					pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
					pc_set_o = 1'b1;
					trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
					exc_pc_mux_o = (debug_mode_q ? cv32e40p_pkg_EXC_PC_DBE : cv32e40p_pkg_EXC_PC_EXCEPTION);
					exc_cause_o = cv32e40p_pkg_EXC_CAUSE_INSTR_FAULT;
				end
				else if (illegal_insn_q) begin
					pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
					pc_set_o = 1'b1;
					trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
					exc_pc_mux_o = (debug_mode_q ? cv32e40p_pkg_EXC_PC_DBE : cv32e40p_pkg_EXC_PC_EXCEPTION);
					illegal_insn_n = 1'b0;
					if (debug_single_step_i && ~debug_mode_q)
						ctrl_fsm_ns = 5'd12;
				end
				else
					case (1'b1)
						ebrk_insn_i: begin
							pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
							pc_set_o = 1'b1;
							trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
							exc_pc_mux_o = cv32e40p_pkg_EXC_PC_EXCEPTION;
							if (debug_single_step_i && ~debug_mode_q)
								ctrl_fsm_ns = 5'd12;
						end
						ecall_insn_i: begin
							pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
							pc_set_o = 1'b1;
							trap_addr_mux_o = cv32e40p_pkg_TRAP_MACHINE;
							exc_pc_mux_o = (debug_mode_q ? cv32e40p_pkg_EXC_PC_DBE : cv32e40p_pkg_EXC_PC_EXCEPTION);
							if (debug_single_step_i && ~debug_mode_q)
								ctrl_fsm_ns = 5'd12;
						end
						mret_insn_i: begin
							csr_restore_mret_id_o = !debug_mode_q;
							ctrl_fsm_ns = 5'd10;
						end
						uret_insn_i: begin
							csr_restore_uret_id_o = !debug_mode_q;
							ctrl_fsm_ns = 5'd10;
						end
						dret_insn_i: begin
							csr_restore_dret_id_o = 1'b1;
							ctrl_fsm_ns = 5'd10;
						end
						csr_status_i: begin
							if (hwlp_end0_eq_pc && hwlp_counter0_gt_1) begin
								pc_mux_o = cv32e40p_pkg_PC_HWLOOP;
								pc_set_o = 1'b1;
								hwlp_dec_cnt_o[0] = 1'b1;
							end
							if (hwlp_end1_eq_pc && hwlp_counter1_gt_1) begin
								pc_mux_o = cv32e40p_pkg_PC_HWLOOP;
								pc_set_o = 1'b1;
								hwlp_dec_cnt_o[1] = 1'b1;
							end
						end
						wfi_i:
							if (debug_req_pending) begin
								ctrl_fsm_ns = 5'd12;
								debug_force_wakeup_n = 1'b1;
							end
							else
								ctrl_fsm_ns = 5'd3;
						fencei_insn_i: begin
							pc_mux_o = cv32e40p_pkg_PC_FENCEI;
							pc_set_o = 1'b1;
						end
						default:
							;
					endcase
			end
			5'd10: begin
				is_decoding_o = 1'b0;
				ctrl_fsm_ns = 5'd5;
				case (1'b1)
					mret_dec_i: begin
						pc_mux_o = (debug_mode_q ? cv32e40p_pkg_PC_EXCEPTION : cv32e40p_pkg_PC_MRET);
						pc_set_o = 1'b1;
						exc_pc_mux_o = cv32e40p_pkg_EXC_PC_DBE;
					end
					uret_dec_i: begin
						pc_mux_o = (debug_mode_q ? cv32e40p_pkg_PC_EXCEPTION : cv32e40p_pkg_PC_URET);
						pc_set_o = 1'b1;
						exc_pc_mux_o = cv32e40p_pkg_EXC_PC_DBE;
					end
					dret_dec_i: begin
						pc_mux_o = cv32e40p_pkg_PC_DRET;
						pc_set_o = 1'b1;
						debug_mode_n = 1'b0;
					end
					default:
						;
				endcase
				if (debug_single_step_i && ~debug_mode_q)
					ctrl_fsm_ns = 5'd12;
			end
			5'd14: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				if (branch_taken_ex_i) begin
					pc_mux_o = cv32e40p_pkg_PC_BRANCH;
					pc_set_o = 1'b1;
				end
				ctrl_fsm_ns = 5'd13;
			end
			5'd11: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
				exc_pc_mux_o = cv32e40p_pkg_EXC_PC_DBD;
				if (~debug_mode_q) begin
					csr_save_cause_o = 1'b1;
					csr_save_id_o = 1'b1;
					debug_csr_save_o = 1'b1;
					if (trigger_match_i)
						debug_cause_o = cv32e40p_pkg_DBG_CAUSE_TRIGGER;
					else if (ebrk_force_debug_mode & ebrk_insn_i)
						debug_cause_o = cv32e40p_pkg_DBG_CAUSE_EBREAK;
					else if (debug_req_entry_q)
						debug_cause_o = cv32e40p_pkg_DBG_CAUSE_HALTREQ;
				end
				debug_req_entry_n = 1'b0;
				ctrl_fsm_ns = 5'd5;
				debug_mode_n = 1'b1;
			end
			5'd12: begin
				is_decoding_o = 1'b0;
				pc_set_o = 1'b1;
				pc_mux_o = cv32e40p_pkg_PC_EXCEPTION;
				exc_pc_mux_o = cv32e40p_pkg_EXC_PC_DBD;
				csr_save_cause_o = 1'b1;
				debug_csr_save_o = 1'b1;
				if (debug_force_wakeup_q)
					debug_cause_o = cv32e40p_pkg_DBG_CAUSE_HALTREQ;
				else if (debug_single_step_i)
					debug_cause_o = cv32e40p_pkg_DBG_CAUSE_STEP;
				csr_save_if_o = 1'b1;
				ctrl_fsm_ns = 5'd5;
				debug_mode_n = 1'b1;
				debug_force_wakeup_n = 1'b0;
			end
			5'd13: begin
				is_decoding_o = 1'b0;
				halt_if_o = 1'b1;
				halt_id_o = 1'b1;
				perf_pipeline_stall_o = data_load_event_i;
				if (data_err_i) begin
					csr_save_ex_o = 1'b1;
					csr_save_cause_o = 1'b1;
					data_err_ack_o = 1'b1;
					csr_cause_o = {1'b0, (data_we_ex_i ? cv32e40p_pkg_EXC_CAUSE_STORE_FAULT : cv32e40p_pkg_EXC_CAUSE_LOAD_FAULT)};
					ctrl_fsm_ns = 5'd9;
				end
				else if ((((debug_mode_q | trigger_match_i) | (ebrk_force_debug_mode & ebrk_insn_i)) | data_load_event_i) | debug_req_entry_q)
					ctrl_fsm_ns = 5'd11;
				else
					ctrl_fsm_ns = 5'd12;
			end
			default: begin
				is_decoding_o = 1'b0;
				instr_req_o = 1'b0;
				ctrl_fsm_ns = 5'd0;
			end
		endcase
	end
	generate
		if (COREV_PULP) begin : gen_hwlp
			assign hwlp_jump_o = (hwlp_end_4_id_d && !hwlp_end_4_id_q ? 1'b1 : 1'b0);
			always @(posedge clk or negedge rst_n)
				if (!rst_n)
					hwlp_end_4_id_q <= 1'b0;
				else
					hwlp_end_4_id_q <= hwlp_end_4_id_d;
			assign hwlp_end0_eq_pc = hwlp_end_addr_i[0+:32] == (pc_id_i + 4);
			assign hwlp_end1_eq_pc = hwlp_end_addr_i[32+:32] == (pc_id_i + 4);
			assign hwlp_counter0_gt_1 = hwlp_counter_i[0+:32] > 1;
			assign hwlp_counter1_gt_1 = hwlp_counter_i[32+:32] > 1;
			assign hwlp_counter0_eq_1 = hwlp_counter_i[0+:32] == 1;
			assign hwlp_counter1_eq_1 = hwlp_counter_i[32+:32] == 1;
			assign hwlp_counter0_eq_0 = hwlp_counter_i[0+:32] == 0;
			assign hwlp_counter1_eq_0 = hwlp_counter_i[32+:32] == 0;
			assign hwlp_end0_eq_pc_plus4 = hwlp_end_addr_i[0+:32] == (pc_id_i + 8);
			assign hwlp_end1_eq_pc_plus4 = hwlp_end_addr_i[32+:32] == (pc_id_i + 8);
			assign hwlp_start0_leq_pc = hwlp_start_addr_i[0+:32] <= pc_id_i;
			assign hwlp_start1_leq_pc = hwlp_start_addr_i[32+:32] <= pc_id_i;
			assign hwlp_end0_geq_pc = hwlp_end_addr_i[0+:32] >= (pc_id_i + 4);
			assign hwlp_end1_geq_pc = hwlp_end_addr_i[32+:32] >= (pc_id_i + 4);
			assign is_hwlp_body = ((hwlp_start0_leq_pc && hwlp_end0_geq_pc) && hwlp_counter0_gt_1) || ((hwlp_start1_leq_pc && hwlp_end1_geq_pc) && hwlp_counter1_gt_1);
		end
		else begin : gen_no_hwlp
			assign hwlp_jump_o = 1'b0;
			wire [1:1] sv2v_tmp_23ABB;
			assign sv2v_tmp_23ABB = 1'b0;
			always @(*) hwlp_end_4_id_q = sv2v_tmp_23ABB;
			assign hwlp_end0_eq_pc = 1'b0;
			assign hwlp_end1_eq_pc = 1'b0;
			assign hwlp_counter0_gt_1 = 1'b0;
			assign hwlp_counter1_gt_1 = 1'b0;
			assign hwlp_counter0_eq_1 = 1'b0;
			assign hwlp_counter1_eq_1 = 1'b0;
			assign hwlp_counter0_eq_0 = 1'b0;
			assign hwlp_counter1_eq_0 = 1'b0;
			assign hwlp_end0_eq_pc_plus4 = 1'b0;
			assign hwlp_end1_eq_pc_plus4 = 1'b0;
			assign hwlp_start0_leq_pc = 1'b0;
			assign hwlp_start1_leq_pc = 1'b0;
			assign hwlp_end0_geq_pc = 1'b0;
			assign hwlp_end1_geq_pc = 1'b0;
			assign is_hwlp_body = 1'b0;
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		load_stall_o = 1'b0;
		deassert_we_o = 1'b0;
		if (~is_decoding_o)
			deassert_we_o = 1'b1;
		if (illegal_insn_i)
			deassert_we_o = 1'b1;
		if ((((data_req_ex_i == 1'b1) && (regfile_we_ex_i == 1'b1)) || ((wb_ready_i == 1'b0) && (regfile_we_wb_i == 1'b1))) && ((((reg_d_ex_is_reg_a_i == 1'b1) || (reg_d_ex_is_reg_b_i == 1'b1)) || (reg_d_ex_is_reg_c_i == 1'b1)) || ((is_decoding_o && (regfile_we_id_i && !data_misaligned_i)) && (regfile_waddr_ex_i == regfile_alu_waddr_id_i)))) begin
			deassert_we_o = 1'b1;
			load_stall_o = 1'b1;
		end
		if ((ctrl_transfer_insn_in_dec_i == cv32e40p_pkg_BRANCH_JALR) && (((((regfile_we_wb_i == 1'b1) && (reg_d_wb_is_reg_a_i == 1'b1)) || ((regfile_we_ex_i == 1'b1) && (reg_d_ex_is_reg_a_i == 1'b1))) || ((regfile_alu_we_fw_i == 1'b1) && (reg_d_alu_is_reg_a_i == 1'b1))) || (FPU && (apu_read_dep_for_jalr_i == 1'b1)))) begin
			jr_stall_o = 1'b1;
			deassert_we_o = 1'b1;
		end
		else
			jr_stall_o = 1'b0;
	end
	assign misaligned_stall_o = data_misaligned_i;
	assign apu_stall_o = apu_read_dep_i | (apu_write_dep_i & ~apu_en_i);
	localparam cv32e40p_pkg_SEL_FW_EX = 2'b01;
	localparam cv32e40p_pkg_SEL_FW_WB = 2'b10;
	localparam cv32e40p_pkg_SEL_REGFILE = 2'b00;
	always @(*) begin
		if (_sv2v_0)
			;
		operand_a_fw_mux_sel_o = cv32e40p_pkg_SEL_REGFILE;
		operand_b_fw_mux_sel_o = cv32e40p_pkg_SEL_REGFILE;
		operand_c_fw_mux_sel_o = cv32e40p_pkg_SEL_REGFILE;
		if (regfile_we_wb_i == 1'b1) begin
			if (reg_d_wb_is_reg_a_i == 1'b1)
				operand_a_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_WB;
			if (reg_d_wb_is_reg_b_i == 1'b1)
				operand_b_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_WB;
			if (reg_d_wb_is_reg_c_i == 1'b1)
				operand_c_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_WB;
		end
		if (regfile_alu_we_fw_i == 1'b1) begin
			if (reg_d_alu_is_reg_a_i == 1'b1)
				operand_a_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_EX;
			if (reg_d_alu_is_reg_b_i == 1'b1)
				operand_b_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_EX;
			if (reg_d_alu_is_reg_c_i == 1'b1)
				operand_c_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_EX;
		end
		if (data_misaligned_i) begin
			operand_a_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_EX;
			operand_b_fw_mux_sel_o = cv32e40p_pkg_SEL_REGFILE;
		end
		else if (mult_multicycle_i)
			operand_c_fw_mux_sel_o = cv32e40p_pkg_SEL_FW_EX;
	end
	always @(posedge clk or negedge rst_n) begin : UPDATE_REGS
		if (rst_n == 1'b0) begin
			ctrl_fsm_cs <= 5'd0;
			jump_done_q <= 1'b0;
			data_err_q <= 1'b0;
			debug_mode_q <= 1'b0;
			illegal_insn_q <= 1'b0;
			debug_req_entry_q <= 1'b0;
			debug_force_wakeup_q <= 1'b0;
		end
		else begin
			ctrl_fsm_cs <= ctrl_fsm_ns;
			jump_done_q <= jump_done & ~id_ready_i;
			data_err_q <= data_err_i;
			debug_mode_q <= debug_mode_n;
			illegal_insn_q <= illegal_insn_n;
			debug_req_entry_q <= debug_req_entry_n;
			debug_force_wakeup_q <= debug_force_wakeup_n;
		end
	end
	assign wake_from_sleep_o = (irq_wu_ctrl_i || debug_req_pending) || debug_mode_q;
	assign debug_mode_o = debug_mode_q;
	assign debug_req_pending = debug_req_i || debug_req_q;
	assign debug_p_elw_no_sleep_o = ((debug_mode_q || debug_req_q) || debug_single_step_i) || trigger_match_i;
	assign debug_wfi_no_sleep_o = (((debug_mode_q || debug_req_pending) || debug_single_step_i) || trigger_match_i) || COREV_CLUSTER;
	assign wfi_active = wfi_i & ~debug_wfi_no_sleep_o;
	always @(posedge clk_ungated_i or negedge rst_n)
		if (!rst_n)
			debug_req_q <= 1'b0;
		else if (debug_req_i)
			debug_req_q <= 1'b1;
		else if (debug_mode_q)
			debug_req_q <= 1'b0;
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0)
			debug_fsm_cs <= 3'b001;
		else
			debug_fsm_cs <= debug_fsm_ns;
	always @(*) begin
		if (_sv2v_0)
			;
		debug_fsm_ns = debug_fsm_cs;
		case (debug_fsm_cs)
			3'b001:
				if (debug_mode_n || (ctrl_fsm_ns == 5'd4)) begin
					if (debug_mode_n)
						debug_fsm_ns = 3'b100;
					else
						debug_fsm_ns = 3'b010;
				end
			3'b010:
				if (debug_mode_n)
					debug_fsm_ns = 3'b100;
			3'b100:
				if (!debug_mode_n)
					debug_fsm_ns = 3'b010;
			default: debug_fsm_ns = 3'b001;
		endcase
	end
	localparam cv32e40p_pkg_HAVERESET_INDEX = 0;
	assign debug_havereset_o = debug_fsm_cs[cv32e40p_pkg_HAVERESET_INDEX];
	localparam cv32e40p_pkg_RUNNING_INDEX = 1;
	assign debug_running_o = debug_fsm_cs[cv32e40p_pkg_RUNNING_INDEX];
	localparam cv32e40p_pkg_HALTED_INDEX = 2;
	assign debug_halted_o = debug_fsm_cs[cv32e40p_pkg_HALTED_INDEX];
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_obi_interface (
	clk,
	rst_n,
	trans_valid_i,
	trans_ready_o,
	trans_addr_i,
	trans_we_i,
	trans_be_i,
	trans_wdata_i,
	trans_atop_i,
	resp_valid_o,
	resp_rdata_o,
	resp_err_o,
	obi_req_o,
	obi_gnt_i,
	obi_addr_o,
	obi_we_o,
	obi_be_o,
	obi_wdata_o,
	obi_atop_o,
	obi_rdata_i,
	obi_rvalid_i,
	obi_err_i
);
	reg _sv2v_0;
	parameter TRANS_STABLE = 0;
	input wire clk;
	input wire rst_n;
	input wire trans_valid_i;
	output wire trans_ready_o;
	input wire [31:0] trans_addr_i;
	input wire trans_we_i;
	input wire [3:0] trans_be_i;
	input wire [31:0] trans_wdata_i;
	input wire [5:0] trans_atop_i;
	output wire resp_valid_o;
	output wire [31:0] resp_rdata_o;
	output wire resp_err_o;
	output reg obi_req_o;
	input wire obi_gnt_i;
	output reg [31:0] obi_addr_o;
	output reg obi_we_o;
	output reg [3:0] obi_be_o;
	output reg [31:0] obi_wdata_o;
	output reg [5:0] obi_atop_o;
	input wire [31:0] obi_rdata_i;
	input wire obi_rvalid_i;
	input wire obi_err_i;
	reg state_q;
	reg next_state;
	assign resp_valid_o = obi_rvalid_i;
	assign resp_rdata_o = obi_rdata_i;
	assign resp_err_o = obi_err_i;
	generate
		if (TRANS_STABLE) begin : gen_trans_stable
			wire [1:1] sv2v_tmp_5942C;
			assign sv2v_tmp_5942C = trans_valid_i;
			always @(*) obi_req_o = sv2v_tmp_5942C;
			wire [32:1] sv2v_tmp_54402;
			assign sv2v_tmp_54402 = trans_addr_i;
			always @(*) obi_addr_o = sv2v_tmp_54402;
			wire [1:1] sv2v_tmp_0A6A4;
			assign sv2v_tmp_0A6A4 = trans_we_i;
			always @(*) obi_we_o = sv2v_tmp_0A6A4;
			wire [4:1] sv2v_tmp_EEB82;
			assign sv2v_tmp_EEB82 = trans_be_i;
			always @(*) obi_be_o = sv2v_tmp_EEB82;
			wire [32:1] sv2v_tmp_E96A4;
			assign sv2v_tmp_E96A4 = trans_wdata_i;
			always @(*) obi_wdata_o = sv2v_tmp_E96A4;
			wire [6:1] sv2v_tmp_AE218;
			assign sv2v_tmp_AE218 = trans_atop_i;
			always @(*) obi_atop_o = sv2v_tmp_AE218;
			assign trans_ready_o = obi_gnt_i;
			wire [1:1] sv2v_tmp_871E0;
			assign sv2v_tmp_871E0 = 1'd0;
			always @(*) state_q = sv2v_tmp_871E0;
			wire [1:1] sv2v_tmp_E3915;
			assign sv2v_tmp_E3915 = 1'd0;
			always @(*) next_state = sv2v_tmp_E3915;
		end
		else begin : gen_no_trans_stable
			reg [31:0] obi_addr_q;
			reg obi_we_q;
			reg [3:0] obi_be_q;
			reg [31:0] obi_wdata_q;
			reg [5:0] obi_atop_q;
			always @(*) begin
				if (_sv2v_0)
					;
				next_state = state_q;
				case (state_q)
					1'd0:
						if (obi_req_o && !obi_gnt_i)
							next_state = 1'd1;
					1'd1:
						if (obi_gnt_i)
							next_state = 1'd0;
				endcase
			end
			always @(*) begin
				if (_sv2v_0)
					;
				if (state_q == 1'd0) begin
					obi_req_o = trans_valid_i;
					obi_addr_o = trans_addr_i;
					obi_we_o = trans_we_i;
					obi_be_o = trans_be_i;
					obi_wdata_o = trans_wdata_i;
					obi_atop_o = trans_atop_i;
				end
				else begin
					obi_req_o = 1'b1;
					obi_addr_o = obi_addr_q;
					obi_we_o = obi_we_q;
					obi_be_o = obi_be_q;
					obi_wdata_o = obi_wdata_q;
					obi_atop_o = obi_atop_q;
				end
			end
			always @(posedge clk or negedge rst_n)
				if (rst_n == 1'b0) begin
					state_q <= 1'd0;
					obi_addr_q <= 32'b00000000000000000000000000000000;
					obi_we_q <= 1'b0;
					obi_be_q <= 4'b0000;
					obi_wdata_q <= 32'b00000000000000000000000000000000;
					obi_atop_q <= 6'b000000;
				end
				else begin
					state_q <= next_state;
					if ((state_q == 1'd0) && (next_state == 1'd1)) begin
						obi_addr_q <= obi_addr_o;
						obi_we_q <= obi_we_o;
						obi_be_q <= obi_be_o;
						obi_wdata_q <= obi_wdata_o;
						obi_atop_q <= obi_atop_o;
					end
				end
			assign trans_ready_o = state_q == 1'd0;
		end
	endgenerate
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_prefetch_controller (
	clk,
	rst_n,
	req_i,
	branch_i,
	branch_addr_i,
	busy_o,
	hwlp_jump_i,
	hwlp_target_i,
	trans_valid_o,
	trans_ready_i,
	trans_addr_o,
	resp_valid_i,
	fetch_ready_i,
	fetch_valid_o,
	fifo_push_o,
	fifo_pop_o,
	fifo_flush_o,
	fifo_flush_but_first_o,
	fifo_cnt_i,
	fifo_empty_i
);
	reg _sv2v_0;
	parameter PULP_OBI = 0;
	parameter COREV_PULP = 1;
	parameter DEPTH = 4;
	parameter FIFO_ADDR_DEPTH = (DEPTH > 1 ? $clog2(DEPTH) : 1);
	input wire clk;
	input wire rst_n;
	input wire req_i;
	input wire branch_i;
	input wire [31:0] branch_addr_i;
	output wire busy_o;
	input wire hwlp_jump_i;
	input wire [31:0] hwlp_target_i;
	output wire trans_valid_o;
	input wire trans_ready_i;
	output reg [31:0] trans_addr_o;
	input wire resp_valid_i;
	input wire fetch_ready_i;
	output wire fetch_valid_o;
	output wire fifo_push_o;
	output wire fifo_pop_o;
	output wire fifo_flush_o;
	output wire fifo_flush_but_first_o;
	input wire [FIFO_ADDR_DEPTH:0] fifo_cnt_i;
	input wire fifo_empty_i;
	reg state_q;
	reg next_state;
	reg [FIFO_ADDR_DEPTH:0] cnt_q;
	reg [FIFO_ADDR_DEPTH:0] next_cnt;
	wire count_up;
	wire count_down;
	reg [FIFO_ADDR_DEPTH:0] flush_cnt_q;
	reg [FIFO_ADDR_DEPTH:0] next_flush_cnt;
	reg [31:0] trans_addr_q;
	wire [31:0] trans_addr_incr;
	wire [31:0] aligned_branch_addr;
	wire fifo_valid;
	wire [FIFO_ADDR_DEPTH:0] fifo_cnt_masked;
	wire hwlp_wait_resp_flush;
	reg hwlp_flush_after_resp;
	reg [FIFO_ADDR_DEPTH:0] hwlp_flush_cnt_delayed_q;
	wire hwlp_flush_resp_delayed;
	wire hwlp_flush_resp;
	assign busy_o = (cnt_q != 3'b000) || trans_valid_o;
	assign fetch_valid_o = (fifo_valid || resp_valid_i) && !(branch_i || (flush_cnt_q > 0));
	assign aligned_branch_addr = {branch_addr_i[31:2], 2'b00};
	assign trans_addr_incr = {trans_addr_q[31:2], 2'b00} + 32'd4;
	generate
		if (PULP_OBI == 0) begin : gen_no_pulp_obi
			assign trans_valid_o = req_i && ((fifo_cnt_masked + cnt_q) < DEPTH);
		end
		else begin : gen_pulp_obi
			assign trans_valid_o = (cnt_q == 3'b000 ? req_i && ((fifo_cnt_masked + cnt_q) < DEPTH) : (req_i && ((fifo_cnt_masked + cnt_q) < DEPTH)) && resp_valid_i);
		end
	endgenerate
	assign fifo_cnt_masked = (branch_i || hwlp_jump_i ? {(FIFO_ADDR_DEPTH >= 0 ? FIFO_ADDR_DEPTH + 1 : 1 - FIFO_ADDR_DEPTH) {1'sb0}} : fifo_cnt_i);
	always @(*) begin
		if (_sv2v_0)
			;
		next_state = state_q;
		trans_addr_o = trans_addr_q;
		case (state_q)
			1'd0: begin
				if (branch_i)
					trans_addr_o = aligned_branch_addr;
				else if (hwlp_jump_i)
					trans_addr_o = hwlp_target_i;
				else
					trans_addr_o = trans_addr_incr;
				if ((branch_i || hwlp_jump_i) && !(trans_valid_o && trans_ready_i))
					next_state = 1'd1;
			end
			1'd1: begin
				trans_addr_o = (branch_i ? aligned_branch_addr : trans_addr_q);
				if (trans_valid_o && trans_ready_i)
					next_state = 1'd0;
			end
		endcase
	end
	assign fifo_valid = !fifo_empty_i;
	assign fifo_push_o = (resp_valid_i && (fifo_valid || !fetch_ready_i)) && !(branch_i || (flush_cnt_q > 0));
	assign fifo_pop_o = fifo_valid && fetch_ready_i;
	assign count_up = trans_valid_o && trans_ready_i;
	assign count_down = resp_valid_i;
	always @(*) begin
		if (_sv2v_0)
			;
		case ({count_up, count_down})
			2'b00: next_cnt = cnt_q;
			2'b01: next_cnt = cnt_q - 1'b1;
			2'b10: next_cnt = cnt_q + 1'b1;
			2'b11: next_cnt = cnt_q;
		endcase
	end
	generate
		if (COREV_PULP) begin : gen_hwlp
			assign fifo_flush_o = branch_i || ((hwlp_jump_i && !fifo_empty_i) && fifo_pop_o);
			assign fifo_flush_but_first_o = (hwlp_jump_i && !fifo_empty_i) && !fifo_pop_o;
			assign hwlp_flush_resp = hwlp_jump_i && !(fifo_empty_i && !resp_valid_i);
			assign hwlp_wait_resp_flush = hwlp_jump_i && (fifo_empty_i && !resp_valid_i);
			always @(posedge clk or negedge rst_n)
				if (~rst_n) begin
					hwlp_flush_after_resp <= 1'b0;
					hwlp_flush_cnt_delayed_q <= 2'b00;
				end
				else if (branch_i) begin
					hwlp_flush_after_resp <= 1'b0;
					hwlp_flush_cnt_delayed_q <= 2'b00;
				end
				else if (hwlp_wait_resp_flush) begin
					hwlp_flush_after_resp <= 1'b1;
					hwlp_flush_cnt_delayed_q <= cnt_q - 1'b1;
				end
				else if (hwlp_flush_resp_delayed) begin
					hwlp_flush_after_resp <= 1'b0;
					hwlp_flush_cnt_delayed_q <= 2'b00;
				end
			assign hwlp_flush_resp_delayed = hwlp_flush_after_resp && resp_valid_i;
		end
		else begin : gen_no_hwlp
			assign fifo_flush_o = branch_i;
			assign fifo_flush_but_first_o = 1'b0;
			assign hwlp_flush_resp = 1'b0;
			assign hwlp_wait_resp_flush = 1'b0;
			wire [1:1] sv2v_tmp_CF421;
			assign sv2v_tmp_CF421 = 1'b0;
			always @(*) hwlp_flush_after_resp = sv2v_tmp_CF421;
			wire [(FIFO_ADDR_DEPTH >= 0 ? FIFO_ADDR_DEPTH + 1 : 1 - FIFO_ADDR_DEPTH):1] sv2v_tmp_424BD;
			assign sv2v_tmp_424BD = 2'b00;
			always @(*) hwlp_flush_cnt_delayed_q = sv2v_tmp_424BD;
			assign hwlp_flush_resp_delayed = 1'b0;
		end
	endgenerate
	always @(*) begin
		if (_sv2v_0)
			;
		next_flush_cnt = flush_cnt_q;
		if (branch_i || hwlp_flush_resp) begin
			next_flush_cnt = cnt_q;
			if (resp_valid_i && (cnt_q > 0))
				next_flush_cnt = cnt_q - 1'b1;
		end
		else if (hwlp_flush_resp_delayed)
			next_flush_cnt = hwlp_flush_cnt_delayed_q;
		else if (resp_valid_i && (flush_cnt_q > 0))
			next_flush_cnt = flush_cnt_q - 1'b1;
	end
	always @(posedge clk or negedge rst_n)
		if (rst_n == 1'b0) begin
			state_q <= 1'd0;
			cnt_q <= 1'sb0;
			flush_cnt_q <= 1'sb0;
			trans_addr_q <= 1'sb0;
		end
		else begin
			state_q <= next_state;
			cnt_q <= next_cnt;
			flush_cnt_q <= next_flush_cnt;
			if ((branch_i || hwlp_jump_i) || (trans_valid_o && trans_ready_i))
				trans_addr_q <= trans_addr_o;
		end
	initial _sv2v_0 = 0;
endmodule
module cv32e40p_sleep_unit (
	clk_ungated_i,
	rst_n,
	clk_gated_o,
	scan_cg_en_i,
	core_sleep_o,
	fetch_enable_i,
	fetch_enable_o,
	if_busy_i,
	ctrl_busy_i,
	lsu_busy_i,
	apu_busy_i,
	pulp_clock_en_i,
	p_elw_start_i,
	p_elw_finish_i,
	debug_p_elw_no_sleep_i,
	wake_from_sleep_i
);
	parameter COREV_CLUSTER = 0;
	input wire clk_ungated_i;
	input wire rst_n;
	output wire clk_gated_o;
	input wire scan_cg_en_i;
	output wire core_sleep_o;
	input wire fetch_enable_i;
	output wire fetch_enable_o;
	input wire if_busy_i;
	input wire ctrl_busy_i;
	input wire lsu_busy_i;
	input wire apu_busy_i;
	input wire pulp_clock_en_i;
	input wire p_elw_start_i;
	input wire p_elw_finish_i;
	input wire debug_p_elw_no_sleep_i;
	input wire wake_from_sleep_i;
	reg fetch_enable_q;
	wire fetch_enable_d;
	reg core_busy_q;
	wire core_busy_d;
	reg p_elw_busy_q;
	wire p_elw_busy_d;
	wire clock_en;
	assign fetch_enable_d = (fetch_enable_i ? 1'b1 : fetch_enable_q);
	generate
		if (COREV_CLUSTER) begin : g_pulp_sleep
			assign core_busy_d = (p_elw_busy_d ? if_busy_i || apu_busy_i : 1'b1);
			assign clock_en = fetch_enable_q && (pulp_clock_en_i || core_busy_q);
			assign core_sleep_o = (p_elw_busy_d && !core_busy_q) && !debug_p_elw_no_sleep_i;
			assign p_elw_busy_d = (p_elw_start_i ? 1'b1 : (p_elw_finish_i ? 1'b0 : p_elw_busy_q));
		end
		else begin : g_no_pulp_sleep
			assign core_busy_d = ((if_busy_i || ctrl_busy_i) || lsu_busy_i) || apu_busy_i;
			assign clock_en = fetch_enable_q && (wake_from_sleep_i || core_busy_q);
			assign core_sleep_o = fetch_enable_q && !clock_en;
			assign p_elw_busy_d = 1'b0;
		end
	endgenerate
	always @(posedge clk_ungated_i or negedge rst_n)
		if (rst_n == 1'b0) begin
			core_busy_q <= 1'b0;
			p_elw_busy_q <= 1'b0;
			fetch_enable_q <= 1'b0;
		end
		else begin
			core_busy_q <= core_busy_d;
			p_elw_busy_q <= p_elw_busy_d;
			fetch_enable_q <= fetch_enable_d;
		end
	assign fetch_enable_o = fetch_enable_q;
	cv32e40p_clock_gate core_clock_gate_i(
		.clk_i(clk_ungated_i),
		.en_i(clock_en),
		.scan_cg_en_i(scan_cg_en_i),
		.clk_o(clk_gated_o)
	);
endmodule
module cv32e40p_core (
	clk_i,
	rst_ni,
	pulp_clock_en_i,
	scan_cg_en_i,
	boot_addr_i,
	mtvec_addr_i,
	dm_halt_addr_i,
	hart_id_i,
	dm_exception_addr_i,
	instr_req_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_addr_o,
	instr_rdata_i,
	data_req_o,
	data_gnt_i,
	data_rvalid_i,
	data_we_o,
	data_be_o,
	data_addr_o,
	data_wdata_o,
	data_rdata_i,
	apu_req_o,
	apu_gnt_i,
	apu_operands_o,
	apu_op_o,
	apu_flags_o,
	apu_rvalid_i,
	apu_result_i,
	apu_flags_i,
	irq_i,
	irq_ack_o,
	irq_id_o,
	debug_req_i,
	debug_havereset_o,
	debug_running_o,
	debug_halted_o,
	fetch_enable_i,
	core_sleep_o
);
	parameter COREV_PULP = 0;
	parameter COREV_CLUSTER = 0;
	parameter FPU = 0;
	parameter FPU_ADDMUL_LAT = 0;
	parameter FPU_OTHERS_LAT = 0;
	parameter ZFINX = 0;
	parameter NUM_MHPMCOUNTERS = 1;
	input wire clk_i;
	input wire rst_ni;
	input wire pulp_clock_en_i;
	input wire scan_cg_en_i;
	input wire [31:0] boot_addr_i;
	input wire [31:0] mtvec_addr_i;
	input wire [31:0] dm_halt_addr_i;
	input wire [31:0] hart_id_i;
	input wire [31:0] dm_exception_addr_i;
	output wire instr_req_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	output wire [31:0] instr_addr_o;
	input wire [31:0] instr_rdata_i;
	output wire data_req_o;
	input wire data_gnt_i;
	input wire data_rvalid_i;
	output wire data_we_o;
	output wire [3:0] data_be_o;
	output wire [31:0] data_addr_o;
	output wire [31:0] data_wdata_o;
	input wire [31:0] data_rdata_i;
	output wire apu_req_o;
	input wire apu_gnt_i;
	localparam cv32e40p_apu_core_pkg_APU_NARGS_CPU = 3;
	output wire [95:0] apu_operands_o;
	localparam cv32e40p_apu_core_pkg_APU_WOP_CPU = 6;
	output wire [5:0] apu_op_o;
	localparam cv32e40p_apu_core_pkg_APU_NDSFLAGS_CPU = 15;
	output wire [14:0] apu_flags_o;
	input wire apu_rvalid_i;
	input wire [31:0] apu_result_i;
	localparam cv32e40p_apu_core_pkg_APU_NUSFLAGS_CPU = 5;
	input wire [4:0] apu_flags_i;
	input wire [31:0] irq_i;
	output wire irq_ack_o;
	output wire [4:0] irq_id_o;
	input wire debug_req_i;
	output wire debug_havereset_o;
	output wire debug_running_o;
	output wire debug_halted_o;
	input wire fetch_enable_i;
	output wire core_sleep_o;
	localparam PULP_SECURE = 0;
	localparam N_PMP_ENTRIES = 16;
	localparam USE_PMP = 0;
	localparam A_EXTENSION = 0;
	localparam DEBUG_TRIGGER_EN = 1;
	localparam PULP_OBI = 0;
	wire [5:0] data_atop_o;
	wire irq_sec_i;
	wire sec_lvl_o;
	localparam N_HWLP = 2;
	localparam APU = (FPU == 1 ? 1 : 0);
	wire instr_valid_id;
	wire [31:0] instr_rdata_id;
	wire is_compressed_id;
	wire illegal_c_insn_id;
	wire is_fetch_failed_id;
	wire clear_instr_valid;
	wire pc_set;
	wire [3:0] pc_mux_id;
	wire [2:0] exc_pc_mux_id;
	wire [4:0] m_exc_vec_pc_mux_id;
	wire [4:0] u_exc_vec_pc_mux_id;
	wire [4:0] exc_cause;
	wire [1:0] trap_addr_mux;
	wire [31:0] pc_if;
	wire [31:0] pc_id;
	wire is_decoding;
	wire useincr_addr_ex;
	wire data_misaligned;
	wire mult_multicycle;
	wire [31:0] jump_target_id;
	wire [31:0] jump_target_ex;
	wire branch_in_ex;
	wire branch_decision;
	wire [1:0] ctrl_transfer_insn_in_dec;
	wire ctrl_busy;
	wire if_busy;
	wire lsu_busy;
	wire apu_busy;
	wire [31:0] pc_ex;
	wire alu_en_ex;
	localparam cv32e40p_pkg_ALU_OP_WIDTH = 7;
	wire [6:0] alu_operator_ex;
	wire [31:0] alu_operand_a_ex;
	wire [31:0] alu_operand_b_ex;
	wire [31:0] alu_operand_c_ex;
	wire [4:0] bmask_a_ex;
	wire [4:0] bmask_b_ex;
	wire [1:0] imm_vec_ext_ex;
	wire [1:0] alu_vec_mode_ex;
	wire alu_is_clpx_ex;
	wire alu_is_subrot_ex;
	wire [1:0] alu_clpx_shift_ex;
	localparam cv32e40p_pkg_MUL_OP_WIDTH = 3;
	wire [2:0] mult_operator_ex;
	wire [31:0] mult_operand_a_ex;
	wire [31:0] mult_operand_b_ex;
	wire [31:0] mult_operand_c_ex;
	wire mult_en_ex;
	wire mult_sel_subword_ex;
	wire [1:0] mult_signed_mode_ex;
	wire [4:0] mult_imm_ex;
	wire [31:0] mult_dot_op_a_ex;
	wire [31:0] mult_dot_op_b_ex;
	wire [31:0] mult_dot_op_c_ex;
	wire [1:0] mult_dot_signed_ex;
	wire mult_is_clpx_ex;
	wire [1:0] mult_clpx_shift_ex;
	wire mult_clpx_img_ex;
	wire fs_off;
	localparam cv32e40p_pkg_C_RM = 3;
	wire [2:0] frm_csr;
	localparam cv32e40p_pkg_C_FFLAG = 5;
	wire [4:0] fflags_csr;
	wire fflags_we;
	wire fregs_we;
	wire apu_en_ex;
	wire [14:0] apu_flags_ex;
	wire [5:0] apu_op_ex;
	wire [1:0] apu_lat_ex;
	wire [95:0] apu_operands_ex;
	wire [5:0] apu_waddr_ex;
	wire [17:0] apu_read_regs;
	wire [2:0] apu_read_regs_valid;
	wire apu_read_dep;
	wire apu_read_dep_for_jalr;
	wire [11:0] apu_write_regs;
	wire [1:0] apu_write_regs_valid;
	wire apu_write_dep;
	wire perf_apu_type;
	wire perf_apu_cont;
	wire perf_apu_dep;
	wire perf_apu_wb;
	wire [5:0] regfile_waddr_ex;
	wire regfile_we_ex;
	wire [5:0] regfile_waddr_fw_wb_o;
	wire regfile_we_wb;
	wire regfile_we_wb_power;
	wire [31:0] regfile_wdata;
	wire [5:0] regfile_alu_waddr_ex;
	wire regfile_alu_we_ex;
	wire [5:0] regfile_alu_waddr_fw;
	wire regfile_alu_we_fw;
	wire regfile_alu_we_fw_power;
	wire [31:0] regfile_alu_wdata_fw;
	wire csr_access_ex;
	localparam cv32e40p_pkg_CSR_OP_WIDTH = 2;
	wire [1:0] csr_op_ex;
	wire [23:0] mtvec;
	wire [23:0] utvec;
	wire [1:0] mtvec_mode;
	wire [1:0] utvec_mode;
	wire [1:0] csr_op;
	wire [11:0] csr_addr;
	wire [11:0] csr_addr_int;
	wire [31:0] csr_rdata;
	wire [31:0] csr_wdata;
	wire [1:0] current_priv_lvl;
	wire data_we_ex;
	wire [5:0] data_atop_ex;
	wire [1:0] data_type_ex;
	wire [1:0] data_sign_ext_ex;
	wire [1:0] data_reg_offset_ex;
	wire data_req_ex;
	wire data_load_event_ex;
	wire data_misaligned_ex;
	wire p_elw_start;
	wire p_elw_finish;
	wire [31:0] lsu_rdata;
	wire halt_if;
	wire id_ready;
	wire ex_ready;
	wire id_valid;
	wire ex_valid;
	wire wb_valid;
	wire lsu_ready_ex;
	wire lsu_ready_wb;
	wire apu_ready_wb;
	wire instr_req_int;
	wire m_irq_enable;
	wire u_irq_enable;
	wire csr_irq_sec;
	wire [31:0] mepc;
	wire [31:0] uepc;
	wire [31:0] depc;
	wire [31:0] mie_bypass;
	wire [31:0] mip;
	wire csr_save_cause;
	wire csr_save_if;
	wire csr_save_id;
	wire csr_save_ex;
	wire [5:0] csr_cause;
	wire csr_restore_mret_id;
	wire csr_restore_uret_id;
	wire csr_restore_dret_id;
	wire csr_mtvec_init;
	wire [31:0] mcounteren;
	wire debug_mode;
	wire [2:0] debug_cause;
	wire debug_csr_save;
	wire debug_single_step;
	wire debug_ebreakm;
	wire debug_ebreaku;
	wire trigger_match;
	wire debug_p_elw_no_sleep;
	wire [63:0] hwlp_start;
	wire [63:0] hwlp_end;
	wire [63:0] hwlp_cnt;
	wire [31:0] hwlp_target;
	wire hwlp_jump;
	wire mhpmevent_minstret;
	wire mhpmevent_load;
	wire mhpmevent_store;
	wire mhpmevent_jump;
	wire mhpmevent_branch;
	wire mhpmevent_branch_taken;
	wire mhpmevent_compressed;
	wire mhpmevent_jr_stall;
	wire mhpmevent_imiss;
	wire mhpmevent_ld_stall;
	wire mhpmevent_pipe_stall;
	wire perf_imiss;
	wire wake_from_sleep;
	wire [511:0] pmp_addr;
	wire [127:0] pmp_cfg;
	wire data_req_pmp;
	wire [31:0] data_addr_pmp;
	wire data_gnt_pmp;
	wire data_err_pmp;
	wire data_err_ack;
	wire instr_req_pmp;
	wire instr_gnt_pmp;
	wire [31:0] instr_addr_pmp;
	wire instr_err_pmp;
	assign m_exc_vec_pc_mux_id = (mtvec_mode == 2'b00 ? 5'h00 : exc_cause);
	assign u_exc_vec_pc_mux_id = (utvec_mode == 2'b00 ? 5'h00 : exc_cause);
	assign irq_sec_i = 1'b0;
	assign apu_flags_o = apu_flags_ex;
	wire clk;
	wire fetch_enable;
	cv32e40p_sleep_unit #(.COREV_CLUSTER(COREV_CLUSTER)) sleep_unit_i(
		.clk_ungated_i(clk_i),
		.rst_n(rst_ni),
		.clk_gated_o(clk),
		.scan_cg_en_i(scan_cg_en_i),
		.core_sleep_o(core_sleep_o),
		.fetch_enable_i(fetch_enable_i),
		.fetch_enable_o(fetch_enable),
		.if_busy_i(if_busy),
		.ctrl_busy_i(ctrl_busy),
		.lsu_busy_i(lsu_busy),
		.apu_busy_i(apu_busy),
		.pulp_clock_en_i(pulp_clock_en_i),
		.p_elw_start_i(p_elw_start),
		.p_elw_finish_i(p_elw_finish),
		.debug_p_elw_no_sleep_i(debug_p_elw_no_sleep),
		.wake_from_sleep_i(wake_from_sleep)
	);
	cv32e40p_if_stage #(
		.COREV_PULP(COREV_PULP),
		.PULP_OBI(PULP_OBI),
		.PULP_SECURE(PULP_SECURE),
		.FPU(FPU),
		.ZFINX(ZFINX)
	) if_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.boot_addr_i(boot_addr_i[31:0]),
		.dm_exception_addr_i(dm_exception_addr_i[31:0]),
		.dm_halt_addr_i(dm_halt_addr_i[31:0]),
		.m_trap_base_addr_i(mtvec),
		.u_trap_base_addr_i(utvec),
		.trap_addr_mux_i(trap_addr_mux),
		.req_i(instr_req_int),
		.instr_req_o(instr_req_pmp),
		.instr_addr_o(instr_addr_pmp),
		.instr_gnt_i(instr_gnt_pmp),
		.instr_rvalid_i(instr_rvalid_i),
		.instr_rdata_i(instr_rdata_i),
		.instr_err_i(1'b0),
		.instr_err_pmp_i(instr_err_pmp),
		.instr_valid_id_o(instr_valid_id),
		.instr_rdata_id_o(instr_rdata_id),
		.is_fetch_failed_o(is_fetch_failed_id),
		.clear_instr_valid_i(clear_instr_valid),
		.pc_set_i(pc_set),
		.mepc_i(mepc),
		.uepc_i(uepc),
		.depc_i(depc),
		.pc_mux_i(pc_mux_id),
		.exc_pc_mux_i(exc_pc_mux_id),
		.pc_id_o(pc_id),
		.pc_if_o(pc_if),
		.is_compressed_id_o(is_compressed_id),
		.illegal_c_insn_id_o(illegal_c_insn_id),
		.m_exc_vec_pc_mux_i(m_exc_vec_pc_mux_id),
		.u_exc_vec_pc_mux_i(u_exc_vec_pc_mux_id),
		.csr_mtvec_init_o(csr_mtvec_init),
		.hwlp_jump_i(hwlp_jump),
		.hwlp_target_i(hwlp_target),
		.jump_target_id_i(jump_target_id),
		.jump_target_ex_i(jump_target_ex),
		.halt_if_i(halt_if),
		.id_ready_i(id_ready),
		.if_busy_o(if_busy),
		.perf_imiss_o(perf_imiss)
	);
	cv32e40p_id_stage #(
		.COREV_PULP(COREV_PULP),
		.COREV_CLUSTER(COREV_CLUSTER),
		.N_HWLP(N_HWLP),
		.PULP_SECURE(PULP_SECURE),
		.USE_PMP(USE_PMP),
		.A_EXTENSION(A_EXTENSION),
		.APU(APU),
		.FPU(FPU),
		.FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
		.FPU_OTHERS_LAT(FPU_OTHERS_LAT),
		.ZFINX(ZFINX),
		.APU_NARGS_CPU(cv32e40p_apu_core_pkg_APU_NARGS_CPU),
		.APU_WOP_CPU(cv32e40p_apu_core_pkg_APU_WOP_CPU),
		.APU_NDSFLAGS_CPU(cv32e40p_apu_core_pkg_APU_NDSFLAGS_CPU),
		.APU_NUSFLAGS_CPU(cv32e40p_apu_core_pkg_APU_NUSFLAGS_CPU),
		.DEBUG_TRIGGER_EN(DEBUG_TRIGGER_EN)
	) id_stage_i(
		.clk(clk),
		.clk_ungated_i(clk_i),
		.rst_n(rst_ni),
		.scan_cg_en_i(scan_cg_en_i),
		.fetch_enable_i(fetch_enable),
		.ctrl_busy_o(ctrl_busy),
		.is_decoding_o(is_decoding),
		.instr_valid_i(instr_valid_id),
		.instr_rdata_i(instr_rdata_id),
		.instr_req_o(instr_req_int),
		.branch_in_ex_o(branch_in_ex),
		.branch_decision_i(branch_decision),
		.jump_target_o(jump_target_id),
		.ctrl_transfer_insn_in_dec_o(ctrl_transfer_insn_in_dec),
		.clear_instr_valid_o(clear_instr_valid),
		.pc_set_o(pc_set),
		.pc_mux_o(pc_mux_id),
		.exc_pc_mux_o(exc_pc_mux_id),
		.exc_cause_o(exc_cause),
		.trap_addr_mux_o(trap_addr_mux),
		.is_fetch_failed_i(is_fetch_failed_id),
		.pc_id_i(pc_id),
		.is_compressed_i(is_compressed_id),
		.illegal_c_insn_i(illegal_c_insn_id),
		.halt_if_o(halt_if),
		.id_ready_o(id_ready),
		.ex_ready_i(ex_ready),
		.wb_ready_i(lsu_ready_wb),
		.id_valid_o(id_valid),
		.ex_valid_i(ex_valid),
		.pc_ex_o(pc_ex),
		.alu_en_ex_o(alu_en_ex),
		.alu_operator_ex_o(alu_operator_ex),
		.alu_operand_a_ex_o(alu_operand_a_ex),
		.alu_operand_b_ex_o(alu_operand_b_ex),
		.alu_operand_c_ex_o(alu_operand_c_ex),
		.bmask_a_ex_o(bmask_a_ex),
		.bmask_b_ex_o(bmask_b_ex),
		.imm_vec_ext_ex_o(imm_vec_ext_ex),
		.alu_vec_mode_ex_o(alu_vec_mode_ex),
		.alu_is_clpx_ex_o(alu_is_clpx_ex),
		.alu_is_subrot_ex_o(alu_is_subrot_ex),
		.alu_clpx_shift_ex_o(alu_clpx_shift_ex),
		.regfile_waddr_ex_o(regfile_waddr_ex),
		.regfile_we_ex_o(regfile_we_ex),
		.regfile_alu_we_ex_o(regfile_alu_we_ex),
		.regfile_alu_waddr_ex_o(regfile_alu_waddr_ex),
		.mult_operator_ex_o(mult_operator_ex),
		.mult_en_ex_o(mult_en_ex),
		.mult_sel_subword_ex_o(mult_sel_subword_ex),
		.mult_signed_mode_ex_o(mult_signed_mode_ex),
		.mult_operand_a_ex_o(mult_operand_a_ex),
		.mult_operand_b_ex_o(mult_operand_b_ex),
		.mult_operand_c_ex_o(mult_operand_c_ex),
		.mult_imm_ex_o(mult_imm_ex),
		.mult_dot_op_a_ex_o(mult_dot_op_a_ex),
		.mult_dot_op_b_ex_o(mult_dot_op_b_ex),
		.mult_dot_op_c_ex_o(mult_dot_op_c_ex),
		.mult_dot_signed_ex_o(mult_dot_signed_ex),
		.mult_is_clpx_ex_o(mult_is_clpx_ex),
		.mult_clpx_shift_ex_o(mult_clpx_shift_ex),
		.mult_clpx_img_ex_o(mult_clpx_img_ex),
		.fs_off_i(fs_off),
		.frm_i(frm_csr),
		.apu_en_ex_o(apu_en_ex),
		.apu_op_ex_o(apu_op_ex),
		.apu_lat_ex_o(apu_lat_ex),
		.apu_operands_ex_o(apu_operands_ex),
		.apu_flags_ex_o(apu_flags_ex),
		.apu_waddr_ex_o(apu_waddr_ex),
		.apu_read_regs_o(apu_read_regs),
		.apu_read_regs_valid_o(apu_read_regs_valid),
		.apu_read_dep_i(apu_read_dep),
		.apu_read_dep_for_jalr_i(apu_read_dep_for_jalr),
		.apu_write_regs_o(apu_write_regs),
		.apu_write_regs_valid_o(apu_write_regs_valid),
		.apu_write_dep_i(apu_write_dep),
		.apu_perf_dep_o(perf_apu_dep),
		.apu_busy_i(apu_busy),
		.csr_access_ex_o(csr_access_ex),
		.csr_op_ex_o(csr_op_ex),
		.current_priv_lvl_i(current_priv_lvl),
		.csr_irq_sec_o(csr_irq_sec),
		.csr_cause_o(csr_cause),
		.csr_save_if_o(csr_save_if),
		.csr_save_id_o(csr_save_id),
		.csr_save_ex_o(csr_save_ex),
		.csr_restore_mret_id_o(csr_restore_mret_id),
		.csr_restore_uret_id_o(csr_restore_uret_id),
		.csr_restore_dret_id_o(csr_restore_dret_id),
		.csr_save_cause_o(csr_save_cause),
		.hwlp_start_o(hwlp_start),
		.hwlp_end_o(hwlp_end),
		.hwlp_cnt_o(hwlp_cnt),
		.hwlp_jump_o(hwlp_jump),
		.hwlp_target_o(hwlp_target),
		.data_req_ex_o(data_req_ex),
		.data_we_ex_o(data_we_ex),
		.atop_ex_o(data_atop_ex),
		.data_type_ex_o(data_type_ex),
		.data_sign_ext_ex_o(data_sign_ext_ex),
		.data_reg_offset_ex_o(data_reg_offset_ex),
		.data_load_event_ex_o(data_load_event_ex),
		.data_misaligned_ex_o(data_misaligned_ex),
		.prepost_useincr_ex_o(useincr_addr_ex),
		.data_misaligned_i(data_misaligned),
		.data_err_i(data_err_pmp),
		.data_err_ack_o(data_err_ack),
		.irq_i(irq_i),
		.irq_sec_i((PULP_SECURE ? irq_sec_i : 1'b0)),
		.mie_bypass_i(mie_bypass),
		.mip_o(mip),
		.m_irq_enable_i(m_irq_enable),
		.u_irq_enable_i(u_irq_enable),
		.irq_ack_o(irq_ack_o),
		.irq_id_o(irq_id_o),
		.debug_mode_o(debug_mode),
		.debug_cause_o(debug_cause),
		.debug_csr_save_o(debug_csr_save),
		.debug_req_i(debug_req_i),
		.debug_havereset_o(debug_havereset_o),
		.debug_running_o(debug_running_o),
		.debug_halted_o(debug_halted_o),
		.debug_single_step_i(debug_single_step),
		.debug_ebreakm_i(debug_ebreakm),
		.debug_ebreaku_i(debug_ebreaku),
		.trigger_match_i(trigger_match),
		.debug_p_elw_no_sleep_o(debug_p_elw_no_sleep),
		.wake_from_sleep_o(wake_from_sleep),
		.regfile_waddr_wb_i(regfile_waddr_fw_wb_o),
		.regfile_we_wb_i(regfile_we_wb),
		.regfile_we_wb_power_i(regfile_we_wb_power),
		.regfile_wdata_wb_i(regfile_wdata),
		.regfile_alu_waddr_fw_i(regfile_alu_waddr_fw),
		.regfile_alu_we_fw_i(regfile_alu_we_fw),
		.regfile_alu_we_fw_power_i(regfile_alu_we_fw_power),
		.regfile_alu_wdata_fw_i(regfile_alu_wdata_fw),
		.mult_multicycle_i(mult_multicycle),
		.mhpmevent_minstret_o(mhpmevent_minstret),
		.mhpmevent_load_o(mhpmevent_load),
		.mhpmevent_store_o(mhpmevent_store),
		.mhpmevent_jump_o(mhpmevent_jump),
		.mhpmevent_branch_o(mhpmevent_branch),
		.mhpmevent_branch_taken_o(mhpmevent_branch_taken),
		.mhpmevent_compressed_o(mhpmevent_compressed),
		.mhpmevent_jr_stall_o(mhpmevent_jr_stall),
		.mhpmevent_imiss_o(mhpmevent_imiss),
		.mhpmevent_ld_stall_o(mhpmevent_ld_stall),
		.mhpmevent_pipe_stall_o(mhpmevent_pipe_stall),
		.perf_imiss_i(perf_imiss),
		.mcounteren_i(mcounteren)
	);
	cv32e40p_ex_stage #(
		.COREV_PULP(COREV_PULP),
		.FPU(FPU),
		.APU_NARGS_CPU(cv32e40p_apu_core_pkg_APU_NARGS_CPU),
		.APU_WOP_CPU(cv32e40p_apu_core_pkg_APU_WOP_CPU),
		.APU_NDSFLAGS_CPU(cv32e40p_apu_core_pkg_APU_NDSFLAGS_CPU),
		.APU_NUSFLAGS_CPU(cv32e40p_apu_core_pkg_APU_NUSFLAGS_CPU)
	) ex_stage_i(
		.clk(clk),
		.rst_n(rst_ni),
		.alu_en_i(alu_en_ex),
		.alu_operator_i(alu_operator_ex),
		.alu_operand_a_i(alu_operand_a_ex),
		.alu_operand_b_i(alu_operand_b_ex),
		.alu_operand_c_i(alu_operand_c_ex),
		.bmask_a_i(bmask_a_ex),
		.bmask_b_i(bmask_b_ex),
		.imm_vec_ext_i(imm_vec_ext_ex),
		.alu_vec_mode_i(alu_vec_mode_ex),
		.alu_is_clpx_i(alu_is_clpx_ex),
		.alu_is_subrot_i(alu_is_subrot_ex),
		.alu_clpx_shift_i(alu_clpx_shift_ex),
		.mult_operator_i(mult_operator_ex),
		.mult_operand_a_i(mult_operand_a_ex),
		.mult_operand_b_i(mult_operand_b_ex),
		.mult_operand_c_i(mult_operand_c_ex),
		.mult_en_i(mult_en_ex),
		.mult_sel_subword_i(mult_sel_subword_ex),
		.mult_signed_mode_i(mult_signed_mode_ex),
		.mult_imm_i(mult_imm_ex),
		.mult_dot_op_a_i(mult_dot_op_a_ex),
		.mult_dot_op_b_i(mult_dot_op_b_ex),
		.mult_dot_op_c_i(mult_dot_op_c_ex),
		.mult_dot_signed_i(mult_dot_signed_ex),
		.mult_is_clpx_i(mult_is_clpx_ex),
		.mult_clpx_shift_i(mult_clpx_shift_ex),
		.mult_clpx_img_i(mult_clpx_img_ex),
		.mult_multicycle_o(mult_multicycle),
		.data_req_i(data_req_o),
		.data_rvalid_i(data_rvalid_i),
		.data_misaligned_ex_i(data_misaligned_ex),
		.data_misaligned_i(data_misaligned),
		.ctrl_transfer_insn_in_dec_i(ctrl_transfer_insn_in_dec),
		.fpu_fflags_we_o(fflags_we),
		.fpu_fflags_o(fflags_csr),
		.apu_en_i(apu_en_ex),
		.apu_op_i(apu_op_ex),
		.apu_lat_i(apu_lat_ex),
		.apu_operands_i(apu_operands_ex),
		.apu_waddr_i(apu_waddr_ex),
		.apu_read_regs_i(apu_read_regs),
		.apu_read_regs_valid_i(apu_read_regs_valid),
		.apu_read_dep_o(apu_read_dep),
		.apu_read_dep_for_jalr_o(apu_read_dep_for_jalr),
		.apu_write_regs_i(apu_write_regs),
		.apu_write_regs_valid_i(apu_write_regs_valid),
		.apu_write_dep_o(apu_write_dep),
		.apu_perf_type_o(perf_apu_type),
		.apu_perf_cont_o(perf_apu_cont),
		.apu_perf_wb_o(perf_apu_wb),
		.apu_ready_wb_o(apu_ready_wb),
		.apu_busy_o(apu_busy),
		.apu_req_o(apu_req_o),
		.apu_gnt_i(apu_gnt_i),
		.apu_operands_o(apu_operands_o),
		.apu_op_o(apu_op_o),
		.apu_rvalid_i(apu_rvalid_i),
		.apu_result_i(apu_result_i),
		.apu_flags_i(apu_flags_i),
		.lsu_en_i(data_req_ex),
		.lsu_rdata_i(lsu_rdata),
		.csr_access_i(csr_access_ex),
		.csr_rdata_i(csr_rdata),
		.branch_in_ex_i(branch_in_ex),
		.regfile_alu_waddr_i(regfile_alu_waddr_ex),
		.regfile_alu_we_i(regfile_alu_we_ex),
		.regfile_waddr_i(regfile_waddr_ex),
		.regfile_we_i(regfile_we_ex),
		.regfile_waddr_wb_o(regfile_waddr_fw_wb_o),
		.regfile_we_wb_o(regfile_we_wb),
		.regfile_we_wb_power_o(regfile_we_wb_power),
		.regfile_wdata_wb_o(regfile_wdata),
		.jump_target_o(jump_target_ex),
		.branch_decision_o(branch_decision),
		.regfile_alu_waddr_fw_o(regfile_alu_waddr_fw),
		.regfile_alu_we_fw_o(regfile_alu_we_fw),
		.regfile_alu_we_fw_power_o(regfile_alu_we_fw_power),
		.regfile_alu_wdata_fw_o(regfile_alu_wdata_fw),
		.is_decoding_i(is_decoding),
		.lsu_ready_ex_i(lsu_ready_ex),
		.lsu_err_i(data_err_pmp),
		.ex_ready_o(ex_ready),
		.ex_valid_o(ex_valid),
		.wb_ready_i(lsu_ready_wb)
	);
	cv32e40p_load_store_unit #(.PULP_OBI(PULP_OBI)) load_store_unit_i(
		.clk(clk),
		.rst_n(rst_ni),
		.data_req_o(data_req_pmp),
		.data_gnt_i(data_gnt_pmp),
		.data_rvalid_i(data_rvalid_i),
		.data_err_i(1'b0),
		.data_err_pmp_i(data_err_pmp),
		.data_addr_o(data_addr_pmp),
		.data_we_o(data_we_o),
		.data_atop_o(data_atop_o),
		.data_be_o(data_be_o),
		.data_wdata_o(data_wdata_o),
		.data_rdata_i(data_rdata_i),
		.data_we_ex_i(data_we_ex),
		.data_atop_ex_i(data_atop_ex),
		.data_type_ex_i(data_type_ex),
		.data_wdata_ex_i(alu_operand_c_ex),
		.data_reg_offset_ex_i(data_reg_offset_ex),
		.data_load_event_ex_i(data_load_event_ex),
		.data_sign_ext_ex_i(data_sign_ext_ex),
		.data_rdata_ex_o(lsu_rdata),
		.data_req_ex_i(data_req_ex),
		.operand_a_ex_i(alu_operand_a_ex),
		.operand_b_ex_i(alu_operand_b_ex),
		.addr_useincr_ex_i(useincr_addr_ex),
		.data_misaligned_ex_i(data_misaligned_ex),
		.data_misaligned_o(data_misaligned),
		.p_elw_start_o(p_elw_start),
		.p_elw_finish_o(p_elw_finish),
		.lsu_ready_ex_o(lsu_ready_ex),
		.lsu_ready_wb_o(lsu_ready_wb),
		.busy_o(lsu_busy)
	);
	assign wb_valid = lsu_ready_wb;
	cv32e40p_cs_registers #(
		.N_HWLP(N_HWLP),
		.A_EXTENSION(A_EXTENSION),
		.FPU(FPU),
		.ZFINX(ZFINX),
		.APU(APU),
		.PULP_SECURE(PULP_SECURE),
		.USE_PMP(USE_PMP),
		.N_PMP_ENTRIES(N_PMP_ENTRIES),
		.NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS),
		.COREV_PULP(COREV_PULP),
		.COREV_CLUSTER(COREV_CLUSTER),
		.DEBUG_TRIGGER_EN(DEBUG_TRIGGER_EN)
	) cs_registers_i(
		.clk(clk),
		.rst_n(rst_ni),
		.hart_id_i(hart_id_i),
		.mtvec_o(mtvec),
		.utvec_o(utvec),
		.mtvec_mode_o(mtvec_mode),
		.utvec_mode_o(utvec_mode),
		.mtvec_addr_i(mtvec_addr_i[31:0]),
		.csr_mtvec_init_i(csr_mtvec_init),
		.csr_addr_i(csr_addr),
		.csr_wdata_i(csr_wdata),
		.csr_op_i(csr_op),
		.csr_rdata_o(csr_rdata),
		.fs_off_o(fs_off),
		.frm_o(frm_csr),
		.fflags_i(fflags_csr),
		.fflags_we_i(fflags_we),
		.fregs_we_i(fregs_we),
		.mie_bypass_o(mie_bypass),
		.mip_i(mip),
		.m_irq_enable_o(m_irq_enable),
		.u_irq_enable_o(u_irq_enable),
		.csr_irq_sec_i(csr_irq_sec),
		.sec_lvl_o(sec_lvl_o),
		.mepc_o(mepc),
		.uepc_o(uepc),
		.mcounteren_o(mcounteren),
		.debug_mode_i(debug_mode),
		.debug_cause_i(debug_cause),
		.debug_csr_save_i(debug_csr_save),
		.depc_o(depc),
		.debug_single_step_o(debug_single_step),
		.debug_ebreakm_o(debug_ebreakm),
		.debug_ebreaku_o(debug_ebreaku),
		.trigger_match_o(trigger_match),
		.priv_lvl_o(current_priv_lvl),
		.pmp_addr_o(pmp_addr),
		.pmp_cfg_o(pmp_cfg),
		.pc_if_i(pc_if),
		.pc_id_i(pc_id),
		.pc_ex_i(pc_ex),
		.csr_save_if_i(csr_save_if),
		.csr_save_id_i(csr_save_id),
		.csr_save_ex_i(csr_save_ex),
		.csr_restore_mret_i(csr_restore_mret_id),
		.csr_restore_uret_i(csr_restore_uret_id),
		.csr_restore_dret_i(csr_restore_dret_id),
		.csr_cause_i(csr_cause),
		.csr_save_cause_i(csr_save_cause),
		.hwlp_start_i(hwlp_start),
		.hwlp_end_i(hwlp_end),
		.hwlp_cnt_i(hwlp_cnt),
		.mhpmevent_minstret_i(mhpmevent_minstret),
		.mhpmevent_load_i(mhpmevent_load),
		.mhpmevent_store_i(mhpmevent_store),
		.mhpmevent_jump_i(mhpmevent_jump),
		.mhpmevent_branch_i(mhpmevent_branch),
		.mhpmevent_branch_taken_i(mhpmevent_branch_taken),
		.mhpmevent_compressed_i(mhpmevent_compressed),
		.mhpmevent_jr_stall_i(mhpmevent_jr_stall),
		.mhpmevent_imiss_i(mhpmevent_imiss),
		.mhpmevent_ld_stall_i(mhpmevent_ld_stall),
		.mhpmevent_pipe_stall_i(mhpmevent_pipe_stall),
		.apu_typeconflict_i(perf_apu_type),
		.apu_contention_i(perf_apu_cont),
		.apu_dep_i(perf_apu_dep),
		.apu_wb_i(perf_apu_wb)
	);
	assign csr_addr = csr_addr_int;
	assign csr_wdata = alu_operand_a_ex;
	assign csr_op = csr_op_ex;
	function automatic [11:0] sv2v_cast_12;
		input reg [11:0] inp;
		sv2v_cast_12 = inp;
	endfunction
	assign csr_addr_int = sv2v_cast_12((csr_access_ex ? alu_operand_b_ex[11:0] : {12 {1'sb0}}));
	assign fregs_we = (FPU & !ZFINX ? (regfile_alu_we_fw && regfile_alu_waddr_fw[5]) || (regfile_we_wb && regfile_waddr_fw_wb_o[5]) : 1'b0);
	generate
		if (1) begin : gen_no_pmp
			assign instr_req_o = instr_req_pmp;
			assign instr_addr_o = instr_addr_pmp;
			assign instr_gnt_pmp = instr_gnt_i;
			assign instr_err_pmp = 1'b0;
			assign data_req_o = data_req_pmp;
			assign data_addr_o = data_addr_pmp;
			assign data_gnt_pmp = data_gnt_i;
			assign data_err_pmp = 1'b0;
		end
	endgenerate
endmodule
module cv32e40p_clock_gate (
	clk_i,
	en_i,
	scan_cg_en_i,
	clk_o
);
	input wire clk_i;
	input wire en_i;
	input wire scan_cg_en_i;
	output wire clk_o;
	assign clk_o = clk_i;
endmodule
module cv32e40p_top (
	clk_i,
	rst_ni,
	pulp_clock_en_i,
	scan_cg_en_i,
	boot_addr_i,
	mtvec_addr_i,
	dm_halt_addr_i,
	hart_id_i,
	dm_exception_addr_i,
	instr_req_o,
	instr_gnt_i,
	instr_rvalid_i,
	instr_addr_o,
	instr_rdata_i,
	data_req_o,
	data_gnt_i,
	data_rvalid_i,
	data_we_o,
	data_be_o,
	data_addr_o,
	data_wdata_o,
	data_rdata_i,
	irq_i,
	irq_ack_o,
	irq_id_o,
	debug_req_i,
	debug_havereset_o,
	debug_running_o,
	debug_halted_o,
	fetch_enable_i,
	core_sleep_o
);
	parameter COREV_PULP = 0;
	parameter COREV_CLUSTER = 0;
	parameter FPU = 0;
	parameter FPU_ADDMUL_LAT = 0;
	parameter FPU_OTHERS_LAT = 0;
	parameter ZFINX = 0;
	parameter NUM_MHPMCOUNTERS = 1;
	input wire clk_i;
	input wire rst_ni;
	input wire pulp_clock_en_i;
	input wire scan_cg_en_i;
	input wire [31:0] boot_addr_i;
	input wire [31:0] mtvec_addr_i;
	input wire [31:0] dm_halt_addr_i;
	input wire [31:0] hart_id_i;
	input wire [31:0] dm_exception_addr_i;
	output wire instr_req_o;
	input wire instr_gnt_i;
	input wire instr_rvalid_i;
	output wire [31:0] instr_addr_o;
	input wire [31:0] instr_rdata_i;
	output wire data_req_o;
	input wire data_gnt_i;
	input wire data_rvalid_i;
	output wire data_we_o;
	output wire [3:0] data_be_o;
	output wire [31:0] data_addr_o;
	output wire [31:0] data_wdata_o;
	input wire [31:0] data_rdata_i;
	input wire [31:0] irq_i;
	output wire irq_ack_o;
	output wire [4:0] irq_id_o;
	input wire debug_req_i;
	output wire debug_havereset_o;
	output wire debug_running_o;
	output wire debug_halted_o;
	input wire fetch_enable_i;
	output wire core_sleep_o;
	wire clk;
	wire apu_req;
	localparam cv32e40p_apu_core_pkg_APU_NARGS_CPU = 3;
	wire [95:0] apu_operands;
	localparam cv32e40p_apu_core_pkg_APU_WOP_CPU = 6;
	wire [5:0] apu_op;
	localparam cv32e40p_apu_core_pkg_APU_NDSFLAGS_CPU = 15;
	wire [14:0] apu_flags;
	wire apu_gnt;
	wire apu_rvalid;
	wire [31:0] apu_rdata;
	localparam cv32e40p_apu_core_pkg_APU_NUSFLAGS_CPU = 5;
	wire [4:0] apu_rflags;
	cv32e40p_core #(
		.COREV_PULP(COREV_PULP),
		.COREV_CLUSTER(COREV_CLUSTER),
		.FPU(FPU),
		.FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
		.FPU_OTHERS_LAT(FPU_OTHERS_LAT),
		.ZFINX(ZFINX),
		.NUM_MHPMCOUNTERS(NUM_MHPMCOUNTERS)
	) core_i(
		.clk_i(clk_i),
		.rst_ni(rst_ni),
		.pulp_clock_en_i(pulp_clock_en_i),
		.scan_cg_en_i(scan_cg_en_i),
		.boot_addr_i(boot_addr_i),
		.mtvec_addr_i(mtvec_addr_i),
		.dm_halt_addr_i(dm_halt_addr_i),
		.hart_id_i(hart_id_i),
		.dm_exception_addr_i(dm_exception_addr_i),
		.instr_req_o(instr_req_o),
		.instr_gnt_i(instr_gnt_i),
		.instr_rvalid_i(instr_rvalid_i),
		.instr_addr_o(instr_addr_o),
		.instr_rdata_i(instr_rdata_i),
		.data_req_o(data_req_o),
		.data_gnt_i(data_gnt_i),
		.data_rvalid_i(data_rvalid_i),
		.data_we_o(data_we_o),
		.data_be_o(data_be_o),
		.data_addr_o(data_addr_o),
		.data_wdata_o(data_wdata_o),
		.data_rdata_i(data_rdata_i),
		.apu_req_o(apu_req),
		.apu_gnt_i(apu_gnt),
		.apu_operands_o(apu_operands),
		.apu_op_o(apu_op),
		.apu_flags_o(apu_flags),
		.apu_rvalid_i(apu_rvalid),
		.apu_result_i(apu_rdata),
		.apu_flags_i(apu_rflags),
		.irq_i(irq_i),
		.irq_ack_o(irq_ack_o),
		.irq_id_o(irq_id_o),
		.debug_req_i(debug_req_i),
		.debug_havereset_o(debug_havereset_o),
		.debug_running_o(debug_running_o),
		.debug_halted_o(debug_halted_o),
		.fetch_enable_i(fetch_enable_i),
		.core_sleep_o(core_sleep_o)
	);
	generate
		if (FPU) begin : fpu_gen
			cv32e40p_clock_gate core_clock_gate_i(
				.clk_i(clk_i),
				.en_i(!core_sleep_o),
				.scan_cg_en_i(scan_cg_en_i),
				.clk_o(clk)
			);
			cv32e40p_fp_wrapper #(
				.FPU_ADDMUL_LAT(FPU_ADDMUL_LAT),
				.FPU_OTHERS_LAT(FPU_OTHERS_LAT)
			) fp_wrapper_i(
				.clk_i(clk),
				.rst_ni(rst_ni),
				.apu_req_i(apu_req),
				.apu_gnt_o(apu_gnt),
				.apu_operands_i(apu_operands),
				.apu_op_i(apu_op),
				.apu_flags_i(apu_flags),
				.apu_rvalid_o(apu_rvalid),
				.apu_rdata_o(apu_rdata),
				.apu_rflags_o(apu_rflags)
			);
		end
		else begin : no_fpu_gen
			assign apu_gnt = 1'sb0;
			assign apu_rvalid = 1'sb0;
			assign apu_rdata = 1'sb0;
			assign apu_rflags = 1'sb0;
		end
	endgenerate
endmodule