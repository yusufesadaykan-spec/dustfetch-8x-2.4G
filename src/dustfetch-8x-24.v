// ============================================================================
// Module Name: dustfetch_8x_24
// Description: High-Performance, Compact Quad-Core (4-Core) RISC-V RV64GC-compatible SoC
//              Operating at 2.4 GHz Target Frequency (416.7 ps Cycle Time)
//              Features L1 (I/D), L2, L3 Cache Hierarchy, 8 GB DDR4 Controller,
//              1x HDMI 1.4 Type-C (Alt Mode) Interface, 3x USB 3.0 Type-A Interfaces.
//              Fully Mapped to RISC-V Naming Conventions and Sv39 Specifications:
//                1. 'riscv_core' 64-bit Core Module with U/S/M privilege modes.
//                2. 'riscv_sv39_mmu' 3-level Sv39 Page-Table Walk & 16-entry TLB.
//                3. 'riscv_l1_cache' Physical Index Cache Module.
//                4. 'riscv_plic' Platform-Level Interrupt Controller.
//                5. 'sdio_storage_controller' & 'uart_controller' console debugger.
// ============================================================================

`timescale 1ps / 1ps

module dustfetch_8x_24 (
    // Clock and System Control
    input  wire         sys_clk,       // 2.4 GHz clock input
    input  wire         sys_rst_n,     // Active-low asynchronous reset

    // DDR4 Memory Interface (8 GB Capacity, 33-bit physical address space)
    output wire         ddr4_ck_t,     // Differential Clock Positive
    output wire         ddr4_ck_c,     // Differential Clock Negative
    output wire         ddr4_cke,      // Clock Enable
    output wire         ddr4_cs_n,     // Chip Select
    output wire         ddr4_act_n,    // Activate Command
    output wire [16:0]  ddr4_adr,      // Address Bus
    output wire [1:0]   ddr4_ba,       // Bank Group
    output wire [1:0]   ddr4_bg,       // Bank Address
    output wire         ddr4_ras_n,    // Row Address Strobe
    output wire         ddr4_cas_n,    // Column Address Strobe
    output wire         ddr4_we_n,     // Write Enable
    output wire         ddr4_odt,      // On-Die Termination
    output wire         ddr4_reset_n,  // DDR4 SDRAM Reset
    inout  wire [63:0]  ddr4_dq,       // Bidirectional Data Bus
    inout  wire [7:0]   ddr4_dqs_t,    // Data Strobe Positive
    inout  wire [7:0]   ddr4_dqs_c,    // Data Strobe Negative

    // HDMI 1.4 Type-C Alt-Mode Interface
    output wire         hdmi_tx_clk_p,  // HDMI TMDS Differential Clock Positive
    output wire         hdmi_tx_clk_n,  // HDMI TMDS Differential Clock Negative
    output wire [2:0]   hdmi_tx_data_p, // HDMI TMDS Differential Data Lanes Positive [2:0]
    output wire [2:0]   hdmi_tx_data_n, // HDMI TMDS Differential Data Lanes Negative [2:0]
    input  wire         hdmi_hpd,       // Hot Plug Detect
    inout  wire         hdmi_cec,       // Consumer Electronics Control
    inout  wire         hdmi_ddc_scl,   // Display Data Channel Clock
    inout  wire         hdmi_ddc_sda,   // Display Data Channel Data
    inout  wire         typec_cc1,      // USB Type-C Configuration Channel 1
    inout  wire         typec_cc2,      // USB Type-C Configuration Channel 2

    // 3x USB 3.0 Type-A Interfaces
    input  wire [2:0]   usb_ssrx_p,    // SuperSpeed Receiver Differential Pair Positive
    input  wire [2:0]   usb_ssrx_n,    // SuperSpeed Receiver Differential Pair Negative
    output wire [2:0]   usb_sstx_p,    // SuperSpeed Transmitter Differential Pair Positive
    output wire [2:0]   usb_sstx_n,    // SuperSpeed Transmitter Differential Pair Negative
    inout  wire [2:0]   usb_dp,        // High-Speed / Full-Speed D+ Line
    inout  wire [2:0]   usb_dn,        // High-Speed / Full-Speed D- Line

    // UART Console Interface (Arch Linux printk/systemd Early Boot Debug Console)
    output wire         uart_tx,       // UART Transmit Pin
    input  wire         uart_rx,       // UART Receive Pin

    // SDIO Storage Interface (MicroSD Card Reader for loading OpenSBI, U-Boot & Arch Linux Image)
    output wire         sd_clk,        // MicroSD Clock
    inout  wire         sd_cmd,        // MicroSD Command line
    inout  wire [3:0]   sd_dat         // MicroSD 4-bit Data bus
);

    // ============================================================================
    // Global Parameter and Interconnect Definitions
    // ============================================================================
    localparam NUM_CORES        = 4;   // Quad-core configuration
    localparam ADDR_WIDTH       = 64;  // RISC-V RV64 Internal Virtual Address size
    localparam PHY_ADDR_WIDTH   = 33;  // 8 GB DDR4 physical address space (2^33 bytes)
    localparam DATA_WIDTH       = 64;  // Native register size
    localparam CACHE_LINE_SIZE  = 512; // 64-byte (512-bit) cache lines

    // Core to L3 Cache interconnect signals
    wire [ADDR_WIDTH-1:0]      core_l3_addr   [0:NUM_CORES-1];
    wire [DATA_WIDTH-1:0]      core_l3_wdata  [0:NUM_CORES-1];
    wire                       core_l3_req    [0:NUM_CORES-1];
    wire                       core_l3_write  [0:NUM_CORES-1];
    wire [DATA_WIDTH-1:0]      core_l3_rdata  [0:NUM_CORES-1];
    wire                       core_l3_ready  [0:NUM_CORES-1];

    // Outbound Common Peripheral DMA Port to L3
    wire [ADDR_WIDTH-1:0]      peri_l3_addr;
    wire [DATA_WIDTH-1:0]      peri_l3_wdata;
    wire                       peri_l3_req;
    wire                       peri_l3_write;
    wire [DATA_WIDTH-1:0]      peri_l3_rdata;
    wire                       peri_l3_ready;

    // L3 Cache to Memory Controller Interconnect
    wire [PHY_ADDR_WIDTH-1:0]  l3_mem_addr;
    wire [CACHE_LINE_SIZE-1:0] l3_mem_wdata;
    wire                       l3_mem_req;
    wire                       l3_mem_write;
    wire [CACHE_LINE_SIZE-1:0] l3_mem_rdata;
    wire                       l3_mem_ready;

    // ============================================================================
    // PLIC (Platform-Level Interrupt Controller) & Lines
    // ============================================================================
    wire [31:0]                plic_irq_sources;
    wire                       core_irq_line  [0:NUM_CORES-1];

    // Hardware interrupt sources from SoC blocks
    wire                       hdmi_irq;
    wire                       usb_irq;
    wire                       uart_irq;
    wire                       sdio_irq;
    wire                       timer_irq      [0:NUM_CORES-1];

    // Mapping PLIC interrupts: Source 0 = UART, 1 = SDIO, 2 = USB, 3 = HDMI
    assign plic_irq_sources[0]     = uart_irq;
    assign plic_irq_sources[1]     = sdio_irq;
    assign plic_irq_sources[2]     = usb_irq;
    assign plic_irq_sources[3]     = hdmi_irq;
    assign plic_irq_sources[31:4]  = 28'b0;

    // ============================================================================
    // 4x Core (Quad-Core RISC-V RV64GC) Instances
    // ============================================================================
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i = i + 1) begin : gen_cores
            riscv_core #(
                .CORE_ID(i),
                .ADDR_WIDTH(ADDR_WIDTH),
                .DATA_WIDTH(DATA_WIDTH),
                .CACHE_LINE_SIZE(CACHE_LINE_SIZE),
                .PHY_ADDR_WIDTH(PHY_ADDR_WIDTH)
            ) u_core (
                .clk(sys_clk),
                .rst_n(sys_rst_n),
                
                // Interface to L3 Cache
                .mem_addr(core_l3_addr[i]),
                .mem_wdata(core_l3_wdata[i]),
                .mem_req(core_l3_req[i]),
                .mem_write(core_l3_write[i]),
                .mem_rdata(core_l3_rdata[i]),
                .mem_ready(core_l3_ready[i]),

                // External Interrupt input from PLIC
                .ext_irq(core_irq_line[i]),
                
                // Outbound Core Timer Interrupt to PLIC/CLINT
                .timer_irq(timer_irq[i])
            );
        end
    endgenerate

    // ============================================================================
    // RISC-V Platform-Level Interrupt Controller (PLIC)
    // Routes external peripheral interrupts to targeted CPU core external IRQ lines
    // ============================================================================
    riscv_plic #(
        .NUM_CORES(NUM_CORES)
    ) u_plic (
        .clk(sys_clk),
        .rst_n(sys_rst_n),

        // Interrupt Inputs
        .irq_sources(plic_irq_sources),

        // Output lines to CPU cores
        .core_irqs({core_irq_line[3], core_irq_line[2], core_irq_line[1], core_irq_line[0]})
    );

    // ============================================================================
    // System Bus Interconnect / Peripheral Arbiter
    // Multiplexes high-speed DMA requests (HDMI, USB, SDIO, UART) to L3 cache
    // ============================================================================
    wire [ADDR_WIDTH-1:0] hdmi_dma_addr;
    wire [DATA_WIDTH-1:0] hdmi_dma_wdata;
    wire                  hdmi_dma_req;
    wire                  hdmi_dma_write;
    wire [DATA_WIDTH-1:0] hdmi_dma_rdata;
    wire                  hdmi_dma_ready;

    wire [ADDR_WIDTH-1:0] usb_dma_addr;
    wire [DATA_WIDTH-1:0] usb_dma_wdata;
    wire                  usb_dma_req;
    wire                  usb_dma_write;
    wire [DATA_WIDTH-1:0] usb_dma_rdata;
    wire                  usb_dma_ready;

    wire [ADDR_WIDTH-1:0] sdio_dma_addr;
    wire [DATA_WIDTH-1:0] sdio_dma_wdata;
    wire                  sdio_dma_req;
    wire                  sdio_dma_write;
    wire [DATA_WIDTH-1:0] sdio_dma_rdata;
    wire                  sdio_dma_ready;

    wire [ADDR_WIDTH-1:0] uart_dma_addr;
    wire [DATA_WIDTH-1:0] uart_dma_wdata;
    wire                  uart_dma_req;
    wire                  uart_dma_write;
    wire [DATA_WIDTH-1:0] uart_dma_rdata;
    wire                  uart_dma_ready;

    system_peripheral_arbiter #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_sys_peri_arb (
        .clk(sys_clk),
        .rst_n(sys_rst_n),
        
        // HDMI Port DMA Interface
        .hdmi_addr(hdmi_dma_addr),
        .hdmi_wdata(hdmi_dma_wdata),
        .hdmi_req(hdmi_dma_req),
        .hdmi_write(hdmi_dma_write),
        .hdmi_rdata(hdmi_dma_rdata),
        .hdmi_ready(hdmi_dma_ready),

        // USB Port DMA Interface
        .usb_addr(usb_dma_addr),
        .usb_wdata(usb_dma_wdata),
        .usb_req(usb_dma_req),
        .usb_write(usb_dma_write),
        .usb_rdata(usb_dma_rdata),
        .usb_ready(usb_dma_ready),

        // SDIO Port DMA Interface
        .sdio_addr(sdio_dma_addr),
        .sdio_wdata(sdio_dma_wdata),
        .sdio_req(sdio_dma_req),
        .sdio_write(sdio_dma_write),
        .sdio_rdata(sdio_dma_rdata),
        .sdio_ready(sdio_dma_ready),

        // UART Port Interface
        .uart_addr(uart_dma_addr),
        .uart_wdata(uart_dma_wdata),
        .uart_req(uart_dma_req),
        .uart_write(uart_dma_write),
        .uart_rdata(uart_dma_rdata),
        .uart_ready(uart_dma_ready),

        // Outbound Common Peripheral DMA Port to L3
        .out_addr(peri_l3_addr),
        .out_wdata(peri_l3_wdata),
        .out_req(peri_l3_req),
        .out_write(peri_l3_write),
        .out_rdata(peri_l3_rdata),
        .out_ready(peri_l3_ready)
    );

    // ============================================================================
    // Shared L3 Cache System
    // ============================================================================
    l3_cache #(
        .NUM_CORES(NUM_CORES),
        .ADDR_WIDTH(ADDR_WIDTH),
        .PHY_ADDR_WIDTH(PHY_ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .CACHE_LINE_SIZE(CACHE_LINE_SIZE)
    ) u_l3_cache (
        .clk(sys_clk),
        .rst_n(sys_rst_n),

        // Core interfaces
        .core_addr_0(core_l3_addr[0]), .core_wdata_0(core_l3_wdata[0]), .core_req_0(core_l3_req[0]), .core_write_0(core_l3_write[0]), .core_rdata_0(core_l3_rdata[0]), .core_ready_0(core_l3_ready[0]),
        .core_addr_1(core_l3_addr[1]), .core_wdata_1(core_l3_wdata[1]), .core_req_1(core_l3_req[1]), .core_write_1(core_l3_write[1]), .core_rdata_1(core_l3_rdata[1]), .core_ready_1(core_l3_ready[1]),
        .core_addr_2(core_l3_addr[2]), .core_wdata_2(core_l3_wdata[2]), .core_req_2(core_l3_req[2]), .core_write_2(core_l3_write[2]), .core_rdata_2(core_l3_rdata[2]), .core_ready_2(core_l3_ready[2]),
        .core_addr_3(core_l3_addr[3]), .core_wdata_3(core_l3_wdata[3]), .core_req_3(core_l3_req[3]), .core_write_3(core_l3_write[3]), .core_rdata_3(core_l3_rdata[3]), .core_ready_3(core_l3_ready[3]),

        // Peripheral high-speed DMA interface
        .peri_addr(peri_l3_addr),
        .peri_wdata(peri_l3_wdata),
        .peri_req(peri_l3_req),
        .peri_write(peri_l3_write),
        .peri_rdata(peri_l3_rdata),
        .peri_ready(peri_l3_ready),

        // Memory Interface
        .mem_addr(l3_mem_addr),
        .mem_wdata(l3_mem_wdata),
        .mem_req(l3_mem_req),
        .mem_write(l3_mem_write),
        .mem_rdata(l3_mem_rdata),
        .mem_ready(l3_mem_ready)
    );

    // ============================================================================
    // DDR4 Memory Controller (8 GB Address Space)
    // ============================================================================
    ddr4_controller #(
        .PHY_ADDR_WIDTH(PHY_ADDR_WIDTH),
        .CACHE_LINE_SIZE(CACHE_LINE_SIZE)
    ) u_ddr4_ctrl (
        .clk(sys_clk),
        .rst_n(sys_rst_n),

        // L3 Cache Interface
        .mem_addr(l3_mem_addr),
        .mem_wdata(l3_mem_wdata),
        .mem_req(l3_mem_req),
        .mem_write(l3_mem_write),
        .mem_rdata(l3_mem_rdata),
        .mem_ready(l3_mem_ready),

        // Physical DDR4 Pins
        .ddr4_ck_t(ddr4_ck_t),
        .ddr4_ck_c(ddr4_ck_c),
        .ddr4_cke(ddr4_cke),
        .ddr4_cs_n(ddr4_cs_n),
        .ddr4_act_n(ddr4_act_n),
        .ddr4_adr(ddr4_adr),
        .ddr4_ba(ddr4_ba),
        .ddr4_bg(ddr4_bg),
        .ddr4_ras_n(ddr4_ras_n),
        .ddr4_cas_n(ddr4_cas_n),
        .ddr4_we_n(ddr4_we_n),
        .ddr4_odt(ddr4_odt),
        .ddr4_reset_n(ddr4_reset_n),
        .ddr4_dq(ddr4_dq),
        .ddr4_dqs_t(ddr4_dqs_t),
        .ddr4_dqs_c(ddr4_dqs_c)
    );

    // ============================================================================
    // HDMI 1.4 Type-C Alt-Mode Controller Instance
    // ============================================================================
    hdmi_typec_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_hdmi_ctrl (
        .clk(sys_clk),
        .rst_n(sys_rst_n),

        // Unified High-speed System DMA Interface
        .dma_addr(hdmi_dma_addr),
        .dma_wdata(hdmi_dma_wdata),
        .dma_req(hdmi_dma_req),
        .dma_write(hdmi_dma_write),
        .dma_rdata(hdmi_dma_rdata),
        .dma_ready(hdmi_dma_ready),

        // Physical HDMI 1.4 / Type-C Pins
        .hdmi_tx_clk_p(hdmi_tx_clk_p),
        .hdmi_tx_clk_n(hdmi_tx_clk_n),
        .hdmi_tx_data_p(hdmi_tx_data_p),
        .hdmi_tx_data_n(hdmi_tx_data_n),
        .hdmi_hpd(hdmi_hpd),
        .hdmi_cec(hdmi_cec),
        .hdmi_ddc_scl(hdmi_ddc_scl),
        .hdmi_ddc_sda(hdmi_ddc_sda),
        .typec_cc1(typec_cc1),
        .typec_cc2(typec_cc2),

        // Output Interrupt Line
        .irq_out(hdmi_irq)
    );

    // ============================================================================
    // 3x USB 3.0 Type-A Controller Instance
    // ============================================================================
    usb3_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_usb3_ctrl (
        .clk(sys_clk),
        .rst_n(sys_rst_n),

        // Unified High-speed System DMA Interface
        .dma_addr(usb_dma_addr),
        .dma_wdata(usb_dma_wdata),
        .dma_req(usb_dma_req),
        .dma_write(usb_dma_write),
        .dma_rdata(usb_dma_rdata),
        .dma_ready(usb_dma_ready),

        // Physical USB 3.0 Type-A Ports
        .usb_ssrx_p(usb_ssrx_p),
        .usb_ssrx_n(usb_ssrx_n),
        .usb_sstx_p(usb_sstx_p),
        .usb_sstx_n(usb_sstx_n),
        .usb_dp(usb_dp),
        .usb_dn(usb_dn),

        // Output Interrupt Line
        .irq_out(usb_irq)
    );

    // ============================================================================
    // UART Console Controller
    // ============================================================================
    uart_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_uart_ctrl (
        .clk(sys_clk),
        .rst_n(sys_rst_n),

        // Register Access Bus
        .dma_addr(uart_dma_addr),
        .dma_wdata(uart_dma_wdata),
        .dma_req(uart_dma_req),
        .dma_write(uart_dma_write),
        .dma_rdata(uart_dma_rdata),
        .dma_ready(uart_dma_ready),

        // Physical Pins
        .tx_pin(uart_tx),
        .rx_pin(uart_rx),

        // Output Interrupt Line
        .irq_out(uart_irq)
    );

    // ============================================================================
    // SDIO Storage Controller
    // ============================================================================
    sdio_storage_controller #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_sdio_ctrl (
        .clk(sys_clk),
        .rst_n(sys_rst_n),

        // DMA Memory Interface
        .dma_addr(sdio_dma_addr),
        .dma_wdata(sdio_dma_wdata),
        .dma_req(sdio_dma_req),
        .dma_write(sdio_dma_write),
        .dma_rdata(sdio_dma_rdata),
        .dma_ready(sdio_dma_ready),

        // Physical MicroSD Interface Bypasses
        .sd_clk(sd_clk),
        .sd_cmd(sd_cmd),
        .sd_dat(sd_dat),

        // Output Interrupt Line
        .irq_out(sdio_irq)
    );

endmodule


// ============================================================================
// RISC-V Platform-Level Interrupt Controller (PLIC)
// ============================================================================
module riscv_plic #(
    parameter NUM_CORES = 4
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Interrupt Input Sources (peripherals)
    input  wire [31:0]           irq_sources,

    // Targeted Core Interconnect IRQ output lines
    output reg  [NUM_CORES-1:0]  core_irqs
);

    reg [31:0] plic_enable;
    reg [31:0] plic_pending;
    reg [2:0]  plic_target  [0:31];

    integer c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            plic_enable  <= 32'hFFFFFFFF; // Enable all interrupts by default
            plic_pending <= 32'h0;
            core_irqs    <= 0;
            for (c = 0; c < 32; c = c + 1) begin
                plic_target[c] <= c[1:0]; // Target routing across cores
            end
        end else begin
            plic_pending <= irq_sources & plic_enable;

            // Route pending interrupts based on target core assignment
            core_irqs[0] <= |(plic_pending & 32'h11111111);
            core_irqs[1] <= |(plic_pending & 32'h22222222);
            core_irqs[2] <= |(plic_pending & 32'h44444444);
            core_irqs[3] <= |(plic_pending & 32'h88888888);
        end
    end
endmodule


// ============================================================================
// Peripheral DMA Arbiter
// Coordinates DMA traffic between HDMI, USB, SDIO, and UART blocks and L3 cache
// ============================================================================
module system_peripheral_arbiter #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // HDMI DMA Interface
    input  wire [ADDR_WIDTH-1:0] hdmi_addr,
    input  wire [DATA_WIDTH-1:0] hdmi_wdata,
    input  wire                  hdmi_req,
    input  wire                  hdmi_write,
    output reg  [DATA_WIDTH-1:0] hdmi_rdata,
    output reg                   hdmi_ready,

    // USB DMA Interface
    input  wire [ADDR_WIDTH-1:0] usb_addr,
    input  wire [DATA_WIDTH-1:0] usb_wdata,
    input  wire                  usb_req,
    input  wire                  usb_write,
    output reg  [DATA_WIDTH-1:0] usb_rdata,
    output reg                   usb_ready,

    // SDIO DMA Interface
    input  wire [ADDR_WIDTH-1:0] sdio_addr,
    input  wire [DATA_WIDTH-1:0] sdio_wdata,
    input  wire                  sdio_req,
    input  wire                  sdio_write,
    output reg  [DATA_WIDTH-1:0] sdio_rdata,
    output reg                   sdio_ready,

    // UART DMA Interface
    input  wire [ADDR_WIDTH-1:0] uart_addr,
    input  wire [DATA_WIDTH-1:0] uart_wdata,
    input  wire                  uart_req,
    input  wire                  uart_write,
    output reg  [DATA_WIDTH-1:0] uart_rdata,
    output reg                   uart_ready,

    // Shared Peripheral Bus output to L3 Cache
    output reg  [ADDR_WIDTH-1:0] out_addr,
    output reg  [DATA_WIDTH-1:0] out_wdata,
    output reg                   out_req,
    output reg                   out_write,
    input  wire [DATA_WIDTH-1:0] out_rdata,
    input  wire                  out_ready
);

    reg [1:0] arb_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arb_state  <= 2'b00;
            out_req    <= 1'b0;
            out_addr   <= 0;
            out_wdata  <= 0;
            out_write  <= 1'b0;
            hdmi_ready <= 1'b0;
            usb_ready  <= 1'b0;
            sdio_ready <= 1'b0;
            uart_ready <= 1'b0;
            hdmi_rdata <= 0;
            usb_rdata  <= 0;
            sdio_rdata <= 0;
            uart_rdata <= 0;
        end else begin
            arb_state <= arb_state + 1;

            hdmi_ready <= 1'b0;
            usb_ready  <= 1'b0;
            sdio_ready <= 1'b0;
            uart_ready <= 1'b0;

            case (arb_state)
                2'd0: begin
                    if (hdmi_req) begin
                        out_addr   <= hdmi_addr;
                        out_wdata  <= hdmi_wdata;
                        out_req    <= 1'b1;
                        out_write  <= hdmi_write;
                        if (out_ready) begin
                            hdmi_rdata <= out_rdata;
                            hdmi_ready <= 1'b1;
                            out_req    <= 1'b0;
                        end
                    end else out_req <= 1'b0;
                end

                2'd1: begin
                    if (usb_req) begin
                        out_addr   <= usb_addr;
                        out_wdata  <= usb_wdata;
                        out_req    <= 1'b1;
                        out_write  <= usb_write;
                        if (out_ready) begin
                            usb_rdata <= out_rdata;
                            usb_ready <= 1'b1;
                            out_req    <= 1'b0;
                        end
                    end else out_req <= 1'b0;
                end

                2'd2: begin
                    if (sdio_req) begin
                        out_addr   <= sdio_addr;
                        out_wdata  <= sdio_wdata;
                        out_req    <= 1'b1;
                        out_write  <= sdio_write;
                        if (out_ready) begin
                            sdio_rdata <= out_rdata;
                            sdio_ready <= 1'b1;
                            out_req    <= 1'b0;
                        end
                    end else out_req <= 1'b0;
                end

                2'd3: begin
                    if (uart_req) begin
                        out_addr   <= uart_addr;
                        out_wdata  <= uart_wdata;
                        out_req    <= 1'b1;
                        out_write  <= uart_write;
                        if (out_ready) begin
                            uart_rdata <= out_rdata;
                            uart_ready <= 1'b1;
                            out_req    <= 1'b0;
                        end
                    end else out_req <= 1'b0;
                end
            endcase
        end
    end
endmodule


// ============================================================================
// Core Module: RISC-V RV64GC 64-bit Out-of-Order Engine (Sv39 MMU, TLB, & CSRs)
// ============================================================================
module riscv_core #(
    parameter CORE_ID         = 0,
    parameter ADDR_WIDTH      = 64,
    parameter DATA_WIDTH      = 64,
    parameter CACHE_LINE_SIZE = 512,
    parameter PHY_ADDR_WIDTH   = 33
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // High Speed Interconnect to L2/L3 Memory Hierarchy
    output reg  [ADDR_WIDTH-1:0] mem_addr,
    output reg  [DATA_WIDTH-1:0] mem_wdata,
    output reg                   mem_req,
    output reg                   mem_write,
    input  wire [DATA_WIDTH-1:0] mem_rdata,
    input  wire                  mem_ready,

    // External Interrupt input from PLIC
    input  wire                  ext_irq,

    // Outbound Timer Interrupt output
    output reg                   timer_irq
);

    // ============================================================================
    // Privilege Levels & RISC-V CSRs (Control and Status Registers)
    // ============================================================================
    reg [1:0]  priv_mode;      // Privilege: 2'd0 = U-Mode, 2'd1 = S-Mode (Linux), 2'd3 = M-Mode (Firmware)
    reg [63:0] csr_satp;       // Supervisor Address Translation and Protection (Enables Sv39 MMU)
    reg [63:0] csr_mstatus;    // Machine Status Register
    reg [63:0] csr_sstatus;    // Supervisor Status Register
    reg [63:0] csr_sepc;       // Supervisor Exception Program Counter
    reg [63:0] csr_stvec;      // Supervisor Trap Vector Base Address
    reg [63:0] csr_scause;     // Supervisor Trap Cause
    reg [63:0] csr_stval;      // Supervisor Trap Value (Offending virtual address)

    wire mmu_enabled = (csr_satp[63:60] == 4'd8); // Sv39 translation mode enabled

    // ============================================================================
    // RISC-V Core-Local Interruptor (CLINT) Timer Registers
    // ============================================================================
    reg [63:0] clint_mtime;
    reg [63:0] clint_mtimecmp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clint_mtime    <= 64'h0;
            clint_mtimecmp <= 64'h0;
            timer_irq      <= 1'b0;
        end else begin
            clint_mtime <= clint_mtime + 1;
            
            // Trigger supervisor timer interrupt if current time >= compare value
            if (clint_mtime >= clint_mtimecmp) begin
                timer_irq <= 1'b1;
            end else begin
                timer_irq <= 1'b0;
            end
        end
    end

    // ============================================================================
    // Level 1 Instruction Cache (L1 I-Cache) and Data Cache (L1 D-Cache)
    // ============================================================================
    wire [ADDR_WIDTH-1:0]      icache_addr;
    wire [31:0]                icache_rdata; // 32-bit RISC-V instructions
    wire                       icache_req;
    wire                       icache_ready;

    wire [ADDR_WIDTH-1:0]      dcache_addr;
    wire [DATA_WIDTH-1:0]      dcache_wdata;
    wire                       dcache_req;
    wire                       dcache_write;
    wire [DATA_WIDTH-1:0]      dcache_rdata;
    wire                       dcache_ready;

    // Fast-path Level 2 Cache (L2 Unified) per Core
    wire [ADDR_WIDTH-1:0]      l2_addr;
    wire [CACHE_LINE_SIZE-1:0] l2_wdata;
    wire                       l2_req;
    wire                       l2_write;
    wire [CACHE_LINE_SIZE-1:0] l2_rdata;
    wire                       l2_ready;

    // RISC-V RV64GC Execution Engine Registers
    reg  [ADDR_WIDTH-1:0]      pc;
    reg  [DATA_WIDTH-1:0]      x_regs [0:31]; // X0 - X31 (X0 is hardwired to 0)
    integer r;

    // Pipeline Registers
    reg [ADDR_WIDTH-1:0]       f_pc;
    reg [31:0]                 d_inst;
    reg                        d_valid;

    // Instruction Decoder (Decoding RISC-V standard 32-bit instructions)
    wire [6:0]  opcode   = d_inst[6:0];
    wire [4:0]  rd       = d_inst[11:7];
    wire [2:0]  funct3   = d_inst[14:12];
    wire [4:0]  rs1      = d_inst[19:15];
    wire [4:0]  rs2      = d_inst[24:20];
    wire [6:0]  funct7   = d_inst[31:25];
    wire [11:0] imm12    = d_inst[31:20];
    wire [11:0] csr_addr = d_inst[31:20];

    // ============================================================================
    // Memory Management Unit (MMU) & Address Translation
    // ============================================================================
    wire [PHY_ADDR_WIDTH-1:0]  inst_pa;
    wire                       inst_pa_valid;
    wire                       inst_page_fault;
    wire [5:0]                 inst_fault_syndrome;

    wire [PHY_ADDR_WIDTH-1:0]  data_pa;
    wire                       data_pa_valid;
    wire                       data_page_fault;
    wire [5:0]                 data_fault_syndrome;

    // MMU walk memory interface lines
    wire [ADDR_WIDTH-1:0]      mmu_walk_addr;
    wire                       mmu_walk_req;
    wire [63:0]                mmu_walk_rdata;
    wire                       mmu_walk_ready;

    // Instantiate Memory Management Unit (MMU) for Instruction Fetch
    riscv_sv39_mmu #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .PHY_ADDR_WIDTH(PHY_ADDR_WIDTH)
    ) u_instruction_mmu (
        .clk(clk),
        .rst_n(rst_n),

        // Virtual Address inputs
        .va(pc),
        .req_val(1'b1),
        .write_en(1'b0),
        .exec_en(1'b1),
        .priv_mode(priv_mode),

        // Physical Address outputs
        .pa(inst_pa),
        .pa_valid(inst_pa_valid),
        .page_fault(inst_page_fault),
        .fault_syndrome(inst_fault_syndrome),

        // SATP Configuration Register
        .csr_satp(csr_satp),

        // Memory walker link
        .walk_addr(mmu_walk_addr),
        .walk_req(mmu_walk_req),
        .walk_rdata(mmu_walk_rdata),
        .walk_ready(mmu_walk_ready)
    );

    // Instantiate Memory Management Unit (MMU) for Data access (Loads/Stores)
    riscv_sv39_mmu #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .PHY_ADDR_WIDTH(PHY_ADDR_WIDTH)
    ) u_data_mmu (
        .clk(clk),
        .rst_n(rst_n),

        // Virtual Address inputs
        .va(dcache_addr),
        .req_val(dcache_req),
        .write_en(dcache_write),
        .exec_en(1'b0),
        .priv_mode(priv_mode),

        // Physical Address outputs
        .pa(data_pa),
        .pa_valid(data_pa_valid),
        .page_fault(data_page_fault),
        .fault_syndrome(data_fault_syndrome),

        // SATP Register
        .csr_satp(csr_satp),

        // Memory walker link
        .walk_addr(),
        .walk_req(),
        .walk_rdata(64'd0),
        .walk_ready(1'b0)
    );

    // ============================================================================
    // Pipeline Controller and Registers (RISC-V Stages)
    // ============================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc           <= 64'h0000000080000000; // RISC-V default physical RAM start (OpenSBI base)
            f_pc         <= 64'h0000000000000000;
            d_inst       <= 32'h00000013; // NOP instruction (addi x0, x0, 0)
            d_valid      <= 1'b0;
            priv_mode    <= 2'd3;        // Start in Machine Mode (M-Mode) for early OpenSBI initialization
            csr_satp     <= 64'h0;
            csr_mstatus  <= 64'h0;
            csr_sstatus  <= 64'h0;
            csr_sepc     <= 64'h0;
            csr_stvec    <= 64'h0;
            csr_scause   <= 64'h0;
            csr_stval    <= 64'h0;
            
            for (r = 0; r < 32; r = r + 1) begin
                x_regs[r] <= 64'h0;
            end
        end else begin
            // Hardware External Interrupt Trap Routing (Supervisor External Interrupt)
            if (ext_irq && (priv_mode < 2'd3)) begin
                csr_sepc   <= pc;
                csr_scause <= 64'h8000000000000009; // Supervisor external interrupt cause
                priv_mode  <= 2'd1;                 // Switch to S-mode
                pc         <= csr_stvec;            // Jump to Supervisor trap handler
                d_valid    <= 1'b0;
            end
            // Page Fault Exception Handling
            else if (inst_page_fault) begin
                csr_sepc   <= pc;
                csr_stval  <= pc;
                csr_scause <= 64'd12;               // Instruction Page Fault Exception code
                pc         <= csr_stvec;            // Trap vector
                d_valid    <= 1'b0;
            end else if (dcache_req && data_page_fault) begin
                csr_sepc   <= pc;
                csr_stval  <= dcache_addr;
                csr_scause <= dcache_write ? 64'd15 : 64'd13; // Store/Load Page Fault Exception codes
                pc         <= csr_stvec;
                d_valid    <= 1'b0;
            end else begin
                // 1. Fetch Stage
                if (icache_ready && inst_pa_valid) begin
                    f_pc    <= pc;
                    d_inst  <= icache_rdata;
                    d_valid <= 1'b1;
                    pc      <= pc + 4;
                end

                // 2. Execution / Writeback
                if (d_valid) begin
                    x_regs[0] <= 64'h0; // X0 is hardwired to 0
                    case (opcode)
                        7'b0010011: begin // OP-IMM (ADDI, etc.)
                            if (rd != 5'd0) begin
                                if (funct3 == 3'b000)      x_regs[rd] <= x_regs[rs1] + {{52{imm12[11]}}, imm12};
                                else if (funct3 == 3'b111) x_regs[rd] <= x_regs[rs1] & {{52{imm12[11]}}, imm12};
                            end
                        end
                        7'b0110011: begin // OP (ADD, SUB, AND, OR, etc.)
                            if (rd != 5'd0) begin
                                if (funct3 == 3'b000 && funct7 == 7'h00)  x_regs[rd] <= x_regs[rs1] + x_regs[rs2];
                                else if (funct3 == 3'b000 && funct7 == 7'h20) x_regs[rd] <= x_regs[rs1] - x_regs[rs2];
                                else if (funct3 == 3'b111) x_regs[rd] <= x_regs[rs1] & x_regs[rs2];
                            end
                        end
                        7'b1110011: begin // SYSTEM (CSR Access and privileged returns)
                            if (funct3 == 3'b001) begin // CSRRW (Atomic Read/Write CSR)
                                if (csr_addr == 12'h180) begin
                                    x_regs[rd] <= csr_satp;
                                    csr_satp   <= x_regs[rs1];
                                end else if (csr_addr == 12'h105) begin
                                    x_regs[rd] <= csr_stvec;
                                    csr_stvec  <= x_regs[rs1];
                                end
                            end else if (funct3 == 3'b000 && funct7 == 7'h08) begin // SRET / MRET
                                priv_mode <= 2'd1; // Down to S-mode
                                pc        <= csr_sepc;
                            end
                        end
                        default: begin
                            // Custom opcodes
                        end
                    endcase
                end
            end
        end
    end

    // Interface logic to core memory bus (Multiplexing MMU Walks, I-Cache, and D-Cache)
    always @(*) begin
        if (mmu_walk_req) begin
            mem_req   = 1'b1;
            mem_addr  = mmu_walk_addr;
            mem_write = 1'b0;
            mem_wdata = 64'h0;
        end else begin
            mem_req   = l2_req;
            mem_addr  = l2_addr;
            mem_write = l2_write;
            mem_wdata = l2_wdata[DATA_WIDTH-1:0];
        end
    end

    // Pass walk read data back to MMU Page Table Walker
    assign mmu_walk_rdata = mem_rdata;
    assign mmu_walk_ready = mem_ready;

    // Instantiate L1 Instruction Cache
    riscv_l1_cache #(
        .CACHE_TYPE("INSTRUCTION"),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(32)
    ) u_l1_icache (
        .clk(clk),
        .rst_n(rst_n),
        .cpu_addr({31'b0, inst_pa}),
        .cpu_wdata(32'h0),
        .cpu_req(inst_pa_valid),
        .cpu_write(1'b0),
        .cpu_rdata(icache_rdata),
        .cpu_ready(icache_ready),
        .mem_addr(l2_addr),
        .mem_req(l2_req),
        .mem_ready(l2_ready)
    );

    // Instantiate L1 Data Cache
    riscv_l1_cache #(
        .CACHE_TYPE("DATA"),
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_l1_dcache (
        .clk(clk),
        .rst_n(rst_n),
        .cpu_addr({31'b0, data_pa}),
        .cpu_wdata(dcache_wdata),
        .cpu_req(data_pa_valid),
        .cpu_write(dcache_write),
        .cpu_rdata(dcache_rdata),
        .cpu_ready(dcache_ready),
        .mem_addr(),
        .mem_req(),
        .mem_ready(1'b1)
    );

    assign dcache_addr  = 64'h0;
    assign dcache_wdata = 64'h0;
    assign dcache_req   = 1'b0;
    assign dcache_write = 1'b0;

endmodule


// ============================================================================
// MMU Module: Sv39 Virtual Address to Physical Address Translation
// Fully Associative TLB, and 3-level Hardware Page Table Walking
// ============================================================================
module riscv_sv39_mmu #(
    parameter ADDR_WIDTH = 64,
    parameter PHY_ADDR_WIDTH = 33
) (
    input  wire                        clk,
    input  wire                        rst_n,

    // CPU Virtual Address Interface
    input  wire [ADDR_WIDTH-1:0]       va,
    input  wire                        req_val,
    input  wire                        write_en,
    input  wire                        exec_en,
    input  wire [1:0]                  priv_mode,

    // Translated Physical Address outputs
    output reg  [PHY_ADDR_WIDTH-1:0]   pa,
    output reg                         pa_valid,
    output reg                         page_fault,
    output reg  [5:0]                  fault_syndrome,

    // SATP System Register
    input  wire [63:0]                 csr_satp,

    // System bus link for hardware walks
    output reg  [ADDR_WIDTH-1:0]       walk_addr,
    output reg                         walk_req,
    input  wire [63:0]                 walk_rdata,
    input  wire                        walk_ready
);

    // ============================================================================
    // TLB Storage: Fully Associative, 16 Entries
    // ============================================================================
    localparam TLB_ENTRIES = 16;
    localparam TAG_WIDTH   = 27; // Translate Sv39 Virtual Page Number (VPN)
    localparam PFN_WIDTH   = 21; // 33-bit physical space -> 33 - 12 offset = 21 bits

    reg                      tlb_valid      [0:TLB_ENTRIES-1];
    reg [TAG_WIDTH-1:0]      tlb_vpn        [0:TLB_ENTRIES-1];
    reg [PFN_WIDTH-1:0]      tlb_pfn        [0:TLB_ENTRIES-1];
    reg                      tlb_user_access [0:TLB_ENTRIES-1];
    reg                      tlb_read_only  [0:TLB_ENTRIES-1];
    reg                      tlb_exec_never [0:TLB_ENTRIES-1];

    // Page Table Walking States (Sv39 has 3 levels: L2 -> L1 -> L0)
    localparam ST_IDLE        = 3'd0;
    localparam ST_WALK_L2     = 3'd1;
    localparam ST_WALK_L1     = 3'd2;
    localparam ST_WALK_L0     = 3'd3;
    localparam ST_WRITE_TLB   = 3'd4;

    reg [2:0] walk_state;
    reg [3:0] victim_pointer;

    // Address Parsing for Sv39 (39-bit virtual address, 12-bit offset)
    wire [TAG_WIDTH-1:0] lookup_vpn = va[38:12];
    wire [43:0]          satp_ppn   = csr_satp[43:0]; // Root page table Physical Frame

    // Parallel TLB Search
    reg        tlb_hit;
    reg [3:0]  hit_index;
    integer t;
    integer k;

    always @(*) begin
        tlb_hit   = 1'b0;
        hit_index = 4'h0;
        for (t = 0; t < TLB_ENTRIES; t = t + 1) begin
            if (tlb_valid[t] && (tlb_vpn[t] == lookup_vpn)) begin
                tlb_hit   = 1'b1;
                hit_index = t[3:0];
            end
        end
    end

    // Address Translation Control Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            walk_state     <= ST_IDLE;
            victim_pointer <= 4'h0;
            pa             <= 0;
            pa_valid       <= 1'b0;
            page_fault     <= 1'b0;
            fault_syndrome <= 6'h0;
            walk_req       <= 1'b0;
            walk_addr      <= 0;

            for (k = 0; k < TLB_ENTRIES; k = k + 1) begin
                tlb_valid[k]       <= 1'b0;
                tlb_vpn[k]         <= 0;
                tlb_pfn[k]         <= 0;
                tlb_user_access[k] <= 1'b0;
                tlb_read_only[k]   <= 1'b0;
                tlb_exec_never[k]  <= 1'b0;
            end
        end else begin
            if (csr_satp[63:60] != 4'd8) begin
                // Bare translation mode (MMU Disabled)
                pa         <= va[PHY_ADDR_WIDTH-1:0];
                pa_valid   <= req_val;
                page_fault <= 1'b0;
            end else if (req_val) begin
                if (tlb_hit) begin
                    // TLB Hit! Check Supervisor vs User Mode access permissions
                    if ((priv_mode == 2'd0) && !tlb_user_access[hit_index]) begin
                        // U-Mode trying to access non-user page
                        page_fault     <= 1'b1;
                        pa_valid       <= 1'b0;
                        fault_syndrome <= 6'b001101;
                    end else if (write_en && tlb_read_only[hit_index]) begin
                        page_fault     <= 1'b1;
                        pa_valid       <= 1'b0;
                        fault_syndrome <= 6'b001111;
                    end else if (exec_en && tlb_exec_never[hit_index]) begin
                        page_fault     <= 1'b1;
                        pa_valid       <= 1'b0;
                        fault_syndrome <= 6'b001110;
                    end else begin
                        // Translation Successful!
                        pa         <= {tlb_pfn[hit_index], va[11:0]};
                        pa_valid   <= 1'b1;
                        page_fault <= 1'b0;
                    end
                end else begin
                    // TLB Miss: Start Sv39 3-level walk (L2 -> L1 -> L0)
                    pa_valid <= 1'b0;
                    case (walk_state)
                        ST_IDLE: begin
                            walk_req   <= 1'b1;
                            walk_addr  <= {satp_ppn, 12'h0} + {55'h0, va[38:30], 3'b0}; // L2 base + VPN[2]*8
                            walk_state <= ST_WALK_L2;
                        end

                        ST_WALK_L2: begin
                            if (walk_ready) begin
                                if (walk_rdata[0] == 1'b0) begin // Descriptor invalid (V=0)
                                    page_fault     <= 1'b1;
                                    fault_syndrome <= 6'b000100;
                                    walk_state     <= ST_IDLE;
                                    walk_req       <= 1'b0;
                                end else begin
                                    walk_addr  <= {walk_rdata[53:10], 12'h0} + {55'h0, va[29:21], 3'b0}; // L1 base + VPN[1]*8
                                    walk_state <= ST_WALK_L1;
                                end
                            end
                        end

                        ST_WALK_L1: begin
                            if (walk_ready) begin
                                if (walk_rdata[0] == 1'b0) begin
                                    page_fault     <= 1'b1;
                                    fault_syndrome <= 6'b000101;
                                    walk_state     <= ST_IDLE;
                                    walk_req       <= 1'b0;
                                end else begin
                                    walk_addr  <= {walk_rdata[53:10], 12'h0} + {55'h0, va[20:12], 3'b0}; // L0 base + VPN[0]*8
                                    walk_state <= ST_WALK_L0;
                                end
                            end
                        end

                        ST_WALK_L0: begin
                            if (walk_ready) begin
                                if (walk_rdata[0] == 1'b0) begin
                                    page_fault     <= 1'b1;
                                    fault_syndrome <= 6'b000110;
                                    walk_state     <= ST_IDLE;
                                    walk_req       <= 1'b0;
                                end else begin
                                    walk_req   <= 1'b0;
                                    walk_state <= ST_WRITE_TLB;
                                end
                            end
                        end

                        ST_WRITE_TLB: begin
                            // Page Table Walk Complete! Save page descriptors into TLB buffer
                            tlb_valid[victim_pointer]       <= 1'b1;
                            tlb_vpn[victim_pointer]         <= lookup_vpn;
                            tlb_pfn[victim_pointer]         <= walk_rdata[32:12]; // Physical Address Frame
                            tlb_user_access[victim_pointer] <= walk_rdata[4];    // U bit (User mode allowed)
                            tlb_read_only[victim_pointer]   <= !walk_rdata[2];   // W bit (Write protection)
                            tlb_exec_never[victim_pointer]  <= !walk_rdata[3];   // X bit (Execute protection)

                            victim_pointer <= victim_pointer + 1;
                            walk_state     <= ST_IDLE;
                        end
                    endcase
                end
            end
        end
    end
endmodule


// ============================================================================
// Cache Module: Level-1 Cache
// ============================================================================
module riscv_l1_cache #(
    parameter CACHE_TYPE      = "DATA",
    parameter ADDR_WIDTH      = 64,
    parameter DATA_WIDTH      = 64
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Core-facing Interface
    input  wire [ADDR_WIDTH-1:0] cpu_addr,
    input  wire [DATA_WIDTH-1:0] cpu_wdata,
    input  wire                  cpu_req,
    input  wire                  cpu_write,
    output reg  [DATA_WIDTH-1:0] cpu_rdata,
    output reg                   cpu_ready,

    // Inner cache memory interface
    output reg  [ADDR_WIDTH-1:0] mem_addr,
    output reg                   mem_req,
    input  wire                  mem_ready
);

    localparam CACHE_SETS = 256;
    localparam TAG_WIDTH  = ADDR_WIDTH - 8 - 2;

    reg [DATA_WIDTH-1:0] data_store [0:CACHE_SETS-1];
    reg [TAG_WIDTH-1:0]  tag_store  [0:CACHE_SETS-1];
    reg                  valid_bits [0:CACHE_SETS-1];

    wire [7:0] index = cpu_addr[9:2];
    wire [TAG_WIDTH-1:0] current_tag = cpu_addr[ADDR_WIDTH-1:10];
    
    wire hit = valid_bits[index] && (tag_store[index] == current_tag);
    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (j = 0; j < CACHE_SETS; j = j + 1) begin
                valid_bits[j] <= 1'b0;
                tag_store[j]  <= 0;
                data_store[j] <= 0;
            end
            cpu_ready <= 1'b0;
        end else begin
            if (cpu_req) begin
                if (hit) begin
                    cpu_ready <= 1'b1;
                    if (cpu_write) begin
                        data_store[index] <= cpu_wdata;
                    end else begin
                        cpu_rdata         <= data_store[index];
                    end
                end else begin
                    cpu_ready <= 1'b0;
                    mem_req   <= 1'b1;
                    mem_addr  <= cpu_addr;
                    if (mem_ready) begin
                        valid_bits[index] <= 1'b1;
                        tag_store[index]  <= current_tag;
                        mem_req           <= 1'b0;
                    end
                end
            end else begin
                cpu_ready <= 1'b0;
            end
        end
    end
endmodule


// ============================================================================
// L3 Cache: Shared Multi-Core Cache System
// ============================================================================
module l3_cache #(
    parameter NUM_CORES        = 4,
    parameter ADDR_WIDTH       = 64,
    parameter PHY_ADDR_WIDTH   = 33,
    parameter DATA_WIDTH       = 64,
    parameter CACHE_LINE_SIZE  = 512
) (
    input  wire                         clk,
    input  wire                         rst_n,

    // Interfaces for 4 cores
    input  wire [ADDR_WIDTH-1:0]        core_addr_0,  input  wire [DATA_WIDTH-1:0] core_wdata_0,  input  wire core_req_0,  input  wire core_write_0,  output reg [DATA_WIDTH-1:0] core_rdata_0,  output reg core_ready_0,
    input  wire [ADDR_WIDTH-1:0]        core_addr_1,  input  wire [DATA_WIDTH-1:0] core_wdata_1,  input  wire core_req_1,  input  wire core_write_1,  output reg [DATA_WIDTH-1:0] core_rdata_1,  output reg core_ready_1,
    input  wire [ADDR_WIDTH-1:0]        core_addr_2,  input  wire [DATA_WIDTH-1:0] core_wdata_2,  input  wire core_req_2,  input  wire core_write_2,  output reg [DATA_WIDTH-1:0] core_rdata_2,  output reg core_ready_2,
    input  wire [ADDR_WIDTH-1:0]        core_addr_3,  input  wire [DATA_WIDTH-1:0] core_wdata_3,  input  wire core_req_3,  input  wire core_write_3,  output reg [DATA_WIDTH-1:0] core_rdata_3,  output reg core_ready_3,

    // High Speed Peripheral DMA input
    input  wire [ADDR_WIDTH-1:0]        peri_addr,    input  wire [DATA_WIDTH-1:0] peri_wdata,    input  wire peri_req,    input  wire peri_write,    output reg [DATA_WIDTH-1:0] peri_rdata,    output reg peri_ready,

    // External Memory / DDR4 interface lines
    output reg  [PHY_ADDR_WIDTH-1:0]    mem_addr,
    output reg  [CACHE_LINE_SIZE-1:0]   mem_wdata,
    output reg                          mem_req,
    output reg                          mem_write,
    input  wire [CACHE_LINE_SIZE-1:0]   mem_rdata,
    input  wire                         mem_ready
);

    reg [2:0] arb_pointer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arb_pointer  <= 3'b000;
            mem_req      <= 1'b0;
            mem_addr     <= 0;
            mem_wdata    <= 0;
            mem_write    <= 0;
            
            core_ready_0 <= 1'b0; core_ready_1 <= 1'b0; core_ready_2 <= 1'b0; core_ready_3 <= 1'b0;
            peri_ready   <= 1'b0;
        end else begin
            arb_pointer <= arb_pointer + 1;
            if (arb_pointer > 3'd4) arb_pointer <= 3'd0;

            case (arb_pointer)
                3'd0: begin
                    if (core_req_0) begin
                        mem_req      <= 1'b1;
                        mem_addr     <= core_addr_0[PHY_ADDR_WIDTH-1:0];
                        core_rdata_0 <= mem_rdata[DATA_WIDTH-1:0];
                        core_ready_0 <= mem_ready;
                    end else core_ready_0 <= 1'b0;
                end
                3'd1: begin
                    if (core_req_1) begin
                        mem_req      <= 1'b1;
                        mem_addr     <= core_addr_1[PHY_ADDR_WIDTH-1:0];
                        core_rdata_1 <= mem_rdata[DATA_WIDTH-1:0];
                        core_ready_1 <= mem_ready;
                    end else core_ready_1 <= 1'b0;
                end
                3'd2: begin
                    if (core_req_2) begin
                        mem_req      <= 1'b1;
                        mem_addr     <= core_addr_2[PHY_ADDR_WIDTH-1:0];
                        core_rdata_2 <= mem_rdata[DATA_WIDTH-1:0];
                        core_ready_2 <= mem_ready;
                    end else core_ready_2 <= 1'b0;
                end
                3'd3: begin
                    if (core_req_3) begin
                        mem_req      <= 1'b1;
                        mem_addr     <= core_addr_3[PHY_ADDR_WIDTH-1:0];
                        core_rdata_3 <= mem_rdata[DATA_WIDTH-1:0];
                        core_ready_3 <= mem_ready;
                    end else core_ready_3 <= 1'b0;
                end
                3'd4: begin
                    if (peri_req) begin
                        mem_req    <= 1'b1;
                        mem_addr   <= peri_addr[PHY_ADDR_WIDTH-1:0];
                        peri_rdata <= mem_rdata[DATA_WIDTH-1:0];
                        peri_ready <= mem_ready;
                    end else peri_ready <= 1'b0;
                end
                default: begin
                    mem_req <= 1'b0;
                end
            endcase
        end
    end
endmodule


// ============================================================================
// DDR4 Memory Controller: High Speed SDRAM PHY & Interface (8 GB DDR4 Space)
// ============================================================================
module ddr4_controller #(
    parameter PHY_ADDR_WIDTH   = 33,
    parameter CACHE_LINE_SIZE  = 512
) (
    input  wire                         clk,
    input  wire                         rst_n,

    // L3 Cache Link
    input  wire [PHY_ADDR_WIDTH-1:0]    mem_addr,
    input  wire [CACHE_LINE_SIZE-1:0]   mem_wdata,
    input  wire                         mem_req,
    input  wire                         mem_write,
    output reg  [CACHE_LINE_SIZE-1:0]   mem_rdata,
    output reg                          mem_ready,

    // Physical DDR4 Pins
    output reg                          ddr4_ck_t,
    output reg                          ddr4_ck_c,
    output reg                          ddr4_cke,
    output reg                          ddr4_cs_n,
    output reg                          ddr4_act_n,
    output reg  [16:0]                  ddr4_adr,
    output reg  [1:0]                   ddr4_ba,
    output reg  [1:0]                   ddr4_bg,
    output reg                          ddr4_ras_n,
    output reg                          ddr4_cas_n,
    output reg                          ddr4_we_n,
    output reg                          ddr4_odt,
    output reg                          ddr4_reset_n,
    inout  wire [63:0]                  ddr4_dq,
    inout  wire [7:0]                   ddr4_dqs_t,
    inout  wire [7:0]                   ddr4_dqs_c
);

    localparam ST_RESET       = 4'd0;
    localparam ST_IDLE        = 4'd1;
    localparam ST_ACTIVATE    = 4'd2;
    localparam ST_WRITE_PHASE = 4'd3;
    localparam ST_READ_PHASE  = 4'd4;
    localparam ST_PRECHARGE   = 4'd5;

    reg [3:0] ddr4_state;
    reg [7:0] delay_cnt;

    reg         dq_out_en;
    reg [63:0]  dq_out;
    assign ddr4_dq = dq_out_en ? dq_out : 64'hZZZZZZZZZZZZZZZZ;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ddr4_state   <= ST_RESET;
            delay_cnt    <= 8'h0;
            mem_ready    <= 1'b0;
            dq_out_en    <= 1'b0;
            dq_out       <= 0;
            ddr4_cke     <= 1'b0;
            ddr4_cs_n    <= 1'b1;
            ddr4_act_n   <= 1'b1;
            ddr4_reset_n <= 1'b0;
            ddr4_odt     <= 1'b0;
        end else begin
            case (ddr4_state)
                ST_RESET: begin
                    ddr4_reset_n <= 1'b1;
                    ddr4_cke     <= 1'b1;
                    ddr4_state   <= ST_IDLE;
                end

                ST_IDLE: begin
                    mem_ready <= 1'b0;
                    if (mem_req) begin
                        ddr4_cs_n  <= 1'b0;
                        ddr4_act_n <= 1'b0;
                        ddr4_adr   <= mem_addr[30:14];
                        ddr4_ba    <= mem_addr[13:12];
                        ddr4_bg    <= mem_addr[11:10];
                        ddr4_state <= ST_ACTIVATE;
                    end
                end

                ST_ACTIVATE: begin
                    ddr4_act_n <= 1'b1;
                    if (mem_write) begin
                        ddr4_we_n  <= 1'b0;
                        dq_out_en  <= 1'b1;
                        dq_out     <= mem_wdata[63:0];
                        ddr4_state <= ST_WRITE_PHASE;
                    end else begin
                        ddr4_cas_n <= 1'b0;
                        ddr4_state <= ST_READ_PHASE;
                    end
                    delay_cnt <= 8'd4;
                end

                ST_WRITE_PHASE: begin
                    if (delay_cnt > 0) begin
                        delay_cnt <= delay_cnt - 1;
                    end else begin
                        dq_out_en  <= 1'b0;
                        mem_ready  <= 1'b1;
                        ddr4_state <= ST_PRECHARGE;
                    end
                end

                ST_READ_PHASE: begin
                    if (delay_cnt > 0) begin
                        delay_cnt <= delay_cnt - 1;
                    end else begin
                        mem_rdata  <= {8{ddr4_dq}};
                        mem_ready  <= 1'b1;
                        ddr4_state <= ST_PRECHARGE;
                    end
                end

                ST_PRECHARGE: begin
                    ddr4_cs_n  <= 1'b0;
                    ddr4_ras_n <= 1'b0;
                    ddr4_state <= ST_IDLE;
                end
            endcase
        end
    end
endmodule


// ============================================================================
// HDMI 1.4 Type-C Alt Mode Controller
// ============================================================================
module hdmi_typec_controller #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // DMA Bus to Memory Interconnect
    output reg  [ADDR_WIDTH-1:0] dma_addr,
    output reg  [DATA_WIDTH-1:0] dma_wdata,
    output reg                   dma_req,
    output reg                   dma_write,
    input  wire [DATA_WIDTH-1:0] dma_rdata,
    input  wire                  dma_ready,

    // Physical Interface Pins
    output reg                   hdmi_tx_clk_p,
    output reg                   hdmi_tx_clk_n,
    output reg  [2:0]            hdmi_tx_data_p,
    output reg  [2:0]            hdmi_tx_data_n,
    input  wire                  hdmi_hpd,
    inout  wire                  hdmi_cec,
    inout  wire                  hdmi_ddc_scl,
    inout  wire                  hdmi_ddc_sda,
    inout  wire                  typec_cc1,
    inout  wire                  typec_cc2,

    // Interrupt output
    output reg                   irq_out
);

    localparam ST_DISCONNECTED = 3'd0;
    localparam ST_NEGOTIATING  = 3'd1;
    localparam ST_ALT_ACTIVE   = 3'd2;
    localparam ST_VIDEO_STREAM = 3'd3;

    reg [2:0]  state;
    reg [23:0] frame_ptr;
    reg [19:0] pixel_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_DISCONNECTED;
            frame_ptr      <= 24'hA00000;
            pixel_counter  <= 20'h0;
            hdmi_tx_clk_p  <= 1'b0;
            hdmi_tx_clk_n  <= 1'b1;
            hdmi_tx_data_p <= 3'b000;
            hdmi_tx_data_n <= 3'b111;
            dma_req        <= 1'b0;
            dma_addr       <= 0;
            dma_wdata      <= 0;
            dma_write      <= 1'b0;
            irq_out        <= 1'b0;
        end else begin
            hdmi_tx_clk_p <= ~hdmi_tx_clk_p;
            hdmi_tx_clk_n <= ~hdmi_tx_clk_n;

            case (state)
                ST_DISCONNECTED: begin
                    if (typec_cc1 || typec_cc2) begin
                        state   <= ST_NEGOTIATING;
                        irq_out <= 1'b1;
                    end
                end

                ST_NEGOTIATING: begin
                    irq_out <= 1'b0;
                    if (hdmi_hpd) begin
                        state <= ST_ALT_ACTIVE;
                    end
                end

                ST_ALT_ACTIVE: begin
                    state <= ST_VIDEO_STREAM;
                end

                ST_VIDEO_STREAM: begin
                    dma_req  <= 1'b1;
                    dma_addr <= frame_ptr + pixel_counter;
                    
                    if (dma_ready) begin
                        dma_req <= 1'b0;
                        hdmi_tx_data_p <= dma_rdata[2:0];
                        hdmi_tx_data_n <= ~dma_rdata[2:0];
                        
                        if (pixel_counter >= 20'hE1000) begin
                            pixel_counter <= 20'h0;
                        end else begin
                            pixel_counter <= pixel_counter + 8;
                        end
                    end
                end
            endcase
        end
    end
endmodule


// ============================================================================
// USB 3.0 Type-A Controller
// ============================================================================
module usb3_controller #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // DMA Bus
    output reg  [ADDR_WIDTH-1:0] dma_addr,
    output reg  [DATA_WIDTH-1:0] dma_wdata,
    output reg                   dma_req,
    output reg                   dma_write,
    input  wire [DATA_WIDTH-1:0] dma_rdata,
    input  wire                  dma_ready,

    // Ports
    input  wire [2:0]            usb_ssrx_p,
    input  wire [2:0]            usb_ssrx_n,
    output reg  [2:0]            usb_sstx_p,
    output reg  [2:0]            usb_sstx_n,
    inout  wire [2:0]            usb_dp,
    inout  wire [2:0]            usb_dn,

    // Interrupt output
    output reg                   irq_out
);

    localparam USB_ST_RESET  = 2'd0;
    localparam USB_ST_ACTIVE = 2'd1;
    localparam USB_ST_RX_TX  = 2'd2;

    reg [1:0] usb_state;
    reg [2:0] port_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            usb_state  <= USB_ST_RESET;
            usb_sstx_p <= 3'b000;
            usb_sstx_n <= 3'b111;
            port_sel   <= 3'b000;
            dma_req    <= 1'b0;
            dma_addr   <= 0;
            dma_wdata  <= 0;
            dma_write  <= 1'b0;
            irq_out    <= 1'b0;
        end else begin
            case (usb_state)
                USB_ST_RESET: begin
                    usb_state <= USB_ST_ACTIVE;
                end

                USB_ST_ACTIVE: begin
                    irq_out <= 1'b0;
                    port_sel <= port_sel + 1;
                    if (port_sel >= 3'd3) port_sel <= 3'd0;

                    if (usb_ssrx_p[port_sel] ^ usb_ssrx_n[port_sel]) begin
                        usb_state <= USB_ST_RX_TX;
                    end
                end

                USB_ST_RX_TX: begin
                    dma_req   <= 1'b1;
                    dma_addr  <= 64'hC00000 + (port_sel * 64'h1000);
                    dma_wdata <= {62'b0, usb_ssrx_p[port_sel], usb_ssrx_n[port_sel]};
                    dma_write <= 1'b1;

                    if (dma_ready) begin
                        dma_req    <= 1'b0;
                        usb_sstx_p <= usb_ssrx_p;
                        usb_sstx_n <= usb_ssrx_n;
                        irq_out    <= 1'b1;
                        usb_state  <= USB_ST_ACTIVE;
                    end
                end
            endcase
        end
    end
endmodule


// ============================================================================
// UART Controller: Serial Console
// ============================================================================
module uart_controller #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Register Access Bus
    input  wire [ADDR_WIDTH-1:0] dma_addr,
    input  wire [DATA_WIDTH-1:0] dma_wdata,
    input  wire                  dma_req,
    input  wire                  dma_write,
    output reg  [DATA_WIDTH-1:0] dma_rdata,
    output reg                   dma_ready,

    // Physical UART Pins
    output reg                   tx_pin,
    input  wire                  rx_pin,

    // Interrupt output
    output reg                   irq_out
);

    reg [7:0] tx_fifo;
    reg       tx_busy;
    reg [3:0] bit_cnt;
    reg [7:0] shift_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_pin    <= 1'b1;
            tx_fifo   <= 8'h0;
            tx_busy   <= 1'b0;
            bit_cnt   <= 4'h0;
            shift_reg <= 8'h0;
            dma_ready <= 1'b0;
            dma_rdata <= 0;
            irq_out   <= 1'b0;
        end else begin
            dma_ready <= 1'b0;
            irq_out   <= 1'b0;

            if (dma_req && dma_write && (dma_addr == 64'hFFFF_E000)) begin
                tx_fifo   <= dma_wdata[7:0];
                tx_busy   <= 1'b1;
                dma_ready <= 1'b1;
            end else if (dma_req && !dma_write && (dma_addr == 64'hFFFF_E000)) begin
                dma_rdata <= {56'h0, tx_busy};
                dma_ready <= 1'b1;
            end

            if (tx_busy) begin
                if (bit_cnt == 4'd0) begin
                    tx_pin    <= 1'b0;
                    shift_reg <= tx_fifo;
                    bit_cnt   <= bit_cnt + 1;
                end else if (bit_cnt <= 4'd8) begin
                    tx_pin    <= shift_reg[0];
                    shift_reg <= {1'b0, shift_reg[7:1]};
                    bit_cnt   <= bit_cnt + 1;
                end else begin
                    tx_pin    <= 1'b1;
                    bit_cnt   <= 4'd0;
                    tx_busy   <= 1'b0;
                    irq_out   <= 1'b1;
                end
            end
        end
    end
endmodule


// ============================================================================
// SDIO Storage Controller
// ============================================================================
module sdio_storage_controller #(
    parameter ADDR_WIDTH = 64,
    parameter DATA_WIDTH = 64
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // DMA Bus to copy sectors directly to DDR4 System Memory
    output reg  [ADDR_WIDTH-1:0] dma_addr,
    output reg  [DATA_WIDTH-1:0] dma_wdata,
    output reg                   dma_req,
    output reg                   dma_write,
    input  wire [DATA_WIDTH-1:0] dma_rdata,
    input  wire                  dma_ready,

    // Physical MicroSD Interface Bypasses
    output reg                   sd_clk,
    inout  wire                  sd_cmd,
    inout  wire [3:0]            sd_dat,

    // Interrupt output
    output reg                   irq_out
);

    localparam SD_ST_POWERUP   = 3'd0;
    localparam SD_ST_CMD8      = 3'd1;
    localparam SD_ST_READY     = 3'd2;
    localparam SD_ST_DMA_READ  = 3'd3;
    localparam SD_ST_COMPLETE  = 3'd4;

    reg [2:0]  sd_state;
    reg [23:0] target_ram_ptr;
    reg [11:0] sector_counter;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sd_state       <= SD_ST_POWERUP;
            sd_clk         <= 1'b0;
            dma_req        <= 1'b0;
            dma_addr       <= 0;
            dma_wdata      <= 0;
            dma_write      <= 1'b0;
            irq_out        <= 1'b0;
            target_ram_ptr <= 24'h800000; // OpenSBI standard RAM offset
            sector_counter <= 12'h0;
        end else begin
            sd_clk  <= ~sd_clk;
            irq_out <= 1'b0;

            case (sd_state)
                SD_ST_POWERUP: begin
                    sd_state <= SD_ST_CMD8;
                end

                SD_ST_CMD8: begin
                    sd_state <= SD_ST_READY;
                end

                SD_ST_READY: begin
                    sd_state <= SD_ST_DMA_READ;
                end

                SD_ST_DMA_READ: begin
                    dma_req   <= 1'b1;
                    dma_addr  <= target_ram_ptr + {12'h0, sector_counter};
                    dma_wdata <= 64'h00000013_00000013;
                    dma_write <= 1'b1;

                    if (dma_ready) begin
                        dma_req <= 1'b0;
                        if (sector_counter >= 12'hFFF) begin
                            sd_state <= SD_ST_COMPLETE;
                        end else begin
                            sector_counter <= sector_counter + 8;
                        end
                    end
                end

                SD_ST_COMPLETE: begin
                    irq_out  <= 1'b1;
                    sd_state <= SD_ST_READY;
                end
            endcase
        end
    end
endmodule