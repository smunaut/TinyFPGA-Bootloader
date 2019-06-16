`default_nettype none

module bootloader (
  input  pin_clk,

  inout  pin_usbp,
  inout  pin_usbn,
  output pin_pu,

  output [2:0] rgb,

  input  pin_29_miso,
  output pin_30_cs,
  output pin_31_mosi,
  output pin_32_sck
);

  wire [2:0] rgb_pwm;

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////
  //////// generate 48 mhz clock
  ////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  wire clk_48mhz;
  reg [7:0] reset_cnt;
  wire reset;
  wire pll_lock;

  SB_PLL40_CORE #(
    .DIVR(4'b0000),
    .DIVF(7'b0111111),
    .DIVQ(3'b100),
    .FILTER_RANGE(3'b001),
    .FEEDBACK_PATH("SIMPLE"),
    .DELAY_ADJUSTMENT_MODE_FEEDBACK("FIXED"),
    .FDA_FEEDBACK(4'b0000),
    .DELAY_ADJUSTMENT_MODE_RELATIVE("FIXED"),
    .FDA_RELATIVE(4'b0000),
    .SHIFTREG_DIV_MODE(2'b00),
    .PLLOUT_SELECT("GENCLK"),
    .ENABLE_ICEGATE(1'b0)
  ) usb_pll_inst (
    .REFERENCECLK(pin_clk),
    .PLLOUTCORE(),
    .PLLOUTGLOBAL(clk_48mhz),
    .EXTFEEDBACK(),
    .DYNAMICDELAY(),
    .RESETB(1'b1),
    .BYPASS(1'b0),
    .LATCHINPUTVALUE(),
    .LOCK(pll_lock),
    .SDI(),
    .SDO(),
    .SCLK()
  );

  always @(posedge clk_48mhz or negedge pll_lock)
    if (!pll_lock)
      reset_cnt <= 8'hff;
    else if (reset_cnt[7])
      reset_cnt <= reset_cnt - 1;

  assign reset = reset_cnt[7];


  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////
  //////// interface with iCE40 warmboot/multiboot capability
  ////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  wire boot;

//  SB_WARMBOOT warmboot_inst (
//    .S1(1'b0),
//    .S0(1'b1),
//    .BOOT(boot)
//  );

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////
  //////// instantiate tinyfpga bootloader
  ////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  wire usb_p_tx, usb_n_tx;
  wire usb_p_rx, usb_n_rx;
  wire usb_p_rx_i, usb_n_rx_i;
  wire usb_tx_en;

  tinyfpga_bootloader tinyfpga_bootloader_inst (
    .clk_48mhz(clk_48mhz),
    .reset(reset),
    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en),
    .led(rgb_pwm[1]),
    .spi_miso(pin_29_miso),
    .spi_cs(pin_30_cs),
    .spi_mosi(pin_31_mosi),
    .spi_sck(pin_32_sck),
    .boot(boot)
  );

  assign pin_pu = 1'b1;

  SB_IO #(
    .PIN_TYPE(6'b101001),
    .PULLUP(1'b0)
  ) usb_io_I[1:0] (
    .PACKAGE_PIN  ({pin_usbp,   pin_usbn}),
    .OUTPUT_ENABLE(usb_tx_en),
    .D_OUT_0      ({usb_p_tx,   usb_n_tx}),
    .D_IN_0       ({usb_p_rx_i, usb_n_rx_i})
  );

  assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_rx_i;
  assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_rx_i;


  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////
  //////// debug leds
  ////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  SB_RGBA_DRV #(
      .CURRENT_MODE("0b1"),
      .RGB0_CURRENT("0b000001"),
      .RGB1_CURRENT("0b000001"),
      .RGB2_CURRENT("0b000001")
  ) rgb_drv_I (
      .RGBLEDEN(1'b1),
      .RGB0PWM(rgb_pwm[0]),
      .RGB1PWM(rgb_pwm[1]),
      .RGB2PWM(rgb_pwm[2]),
      .CURREN(1'b1),
      .RGB0(rgb[0]),
      .RGB1(rgb[1]),
      .RGB2(rgb[2])
  );

  assign rgb_pwm[0] = reset;
  assign rgb_pwm[2] = 1'b0;

endmodule
