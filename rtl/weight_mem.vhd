-- INT8 weight rows (LANES weights per 32-bit word) and Q5.10 biases in M10K.
-- Row order: Z rows, R rows, H rows (N_HID each), then N_CLS dense rows.
-- Bias order: bz, br, b_in, b_hn (N_HID each), then dense bias.
-- Initialised from .mif (Quartus), can be overwritten by the HPS at runtime.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity weight_mem is
  port (
    clk        : in  std_logic;
    hps_we     : in  std_logic;
    hps_addr   : in  unsigned(WROW_AW-1 downto 0);
    hps_wdata  : in  std_logic_vector(31 downto 0);
    hps_bwe    : in  std_logic;
    hps_baddr  : in  unsigned(BIAS_AW-1 downto 0);
    rd_addr    : in  unsigned(WROW_AW-1 downto 0);
    w_out      : out int8_array(0 to LANES-1);
    b_rd_addr  : in  unsigned(BIAS_AW-1 downto 0);
    b_out      : out fx_t
  );
end entity;

architecture rtl of weight_mem is
  type wmem_t is array (0 to WROW_DEPTH-1) of std_logic_vector(LANES*W_DATA-1 downto 0);
  type bmem_t is array (0 to N_BIAS-1) of std_logic_vector(W_FX-1 downto 0);
  signal wmem : wmem_t;
  signal bmem : bmem_t;
  signal w_q  : std_logic_vector(LANES*W_DATA-1 downto 0) := (others => '0');
  signal b_q  : std_logic_vector(W_FX-1 downto 0) := (others => '0');

  attribute ramstyle : string;
  attribute ramstyle of wmem : signal is "M10K";
  attribute ram_init_file : string;
  attribute ram_init_file of wmem : signal is "tinygru_wrow.mif";
  attribute ram_init_file of bmem : signal is "tinygru_bias.mif";
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if hps_we = '1' and hps_addr < WROW_DEPTH then
        wmem(to_integer(hps_addr)) <= hps_wdata(LANES*W_DATA-1 downto 0);
      end if;
      w_q <= wmem(to_integer(rd_addr));
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      if hps_bwe = '1' and hps_baddr < N_BIAS then
        bmem(to_integer(hps_baddr)) <= hps_wdata(W_FX-1 downto 0);
      end if;
      b_q <= bmem(to_integer(b_rd_addr));
    end if;
  end process;

  g_lane : for l in 0 to LANES-1 generate
    w_out(l) <= signed(w_q(8*l+7 downto 8*l));
  end generate;

  b_out <= signed(b_q);
end architecture;
