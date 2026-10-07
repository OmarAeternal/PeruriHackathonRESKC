-- IMU window buffer, WIN_SLOTS windows of SEQ_LEN x N_IN INT8 samples in M10K.
-- HPS writes 32-bit words (4 samples, little-endian); the controller reads one
-- sample per cycle using a byte address (read latency 1).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity input_window_buf is
  port (
    clk      : in  std_logic;
    wr_en    : in  std_logic;
    wr_addr  : in  unsigned(WINW_AW-1 downto 0);
    wr_data  : in  std_logic_vector(31 downto 0);
    rd_addr  : in  unsigned(WIN_AW-1 downto 0);
    x_out    : out int8_t
  );
end entity;

architecture rtl of input_window_buf is
  type win_ram_t is array (0 to WINW_DEPTH-1) of std_logic_vector(31 downto 0);
  signal win_ram : win_ram_t;
  signal rd_word : std_logic_vector(31 downto 0) := (others => '0');
  signal rd_sel  : unsigned(1 downto 0) := (others => '0');

  attribute ramstyle : string;
  attribute ramstyle of win_ram : signal is "M10K";
  attribute ram_init_file : string;
  attribute ram_init_file of win_ram : signal is "imu_window.mif";
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if wr_en = '1' and wr_addr < WINW_DEPTH then
        win_ram(to_integer(wr_addr)) <= wr_data;
      end if;
      rd_word <= win_ram(to_integer(rd_addr(WIN_AW-1 downto 2)));
      rd_sel  <= rd_addr(1 downto 0);
    end if;
  end process;

  with rd_sel select x_out <=
    signed(rd_word( 7 downto  0)) when "00",
    signed(rd_word(15 downto  8)) when "01",
    signed(rd_word(23 downto 16)) when "10",
    signed(rd_word(31 downto 24)) when others;
end architecture;
