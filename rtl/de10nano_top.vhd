-- Standalone DE10-Nano demo (no HPS needed). Weights and two IMU windows come
-- from the .mif files.
--   KEY[0]   : reset (active low)
--   KEY[1]   : start inference
--   SW[1:0]  : CORDIC mode (00 = 8, 01 = 12, 1x = 16 iterations)
--   SW[2]    : window slot (0 = NORMAL example, 1 = ANOMALI example)
--   LED[0]   : class (1 = ANOMALI)
--   LED[1]   : busy
--   LED[2]   : done
--   LED[3]   : heartbeat
--   LED[7:4] : integer part of the score (logit margin)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity de10nano_top is
  port (
    FPGA_CLK1_50 : in  std_logic;
    KEY          : in  std_logic_vector(1 downto 0);
    SW           : in  std_logic_vector(3 downto 0);
    LED          : out std_logic_vector(7 downto 0)
  );
end entity;

architecture rtl of de10nano_top is
  constant DEB_BITS : integer := 20;

  signal clk        : std_logic;
  signal rst_sync   : std_logic_vector(1 downto 0) := (others => '1');
  signal rst        : std_logic;
  signal key_sync   : std_logic_vector(1 downto 0) := (others => '1');
  signal key_stable : std_logic := '1';
  signal key_prev   : std_logic := '1';
  signal deb_cnt    : unsigned(DEB_BITS-1 downto 0) := (others => '0');
  signal start      : std_logic := '0';
  signal sw_sync    : std_logic_vector(2 downto 0) := (others => '0');
  signal hb_cnt     : unsigned(24 downto 0) := (others => '0');

  signal st_busy, st_done : std_logic;
  signal st_class         : std_logic_vector(7 downto 0);
  signal st_score         : std_logic_vector(15 downto 0);
begin
  clk <= FPGA_CLK1_50;
  rst <= rst_sync(1);

  process(clk)
  begin
    if rising_edge(clk) then
      rst_sync <= rst_sync(0) & not KEY(0);
    end if;
  end process;

  process(clk)
  begin
    if rising_edge(clk) then
      key_sync <= key_sync(0) & KEY(1);
      sw_sync  <= SW(2 downto 0);
      start    <= '0';
      if key_sync(1) /= key_stable then
        deb_cnt <= deb_cnt + 1;
        if deb_cnt = (deb_cnt'range => '1') then
          key_stable <= key_sync(1);
          deb_cnt    <= (others => '0');
        end if;
      else
        deb_cnt <= (others => '0');
      end if;
      key_prev <= key_stable;
      if key_prev = '1' and key_stable = '0' then
        start <= '1';
      end if;
      hb_cnt <= hb_cnt + 1;
    end if;
  end process;

  u_nf : entity work.neuroflex_top
    port map (clk => clk, reset => rst,
              avs_address => (others => '0'), avs_write => '0',
              avs_writedata => (others => '0'), avs_read => '0', avs_readdata => open,
              ext_start => start, ext_mode => sw_sync(1 downto 0), ext_win_sel => sw_sync(2),
              st_busy => st_busy, st_done => st_done,
              st_class => st_class, st_score => st_score);

  LED(0)          <= st_class(0);
  LED(1)          <= st_busy;
  LED(2)          <= st_done;
  LED(3)          <= hb_cnt(hb_cnt'high);
  LED(7 downto 4) <= st_score(13 downto 10);
end architecture;
