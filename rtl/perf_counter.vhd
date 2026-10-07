-- Cycle counter per inference and total inference counter.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity perf_counter is
  port (
    clk, rst    : in  std_logic;
    start       : in  std_logic;
    busy        : in  std_logic;
    done        : in  std_logic;
    last_cycles : out unsigned(31 downto 0);
    infer_cnt   : out unsigned(31 downto 0)
  );
end entity;

architecture rtl of perf_counter is
  signal cycle_cnt : unsigned(31 downto 0) := (others => '0');
  signal last_r    : unsigned(31 downto 0) := (others => '0');
  signal infer_r   : unsigned(31 downto 0) := (others => '0');
begin
  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        cycle_cnt <= (others => '0');
        last_r    <= (others => '0');
        infer_r   <= (others => '0');
      else
        if start = '1' then
          cycle_cnt <= (others => '0');
        elsif busy = '1' then
          cycle_cnt <= cycle_cnt + 1;
        end if;
        if done = '1' then
          infer_r <= infer_r + 1;
          last_r  <= cycle_cnt;
        end if;
      end if;
    end if;
  end process;

  last_cycles <= last_r;
  infer_cnt   <= infer_r;
end architecture;
