-- INT8 multiply-accumulate engine, LANES products per cycle into a 24-bit accumulator.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity int8_mac_engine is
  generic ( LANES : integer := 4 );
  port (
    clk, rst  : in  std_logic;
    clr, en   : in  std_logic;
    a_vec     : in  int8_array(0 to LANES-1);
    b_vec     : in  int8_array(0 to LANES-1);
    acc_out   : out acc_t
  );
end entity;

architecture rtl of int8_mac_engine is
  signal acc : acc_t := (others => '0');
begin
  process(clk)
    variable sum : acc_t;
  begin
    if rising_edge(clk) then
      if rst = '1' or clr = '1' then
        acc <= (others => '0');
      elsif en = '1' then
        sum := acc;
        for l in 0 to LANES-1 loop
          sum := sum + resize(a_vec(l) * b_vec(l), W_ACC);
        end loop;
        acc <= sum;
      end if;
    end if;
  end process;

  acc_out <= acc;
end architecture;
