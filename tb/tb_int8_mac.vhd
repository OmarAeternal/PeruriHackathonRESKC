-- Unit test INT8 MAC: corner case -128 x -128, clear/enable behaviour and
-- random vectors compared against a direct computation in the testbench.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.neuroflex_pkg.all;

entity tb_int8_mac is
end entity;

architecture sim of tb_int8_mac is
  constant NL : integer := 4;
  signal clk      : std_logic := '0';
  signal rst      : std_logic := '0';
  signal clr, en  : std_logic := '0';
  signal a_vec    : int8_array(0 to NL-1) := (others => (others => '0'));
  signal b_vec    : int8_array(0 to NL-1) := (others => (others => '0'));
  signal acc_out  : acc_t;
  signal finished : boolean := false;
begin
  clk <= not clk after 10 ns when not finished;

  dut : entity work.int8_mac_engine
    generic map (LANES => NL)
    port map (clk => clk, rst => rst, clr => clr, en => en,
              a_vec => a_vec, b_vec => b_vec, acc_out => acc_out);

  stim : process
    variable s1, s2    : positive := 7;
    variable r         : real;
    variable a, b      : integer;
    variable expected  : integer;
    variable errors    : integer := 0;
  begin
    rst <= '1'; wait until rising_edge(clk); rst <= '0';

    a_vec <= (others => to_signed(-128, 8));
    b_vec <= (others => to_signed(-128, 8));
    clr <= '1'; wait until rising_edge(clk); clr <= '0';
    en  <= '1';
    for n in 1 to 32 loop wait until rising_edge(clk); end loop;
    en  <= '0'; wait until rising_edge(clk);
    if acc_out /= to_signed(32*4*16384, W_ACC) then
      report "MAC overflow / sign-extension salah" severity error;
      errors := errors + 1;
    end if;

    a_vec <= (others => to_signed(127, 8));
    b_vec <= (others => to_signed(-128, 8));
    en  <= '1'; clr <= '1'; wait until rising_edge(clk);
    clr <= '0'; wait until rising_edge(clk);
    en  <= '0'; wait until rising_edge(clk);
    if acc_out /= to_signed(4*127*(-128), W_ACC) then
      report "clear tidak memiliki prioritas di atas enable" severity error;
      errors := errors + 1;
    end if;

    for blk in 0 to 3 loop
      clr <= '1'; wait until rising_edge(clk); clr <= '0';
      expected := 0;
      for n in 0 to 249 loop
        for l in 0 to NL-1 loop
          uniform(s1, s2, r);  a := integer(floor(r * 256.0)) - 128;
          uniform(s1, s2, r);  b := integer(floor(r * 256.0)) - 128;
          a_vec(l) <= to_signed(a, 8);
          b_vec(l) <= to_signed(b, 8);
          expected := expected + a*b;
        end loop;
        en <= '1';
        wait until rising_edge(clk);
      end loop;
      en <= '0'; wait until rising_edge(clk);
      if acc_out /= to_signed(expected, W_ACC) then
        report "Random block " & integer'image(blk) & ": acc=" & integer'image(to_integer(acc_out))
             & " expected=" & integer'image(expected) severity error;
        errors := errors + 1;
      end if;
    end loop;

    if errors = 0 then
      report "tb_int8_mac: PASS" severity note;
    else
      report "tb_int8_mac: FAIL (" & integer'image(errors) & " error)" severity failure;
    end if;
    finished <= true;
    wait;
  end process;
end architecture;
