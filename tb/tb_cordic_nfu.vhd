-- Unit test CORDIC NFU: sweeps z over the model range for every mode (8/12/16
-- iterations) and function (tanh/sigmoid). Output must match the golden model
-- bit-exactly; the error versus the floating-point reference is reported.
-- Line format of REF_FILE: mode is_sigmoid z golden ideal (Q5.10 integers).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.neuroflex_pkg.all;

entity tb_cordic_nfu is
  generic ( REF_FILE : string := "cordic_ref.txt" );
end entity;

architecture sim of tb_cordic_nfu is
  type err_arr_t is array (0 to 2, 0 to 1) of integer;
  constant ERR_LIMIT : err_arr_t := ((16, 16), (2, 2), (2, 2));
  type fn_name_t is array (0 to 1) of string(1 to 7);
  constant FN_NAME   : fn_name_t := ("tanh   ", "sigmoid");

  signal clk        : std_logic := '0';
  signal rst        : std_logic := '0';
  signal start      : std_logic := '0';
  signal mode       : std_logic_vector(1 downto 0) := "10";
  signal is_sigmoid : std_logic := '0';
  signal z_in       : fx_t := (others => '0');
  signal f_out      : fx_t;
  signal valid      : std_logic;
  signal finished   : boolean := false;
begin
  clk <= not clk after 10 ns when not finished;

  dut : entity work.cordic_nfu
    port map (clk => clk, rst => rst, start => start, mode => mode,
              is_sigmoid => is_sigmoid, z_in => z_in, f_out => f_out,
              valid => valid, busy => open);

  stim : process
    file     f_ref     : text open read_mode is REF_FILE;
    variable l         : line;
    variable m, s, x   : integer;
    variable gold, ref : integer;
    variable err       : integer;
    variable max_err   : err_arr_t := (others => (others => 0));
    variable sum_err   : err_arr_t := (others => (others => 0));
    variable cnt       : err_arr_t := (others => (others => 0));
    variable mismatch  : integer := 0;
    variable cycles    : integer;
    variable lat_max   : integer := 0;
  begin
    rst <= '1'; wait until rising_edge(clk); rst <= '0';
    while not endfile(f_ref) loop
      readline(f_ref, l);
      read(l, m); read(l, s); read(l, x); read(l, gold); read(l, ref);
      mode       <= std_logic_vector(to_unsigned(m, 2));
      is_sigmoid <= '1' when s = 1 else '0';
      z_in       <= to_signed(x, W_FX);
      start      <= '1';
      wait until rising_edge(clk);
      start      <= '0';
      cycles     := 0;
      loop
        wait until rising_edge(clk);
        cycles := cycles + 1;
        exit when valid = '1';
      end loop;
      if cycles > lat_max then lat_max := cycles; end if;
      if to_integer(f_out) /= gold then
        mismatch := mismatch + 1;
        report "Mismatch mode=" & integer'image(m) & " sig=" & integer'image(s)
             & " z=" & integer'image(x) & " rtl=" & integer'image(to_integer(f_out))
             & " golden=" & integer'image(gold) severity error;
      end if;
      err := abs(to_integer(f_out) - ref);
      if err > max_err(m, s) then max_err(m, s) := err; end if;
      sum_err(m, s) := sum_err(m, s) + err;
      cnt(m, s)     := cnt(m, s) + 1;
    end loop;

    for mi in 0 to 2 loop
      for si in 0 to 1 loop
        if cnt(mi, si) > 0 then
          report "mode " & integer'image(mi) & " " & FN_NAME(si)
               & " : max error = " & integer'image(max_err(mi, si))
               & " LSB, mean error x1000 = " & integer'image((1000 * sum_err(mi, si)) / cnt(mi, si))
               & " LSB (Q5.10)" severity note;
          assert max_err(mi, si) <= ERR_LIMIT(mi, si)
            report "CORDIC error melebihi batas" severity error;
        end if;
      end loop;
    end loop;
    report "Latency maksimum = " & integer'image(lat_max) & " cycle" severity note;

    if mismatch = 0 then
      report "tb_cordic_nfu: PASS (bit-exact dengan golden model)" severity note;
    else
      report "tb_cordic_nfu: FAIL (" & integer'image(mismatch) & " mismatch)" severity failure;
    end if;
    finished <= true;
    wait;
  end process;
end architecture;
