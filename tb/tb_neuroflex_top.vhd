-- Integration test: loads weights, biases and two IMU windows through the
-- Avalon-MM slave (as the HPS driver does), runs every window in every CORDIC
-- mode and compares class/score with the Python golden model.
-- A final run uses the ext_* conduit (standalone board path).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use work.neuroflex_pkg.all;

entity tb_neuroflex_top is
  generic (
    MODEL_FILE    : string := "model_load.txt";
    WINDOWS_FILE  : string := "windows_load.txt";
    EXPECTED_FILE : string := "expected.txt"
  );
end entity;

architecture sim of tb_neuroflex_top is
  constant REG_CTRL   : natural := 16#00#;
  constant REG_STATUS : natural := 16#04#;
  constant REG_RESULT : natural := 16#08#;
  constant REG_CYCLES : natural := 16#0C#;
  constant REG_INFER  : natural := 16#10#;
  constant REG_ID     : natural := 16#14#;
  constant REG_CONFIG : natural := 16#18#;

  signal clk           : std_logic := '0';
  signal reset         : std_logic := '1';
  signal avs_address   : std_logic_vector(10 downto 0) := (others => '0');
  signal avs_write     : std_logic := '0';
  signal avs_read      : std_logic := '0';
  signal avs_writedata : std_logic_vector(31 downto 0) := (others => '0');
  signal avs_readdata  : std_logic_vector(31 downto 0);
  signal ext_start     : std_logic := '0';
  signal ext_mode      : std_logic_vector(1 downto 0) := "10";
  signal ext_win_sel   : std_logic := '0';
  signal st_busy       : std_logic;
  signal st_done       : std_logic;
  signal st_class      : std_logic_vector(7 downto 0);
  signal st_score      : std_logic_vector(15 downto 0);
  signal finished      : boolean := false;
begin
  clk <= not clk after 10 ns when not finished;

  dut : entity work.neuroflex_top
    port map (clk => clk, reset => reset,
              avs_address => avs_address, avs_write => avs_write,
              avs_writedata => avs_writedata, avs_read => avs_read,
              avs_readdata => avs_readdata,
              ext_start => ext_start, ext_mode => ext_mode, ext_win_sel => ext_win_sel,
              st_busy => st_busy, st_done => st_done,
              st_class => st_class, st_score => st_score);

  stim : process
    variable errors : integer := 0;
    variable rd     : std_logic_vector(31 downto 0);

    procedure av_write(addr : natural; data : std_logic_vector(31 downto 0)) is
    begin
      avs_address   <= std_logic_vector(to_unsigned(addr / 4, 11));
      avs_writedata <= data;
      avs_write     <= '1';
      wait until rising_edge(clk);
      avs_write     <= '0';
    end procedure;

    procedure av_read(addr : natural; data : out std_logic_vector(31 downto 0)) is
    begin
      avs_address <= std_logic_vector(to_unsigned(addr / 4, 11));
      avs_read    <= '1';
      wait until rising_edge(clk);
      avs_read    <= '0';
      wait for 1 ns;
      data := avs_readdata;
    end procedure;

    procedure load_file(name : string) is
      file     f    : text open read_mode is name;
      variable l    : line;
      variable addr : std_logic_vector(15 downto 0);
      variable data : std_logic_vector(31 downto 0);
      variable n    : integer := 0;
    begin
      while not endfile(f) loop
        readline(f, l);
        hread(l, addr);
        hread(l, data);
        av_write(to_integer(unsigned(addr)), data);
        n := n + 1;
      end loop;
      report "Loaded " & integer'image(n) & " word dari " & name severity note;
    end procedure;

    procedure wait_done(cycles : out integer) is
      variable st : std_logic_vector(31 downto 0);
      variable n  : integer := 0;
    begin
      loop
        for i in 1 to 500 loop
          wait until rising_edge(clk);
        end loop;
        n := n + 500;
        av_read(REG_STATUS, st);
        exit when st(1) = '1';
        assert n < 2_000_000 report "Timeout menunggu DONE" severity failure;
      end loop;
      cycles := n;
    end procedure;

    file     f_exp   : text;
    variable l       : line;
    variable slot, mode, cls_exp, score_exp : integer;
    variable cls, score, cyc, n : integer;
    variable ext_slot_mode_cls : integer := -1;
    variable n_runs  : integer := 0;
  begin
    reset <= '1';
    for i in 1 to 4 loop wait until rising_edge(clk); end loop;
    reset <= '0';
    wait until rising_edge(clk);

    av_read(REG_ID, rd);
    if rd /= NF_ID then
      report "ID register salah: " & to_hstring(rd) severity error;
      errors := errors + 1;
    end if;
    av_read(REG_CONFIG, rd);
    report "CONFIG = 0x" & to_hstring(rd) & " (SEQ_LEN|N_CLS|N_HID|N_IN)" severity note;

    load_file(MODEL_FILE);
    load_file(WINDOWS_FILE);

    file_open(f_exp, EXPECTED_FILE, read_mode);
    while not endfile(f_exp) loop
      readline(f_exp, l);
      read(l, slot); read(l, mode); read(l, cls_exp); read(l, score_exp);

      av_write(REG_CTRL, std_logic_vector(to_unsigned(slot*8 + mode*2 + 1, 32)));
      wait_done(n);
      av_read(REG_RESULT, rd);
      cls   := to_integer(unsigned(rd(7 downto 0)));
      score := to_integer(unsigned(rd(31 downto 16)));
      av_read(REG_CYCLES, rd);
      cyc   := to_integer(unsigned(rd));
      n_runs := n_runs + 1;

      report "slot=" & integer'image(slot) & " mode=" & integer'image(mode)
           & " class=" & integer'image(cls) & " score=" & integer'image(score)
           & " (golden " & integer'image(cls_exp) & "/" & integer'image(score_exp) & ")"
           & " cycles=" & integer'image(cyc) severity note;
      if cls /= cls_exp or score /= score_exp then
        report "Mismatch terhadap golden model" severity error;
        errors := errors + 1;
      end if;
      if slot = 1 and mode = 2 then
        ext_slot_mode_cls := cls_exp;
      end if;
    end loop;
    file_close(f_exp);

    av_read(REG_INFER, rd);
    if to_integer(unsigned(rd)) /= n_runs then
      report "INFER_CNT salah" severity error;
      errors := errors + 1;
    end if;

    ext_mode    <= "10";
    ext_win_sel <= '1';
    ext_start   <= '1';
    wait until rising_edge(clk);
    ext_start   <= '0';
    wait until rising_edge(clk);
    if st_busy /= '1' then
      report "ext_start tidak memulai inferensi" severity error;
      errors := errors + 1;
    end if;
    wait until st_done = '1';
    if ext_slot_mode_cls >= 0 and to_integer(unsigned(st_class)) /= ext_slot_mode_cls then
      report "Conduit: class salah" severity error;
      errors := errors + 1;
    end if;
    report "Conduit run: class=" & integer'image(to_integer(unsigned(st_class)))
         & " score=" & integer'image(to_integer(unsigned(st_score))) severity note;

    if errors = 0 then
      report "tb_neuroflex_top: PASS" severity note;
    else
      report "tb_neuroflex_top: FAIL (" & integer'image(errors) & " error)" severity failure;
    end if;
    finished <= true;
    wait;
  end process;
end architecture;
