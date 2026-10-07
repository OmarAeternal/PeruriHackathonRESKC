-- NeuroFlex-CORDIC accelerator top level (Avalon-MM slave + status conduit).
-- Used as a Platform Designer component behind the HPS Lightweight bridge,
-- or standalone through the ext_* conduit (see de10nano_top).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity neuroflex_top is
  port (
    clk           : in  std_logic;
    reset         : in  std_logic;
    avs_address   : in  std_logic_vector(10 downto 0);
    avs_write     : in  std_logic;
    avs_writedata : in  std_logic_vector(31 downto 0);
    avs_read      : in  std_logic;
    avs_readdata  : out std_logic_vector(31 downto 0);
    ext_start     : in  std_logic;
    ext_mode      : in  std_logic_vector(1 downto 0);
    ext_win_sel   : in  std_logic;
    st_busy       : out std_logic;
    st_done       : out std_logic;
    st_class      : out std_logic_vector(7 downto 0);
    st_score      : out std_logic_vector(15 downto 0)
  );
end entity;

architecture rtl of neuroflex_top is
  signal rst          : std_logic;
  signal br_start     : std_logic;
  signal br_mode      : std_logic_vector(1 downto 0);
  signal br_win       : std_logic;
  signal start        : std_logic;
  signal start_win    : std_logic;
  signal run_mode     : std_logic_vector(1 downto 0) := "10";
  signal busy         : std_logic;
  signal done         : std_logic;
  signal done_flag    : std_logic := '0';
  signal class_out    : std_logic_vector(7 downto 0);
  signal score        : fx_t;
  signal last_cycles  : unsigned(31 downto 0);
  signal infer_cnt    : unsigned(31 downto 0);

  signal wrow_we, bias_we, win_we : std_logic;
  signal mem_addr     : unsigned(8 downto 0);
  signal mem_wdata    : std_logic_vector(31 downto 0);

  signal x_rd_addr    : unsigned(WIN_AW-1 downto 0);
  signal x_in         : int8_t;
  signal x_vec        : int8_array(0 to N_IN-1);

  signal c_w_addr, d_w_addr, w_rd_addr : unsigned(WROW_AW-1 downto 0);
  signal c_b_addr, d_b_addr, b_rd_addr : unsigned(BIAS_AW-1 downto 0);
  signal c_issue, d_issue, issue       : std_logic;
  signal c_word, d_word, word          : integer range 0 to ROW_WORDS-1;
  signal c_clr, d_clr, mac_clr         : std_logic;
  signal d_active                      : std_logic;
  signal mac_en                        : std_logic := '0';
  signal word_d                        : integer range 0 to ROW_WORDS-1 := 0;

  signal w_vec, xh_vec : int8_array(0 to LANES-1);
  signal op_vec        : int8_array(0 to ROW_LEN-1);
  signal b_val         : fx_t;
  signal mac_acc       : acc_t;

  signal cordic_start, is_sigmoid, act_valid : std_logic;
  signal act_z, act_out : fx_t;

  signal clr_state, upd_en, swap : std_logic;
  signal j_idx      : integer range 0 to N_HID-1;
  signal z_j, n_j   : fx_t;
  signal h_vec      : hid_vec_t;

  signal dense_start, dense_done : std_logic;
begin
  assert WROW_DEPTH <= 2**WROW_AW and N_BIAS <= 2**BIAS_AW and WINW_DEPTH <= 2**WINW_AW
    report "NeuroFlex: model does not fit the Avalon memory map" severity failure;
  assert LANES = 4 report "NeuroFlex: HPS word packing assumes LANES = 4" severity failure;

  rst <= reset;

  start     <= (br_start or ext_start) and not busy;
  start_win <= br_win when br_start = '1' else ext_win_sel;

  process(clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        run_mode  <= "10";
        done_flag <= '0';
      elsif start = '1' then
        done_flag <= '0';
        if br_start = '1' then
          run_mode <= br_mode;
        else
          run_mode <= ext_mode;
        end if;
      elsif done = '1' then
        done_flag <= '1';
      end if;
    end if;
  end process;

  u_bridge : entity work.hps_bridge_if
    port map (clk => clk, rst => rst,
              avs_address => avs_address, avs_write => avs_write,
              avs_writedata => avs_writedata, avs_read => avs_read,
              avs_readdata => avs_readdata,
              start_pulse => br_start, cordic_mode => br_mode, win_sel => br_win,
              busy => busy, done => done_flag, class_out => class_out,
              score => score, cycle_cnt => last_cycles, infer_cnt => infer_cnt,
              wrow_we => wrow_we, bias_we => bias_we, win_we => win_we,
              mem_addr => mem_addr, mem_wdata => mem_wdata);

  u_win : entity work.input_window_buf
    port map (clk => clk, wr_en => win_we, wr_addr => mem_addr, wr_data => mem_wdata,
              rd_addr => x_rd_addr, x_out => x_in);

  u_wmem : entity work.weight_mem
    port map (clk => clk,
              hps_we => wrow_we, hps_addr => mem_addr, hps_wdata => mem_wdata,
              hps_bwe => bias_we, hps_baddr => mem_addr,
              rd_addr => w_rd_addr, w_out => w_vec,
              b_rd_addr => b_rd_addr, b_out => b_val);

  -- Weight port / MAC arbitration between the GRU controller and the dense stage
  w_rd_addr <= d_w_addr when d_active = '1' else c_w_addr;
  b_rd_addr <= d_b_addr when d_active = '1' else c_b_addr;
  word      <= d_word   when d_active = '1' else c_word;
  issue     <= c_issue or d_issue;
  mac_clr   <= c_clr or d_clr;

  -- Weight memory has one cycle read latency: delay enable and operand select
  process(clk)
  begin
    if rising_edge(clk) then
      mac_en <= issue;
      word_d <= word;
    end if;
  end process;

  -- Operand vector [x(t) | pad | int8(h(t-1)) | pad], same layout as a weight row
  g_op : for p in 0 to ROW_LEN-1 generate
    g_x : if p < N_IN generate
      op_vec(p) <= x_vec(p);
    end generate;
    g_h : if p >= H_OFS and p < H_OFS + N_HID generate
      op_vec(p) <= fx_to_int8(h_vec(p - H_OFS));
    end generate;
    g_z : if (p >= N_IN and p < H_OFS) or p >= H_OFS + N_HID generate
      op_vec(p) <= (others => '0');
    end generate;
  end generate;

  g_lane : for l in 0 to LANES-1 generate
    xh_vec(l) <= op_vec(word_d*LANES + l);
  end generate;

  u_mac : entity work.int8_mac_engine
    generic map (LANES => LANES)
    port map (clk => clk, rst => rst, clr => mac_clr, en => mac_en,
              a_vec => w_vec, b_vec => xh_vec, acc_out => mac_acc);

  u_nfu : entity work.cordic_nfu
    port map (clk => clk, rst => rst, start => cordic_start,
              mode => run_mode, is_sigmoid => is_sigmoid,
              z_in => act_z, f_out => act_out, valid => act_valid, busy => open);

  u_hbuf : entity work.hidden_state_buf
    port map (clk => clk, rst => rst, clr_state => clr_state,
              upd_en => upd_en, swap => swap, j => j_idx,
              z_j => z_j, h_cand => n_j, h_vec => h_vec);

  u_ctrl : entity work.gru_controller
    port map (clk => clk, rst => rst, start => start, win_sel => start_win,
              busy => busy, done => done,
              x_rd_addr => x_rd_addr, x_in => x_in, x_vec => x_vec,
              w_rd_addr => c_w_addr, w_issue => c_issue, w_word => c_word,
              b_rd_addr => c_b_addr, b_data => b_val,
              mac_clr => c_clr, mac_acc => mac_acc,
              cordic_start => cordic_start, is_sigmoid => is_sigmoid,
              act_z => act_z, act_in => act_out, act_valid => act_valid,
              clr_state => clr_state, upd_en => upd_en, swap => swap,
              j_idx => j_idx, z_j => z_j, n_j => n_j,
              dense_start => dense_start, dense_done => dense_done);

  u_dense : entity work.dense_argmax
    port map (clk => clk, rst => rst, dense_start => dense_start, active => d_active,
              w_rd_addr => d_w_addr, w_issue => d_issue, w_word => d_word,
              b_rd_addr => d_b_addr, b_data => b_val,
              mac_clr => d_clr, mac_acc => mac_acc,
              dense_done => dense_done, class_out => class_out, score => score);

  u_perf : entity work.perf_counter
    port map (clk => clk, rst => rst, start => start, busy => busy, done => done,
              last_cycles => last_cycles, infer_cnt => infer_cnt);

  st_busy  <= busy;
  st_done  <= done_flag;
  st_class <= class_out;
  st_score <= std_logic_vector(score);
end architecture;
