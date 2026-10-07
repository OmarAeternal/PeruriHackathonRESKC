-- Main FSM. Per timestep: load x(t), then for every neuron j:
--   z = sigmoid(Wz x + Uz h + bz)
--   r = sigmoid(Wr x + Ur h + br)
--   n = tanh(Wn x + b_in + r * (Un h + b_hn))
--   h(t) = n + z * (h(t-1) - n)
-- The MAC engine and the shared CORDIC are reused for every gate.
-- After the last timestep the dense/argmax stage is started.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity gru_controller is
  port (
    clk, rst     : in  std_logic;
    start        : in  std_logic;
    win_sel      : in  std_logic;
    busy         : out std_logic;
    done         : out std_logic;
    x_rd_addr    : out unsigned(WIN_AW-1 downto 0) := (others => '0');
    x_in         : in  int8_t;
    x_vec        : out int8_array(0 to N_IN-1);
    w_rd_addr    : out unsigned(WROW_AW-1 downto 0) := (others => '0');
    w_issue      : out std_logic;
    w_word       : out integer range 0 to ROW_WORDS-1;
    b_rd_addr    : out unsigned(BIAS_AW-1 downto 0) := (others => '0');
    b_data       : in  fx_t;
    mac_clr      : out std_logic;
    mac_acc      : in  acc_t;
    cordic_start : out std_logic;
    is_sigmoid   : out std_logic;
    act_z        : out fx_t;
    act_in       : in  fx_t;
    act_valid    : in  std_logic;
    clr_state    : out std_logic;
    upd_en       : out std_logic;
    swap         : out std_logic;
    j_idx        : out integer range 0 to N_HID-1;
    z_j          : out fx_t;
    n_j          : out fx_t;
    dense_start  : out std_logic;
    dense_done   : in  std_logic
  );
end entity;

architecture rtl of gru_controller is
  type state_t is (S_IDLE, S_LOADX, S_MAC, S_MAC_WAIT, S_PRE, S_ACT, S_ACT_WAIT,
                   S_UPDATE, S_NEXT, S_DENSE, S_DONE);
  type step_t is (STEP_Z, STEP_R, STEP_HX, STEP_HH);
  signal st       : state_t := S_IDLE;
  signal step     : step_t  := STEP_Z;

  signal t        : integer range 0 to SEQ_LEN-1 := 0;
  signal j        : integer range 0 to N_HID-1 := 0;
  signal k        : integer range 0 to ROW_WORDS-1 := 0;
  signal k_last   : integer range 0 to ROW_WORDS-1 := 0;
  signal w_base   : integer range 0 to WROW_DEPTH-1 := 0;
  signal win_base : integer range 0 to WIN_DEPTH-1 := 0;
  signal wait_cnt : std_logic := '0';

  signal ld_cnt   : integer range 0 to N_IN := 0;
  signal ld_v1    : std_logic := '0';
  signal ld_v2    : std_logic := '0';
  signal ld_i1    : integer range 0 to N_IN-1 := 0;
  signal ld_i2    : integer range 0 to N_IN-1 := 0;

  signal x_r      : int8_array(0 to N_IN-1) := (others => (others => '0'));
  signal z_r, r_r, n_r, ax_r, pre_r : fx_t := (others => '0');
  signal busy_r   : std_logic := '0';
begin
  busy  <= busy_r;
  x_vec <= x_r;
  j_idx <= j;
  z_j   <= z_r;
  n_j   <= n_r;
  act_z <= pre_r;

  process(clk)
    variable bh   : fx_t;
    variable prod : signed(2*W_FX-1 downto 0);

    procedure begin_mac(row, k0, k1, bias : in integer) is
    begin
      w_base    <= row * ROW_WORDS;
      k         <= k0;
      k_last    <= k1;
      b_rd_addr <= to_unsigned(bias, BIAS_AW);
      mac_clr   <= '1';
      st        <= S_MAC;
    end procedure;
  begin
    if rising_edge(clk) then
      mac_clr      <= '0';
      w_issue      <= '0';
      cordic_start <= '0';
      upd_en       <= '0';
      swap         <= '0';
      clr_state    <= '0';
      dense_start  <= '0';
      done         <= '0';

      ld_v2 <= ld_v1;
      ld_i2 <= ld_i1;
      if ld_v2 = '1' then
        x_r(ld_i2) <= x_in;
      end if;

      if rst = '1' then
        st     <= S_IDLE;
        busy_r <= '0';
        ld_v1  <= '0';
        ld_v2  <= '0';
      else
        case st is
          when S_IDLE =>
            ld_v1 <= '0';
            if start = '1' then
              t         <= 0;
              j         <= 0;
              ld_cnt    <= 0;
              busy_r    <= '1';
              clr_state <= '1';
              if win_sel = '1' then
                win_base <= WIN_LEN;
              else
                win_base <= 0;
              end if;
              st <= S_LOADX;
            end if;

          when S_LOADX =>
            if ld_cnt < N_IN then
              x_rd_addr <= to_unsigned(win_base + t*N_IN + ld_cnt, WIN_AW);
              ld_v1     <= '1';
              ld_i1     <= ld_cnt;
              ld_cnt    <= ld_cnt + 1;
            else
              ld_v1 <= '0';
              if ld_v1 = '0' and ld_v2 = '0' then
                step <= STEP_Z;
                begin_mac(ROW_Z + j, 0, ROW_WORDS-1, B_Z + j);
              end if;
            end if;

          when S_MAC =>
            w_issue   <= '1';
            w_rd_addr <= to_unsigned(w_base + k, WROW_AW);
            w_word    <= k;
            if k = k_last then
              wait_cnt <= '0';
              st       <= S_MAC_WAIT;
            else
              k <= k + 1;
            end if;

          when S_MAC_WAIT =>
            if wait_cnt = '1' then
              st <= S_PRE;
            end if;
            wait_cnt <= '1';

          when S_PRE =>
            case step is
              when STEP_Z | STEP_R =>
                pre_r      <= preact(mac_acc, b_data);
                is_sigmoid <= '1';
                st         <= S_ACT;
              when STEP_HX =>
                ax_r <= preact(mac_acc, b_data);
                step <= STEP_HH;
                begin_mac(ROW_H + j, X_WORDS, ROW_WORDS-1, B_HN + j);
              when STEP_HH =>
                bh   := preact(mac_acc, b_data);
                prod := r_r * bh;
                pre_r      <= sat_fx(resize(ax_r, 2*W_FX) + shift_right(prod, FRAC));
                is_sigmoid <= '0';
                st         <= S_ACT;
            end case;

          when S_ACT =>
            cordic_start <= '1';
            st           <= S_ACT_WAIT;

          when S_ACT_WAIT =>
            if act_valid = '1' then
              case step is
                when STEP_Z =>
                  z_r  <= act_in;
                  step <= STEP_R;
                  begin_mac(ROW_R + j, 0, ROW_WORDS-1, B_R + j);
                when STEP_R =>
                  r_r  <= act_in;
                  step <= STEP_HX;
                  begin_mac(ROW_H + j, 0, X_WORDS-1, B_IN + j);
                when others =>
                  n_r <= act_in;
                  st  <= S_UPDATE;
              end case;
            end if;

          when S_UPDATE =>
            upd_en <= '1';
            st     <= S_NEXT;

          when S_NEXT =>
            if j < N_HID-1 then
              j    <= j + 1;
              step <= STEP_Z;
              begin_mac(ROW_Z + j + 1, 0, ROW_WORDS-1, B_Z + j + 1);
            elsif t < SEQ_LEN-1 then
              j      <= 0;
              t      <= t + 1;
              swap   <= '1';
              ld_cnt <= 0;
              st     <= S_LOADX;
            else
              swap        <= '1';
              dense_start <= '1';
              st          <= S_DENSE;
            end if;

          when S_DENSE =>
            if dense_done = '1' then
              st <= S_DONE;
            end if;

          when S_DONE =>
            busy_r <= '0';
            done   <= '1';
            st     <= S_IDLE;
        end case;
      end if;
    end if;
  end process;
end architecture;
