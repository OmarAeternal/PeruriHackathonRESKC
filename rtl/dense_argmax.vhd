-- Dense layer N_HID -> N_CLS reusing the MAC engine, followed by argmax.
-- score = best logit - runner-up logit (Q5.10, saturated), class 0 = NORMAL, 1 = ANOMALI.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity dense_argmax is
  port (
    clk, rst    : in  std_logic;
    dense_start : in  std_logic;
    active      : out std_logic;
    w_rd_addr   : out unsigned(WROW_AW-1 downto 0) := (others => '0');
    w_issue     : out std_logic;
    w_word      : out integer range 0 to ROW_WORDS-1;
    b_rd_addr   : out unsigned(BIAS_AW-1 downto 0) := (others => '0');
    b_data      : in  fx_t;
    mac_clr     : out std_logic;
    mac_acc     : in  acc_t;
    dense_done  : out std_logic;
    class_out   : out std_logic_vector(7 downto 0);
    score       : out fx_t
  );
end entity;

architecture rtl of dense_argmax is
  type dstate_t is (D_IDLE, D_SETUP, D_MAC, D_WAIT, D_STORE, D_ARGMAX);
  subtype logit_t is signed(W_ACC downto 0);
  signal dst      : dstate_t := D_IDLE;
  signal k        : integer range 0 to N_CLS-1 := 0;
  signal wd       : integer range 0 to ROW_WORDS-1 := 0;
  signal w_base   : integer range 0 to WROW_DEPTH-1 := 0;
  signal wait_cnt : std_logic := '0';
  signal best     : logit_t := (others => '0');
  signal second   : logit_t := (others => '0');
  signal best_k   : integer range 0 to N_CLS-1 := 0;
  signal active_r : std_logic := '0';
begin
  active <= active_r;

  process(clk)
    variable logit : logit_t;
  begin
    if rising_edge(clk) then
      mac_clr    <= '0';
      w_issue    <= '0';
      dense_done <= '0';
      if rst = '1' then
        dst       <= D_IDLE;
        active_r  <= '0';
        class_out <= (others => '0');
        score     <= (others => '0');
      else
        case dst is
          when D_IDLE =>
            if dense_start = '1' then
              k        <= 0;
              active_r <= '1';
              dst      <= D_SETUP;
            end if;

          when D_SETUP =>
            mac_clr   <= '1';
            b_rd_addr <= to_unsigned(B_D + k, BIAS_AW);
            w_base    <= (ROW_D + k) * ROW_WORDS;
            wd        <= X_WORDS;
            dst       <= D_MAC;

          when D_MAC =>
            w_issue   <= '1';
            w_rd_addr <= to_unsigned(w_base + wd, WROW_AW);
            w_word    <= wd;
            if wd = ROW_WORDS-1 then
              wait_cnt <= '0';
              dst      <= D_WAIT;
            else
              wd <= wd + 1;
            end if;

          when D_WAIT =>
            if wait_cnt = '1' then
              dst <= D_STORE;
            end if;
            wait_cnt <= '1';

          when D_STORE =>
            logit := resize(shift_right(mac_acc, ACC_SHIFT), W_ACC+1) + resize(b_data, W_ACC+1);
            if k = 0 then
              best   <= logit;
              best_k <= 0;
              second <= (logit_t'high => '1', others => '0');
            elsif logit > best then
              second <= best;
              best   <= logit;
              best_k <= k;
            elsif logit > second then
              second <= logit;
            end if;
            if k = N_CLS-1 then
              dst <= D_ARGMAX;
            else
              k   <= k + 1;
              dst <= D_SETUP;
            end if;

          when D_ARGMAX =>
            class_out  <= std_logic_vector(to_unsigned(best_k, 8));
            score      <= sat_fx(resize(best, W_ACC+2) - resize(second, W_ACC+2));
            dense_done <= '1';
            active_r   <= '0';
            dst        <= D_IDLE;
        end case;
      end if;
    end if;
  end process;
end architecture;
