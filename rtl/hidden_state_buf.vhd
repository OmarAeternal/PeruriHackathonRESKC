-- Ping-pong hidden state register file. h(t-1) is read from the active bank
-- while h(t) = n + z * (h(t-1) - n) is written into the other bank;
-- swap makes the new bank active at the end of a timestep.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity hidden_state_buf is
  port (
    clk, rst  : in  std_logic;
    clr_state : in  std_logic;
    upd_en    : in  std_logic;
    swap      : in  std_logic;
    j         : in  integer range 0 to N_HID-1;
    z_j       : in  fx_t;
    h_cand    : in  fx_t;
    h_vec     : out hid_vec_t
  );
end entity;

architecture rtl of hidden_state_buf is
  type h_bank_t is array (0 to 1) of hid_vec_t;
  signal h_bank : h_bank_t := (others => (others => (others => '0')));
  signal sel    : integer range 0 to 1 := 0;
begin
  h_vec <= h_bank(sel);

  process(clk)
    variable h_prev : fx_t;
    variable diff   : signed(W_FX downto 0);
    variable prod   : signed(2*W_FX downto 0);
  begin
    if rising_edge(clk) then
      if rst = '1' or clr_state = '1' then
        h_bank <= (others => (others => (others => '0')));
        sel    <= 0;
      else
        if upd_en = '1' then
          h_prev := h_bank(sel)(j);
          diff   := resize(h_prev, W_FX+1) - resize(h_cand, W_FX+1);
          prod   := z_j * diff;
          h_bank(1 - sel)(j) <= sat_fx(resize(h_cand, 2*W_FX+1) + shift_right(prod, FRAC));
        end if;
        if swap = '1' then
          sel <= 1 - sel;
        end if;
      end if;
    end if;
  end process;
end architecture;
