library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package neuroflex_pkg is
  constant N_IN      : integer := 6;
  constant N_HID     : integer := 16;
  constant N_CLS     : integer := 2;
  constant SEQ_LEN   : integer := 64;
  constant WIN_SLOTS : integer := 2;
  constant LANES     : integer := 4;
  constant W_DATA    : integer := 8;
  constant W_ACC     : integer := 24;
  constant W_FX      : integer := 16;
  constant FRAC      : integer := 10;
  constant ACC_SHIFT : integer := 3;
  constant X_SHIFT   : integer := 7;

  constant FX_ONE    : integer := 2**FRAC;
  constant FX_MAX    : integer := 2**(W_FX-1) - 1;
  constant FX_MIN    : integer := -2**(W_FX-1);

  -- Weight row layout: [x part padded to X_WORDS words][h part padded to H_WORDS words]
  constant X_WORDS    : integer := (N_IN  + LANES - 1) / LANES;
  constant H_WORDS    : integer := (N_HID + LANES - 1) / LANES;
  constant ROW_WORDS  : integer := X_WORDS + H_WORDS;
  constant ROW_LEN    : integer := ROW_WORDS * LANES;
  constant H_OFS      : integer := X_WORDS * LANES;

  constant ROW_Z      : integer := 0;
  constant ROW_R      : integer := N_HID;
  constant ROW_H      : integer := 2*N_HID;
  constant ROW_D      : integer := 3*N_HID;
  constant N_ROWS     : integer := 3*N_HID + N_CLS;
  constant WROW_DEPTH : integer := N_ROWS * ROW_WORDS;

  constant B_Z        : integer := 0;
  constant B_R        : integer := N_HID;
  constant B_IN       : integer := 2*N_HID;
  constant B_HN       : integer := 3*N_HID;
  constant B_D        : integer := 4*N_HID;
  constant N_BIAS     : integer := 4*N_HID + N_CLS;

  constant WIN_LEN    : integer := SEQ_LEN * N_IN;
  constant WIN_DEPTH  : integer := WIN_SLOTS * WIN_LEN;
  constant WINW_DEPTH : integer := (WIN_DEPTH + 3) / 4;

  -- Address widths are fixed by the Avalon map (512 words per region)
  constant WROW_AW    : integer := 9;
  constant BIAS_AW    : integer := 9;
  constant WINW_AW    : integer := 9;
  constant WIN_AW     : integer := WINW_AW + 2;

  constant NF_ID      : std_logic_vector(31 downto 0) := x"4E460100";

  subtype int8_t  is signed(W_DATA-1 downto 0);
  subtype acc_t   is signed(W_ACC-1  downto 0);
  subtype fx_t    is signed(W_FX-1   downto 0);
  type    int8_array is array (natural range <>) of int8_t;
  type    hid_vec_t  is array (0 to N_HID-1) of fx_t;

  function sat_fx(a : signed) return fx_t;
  function preact(acc : acc_t; b : fx_t) return fx_t;
  function fx_to_int8(h : fx_t) return int8_t;
end package;

package body neuroflex_pkg is
  function sat_fx(a : signed) return fx_t is
  begin
    if    a > FX_MAX then return to_signed(FX_MAX, W_FX);
    elsif a < FX_MIN then return to_signed(FX_MIN, W_FX);
    else  return resize(a, W_FX);
    end if;
  end function;

  function preact(acc : acc_t; b : fx_t) return fx_t is
  begin
    return sat_fx(resize(shift_right(acc, ACC_SHIFT), W_ACC+1) + resize(b, W_ACC+1));
  end function;

  function fx_to_int8(h : fx_t) return int8_t is
    variable v : fx_t;
  begin
    v := shift_right(h, FRAC - X_SHIFT);
    if    v >  127 then return to_signed( 127, W_DATA);
    elsif v < -128 then return to_signed(-128, W_DATA);
    else  return resize(v, W_DATA);
    end if;
  end function;
end package body;
