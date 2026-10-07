-- Shared CORDIC nonlinear function unit (hyperbolic rotation mode).
-- tanh(z): CORDIC on z/4 gives (cosh, sinh), two double-angle steps give
-- (cosh z, sinh z), then tanh = sinh / cosh via the sequential divider.
-- sigmoid(z) = 0.5 * (1 + tanh(z/2)). Iterations 8/12/16 (mode "00"/"01"/"1x"),
-- iterations i = 4 and 13 are repeated. Internal format Q7.16, I/O Q5.10.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity cordic_nfu is
  port (
    clk, rst   : in  std_logic;
    start      : in  std_logic;
    mode       : in  std_logic_vector(1 downto 0);
    is_sigmoid : in  std_logic;
    z_in       : in  fx_t;
    f_out      : out fx_t;
    valid      : out std_logic;
    busy       : out std_logic
  );
end entity;

architecture rtl of cordic_nfu is
  constant W_C   : integer := 24;
  constant F_C   : integer := 16;
  subtype cx_t is signed(W_C-1 downto 0);

  type atanh_lut_t is array (1 to 16) of cx_t;
  constant ATANH_LUT : atanh_lut_t := (
    to_signed(35999, W_C), to_signed(16739, W_C), to_signed(8235, W_C),
    to_signed(4101,  W_C), to_signed(2049,  W_C), to_signed(1024, W_C),
    to_signed(512,   W_C), to_signed(256,   W_C), to_signed(128,  W_C),
    to_signed(64,    W_C), to_signed(32,    W_C), to_signed(16,   W_C),
    to_signed(8,     W_C), to_signed(4,     W_C), to_signed(2,    W_C),
    to_signed(1,     W_C));

  constant X0_KH    : cx_t := to_signed(79135, W_C);
  constant Z_SAT    : integer := 4 * 2**F_C;
  constant TANH_SAT : integer := FX_ONE - 1;

  type st_t is (IDLE, ROTATE, DOUBLE, DIVIDE, WAIT_DIV, OUTPUT);
  signal st : st_t := IDLE;

  signal x, y, z     : cx_t := (others => '0');
  signal i           : integer range 1 to 16 := 1;
  signal n_iter      : integer range 8 to 16 := 16;
  signal rep_done    : std_logic := '0';
  signal dbl_cnt     : std_logic := '0';
  signal sig_r, neg  : std_logic := '0';
  signal tanh_r      : fx_t := (others => '0');

  signal div_start   : std_logic := '0';
  signal div_done    : std_logic;
  signal div_num     : unsigned(W_C-1 downto 0) := (others => '0');
  signal div_den     : unsigned(W_C-1 downto 0) := (others => '0');
  signal div_q       : unsigned(FRAC downto 0);
begin
  u_div : entity work.seq_divider
    generic map (W => W_C, QB => FRAC + 1)
    port map (clk => clk, rst => rst, start => div_start,
              num => div_num, den => div_den, q => div_q, done => div_done);

  busy <= '0' when st = IDLE else '1';

  process(clk)
    variable zi     : cx_t;
    variable c2, s2 : signed(2*W_C downto 0);
    variable sc     : signed(2*W_C-1 downto 0);
    variable mag    : fx_t;
  begin
    if rising_edge(clk) then
      valid     <= '0';
      div_start <= '0';
      if rst = '1' then
        st <= IDLE;
      else
        case st is
          when IDLE =>
            if start = '1' then
              zi := shift_left(resize(z_in, W_C), F_C - FRAC);
              if is_sigmoid = '1' then
                zi := shift_right(zi, 1);
              end if;
              sig_r <= is_sigmoid;
              case mode is
                when "00"   => n_iter <= 8;
                when "01"   => n_iter <= 12;
                when others => n_iter <= 16;
              end case;
              if zi >= Z_SAT then
                tanh_r <= to_signed(TANH_SAT, W_FX);
                st     <= OUTPUT;
              elsif zi <= -Z_SAT then
                tanh_r <= to_signed(-TANH_SAT, W_FX);
                st     <= OUTPUT;
              else
                x <= X0_KH;
                y <= (others => '0');
                z <= shift_right(zi, 2);
                i <= 1;
                rep_done <= '0';
                st <= ROTATE;
              end if;
            end if;

          when ROTATE =>
            if z(z'high) = '0' then
              x <= x + shift_right(y, i);
              y <= y + shift_right(x, i);
              z <= z - ATANH_LUT(i);
            else
              x <= x - shift_right(y, i);
              y <= y - shift_right(x, i);
              z <= z + ATANH_LUT(i);
            end if;
            if (i = 4 or i = 13) and rep_done = '0' then
              rep_done <= '1';
            else
              rep_done <= '0';
              if i = n_iter then
                dbl_cnt <= '0';
                st <= DOUBLE;
              else
                i <= i + 1;
              end if;
            end if;

          when DOUBLE =>
            c2 := resize(x * x, 2*W_C+1) + resize(y * y, 2*W_C+1);
            sc := x * y;
            x <= resize(shift_right(c2, F_C), W_C);
            y <= resize(shift_right(sc, F_C - 1), W_C);
            if dbl_cnt = '1' then
              st <= DIVIDE;
            end if;
            dbl_cnt <= '1';

          when DIVIDE =>
            neg       <= y(y'high);
            div_num   <= unsigned(abs(y));
            div_den   <= unsigned(x);
            div_start <= '1';
            st        <= WAIT_DIV;

          when WAIT_DIV =>
            if div_done = '1' then
              if div_q(FRAC) = '1' then
                mag := to_signed(FX_ONE, W_FX);
              else
                mag := signed(resize(div_q, W_FX));
              end if;
              if neg = '1' then
                tanh_r <= -mag;
              else
                tanh_r <= mag;
              end if;
              st <= OUTPUT;
            end if;

          when OUTPUT =>
            if sig_r = '1' then
              f_out <= shift_right(to_signed(FX_ONE, W_FX) + tanh_r, 1);
            else
              f_out <= tanh_r;
            end if;
            valid <= '1';
            st    <= IDLE;
        end case;
      end if;
    end if;
  end process;
end architecture;
