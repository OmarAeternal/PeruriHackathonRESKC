-- Sequential restoring divider: q = floor(num * 2^(QB-1) / den), valid when num < 2*den.
-- q(QB-1) = '1' means num >= den (result >= 1.0).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity seq_divider is
  generic (
    W  : integer := 24;
    QB : integer := 11
  );
  port (
    clk, rst : in  std_logic;
    start    : in  std_logic;
    num      : in  unsigned(W-1 downto 0);
    den      : in  unsigned(W-1 downto 0);
    q        : out unsigned(QB-1 downto 0);
    done     : out std_logic
  );
end entity;

architecture rtl of seq_divider is
  signal r       : unsigned(W downto 0) := (others => '0');
  signal d       : unsigned(W downto 0) := (others => '0');
  signal qr      : unsigned(QB-1 downto 0) := (others => '0');
  signal cnt     : integer range 0 to QB-1 := 0;
  signal running : std_logic := '0';
begin
  process(clk)
    variable rv : unsigned(W downto 0);
  begin
    if rising_edge(clk) then
      done <= '0';
      if rst = '1' then
        running <= '0';
      elsif start = '1' then
        r       <= '0' & num;
        d       <= '0' & den;
        qr      <= (others => '0');
        cnt     <= 0;
        running <= '1';
      elsif running = '1' then
        rv := r;
        if cnt /= 0 then
          rv := r(W-1 downto 0) & '0';
        end if;
        if rv >= d then
          rv := rv - d;
          qr <= qr(QB-2 downto 0) & '1';
        else
          qr <= qr(QB-2 downto 0) & '0';
        end if;
        r <= rv;
        if cnt = QB-1 then
          running <= '0';
          done    <= '1';
        else
          cnt <= cnt + 1;
        end if;
      end if;
    end if;
  end process;

  q <= qr;
end architecture;
