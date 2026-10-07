-- Avalon-MM slave on the Lightweight HPS-to-FPGA bridge (word addressing, read latency 1).
-- Byte map: 0x0000 registers, 0x0800 weight rows, 0x1000 IMU windows, 0x1800 biases.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.neuroflex_pkg.all;

entity hps_bridge_if is
  port (
    clk, rst      : in  std_logic;
    avs_address   : in  std_logic_vector(10 downto 0);
    avs_write     : in  std_logic;
    avs_writedata : in  std_logic_vector(31 downto 0);
    avs_read      : in  std_logic;
    avs_readdata  : out std_logic_vector(31 downto 0);
    start_pulse   : out std_logic;
    cordic_mode   : out std_logic_vector(1 downto 0);
    win_sel       : out std_logic;
    busy          : in  std_logic;
    done          : in  std_logic;
    class_out     : in  std_logic_vector(7 downto 0);
    score         : in  fx_t;
    cycle_cnt     : in  unsigned(31 downto 0);
    infer_cnt     : in  unsigned(31 downto 0);
    wrow_we       : out std_logic;
    bias_we       : out std_logic;
    win_we        : out std_logic;
    mem_addr      : out unsigned(8 downto 0);
    mem_wdata     : out std_logic_vector(31 downto 0)
  );
end entity;

architecture rtl of hps_bridge_if is
  signal mode_r : std_logic_vector(1 downto 0) := "10";
  signal win_r  : std_logic := '0';
  signal region : std_logic_vector(1 downto 0);
begin
  region      <= avs_address(10 downto 9);
  cordic_mode <= mode_r;
  win_sel     <= win_r;

  mem_addr  <= unsigned(avs_address(8 downto 0));
  mem_wdata <= avs_writedata;
  wrow_we   <= avs_write when region = "01" else '0';
  win_we    <= avs_write when region = "10" else '0';
  bias_we   <= avs_write when region = "11" else '0';

  process(clk)
  begin
    if rising_edge(clk) then
      start_pulse <= '0';
      if rst = '1' then
        mode_r       <= "10";
        win_r        <= '0';
        avs_readdata <= (others => '0');
      else
        if avs_write = '1' and region = "00" then
          case avs_address(2 downto 0) is
            when "000" =>
              start_pulse <= avs_writedata(0) and not busy;
              mode_r      <= avs_writedata(2 downto 1);
              win_r       <= avs_writedata(3);
            when others => null;
          end case;
        end if;

        if avs_read = '1' then
          avs_readdata <= (others => '0');
          if region = "00" then
            case avs_address(2 downto 0) is
              when "000" => avs_readdata(3 downto 1) <= win_r & mode_r;
              when "001" => avs_readdata(1 downto 0) <= done & busy;
              when "010" => avs_readdata <= std_logic_vector(score) & x"00" & class_out;
              when "011" => avs_readdata <= std_logic_vector(cycle_cnt);
              when "100" => avs_readdata <= std_logic_vector(infer_cnt);
              when "101" => avs_readdata <= NF_ID;
              when "110" => avs_readdata <= std_logic_vector(to_unsigned(SEQ_LEN, 8))
                                          & std_logic_vector(to_unsigned(N_CLS, 8))
                                          & std_logic_vector(to_unsigned(N_HID, 8))
                                          & std_logic_vector(to_unsigned(N_IN, 8));
              when others => null;
            end case;
          end if;
        end if;
      end if;
    end if;
  end process;
end architecture;
