-- (c) Copyright 1995-2018 Xilinx, Inc. All rights reserved.
-- 
-- This file contains confidential and proprietary information
-- of Xilinx, Inc. and is protected under U.S. and
-- international copyright and other intellectual property
-- laws.
-- 
-- DISCLAIMER
-- This disclaimer is not a license and does not grant any
-- rights to the materials distributed herewith. Except as
-- otherwise provided in a valid license issued to you by
-- Xilinx, and to the maximum extent permitted by applicable
-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
-- WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
-- (2) Xilinx shall not be liable (whether in contract or tort,
-- including negligence, or under any other theory of
-- liability) for any loss or damage of any kind or nature
-- related to, arising under or in connection with these
-- materials, including for any direct, or any indirect,
-- special, incidental, or consequential loss or damage
-- (including loss of data, profits, goodwill, or any type of
-- loss or damage suffered as a result of any action brought
-- by a third party) even if such damage or loss was
-- reasonably foreseeable or Xilinx had been advised of the
-- possibility of the same.
-- 
-- CRITICAL APPLICATIONS
-- Xilinx products are not designed or intended to be fail-
-- safe, or for use in any application requiring fail-safe
-- performance, such as life-support or safety devices or
-- systems, Class III medical devices, nuclear facilities,
-- applications related to the deployment of airbags, or any
-- other applications that could lead to death, personal
-- injury, or severe property or environmental damage
-- (individually and collectively, "Critical
-- Applications"). Customer assumes the sole risk and
-- liability of any use of Xilinx products in Critical
-- Applications, subject only to applicable laws and
-- regulations governing limitations on product liability.
-- 
-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
-- PART OF THIS FILE AT ALL TIMES.
-- 
-- DO NOT MODIFY THIS FILE.

-- IP VLNV: xilinx.com:ip:cordic:6.0
-- IP Revision: 11

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

LIBRARY cordic_v6_0_11;
USE cordic_v6_0_11.cordic_v6_0_11;

ENTITY cordic_rotator IS
  PORT (
    aclk : IN STD_LOGIC;
    aresetn : IN STD_LOGIC;
    s_axis_phase_tvalid : IN STD_LOGIC;
    s_axis_phase_tready : OUT STD_LOGIC;
    s_axis_phase_tdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
    s_axis_cartesian_tvalid : IN STD_LOGIC;
    s_axis_cartesian_tready : OUT STD_LOGIC;
    s_axis_cartesian_tlast : IN STD_LOGIC;
    s_axis_cartesian_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
    m_axis_dout_tvalid : OUT STD_LOGIC;
    m_axis_dout_tready : IN STD_LOGIC;
    m_axis_dout_tlast : OUT STD_LOGIC;
    m_axis_dout_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
  );
END cordic_rotator;

ARCHITECTURE cordic_rotator_arch OF cordic_rotator IS
  ATTRIBUTE DowngradeIPIdentifiedWarnings : STRING;
  ATTRIBUTE DowngradeIPIdentifiedWarnings OF cordic_rotator_arch: ARCHITECTURE IS "yes";
  COMPONENT cordic_v6_0_11 IS
    GENERIC (
      C_ARCHITECTURE : INTEGER;
      C_CORDIC_FUNCTION : INTEGER;
      C_COARSE_ROTATE : INTEGER;
      C_DATA_FORMAT : INTEGER;
      C_XDEVICEFAMILY : STRING;
      C_HAS_ACLKEN : INTEGER;
      C_HAS_ACLK : INTEGER;
      C_HAS_S_AXIS_CARTESIAN : INTEGER;
      C_HAS_S_AXIS_PHASE : INTEGER;
      C_HAS_ARESETN : INTEGER;
      C_INPUT_WIDTH : INTEGER;
      C_ITERATIONS : INTEGER;
      C_OUTPUT_WIDTH : INTEGER;
      C_PHASE_FORMAT : INTEGER;
      C_PIPELINE_MODE : INTEGER;
      C_PRECISION : INTEGER;
      C_ROUND_MODE : INTEGER;
      C_SCALE_COMP : INTEGER;
      C_THROTTLE_SCHEME : INTEGER;
      C_TLAST_RESOLUTION : INTEGER;
      C_HAS_S_AXIS_PHASE_TUSER : INTEGER;
      C_HAS_S_AXIS_PHASE_TLAST : INTEGER;
      C_S_AXIS_PHASE_TDATA_WIDTH : INTEGER;
      C_S_AXIS_PHASE_TUSER_WIDTH : INTEGER;
      C_HAS_S_AXIS_CARTESIAN_TUSER : INTEGER;
      C_HAS_S_AXIS_CARTESIAN_TLAST : INTEGER;
      C_S_AXIS_CARTESIAN_TDATA_WIDTH : INTEGER;
      C_S_AXIS_CARTESIAN_TUSER_WIDTH : INTEGER;
      C_M_AXIS_DOUT_TDATA_WIDTH : INTEGER;
      C_M_AXIS_DOUT_TUSER_WIDTH : INTEGER
    );
    PORT (
      aclk : IN STD_LOGIC;
      aclken : IN STD_LOGIC;
      aresetn : IN STD_LOGIC;
      s_axis_phase_tvalid : IN STD_LOGIC;
      s_axis_phase_tready : OUT STD_LOGIC;
      s_axis_phase_tuser : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
      s_axis_phase_tlast : IN STD_LOGIC;
      s_axis_phase_tdata : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
      s_axis_cartesian_tvalid : IN STD_LOGIC;
      s_axis_cartesian_tready : OUT STD_LOGIC;
      s_axis_cartesian_tuser : IN STD_LOGIC_VECTOR(0 DOWNTO 0);
      s_axis_cartesian_tlast : IN STD_LOGIC;
      s_axis_cartesian_tdata : IN STD_LOGIC_VECTOR(31 DOWNTO 0);
      m_axis_dout_tvalid : OUT STD_LOGIC;
      m_axis_dout_tready : IN STD_LOGIC;
      m_axis_dout_tuser : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
      m_axis_dout_tlast : OUT STD_LOGIC;
      m_axis_dout_tdata : OUT STD_LOGIC_VECTOR(31 DOWNTO 0)
    );
  END COMPONENT cordic_v6_0_11;
  ATTRIBUTE X_INTERFACE_INFO : STRING;
  ATTRIBUTE X_INTERFACE_INFO OF aclk: SIGNAL IS "xilinx.com:signal:clock:1.0 aclk_intf CLK";
  ATTRIBUTE X_INTERFACE_INFO OF aresetn: SIGNAL IS "xilinx.com:signal:reset:1.0 aresetn_intf RST";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_phase_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_PHASE TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_phase_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_PHASE TREADY";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_phase_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_PHASE TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_cartesian_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_CARTESIAN TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_cartesian_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_CARTESIAN TREADY";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_cartesian_tlast: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_CARTESIAN TLAST";
  ATTRIBUTE X_INTERFACE_INFO OF s_axis_cartesian_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 S_AXIS_CARTESIAN TDATA";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_dout_tvalid: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_DOUT TVALID";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_dout_tready: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_DOUT TREADY";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_dout_tlast: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_DOUT TLAST";
  ATTRIBUTE X_INTERFACE_INFO OF m_axis_dout_tdata: SIGNAL IS "xilinx.com:interface:axis:1.0 M_AXIS_DOUT TDATA";
BEGIN
  U0 : cordic_v6_0_11
    GENERIC MAP (
      C_ARCHITECTURE => 2,
      C_CORDIC_FUNCTION => 0,
      C_COARSE_ROTATE => 1,
      C_DATA_FORMAT => 0,
      C_XDEVICEFAMILY => "zynq",
      C_HAS_ACLKEN => 0,
      C_HAS_ACLK => 1,
      C_HAS_S_AXIS_CARTESIAN => 1,
      C_HAS_S_AXIS_PHASE => 1,
      C_HAS_ARESETN => 1,
      C_INPUT_WIDTH => 16,
      C_ITERATIONS => 0,
      C_OUTPUT_WIDTH => 16,
      C_PHASE_FORMAT => 1,
      C_PIPELINE_MODE => -2,
      C_PRECISION => 0,
      C_ROUND_MODE => 2,
      C_SCALE_COMP => 0,
      C_THROTTLE_SCHEME => 2,
      C_TLAST_RESOLUTION => 1,
      C_HAS_S_AXIS_PHASE_TUSER => 0,
      C_HAS_S_AXIS_PHASE_TLAST => 0,
      C_S_AXIS_PHASE_TDATA_WIDTH => 16,
      C_S_AXIS_PHASE_TUSER_WIDTH => 1,
      C_HAS_S_AXIS_CARTESIAN_TUSER => 0,
      C_HAS_S_AXIS_CARTESIAN_TLAST => 1,
      C_S_AXIS_CARTESIAN_TDATA_WIDTH => 32,
      C_S_AXIS_CARTESIAN_TUSER_WIDTH => 1,
      C_M_AXIS_DOUT_TDATA_WIDTH => 32,
      C_M_AXIS_DOUT_TUSER_WIDTH => 1
    )
    PORT MAP (
      aclk => aclk,
      aclken => '1',
      aresetn => aresetn,
      s_axis_phase_tvalid => s_axis_phase_tvalid,
      s_axis_phase_tready => s_axis_phase_tready,
      s_axis_phase_tuser => STD_LOGIC_VECTOR(TO_UNSIGNED(0, 1)),
      s_axis_phase_tlast => '0',
      s_axis_phase_tdata => s_axis_phase_tdata,
      s_axis_cartesian_tvalid => s_axis_cartesian_tvalid,
      s_axis_cartesian_tready => s_axis_cartesian_tready,
      s_axis_cartesian_tuser => STD_LOGIC_VECTOR(TO_UNSIGNED(0, 1)),
      s_axis_cartesian_tlast => s_axis_cartesian_tlast,
      s_axis_cartesian_tdata => s_axis_cartesian_tdata,
      m_axis_dout_tvalid => m_axis_dout_tvalid,
      m_axis_dout_tready => m_axis_dout_tready,
      m_axis_dout_tlast => m_axis_dout_tlast,
      m_axis_dout_tdata => m_axis_dout_tdata
    );
END cordic_rotator_arch;
