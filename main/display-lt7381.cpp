#include "display.h"
#include "display-module.h"
#include "display-spi.h"
#include "display-lt7381.h"
#include "exception.h"

#include "log.h"

#include <math.h>

#include <format>
#include <thread>
#include <chrono>

static constexpr const unsigned char if_write_register =	0b00000000;
static constexpr const unsigned char if_write_data =		0b10000000;
static constexpr const unsigned char if_read_status = 		0b01000000;
static constexpr const unsigned char if_read_register = 	0b11000000;

static constexpr const unsigned int spi_speed_initial =  1'000'000;
static constexpr const unsigned int spi_speed_normal =  40'000'000;

enum
{

	reg_srr =		0x00,
	reg_ccr =		0x01,
	reg_macr =		0x02,
	reg_icr =		0x03,
	reg_mrwdp =		0x04,
	reg_ppllc1 = 	0x05,
	reg_ppllc2 = 	0x06,
	reg_mpllc1 = 	0x07,
	reg_mpllc2 = 	0x08,
	reg_cpllc1 = 	0x09,
	reg_cpllc2 = 	0x0a,
	reg_mpwctr =	0x10,
	reg_dpcr =		0x12,
	reg_pcsr =		0x13,
	reg_hdwr =		0x14,
	reg_hdwftr =	0x15,
	reg_hndr =		0x16,
	reg_hndftr =	0x17,
	reg_hstr =		0x18,
	reg_hpwr =		0x19,
	reg_vdhr_l =	0x1a,
	reg_vdhr_h =	0x1b,
	reg_vndr_l =	0x1c,
	reg_vndr_h =	0x1d,
	reg_vstr =		0x1e,
	reg_vpwr =		0x1f,

	reg_misa_0 =	0x20,
	reg_misa_1 =	0x21,
	reg_misa_2 =	0x22,
	reg_misa_3 =	0x23,
	reg_miw_l =		0x24,
	reg_miw_h =		0x25,
	reg_mwulx_l =	0x26,
	reg_mwulx_h =	0x27,
	reg_mwuly_l = 	0x28,
	reg_mwuly_h = 	0x29,

	reg_cvssa_0	=	0x50,
	reg_cvssa_1	=	0x51,
	reg_cvssa_2	=	0x52,
	reg_cvssa_3	=	0x53,
	reg_cvs_imwth_l = 0x54,
	reg_cvs_imwth_h = 0x55,
	reg_awul_x_l =	0x56,
	reg_awul_x_h =	0x57,
	reg_awul_y_l =	0x58,
	reg_awul_y_h =	0x59,
	reg_aw_wth_l =	0x5a,
	reg_aw_wth_h =	0x5b,
	reg_aw_ht_l =	0x5c,
	reg_aw_ht_h =	0x5d,
	reg_aw_color =	0x5e,
	reg_curh_l =	0x5f,
	reg_curh_h =	0x60,
	reg_curv_l =	0x61,
	reg_curv_h =	0x62,

	reg_dlhsr_l =	0x68,
	reg_dlhsr_h =	0x69,
	reg_dlvsr_l =	0x6a,
	reg_dlvsr_h	=	0x6b,
	reg_dlher_l =	0x6c,
	reg_dlher_h =	0x6d,
	reg_dlver_l =	0x6e,
	reg_dlver_h =	0x6f,

	reg_dcr1 =		0x76,

	reg_psclr =		0x84,
	reg_pmuxr =		0x85,
	reg_pcfgr =		0x86,
	reg_tcmpb0_l =	0x88,
	reg_tcmpb0_h =	0x89,
	reg_tcntb0_l =	0x8a,
	reg_tcntb0_h =	0x8b,
	reg_tcmpb1_l =	0x8c,
	reg_tcmpb1_h =	0x8d,
	reg_tcntb1_l =	0x8e,
	reg_tcntb1_h =	0x8f,

	reg_fgcr =		0xd2,
	reg_fgcg =		0xd3,
	reg_fgcb =		0xd4,
	reg_bgcr =		0xd5,
	reg_bgcg =		0xd6,
	reg_bgcb =		0xd7,

	reg_sdrar =		0xe0,
	reg_sdrmd =		0xe1,
	reg_sdrref_l =	0xe2,
	reg_sdrref_h =	0xe3,
	reg_sdrcr =		0xe4,

	status_write_fifo_full =		0b10000000,
	status_write_fifo_empty =		0b01000000,
	status_read_fifo_full =			0b00100000,
	status_read_fifo_empty =		0b00010000,
	status_core_busy =				0b00001000,
	status_ram_ready =				0b00000100,
	status_mode_status =			0b00000010,
	status_interrupt_state =		0b00000001,

	reg_srr_reconfig_pll =			0b10000000,
	reg_srr_reset =					0b00000001,

	reg_ccr_pll_ready =				0b100'00'00'0,
	reg_ccr_mask_wait =				0b010'00'00'0,
	reg_ccr_keypad_enable =			0b001'00'00'0,
	reg_ccr_tft_if_none =			0b000'11'00'0,
	reg_ccr_tft_if_16b =			0b000'10'00'0,
	reg_ccr_tft_if_18b =			0b000'01'00'0,
	reg_ccr_tft_if_24b =			0b000'00'00'0,
	reg_ccr_i2c_master_enable =		0b000'00'10'0,
	reg_ccr_spi_master_enable =		0b000'00'01'0,
	reg_ccr_host_if_8bit =			0b000'00'00'0,
	reg_ccr_host_if_16bit =			0b000'00'00'1,

	reg_macr_host_if_mask_even =	0b11'00'0'00'0,
	reg_macr_host_if_mask_all =		0b10'00'0'00'0,
	reg_macr_host_if_mask_none =	0b00'00'0'00'0,
	reg_macr_read_dir_btlr =		0b00'11'0'00'0,
	reg_macr_read_dir_tblr =		0b00'10'0'00'0,
	reg_macr_read_dir_rltb =		0b00'01'0'00'0,
	reg_macr_read_dir_lrtb =		0b00'00'0'00'0,
	reg_macr_write_dir_btlr =		0b00'00'0'11'0,
	reg_macr_write_dir_tblr =		0b00'00'0'10'0,
	reg_macr_write_dir_rltb =		0b00'00'0'01'0,
	reg_macr_write_dir_lrtb =		0b00'00'0'00'0,

	reg_icr_int_active_level_low =	0b00000000,
	reg_icr_int_active_level_high =	0b10000000,
	reg_icr_psm0_debounce =			0b01000000,
	reg_icr_psm0_trigger_low =		0b00000000,
	reg_icr_psm0_trigger_falling =	0b00010000,
	reg_icr_psm0_trigger_high =		0b00100000,
	reg_icr_psm0_trigger_rising =	0b00110000,
	reg_icr_mode_graphic =			0b00000000,
	reg_icr_mode_text =				0b00000100,
	reg_icr_write_dest_palette =	0b00000011,
	reg_icr_write_dest_cursor =		0b00000010,
	reg_icr_write_dest_gamma =		0b00000001,
	reg_icr_write_dest_ram =		0b00000000,

	reg_mpwctr_pip1_enable =		0b10000000,
	reg_mpwctr_pip2_enable =		0b01000000,
	reg_mpwctr_pip1_config =		0b00000000,
	reg_mpwctr_pip2_config =		0b00010000,
	reg_mpwctr_color_depth_24bpp =	0b00001000,
	reg_mpwctr_color_depth_16bpp =	0b00000100,
	reg_mpwctr_color_depth_8bpp =	0b00000000,
	reg_mpwctr_sync_mode =			0b00000000,
	reg_mpwctr_de_mode =			0b00000001,

	reg_dpcr_clk_invert =			0b1'0'0'0'0'000,
	reg_dpcr_clk_normal =			0b0'0'0'0'0'000,
	reg_dpcr_display_on =			0b0'1'0'0'0'000,
	reg_dpcr_display_off =			0b0'0'0'0'0'000,
	reg_dpcr_test =					0b0'0'1'0'0'000,
	reg_dpcr_hdir_toptobot =		0b0'0'0'0'0'000,
	reg_dpcr_hdir_bottotop =		0b0'0'0'1'0'000,
	reg_dpcr_vdir_toptobot =		0b0'0'0'0'0'000,
	reg_dpcr_vdir_bottotop =		0b0'0'0'0'1'000,
	reg_dpcr_rgb_rgb =				0b0'0'0'0'0'000,
	reg_dpcr_rgb_rbg =				0b0'0'0'0'0'001,
	reg_dpcr_rgb_grb =				0b0'0'0'0'0'010,
	reg_dpcr_rgb_gbr =				0b0'0'0'0'0'011,
	reg_dpcr_rgb_brg =				0b0'0'0'0'0'100,
	reg_dpcr_rgb_bgr =				0b0'0'0'0'0'101,
	reg_dpcr_rgb_gray =				0b0'0'0'0'0'110,
	reg_dpcr_rgb_idle =				0b0'0'0'0'0'111,

	reg_pcsr_hsync_low =			0b00000000,
	reg_pcsr_hsync_high =			0b10000000,
	reg_pcsr_vsync_low =			0b00000000,
	reg_pcsr_vsync_high =			0b01000000,
	reg_pcsr_pde_low =				0b00100000,
	reg_pcsr_pde_high =				0b00000000,
	reg_pcsr_pde_idle_low =			0b00000000,
	reg_pcsr_pde_idle_high =		0b00010000,
	reg_pcsr_pclk_idle_low =		0b00000000,
	reg_pcsr_pclk_idle_high =		0b00001000,
	reg_pcsr_pd_idle_low =			0b00000000,
	reg_pcsr_pd_idle_high =			0b00000100,
	reg_pcsr_hsync_idle_low =		0b00000000,
	reg_pcsr_hsync_idle_high =		0b00000010,
	reg_pcsr_vsync_idle_low =		0b00000000,
	reg_pcsr_vsync_idle_high =		0b00000001,

	reg_dcr1_drawing_active =		0b10000000,
	reg_dcr1_fill =					0b01000000,
	reg_dcr1_draw_round_rect =		0b00110000,
	reg_dcr1_draw_rectangle =		0b00100000,
	reg_dcr1_draw_arc =				0b00010000,
	reg_dcr1_draw_ellipse =			0b00000000,
	reg_dcr1_ellipse_botright =		0b00000011,
	reg_dcr1_ellipse_upright =		0b00000010,
	reg_dcr1_ellipse_upleft =		0b00000001,
	reg_dcr1_ellipse_botleft =		0b00000000,

	reg_pmuxr_timer1_div_8 =		0b11'00'00'00,
	reg_pmuxr_timer1_div_4 =		0b10'00'00'00,
	reg_pmuxr_timer1_div_2 =		0b01'00'00'00,
	reg_pmuxr_timer1_div_1 =		0b00'00'00'00,
	reg_pmuxr_timer0_div_8 =		0b00'11'00'00,
	reg_pmuxr_timer0_div_4 =		0b00'10'00'00,
	reg_pmuxr_timer0_div_2 =		0b00'01'00'00,
	reg_pmuxr_timer0_div_1 =		0b00'00'00'00,
	reg_pmuxr_pwm1_func_cclck =		0b00'00'11'00,
	reg_pmuxr_pwm1_func_timer1 =	0b00'00'10'00,
	reg_pmuxr_pwm1_error =			0b00'00'00'00,
	reg_pmuxr_pwm0_func_cclk =		0b00'00'00'11,
	reg_pmuxr_pwm0_func_timer0 =	0b00'00'00'10,
	reg_pmuxr_pwm0_func_gpio7 =		0b00'00'00'00,

	reg_pcfgr_timer1_invert =		0b01000000,
	reg_pcfgr_timer1_autoreload =	0b00100000,
	reg_pcfgr_timer1_startstop =	0b00010000,
	reg_pcfgr_timer0_deadzone =		0b00001000,
	reg_pcfgr_timer0_invert =		0b00000100,
	reg_pcfgr_timer0_autoreload =	0b00000010,
	reg_pcfgr_timer0_startstop =	0b00000001,

	reg_aw_color_read_write_pos =	0b0000'0'0'00,
	reg_aw_color_read_read_pos =	0b0000'1'0'00,
	reg_aw_color_addr_mode_block =	0b0000'0'0'00,
	reg_aw_color_addr_mode_linear =	0b0000'0'1'00,
	reg_aw_color_col_depth_24bpp =	0b0000'0'0'10,
	reg_aw_color_col_depth_16bpp =	0b0000'0'0'01,
	reg_aw_color_col_depth_8bpp =	0b0000'0'0'00,

	reg_sdrar_power_saving_pdown =	0b0'0'0'00'000,
	reg_sdrar_power_saving_self =	0b1'0'0'00'000,
	reg_sdrar_sdr_bank_4 =			0b0'0'1'00'000,
	reg_sdrar_sdr_row_2k =			0b0'0'0'00'000,
	reg_sdrar_sdr_col_256 =			0b0'0'0'00'000,

	reg_sdrmd_sdr_caslat_2 =		0b00000'010,
	reg_sdrmd_sdr_caslat_3 =		0b00000'011,

	reg_sdrcr_warning =				0b0000'1'0'0'0,
	reg_sdrcr_sdr_paramen =			0b0000'0'1'0'0,
	reg_sdrcr_sdr_psaving =			0b0000'0'0'1'0,
	reg_sdrcr_sdr_initdone =		0b0000'0'0'0'1,
};

DisplayModuleLT7381::DisplayModuleLT7381(Config& config_in, Log& log_in, Util& util_in, SPI& spi_in, LedPWM& ledpwm_in,
			int module_index_in, int x_size_in, int y_size_in, bool flip_in, bool invert_in, bool rotate_in, bool blinvert_in)
		:
			DisplayModuleSPI(config_in, log_in, util_in, spi_in, ledpwm_in, module_index_in, x_size_in, y_size_in, flip_in, invert_in, rotate_in, blinvert_in)
{
	int wait_cycles;
	int raw_status;
	int cooked_status;

	this->speed(spi_speed_initial);

	raw_status = this->read_status();
	cooked_status = raw_status & ~(status_ram_ready | status_core_busy);

	if(cooked_status != (status_write_fifo_empty | status_read_fifo_empty))
		throw(transient_exception(std::format("DisplayLT7381: invalid status: {:02x}/{:02}", raw_status, cooked_status)));

	static constexpr const int pclk_od = 2;		/* pixel clock */
	static constexpr const int pclk_r  = 5;
	static constexpr const int pclk_n  = 10;
	static constexpr const int mclk_od = 2;		/* RAM */
	static constexpr const int mclk_r  = 5;
	static constexpr const int mclk_n  = 100;
	static constexpr const int cclk_od = 2;		/* host interface, BTE etc. */
	static constexpr const int cclk_r  = 5;
	static constexpr const int cclk_n  = 100;

	static constexpr const int pplc1 = (pclk_od << 6) | (pclk_r << 1) | ((pclk_n >> 8) & 0x01);
	static constexpr const int pplc2 = pclk_n;
	static constexpr const int mplc1 = (mclk_od << 6) | (mclk_r << 1) | ((mclk_n >> 8) & 0x01);
	static constexpr const int mplc2 = mclk_n;
	static constexpr const int cplc1 = (cclk_od << 6) | (cclk_r << 1) | ((cclk_n >> 8) & 0x01);
	static constexpr const int cplc2 = cclk_n;

	this->write_register(reg_ppllc1, pplc1);
	this->write_register(reg_ppllc2, pplc2);
	this->write_register(reg_mpllc1, mplc1);
	this->write_register(reg_mpllc2, mplc2);
	this->write_register(reg_cpllc1, cplc1);
	this->write_register(reg_cpllc2, cplc2);

	this->write_register(reg_srr, reg_srr_reconfig_pll);

	for(wait_cycles = 0; wait_cycles < 8; wait_cycles++)
		if(this->read_register(reg_ccr) & reg_ccr_pll_ready)
			break;

	if(wait_cycles > 0)
		this->log << std::format("lt7381: pll wait cycles: {:d}", wait_cycles);

	this->check_register(reg_ppllc1, pplc1);
	this->check_register(reg_ppllc2, pplc2);
	this->check_register(reg_mpllc1, mplc1);
	this->check_register(reg_mpllc2, mplc2);
	this->check_register(reg_cpllc1, cplc1);
	this->check_register(reg_cpllc2, cplc2);

	this->speed(spi_speed_normal);

#if 0
	unsigned int freq_p, freq_m, freq_c;

	this->log << std::format("p-plc: {}", this->pll_status(pplc1, pplc2, freq_p));
	this->log << std::format("m-plc: {}", this->pll_status(mplc1, mplc2, freq_m));
	this->log << std::format("c-plc: {}", this->pll_status(cplc1, cplc2, freq_c));

	this->log << std::format("mclk({:d}) between cclk({:d}) and cclk*2({:d})", freq_m / 1000000, freq_c / 1000000, freq_c * 2 / 1000000);
	this->log << std::format("cclk({:d}) >= pclk * 1.5 ({:d})", freq_c / 1000000, freq_p * 15 / 10000000);
#endif

	this->write_and_check_register(reg_sdrar, reg_sdrar_power_saving_pdown | reg_sdrar_sdr_bank_4 | reg_sdrar_sdr_row_2k | reg_sdrar_sdr_col_256); // according to datasheet
	//this->write_and_check_register(reg_sdrar, 0x29); // according to sample source

	this->write_and_check_register(reg_sdrmd, reg_sdrmd_sdr_caslat_3); // according to datasheet
	this->write_and_check_register(reg_sdrref_l, 0x1a); // according to datasheet
	this->write_and_check_register(reg_sdrref_h, 0x06);
	this->write_register(reg_sdrcr, reg_sdrcr_sdr_initdone);

	for(wait_cycles = 0; wait_cycles < 256; wait_cycles++)
		if(this->read_status() & status_ram_ready)
			break;

	if(wait_cycles > 0)
		this->log << std::format("lt7381: ram init: wait cyles: {:d}", wait_cycles);

	this->write_register(reg_ccr, reg_ccr_tft_if_18b | reg_ccr_host_if_8bit);
	this->write_and_check_register(reg_macr, reg_macr_host_if_mask_none | reg_macr_read_dir_lrtb | reg_macr_write_dir_lrtb);
	this->write_and_check_register(reg_icr, reg_icr_mode_graphic | reg_icr_write_dest_ram);
	this->write_and_check_register(reg_pcsr, reg_pcsr_hsync_low | reg_pcsr_vsync_low | reg_pcsr_pde_high | reg_pcsr_hsync_idle_high | reg_pcsr_vsync_idle_high);

	this->write_and_check_register(reg_hdwr,	(this->x_size / 8) - 1);
	this->write_and_check_register(reg_hdwftr,	(this->x_size % 8));
	this->write_and_check_register(reg_vdhr_l,	(this->y_size & 0x00ff) >> 0);
	this->write_and_check_register(reg_vdhr_h,	(this->y_size & 0xff00) >> 8);

	this->write_and_check_register(reg_hndr, 0);
	this->write_and_check_register(reg_hndftr, 0);
	this->write_and_check_register(reg_hstr, 0);
	this->write_and_check_register(reg_hpwr, 0);
	this->write_and_check_register(reg_vndr_l, 0);
	this->write_and_check_register(reg_vndr_h, 0);
	this->write_and_check_register(reg_vstr, 0);
	this->write_and_check_register(reg_vpwr, 0);

	this->write_register(reg_dpcr, reg_dpcr_clk_normal | reg_dpcr_display_on | reg_dpcr_hdir_toptobot | reg_dpcr_vdir_toptobot | reg_dpcr_rgb_rgb);

	this->write_and_check_register(reg_cvssa_0, 0);
	this->write_and_check_register(reg_cvssa_1, 0);
	this->write_and_check_register(reg_cvssa_2, 0);
	this->write_and_check_register(reg_cvssa_3, 0);
	this->write_and_check_register(reg_cvs_imwth_l, (this->x_size & 0x00ff) >> 0);
	this->write_and_check_register(reg_cvs_imwth_h, (this->x_size & 0xff00) >> 8);

	this->write_and_check_register(reg_aw_color, reg_aw_color_read_write_pos | reg_aw_color_addr_mode_block | reg_aw_color_col_depth_24bpp);
	this->write_and_check_register(reg_mpwctr, reg_mpwctr_color_depth_24bpp | reg_mpwctr_de_mode);

	this->write_and_check_register(reg_misa_0, 0);
	this->write_and_check_register(reg_misa_1, 0);
	this->write_and_check_register(reg_misa_2, 0);
	this->write_and_check_register(reg_misa_3, 0);

	this->write_and_check_register(reg_miw_l, (this->x_size & 0x00ff) >> 0);
	this->write_and_check_register(reg_miw_h, (this->x_size & 0xff00) >> 8);

	this->set_main_window_position(0, 0);
	this->_set_active_layer(0);
	this->_show_layer(0);
}

std::string DisplayModuleLT7381::pll_status(unsigned int reg1, unsigned int reg2, unsigned int &freq)
{
	static const constexpr float crystal = 10000000;
	unsigned int od;
	unsigned int r;
	unsigned int n8, n07, n;
	float input_freq;
	float pll_freq;
	std::string out;

	od =	(reg1 & 0b11000000) >> 6;
	r =		(reg1 & 0b00111110) >> 1;
	n8 =	(reg1 & 0b00000001) >> 0;
	n07 =	(reg2 & 0b11111111) >> 0;
	n =		(n8 << 8) | (n07 << 0);
	input_freq = crystal / static_cast<float>(r);
	pll_freq = crystal * (static_cast<float>(n) / static_cast<float>(r)) / static_cast<float>(od);

	out = std::format("od: {:d}, r: {:d}, n: {:d}, input_freq: {:f}, pll_freq: {:f}", od, r, n, input_freq / 1000000.0F, pll_freq / 1000000.0F);

	freq = static_cast<unsigned int>(pll_freq);

	return(out);
}

void DisplayModuleLT7381::write_register(unsigned char reg, unsigned char reg_data)
{
	uint64_t data = (if_write_register << 24) | (reg << 16) | (if_write_data << 8) | (reg_data << 0);

	//this->log << std::format("write {:02x}: {:02x}", reg, reg_data);

	this->transfer({
			.send =
			{
				.command = {},
				.address =
				{
					.bits = 32,
					.data = data,
				},
			},
			.receive = {},
	});
}

int DisplayModuleLT7381::read_status()
{
	uint16_t data = if_read_status;
	SPI::data_t in;

	this->transfer({
			.send =
			{
				.command =
				{
					.bits = 8,
					.data = data,
				},
				.address = {},
			},
			.receive =
			{
				.length = 1,
				.data = &in,
			},
	});

	return(in[0]);
}

int DisplayModuleLT7381::read_register(unsigned char reg)
{
	uint64_t command = (if_write_register << 16) | (reg << 8) | (if_read_register << 0);
	SPI::data_t in;

	this->transfer({
			.send =
			{
				.command = {},
				.address =
				{
					.bits = 24,
					.data = command,
				}
			},
			.receive =
			{
				.length = 1,
				.data = &in,
			},
	});

	return(in[0]);
}

void DisplayModuleLT7381::check_register(unsigned char reg, unsigned char data_out)
{
	int data_in;

	data_in = this->read_register(reg);

	if(data_out != data_in)
		throw(transient_exception(std::format("lt7381: check_register: check failed: {:02x}: {:02x}/{:02x}", reg, data_out, data_in)));
}

void DisplayModuleLT7381::write_and_check_register(unsigned char reg, unsigned char data)
{
	this->write_register(reg, data);
	this->check_register(reg, data);
}

void DisplayModuleLT7381::set_active_window(int from_x, int from_y, int to_x, int to_y, int page)
{
	int width;
	int height;

	if(page)
	{
		from_y += this->y_size;
		to_y += this->y_size;
	}

	width = to_x - from_x + 1;
	height = to_y - from_y + 1;

	this->write_and_check_register(reg_awul_x_l, (from_x & 0x00ff) >> 0);
	this->write_and_check_register(reg_awul_x_h, (from_x & 0xff00) >> 8);
	this->write_and_check_register(reg_awul_y_l, (from_y & 0x00ff) >> 0);
	this->write_and_check_register(reg_awul_y_h, (from_y & 0xff00) >> 8);

	this->write_and_check_register(reg_aw_wth_l, (width & 0x00ff) >> 0);
	this->write_and_check_register(reg_aw_wth_h, (width & 0xff00) >> 8);
	this->write_and_check_register(reg_aw_ht_l,  (height & 0x00ff) >> 0);
	this->write_and_check_register(reg_aw_ht_h,  (height & 0xff00) >> 8);

	this->write_register(reg_curh_l, (from_x & 0x00ff) >> 0);
	this->write_register(reg_curh_h, (from_x & 0xff00) >> 8);
	this->write_register(reg_curv_l, (from_y & 0x00ff) >> 0);
	this->write_register(reg_curv_h, (from_y & 0xff00) >> 8);
}

void DisplayModuleLT7381::box(int r, int g, int b, int from_x, int from_y, int to_x, int to_y, int page)
{
	int wait_cycles;

	this->set_active_window(0, 0, this->x_size - 1, this->y_size - 1, page);

	if(page)
	{
		from_y += this->y_size;
		to_y += this->y_size;
	}

	this->write_and_check_register(reg_dlhsr_l, (from_x & 0x00ff) >> 0);
	this->write_and_check_register(reg_dlhsr_h, (from_x & 0xff00) >> 8);
	this->write_and_check_register(reg_dlvsr_l, (from_y & 0x00ff) >> 0);
	this->write_and_check_register(reg_dlvsr_h, (from_y & 0xff00) >> 8);

	this->write_and_check_register(reg_dlher_l, (to_x & 0x00ff) >> 0);
	this->write_and_check_register(reg_dlher_h, (to_x & 0xff00) >> 8);
	this->write_and_check_register(reg_dlver_l, (to_y & 0x00ff) >> 0);
	this->write_and_check_register(reg_dlver_h, (to_y & 0xff00) >> 8);

	this->write_and_check_register(reg_fgcr, r);
	this->write_and_check_register(reg_fgcg, g);
	this->write_and_check_register(reg_fgcb, b);

	this->write_register(reg_dcr1, reg_dcr1_drawing_active | reg_dcr1_fill | reg_dcr1_draw_rectangle);

	for(wait_cycles = 0; wait_cycles < 1024; wait_cycles++)
		if(!(this->read_register(reg_dcr1) & reg_dcr1_drawing_active))
			break;

	if(wait_cycles >= 1024)
		this->log << std::format("lt7381: box finished after {:d} cycles", wait_cycles);
}

void DisplayModuleLT7381::set_main_window_position(int x, int y)
{
	this->write_and_check_register(reg_mwulx_l, (x & 0x00ff) >> 0);
	this->write_and_check_register(reg_mwulx_h, (x & 0xff00) >> 8);
	this->write_and_check_register(reg_mwuly_l, (y & 0x00ff) >> 0);
	this->write_and_check_register(reg_mwuly_h, (y & 0xff00) >> 8);
}

std::string DisplayModuleLT7381::_name()
{
	return("Levetop LT7381");
}

void DisplayModuleLT7381::_brightness(int brightness)
{
	static const constexpr int max_duty = 0xffff;
	int duty = 0;
	float duty_float = 0;

	duty_float = powf(2, brightness / 10.0f) * 64.0f + 180.0f;
	duty = static_cast<int>(duty_float);

	if(duty < 245)
		duty = 0;

	if(duty >= max_duty)
		duty = max_duty;

	this->write_and_check_register(reg_pcfgr, 0); // turn off PWM
	this->write_and_check_register(reg_psclr, 12); // 100 Mhz / [12] / max_duty = 127 Hz
	this->write_and_check_register(reg_pmuxr, reg_pmuxr_timer1_div_1 | reg_pmuxr_timer0_div_1 | reg_pmuxr_pwm1_func_timer1 | reg_pmuxr_pwm0_func_timer0);
	this->write_and_check_register(reg_tcntb1_l, (max_duty & 0x00ff) >> 0);
	this->write_and_check_register(reg_tcntb1_h, (max_duty & 0xff00) >> 8);
	this->write_and_check_register(reg_tcmpb1_l, (duty & 0x00ff) >> 0);
	this->write_and_check_register(reg_tcmpb1_h, (duty & 0xff00) >> 8);
	this->write_and_check_register(reg_pcfgr, reg_pcfgr_timer1_autoreload | reg_pcfgr_timer1_startstop);
}

void DisplayModuleLT7381::_clear(int r, int g, int b)
{
	this->box(r, g, b, 0, 0, this->x_size - 1, this->y_size - 1, this->active_page);
}

void DisplayModuleLT7381::_box(const Display::box_rgb_args_t& args)
{
	this->box(args.r, args.g, args.b, args.geometry.from_x, args.geometry.from_y, args.geometry.to_x, args.geometry.to_y, this->active_page);
}

void DisplayModuleLT7381::_set_window(const Display::geometry_t& geo)
{
	this->set_active_window(geo.from_x, geo.from_y, geo.to_x, geo.to_y, this->active_page);
}

void DisplayModuleLT7381::_plot(int length, const Display::rgb_t* pixels)
{
	int current;

	this->set_leader(SPI::data_t{if_write_register, reg_mrwdp, if_write_data});

	for(current = 0; current < length; current++)
		this->push(SPI::data_t{pixels[current].b, pixels[current].g, pixels[current].r});

	this->flush();
	this->clear_leader();
}

void DisplayModuleLT7381::_set_active_layer(int layer)
{
	this->active_page = layer;
}

void DisplayModuleLT7381::_show_layer(int layer)
{
	this->set_main_window_position(0, layer ? this->y_size : 0);
}
