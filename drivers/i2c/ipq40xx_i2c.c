/*
 * Copyright (c) 2015 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <i2c.h>
#include "ipq40xx_i2c.h"
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch-qcom-common/clk.h>
#include "ipq40xx_cdp.h"

static int i2c_base_addr;
static int i2c_hw_initialized;
static int i2c_board_initialized;

static unsigned int i2c_bus_num = 1; /* default use i2c1 (gpio 34/35) */

#ifdef CONFIG_I2C_MULTI_BUS
unsigned int i2c_get_bus_num(void)
{
	return i2c_bus_num;
}

int i2c_set_bus_num(unsigned int bus_num)
{
	i2c_bus_num = bus_num;
	return 0;
}
#endif

/*
 * Reset entire QUP and all mini cores
 */
static void i2c_reset(void)
{
	writel(0x1, (i2c_base_addr + QUP_SW_RESET_OFFSET));
	udelay(5);
}

static int check_bit_state(uint32_t reg_addr, int bit_num, int val,
				int us_delay)
{
	unsigned int count = TIMEOUT_CNT;
	unsigned int bit_val = ((readl(reg_addr) >> bit_num) & 0x01);

	while (bit_val != val) {
		count--;
		if (count == 0) {
			return -ETIMEDOUT;
		}
		udelay(us_delay);
		bit_val = ((readl(reg_addr) >> bit_num) & 0x01);
	}

	return SUCCESS;
}

/*
 * Check whether QUP State is valid
 */
static int check_qup_state_valid(void)
{
	return check_bit_state(i2c_base_addr + QUP_STATE_OFFSET,
				QUP_STATE_VALID_BIT,
				QUP_STATE_VALID, 1);
}

/*
 * Configure QUP Core state
 */
static int config_i2c_state(unsigned int state)
{
	uint32_t val;
	int ret = SUCCESS;

	ret = check_qup_state_valid();
	if (ret != SUCCESS)
		return ret;

	/* Set the state  */
	val = readl(i2c_base_addr + QUP_STATE_OFFSET);
	val = ((val & ~QUP_STATE_MASK) | state);
	writel(val, (i2c_base_addr + QUP_STATE_OFFSET));
	ret = check_qup_state_valid();

	return ret;
}

/*
 * Configure I2C IO Mode.
 */
void config_i2c_mode(void)
{
	int cfg;

	cfg = readl(i2c_base_addr + QUP_IO_MODES_OFFSET);
	cfg |= (INPUT_FIFO_MODE | OUTPUT_FIFO_MODE | PACK_EN | UNPACK_EN);
	writel(cfg, i2c_base_addr + QUP_IO_MODES_OFFSET);
}

void i2c_qca_board_init(void)
{
	if (i2c_bus_num == 0) {
		i2c_base_addr = gboard_param->i2c0_cfg->i2c_base;
		qca_configure_gpio(gboard_param->i2c0_cfg->i2c_gpio, NO_OF_I2C_GPIOS);
	} else {
		i2c_base_addr = gboard_param->i2c1_cfg->i2c_base;
		qca_configure_gpio(gboard_param->i2c1_cfg->i2c_gpio, NO_OF_I2C_GPIOS);
	}

	/* Configure the I2C clock */
	i2c_clock_config(i2c_bus_num);

	i2c_hw_initialized = 0;
	i2c_board_initialized = 1;
}

void i2c_qup_mini_core_init(void)
{
	int cfg;

	cfg = readl(i2c_base_addr + QUP_CONFIG_OFFSET);
	cfg |= (QUP_CONFIG_MINI_CORE_I2C) | (I2C_BIT_WORD);

	writel(cfg, i2c_base_addr + QUP_CONFIG_OFFSET);

	writel(QUP_EN_VERSION_TWO_TAG, (i2c_base_addr +
					QUP_I2C_MASTER_CONFIG_OFFSET));
}

/*
 * QUP I2C Hardware Initialisation
 */
static int i2c_hw_init(void)
{
	int ret;

	/* QUP configuration */
	i2c_reset();

	/* Set the BLSP QUP state */
	ret = check_qup_state_valid();
	if (ret)
		return ret;

	writel(0,(i2c_base_addr + QUP_CONFIG_OFFSET));

	writel(QUP_APP_CLK_ON_EN | QUP_CORE_CLK_ON_EN |
		QUP_FIFO_CLK_GATE_EN, (i2c_base_addr + QUP_CONFIG_OFFSET));

	writel(0, i2c_base_addr + QUP_I2C_MASTER_CLK_CTL_OFFSET);
	writel(0, i2c_base_addr + QUP_TEST_CTRL_OFFSET);
	writel(0, i2c_base_addr + QUP_IO_MODES_OFFSET);
	writel(0, i2c_base_addr + QUP_OPERATIONAL_MASK_OFFSET);

	i2c_qup_mini_core_init();

	config_i2c_mode();

	writel(QUP_MX_READ_COUNT, i2c_base_addr + QUP_MX_READ_COUNT_OFFSET);
	writel(QUP_MX_WRITE_COUNT, i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);


	writel(QUP_MX_INPUT_COUNT, i2c_base_addr + QUP_MX_INPUT_COUNT_OFFSET);
	writel(QUP_MX_OUTPUT_COUNT, i2c_base_addr + QUP_MX_OUTPUT_COUNT_OFFSET);

	ret = config_i2c_state(QUP_STATE_RESET);
	if (ret)
		return ret;

	i2c_hw_initialized = 1;

	return SUCCESS;
}

/*
 * Function to check wheather Input or Output FIFO
 * has data to be serviced. For invalid slaves, this
 * flag will not be set.
 */
static int check_fifo_status(uint dir)
{
	unsigned int count = TIMEOUT_CNT;
	unsigned int status_flag;
	unsigned int val;

	if (dir == READ) {
		do {
			val = readl(i2c_base_addr
				+ QUP_OPERATIONAL_OFFSET);
			count--;
			if (count == 0)
				return -ETIMEDOUT;
			status_flag = val & INPUT_SERVICE_FLAG;
			udelay(10);
		} while (!status_flag);

	} else if (dir == WRITE) {
		do {
			val = readl(i2c_base_addr
				+ QUP_OPERATIONAL_OFFSET);
			count--;
			if (count == 0)
				return -ETIMEDOUT;
			status_flag = val & OUTPUT_FIFO_FULL;
			udelay(10);
		} while (status_flag);
	}
	return SUCCESS;
}

/*
 * Check whether the values in the OUTPUT FIFO are shifted out.
 */
static int check_write_done(void)
{
	unsigned int count = TIMEOUT_CNT;
	unsigned int status_flag;
	unsigned int val;

	do {
		val = readl(i2c_base_addr
				+ QUP_OPERATIONAL_OFFSET);
		count--;
		if (count == 0)
			return -ETIMEDOUT;
		status_flag = val & OUTPUT_FIFO_NOT_EMPTY;
		udelay(10);
	} while (status_flag);

	return SUCCESS;
}

int i2c_process_read_data(uint32_t data, uchar *buffer, int len)
{
	int idx = 0;
	uint8_t data_8 = 0;
	int index = 0;
	int rd_len = len;

	while (index < 4 && rd_len) {
		data_8 = QUP_I2C_DATA(data);
		index++;
		if (data_8 == QUP_I2C_DATA_READ_AND_STOP_SEQ) {
			index++;
			data = (data >> 16);
			continue;
		}
		if (data_8 == QUP_I2C_STOP_SEQ)
			break;
		if (data_8 == QUP_I2C_NOP_PADDING) {
			data = (data >> 8);
			continue;
		}
		buffer[idx] = data_8;
		rd_len--;
		idx++;
		data = (data >> 8);
	}
	return idx;
}

uint32_t i2c_write_read_offset(uchar chip, int alen)
{
	uint32_t tag;
	uint32_t *fifo;

	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);
	tag = QUP_I2C_START_SEQ;
	tag |= ((QUP_I2C_ADDR(chip)) | (I2C_WRITE)) << 8;
	tag |= QUP_I2C_DATA_WRITE_SEQ << 16;
	tag |= alen << 24;
	writel(tag, fifo);

	return tag;
}

uint32_t i2c_write_read_tag(uchar chip, uint addr, int alen, int data_len)
{
	uint32_t tag = 0;
	uint32_t *fifo;

	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);

	if (alen == 2) {
		/* based on the slave send msb 8 bits or lsb 8 bits first */
		tag = QUP_I2C_DATA(addr);
		tag |= QUP_I2C_DATA(addr >> 8) << 8;
		tag |= QUP_I2C_START_SEQ << 16;
		tag |= ((QUP_I2C_ADDR(chip)) | (I2C_READ)) << 24;
		writel(tag, fifo);

		tag = 0;
		tag |= QUP_I2C_DATA_READ_AND_STOP_SEQ;
		tag |= data_len << 8;
		writel(tag, fifo);
	} else if (alen == 1) {
		tag = QUP_I2C_DATA(addr);
		tag |= QUP_I2C_START_SEQ << 8;
		tag |= ((QUP_I2C_ADDR(chip)) | (I2C_READ)) << 16;
		tag |= (QUP_I2C_DATA_READ_AND_STOP_SEQ << 24);
		writel(tag, fifo);

		tag = 0;
		tag |= data_len;
		writel(tag, fifo);
	} else if (alen == 0) {
		tag |= QUP_I2C_START_SEQ;
		tag |= ((QUP_I2C_ADDR(chip)) | (I2C_READ)) << 8;
		tag |= (QUP_I2C_DATA_READ_AND_STOP_SEQ << 16);
		tag |= data_len << 24;
		writel(tag, fifo);
	}
	return 0;
}

int i2c_read(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int ret = 0;
	int nack = 0;
	uint32_t data = 0;
	uint8_t data_len = len;
	uint32_t *fifo;
	int idx = 0;
	int cfg;

	if (i2c_bus_num == 0)
		i2c_base_addr = gboard_param->i2c0_cfg->i2c_base;
	else
		i2c_base_addr = gboard_param->i2c1_cfg->i2c_base;
 

	config_i2c_state(QUP_STATE_RESET);

	if (!i2c_board_initialized) {
		i2c_qca_board_init();
	}

	if (!i2c_hw_initialized) {
		i2c_hw_init();
	}

	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_OFFSET);
	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_EN_OFFSET);
	writel(0, i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET);

	if (alen != 0)
		writel((OUT_FIFO_RD_TAG_BYTE_CNT + alen),
			i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);
	else
		writel(OUT_FIFO_WR_TAG_BYTE_CNT,
			i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);

	writel((IN_FIFO_TAG_BYTE_CNT + data_len),
			i2c_base_addr + QUP_MX_READ_COUNT_OFFSET);

	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State run failed\n");
		goto out;
	}

	/* Configure the I2C Master clock */
	cfg = (QUP_INPUT_CLK / (I2C_CLK_100KHZ * 2)) - 3;
	writel(cfg, i2c_base_addr + QUP_I2C_MASTER_CLK_CTL_OFFSET);
	/* Write to FIFO in Pause State */
	/* Set to PAUSE state */
	ret = config_i2c_state(QUP_STATE_PAUSE);
	if (ret != SUCCESS) {
		debug("State Pause failed\n");
		goto out;
	}

	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);

	if (alen != 0)
		data = i2c_write_read_offset(chip, alen);

	data = i2c_write_read_tag(chip, addr, alen, data_len);

	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State run failed\n");
		goto out;
	}

	mdelay(2);
	ret = check_write_done();
	if (ret != SUCCESS) {
		debug("Write done failed\n");
		goto out;
	}

	nack = readl(i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET) & NACK_BIT_MASK;
	nack = nack >> NACK_BIT_SHIFT;
	if (nack == 1) {
		debug("NACK RECVD\n");
		return -ENACK;
	}

	if (readl(i2c_base_addr + QUP_OPERATIONAL_OFFSET)
			& OUTPUT_SERVICE_FLAG) {
		writel(OUTPUT_SERVICE_FLAG,
			i2c_base_addr + QUP_OPERATIONAL_OFFSET);
	}

	fifo = (uint32_t *)(i2c_base_addr + QUP_INPUT_FIFO_OFFSET);

	mdelay(2);
	ret = check_fifo_status(READ);
	if (ret != SUCCESS) {
		debug("Read status failed\n");
		goto out;
	}
	while (len) {
		/* Read the data from the FIFO */
		data = readl(fifo);

		ret = i2c_process_read_data(data, buffer + idx, len);
		if (ret) {
		    idx += ret;
		    len -= ret;
		}
	}

	if (readl(i2c_base_addr + QUP_OPERATIONAL_OFFSET)
		& INPUT_SERVICE_FLAG) {
		writel(INPUT_SERVICE_FLAG,
			i2c_base_addr + QUP_OPERATIONAL_OFFSET);
	}
	(void)config_i2c_state(QUP_STATE_RESET);
	return SUCCESS;
out:
	/*
	 * Put the I2C Core back in the Reset State to end the transfer.
	 */
	(void)config_i2c_state(QUP_STATE_RESET);
	writel(QUP_MX_READ_COUNT, i2c_base_addr + QUP_MX_READ_COUNT_OFFSET);
	return ret;
}

int create_data_byte(uint16_t *data, uchar *buffer, int len)
{
	int idx = 0;
	if (len == 0) {
		return 0;
	} else {
		*data = QUP_I2C_DATA(buffer[idx]);
		idx++;
		len--;
	}
	if (len == 0) {
	    return idx;
	} else {
		*data = (*data << 8);
		*data |= QUP_I2C_DATA(buffer[idx]);
		idx++;
		len--;
	}
	return idx;
}

uint32_t i2c_frame_wr_tag(uchar chip, uint8_t data_len, int alen)
{
	uint32_t tag;

	tag = QUP_I2C_START_SEQ;
	tag |= (((QUP_I2C_ADDR(chip)) | (I2C_WRITE)) << 8);
	tag |= (QUP_I2C_DATA_WRITE_AND_STOP_SEQ << 16);
	tag |= (data_len + alen) << 24;
	return tag;
}

int i2c_write(uchar chip, uint addr, int alen, uchar *buffer, int len)
{
	int ret = 0;
	int nack = 0;
	int idx = 0;
	int first = 1;
	uint32_t data = 0;
	uint16_t data_lsb_16 = 0;
	uint16_t data_msb_16 = 0;
	uint8_t data_len = len;
	uint32_t *fifo;
	uint32_t cfg;

	if (i2c_bus_num == 0)
		i2c_base_addr = gboard_param->i2c0_cfg->i2c_base;
	else
		i2c_base_addr = gboard_param->i2c1_cfg->i2c_base;
	/* Set to Reset State */
	ret = config_i2c_state(QUP_STATE_RESET);

	if (!i2c_board_initialized) {
		i2c_qca_board_init();
	}

	if (!i2c_hw_initialized) {
		i2c_hw_init();
	}

	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_OFFSET);
	writel(0x3C, i2c_base_addr + QUP_ERROR_FLAGS_EN_OFFSET);
	writel(0, i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET);

	writel((OUT_FIFO_WR_TAG_BYTE_CNT + len + alen),
		i2c_base_addr + QUP_MX_WRITE_COUNT_OFFSET);


	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State run failed\n");
		goto out;
	}

	/* Configure the I2C Master clock */
	cfg = (QUP_INPUT_CLK / (I2C_CLK_100KHZ * 2)) - 3;
	writel(cfg, i2c_base_addr + QUP_I2C_MASTER_CLK_CTL_OFFSET);


	/* Write to FIFO in Pause State */
	/* Set to PAUSE state */
	ret = config_i2c_state(QUP_STATE_PAUSE);
	if (ret != SUCCESS) {
		debug("State Pause failed\n");
		goto out;
	}
	fifo = (uint32_t *) (i2c_base_addr + QUP_OUTPUT_FIFO_OFFSET);
	data = i2c_frame_wr_tag(chip, data_len, alen);

	/* Write tags to the FIFO along with Slave address
	 * and Write len */
	writel(data, fifo);

	while (len > 0) {
		data_lsb_16 = 0;
		data_msb_16 = 0;
		data = 0;
		if ((first == 1) && (alen != 0)) {
			if (alen == 2) {
				/* based on the slave send msb 8 bits or lsb 8 bits first */
				data_lsb_16 = QUP_I2C_DATA(addr);
				data_lsb_16 |= QUP_I2C_DATA(addr >> 8) << 8;
			} else if (alen == 1) {
				data_lsb_16 = QUP_I2C_DATA(addr);
				data_lsb_16 |= QUP_I2C_DATA(buffer[idx]) << 8;
				idx++;
				len --;
			}
			first = 0;
			ret = 2;
		} else {
			ret = create_data_byte(&data_lsb_16, buffer + idx, len);
			idx += ret;
			len -= ret;
		}
		if(ret == 2) {
			ret = create_data_byte(&data_msb_16, buffer + idx, len);
			idx += ret;
			len -= ret;
		}
		data |= data_msb_16;
		data = (data << 16);
		data |= data_lsb_16;
		writel(data, fifo);
	}

	/* Set to RUN state */
	ret = config_i2c_state(QUP_STATE_RUN);
	if (ret != SUCCESS) {
		debug("State Run failed\n");
		goto out;
	}

	/* Clear Operational Flag */
	if (readl(i2c_base_addr + QUP_OPERATIONAL_OFFSET)
			& OUTPUT_SERVICE_FLAG) {
		writel(OUTPUT_SERVICE_FLAG,
			i2c_base_addr + QUP_OPERATIONAL_OFFSET);
	}

	mdelay(2);
	ret = check_write_done();
	if (ret != SUCCESS) {
		debug("Write done failed\n");
		goto out;
	}

	nack = readl(i2c_base_addr + QUP_I2C_MASTER_STATUS_OFFSET) & NACK_BIT_MASK;
	nack = nack >> NACK_BIT_SHIFT;
	if (nack == 1) {
		debug("NACK RECVD\n");
		return -ENACK;
	}
out:
	/*
	 * Put the I2C Core back in the Reset State to end the transfer.
	 */
	(void)config_i2c_state(QUP_STATE_RESET);
	return ret;
}

/*
 * Probe the given I2C chip address.
 * Returns 0 if a chip responded.
 */
int i2c_probe(uchar chip)
{
	uchar buf;

	/*send the third parameter alen as per the i2c slave*/
	return i2c_read(chip, 0x0, 0x0, &buf, 0x1);
}

void i2c_init(int speed, int slaveaddr)
{
	debug("i2c_init(speed=%u, slaveaddr=0x%x)\n", speed, slaveaddr);
}

/* Foxconn add start, Max Ding, 08/08/2016 led control for I2C LED chip in uboot: TI LP5562*/
/* For I2C led device: lp5562 */
/* led_color: enum LED_LP5562_COLOR
 * led_action: 0: off, 1: on, 2: blink 
 * led_percent: 0-100 percentage of brightness;
 */
int i2c_led_ctrl(unsigned int led_color, unsigned int led_action, unsigned int led_percent)
{
    struct s_led_color {
        unsigned char r_cur;
        unsigned char r_pwm;
        unsigned char g_cur;
        unsigned char g_pwm;
        unsigned char b_cur;
        unsigned char b_pwm;
        unsigned char w_cur;
        unsigned char w_pwm;
    };

    int i;
    unsigned char byte;

    /* current RGB maximum lumens: RGB: 45, 112, 28.5(it will be adjusted later), so we use 90:224:57 as Proportion */
    /* current RGB maximum lumens: RGB: 45, 112, 28.5(it will be adjusted later), so we use 90:224:57 as Proportion */
    struct s_led_color led_colors[LED_LP5562_NUM] = {
        {0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, //Red
        {0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00}, //Green
        {0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00}, //Blue
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff}, //White
        {0xff, 0xff, 0x1a, 0xff, 0x00, 0x00, 0x00, 0x00}, //Amber --> adjust it by my eye
        {0x00, 0x00, 0x35, 0xff, 0xff, 0xff, 0x00, 0x00}, //Cyan --> adjust it by my eye
        {0xff, 0xff, 0x00, 0x00, 0x80, 0xff, 0x00, 0x00}, //Magenta --> adjust it by my eye
        {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},  //All --> for TE test
        {0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff}  //Custom --> for ID test        
    };


    if (led_color >= LED_LP5562_NUM)
        return -1;

    if (led_action > 3)
        return -1;

    for (i=0; i<LED_LP5562_NUM; i++) {
        led_colors[i].r_cur = led_colors[i].r_cur*led_percent/100;
        led_colors[i].g_cur = led_colors[i].g_cur*led_percent/100;
        led_colors[i].b_cur = led_colors[i].b_cur*led_percent/100;
        led_colors[i].w_cur = led_colors[i].w_cur*led_percent/100;
    }
    
    byte = 0xff;
    i2c_write(0x30, 0x0d, 1, &byte, 1);
    //i2c_write(0x31, 0x0d, 1, &byte, 1);
    i2c_write(0x32, 0x0d, 1, &byte, 1);
    i2c_write(0x33, 0x0d, 1, &byte, 1);
        
    if ((0 == led_action) || (led_percent == 0)) {
        //system("i2cset -y 1 0x30 0x0d 0xff"); //reset
        byte = 0xff;
        //i2c_write(0x30, 0x0d, 1, &byte, 1);
        i2c_write(0x31, 0x0d, 1, &byte, 1);
        //i2c_write(0x32, 0x0d, 1, &byte, 1);
        //i2c_write(0x33, 0x0d, 1, &byte, 1);
    } else if (1 == led_action) {
        //system("i2cset -y 1 0x30 0x0d 0xff"); //reset
        byte = 0xff;
        //i2c_write(0x30, 0x0d, 1, &byte, 1);
        i2c_write(0x31, 0x0d, 1, &byte, 1);
        //i2c_write(0x32, 0x0d, 1, &byte, 1);
        //i2c_write(0x33, 0x0d, 1, &byte, 1);
        
        //system("i2cset -y 1 0x30 0x00 0x40"); //enable chip
        byte = 0x40;
        //i2c_write(0x30, 0x00, 1, &byte, 1);
        i2c_write(0x31, 0x00, 1, &byte, 1);
        //i2c_write(0x32, 0x00, 1, &byte, 1);
        //i2c_write(0x33, 0x00, 1, &byte, 1); 
        //usleep(600); // should be > 500us per LP5562 data sheet
        udelay(600);
        //system("i2cset -y 1 0x30 0x08 0x01"); //use internal clock
        byte = 0x01;
        //i2c_write(0x30, 0x08, 1, &byte, 1); 
        i2c_write(0x31, 0x08, 1, &byte, 1);
        //i2c_write(0x32, 0x08, 1, &byte, 1);
        //i2c_write(0x33, 0x08, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x70 0x00"); //(Configure all LED outputs to be controlled from I2C registers
        byte = 0x00;
        //i2c_write(0x30, 0x70, 1, &byte, 1);
        i2c_write(0x31, 0x70, 1, &byte, 1);
        //i2c_write(0x32, 0x70, 1, &byte, 1);
        //i2c_write(0x33, 0x70, 1, &byte, 1);
        //set current and pwm 0-255
        if (led_colors[led_color].r_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x07 0x%02x", led_colors[led_color].r_cur);
            //system(cmd);
            byte = led_colors[led_color].r_cur;
            //i2c_write(0x30, 0x07, 1, &byte, 1);
            i2c_write(0x31, 0x07, 1, &byte, 1);
            //i2c_write(0x32, 0x07, 1, &byte, 1);
            //i2c_write(0x33, 0x07, 1, &byte, 1);
        }
        if (led_colors[led_color].r_pwm) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x04 0x%02x", led_colors[led_color].r_pwm);
            //system(cmd);
            byte = led_colors[led_color].r_pwm;
            //i2c_write(0x30, 0x04, 1, &byte, 1);
            i2c_write(0x31, 0x04, 1, &byte, 1);
            //i2c_write(0x32, 0x04, 1, &byte, 1);
            //i2c_write(0x33, 0x04, 1, &byte, 1);
        }
        if (led_colors[led_color].g_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x06 0x%02x", led_colors[led_color].g_cur);
            //system(cmd);
            byte = led_colors[led_color].g_cur;
            //i2c_write(0x30, 0x06, 1, &byte, 1);
            i2c_write(0x31, 0x06, 1, &byte, 1);
            //i2c_write(0x32, 0x06, 1, &byte, 1);
            //i2c_write(0x33, 0x06, 1, &byte, 1);
        }
        if (led_colors[led_color].g_pwm) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x03 0x%02x", led_colors[led_color].g_pwm);
            //system(cmd);
            byte = led_colors[led_color].g_pwm;
            //i2c_write(0x30, 0x03, 1, &byte, 1);
            i2c_write(0x31, 0x03, 1, &byte, 1);
            //i2c_write(0x32, 0x03, 1, &byte, 1);
            //i2c_write(0x33, 0x03, 1, &byte, 1);
            
        }
        if (led_colors[led_color].b_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x05 0x%02x", led_colors[led_color].b_cur);
            //system(cmd);
            byte = led_colors[led_color].b_cur;
            //i2c_write(0x30, 0x05, 1, &byte, 1);
            i2c_write(0x31, 0x05, 1, &byte, 1);
            //i2c_write(0x32, 0x05, 1, &byte, 1);
            //i2c_write(0x33, 0x05, 1, &byte, 1);
        }
        if (led_colors[led_color].b_pwm) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x02 0x%02x", led_colors[led_color].b_pwm);
            //system(cmd);
            byte = led_colors[led_color].b_pwm;
            //i2c_write(0x30, 0x02, 1, &byte, 1);
            i2c_write(0x31, 0x02, 1, &byte, 1);
            //i2c_write(0x32, 0x02, 1, &byte, 1);
            //i2c_write(0x33, 0x02, 1, &byte, 1);
        }
        if (led_colors[led_color].w_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x0f 0x%02x", led_colors[led_color].w_cur);
            //system(cmd);
            byte = led_colors[led_color].w_cur;
            //i2c_write(0x30, 0x0f, 1, &byte, 1);
            i2c_write(0x31, 0x0f, 1, &byte, 1);
            //i2c_write(0x32, 0x0f, 1, &byte, 1);
            //i2c_write(0x33, 0x0f, 1, &byte, 1);
        }
        if (led_colors[led_color].w_pwm) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x0e 0x%02x", led_colors[led_color].w_pwm);
            //system(cmd);
            byte = led_colors[led_color].w_pwm;
            //i2c_write(0x30, 0x0e, 1, &byte, 1);
            i2c_write(0x31, 0x0e, 1, &byte, 1);
            //i2c_write(0x32, 0x0e, 1, &byte, 1);
            //i2c_write(0x33, 0x0e, 1, &byte, 1);
        }
    } else if (2 == led_action) {
        unsigned int led_engine1_map = 0;

        //system("i2cset -y 1 0x30 0x0d 0xff"); //reset
        byte = 0xff;
        //i2c_write(0x30, 0x0d, 1, &byte, 1);
        i2c_write(0x31, 0x0d, 1, &byte, 1);
        //i2c_write(0x32, 0x0d, 1, &byte, 1);
        //i2c_write(0x33, 0x0d, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x00 0xc0"); //enable chip & LOG bit(logarithmic scale)
        byte = 0xc0;
        //i2c_write(0x30, 0x00, 1, &byte, 1);
        i2c_write(0x31, 0x00, 1, &byte, 1);
        //i2c_write(0x32, 0x00, 1, &byte, 1);
        //i2c_write(0x33, 0x00, 1, &byte, 1);
        //usleep(600); // should be > 500us per LP5562 data sheet
        udelay(600);
        //system("i2cset -y 1 0x30 0x01 0x10"); //use engine 1
        byte = 0x10;
        //i2c_write(0x30, 0x01, 1, &byte, 1);
        i2c_write(0x31, 0x01, 1, &byte, 1);
        //i2c_write(0x32, 0x01, 1, &byte, 1);
        //i2c_write(0x33, 0x01, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x08 0x01"); //use internal clock
        byte = 0x01;
        //i2c_write(0x30, 0x08, 1, &byte, 1);
        i2c_write(0x31, 0x08, 1, &byte, 1);
        //i2c_write(0x32, 0x08, 1, &byte, 1);
        //i2c_write(0x33, 0x08, 1, &byte, 1);

        //set current 0-255
        if (led_colors[led_color].r_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x07 0x%02x", led_colors[led_color].r_cur);
            //system(cmd);
            byte = led_colors[led_color].r_cur;
            //i2c_write(0x30, 0x07, 1, &byte, 1);
            i2c_write(0x31, 0x07, 1, &byte, 1);
            //i2c_write(0x32, 0x07, 1, &byte, 1);
            //i2c_write(0x33, 0x07, 1, &byte, 1);
            led_engine1_map |= 0x10;
        }
        if (led_colors[led_color].g_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x06 0x%02x", led_colors[led_color].g_cur);
            //system(cmd);
            byte = led_colors[led_color].g_cur;
            //i2c_write(0x30, 0x06, 1, &byte, 1);
            //i2c_write(0x31, 0x06, 1, &byte, 1);
            i2c_write(0x32, 0x06, 1, &byte, 1);
            //i2c_write(0x33, 0x06, 1, &byte, 1);
            led_engine1_map |= 0x04;
        }
        if (led_colors[led_color].b_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x05 0x%02x", led_colors[led_color].b_cur);
            //system(cmd);
            byte = led_colors[led_color].b_cur;
            //i2c_write(0x30, 0x05, 1, &byte, 1);
            i2c_write(0x31, 0x05, 1, &byte, 1);
            //i2c_write(0x32, 0x05, 1, &byte, 1);
            //i2c_write(0x33, 0x05, 1, &byte, 1);
            led_engine1_map |= 0x01;
        }
        if (led_colors[led_color].w_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x0f 0x%02x", led_colors[led_color].w_cur);
            //system(cmd);
            byte = led_colors[led_color].w_cur;
            //i2c_write(0x30, 0x0f, 1, &byte, 1);
            i2c_write(0x31, 0x0f, 1, &byte, 1);
            //i2c_write(0x32, 0x0f, 1, &byte, 1);
            //i2c_write(0x33, 0x0f, 1, &byte, 1);
            led_engine1_map |= 0x40;
        }

        //sprintf(cmd, "i2cset -y 1 0x30 0x70 0x%02x", led_engine1_map); //Configure related LED outputs to be controlled by engine1
        //system(cmd);
        byte = led_engine1_map;
        //i2c_write(0x30, 0x70, 1, &byte, 1);
        i2c_write(0x31, 0x70, 1, &byte, 1);
        //i2c_write(0x32, 0x70, 1, &byte, 1);
        //i2c_write(0x33, 0x70, 1, &byte, 1);

        //add program for engine 1
        //make brightness(pwm) up to 255, blink every 3 second
        //system("i2cset -y 1 0x30 0x10 0x02"); //ramp up to PWM value 128 in 125*5 ms
        byte = 0x05;
        //i2c_write(0x30, 0x10, 1, &byte, 1);
        i2c_write(0x31, 0x10, 1, &byte, 1);
        //i2c_write(0x32, 0x10, 1, &byte, 1);
        //i2c_write(0x33, 0x10, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x11 0x7f");
        byte = 0x7f;
        //i2c_write(0x30, 0x11, 1, &byte, 1);
        i2c_write(0x31, 0x11, 1, &byte, 1);
        //i2c_write(0x32, 0x11, 1, &byte, 1);
        //i2c_write(0x33, 0x11, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x12 0x02"); //ramp up to PWM value 255 in 125*5 ms
        byte = 0x05;
        //i2c_write(0x30, 0x12, 1, &byte, 1);
        i2c_write(0x31, 0x12, 1, &byte, 1);
        //i2c_write(0x32, 0x12, 1, &byte, 1);
        //i2c_write(0x33, 0x12, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x13 0x7e");
        byte = 0x7e;
        //i2c_write(0x30, 0x13, 1, &byte, 1);
        i2c_write(0x31, 0x13, 1, &byte, 1);
        //i2c_write(0x32, 0x13, 1, &byte, 1);
        //i2c_write(0x33, 0x13, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x14 0x4d"); //wait 200 ms
        byte = 0x5a;
        //i2c_write(0x30, 0x14, 1, &byte, 1);
        i2c_write(0x31, 0x14, 1, &byte, 1);
        //i2c_write(0x32, 0x14, 1, &byte, 1);
        //i2c_write(0x33, 0x14, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x15 0x00");
        byte = 0x00;
        //i2c_write(0x30, 0x15, 1, &byte, 1);
        i2c_write(0x31, 0x15, 1, &byte, 1);
        //i2c_write(0x32, 0x15, 1, &byte, 1);
        //i2c_write(0x33, 0x15, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x16 0x02"); //ramp down to PWM value 128 in 125*5 ms
        byte = 0x08;
        //i2c_write(0x30, 0x16, 1, &byte, 1);
        i2c_write(0x31, 0x16, 1, &byte, 1);
        //i2c_write(0x32, 0x16, 1, &byte, 1);
        //i2c_write(0x33, 0x16, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x17 0xfe");
        byte = 0xfe;
        //i2c_write(0x30, 0x17, 1, &byte, 1);
        i2c_write(0x31, 0x17, 1, &byte, 1);
        //i2c_write(0x32, 0x17, 1, &byte, 1);
        //i2c_write(0x33, 0x17, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x18 0x02"); //ramp down to PWM value 0 in 125*5 ms
        byte = 0x08;
        //i2c_write(0x30, 0x18, 1, &byte, 1);
        i2c_write(0x31, 0x18, 1, &byte, 1);
        //i2c_write(0x32, 0x18, 1, &byte, 1);
        //i2c_write(0x33, 0x18, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x19 0xff");
        byte = 0xff;
        //i2c_write(0x30, 0x19, 1, &byte, 1);
        i2c_write(0x31, 0x19, 1, &byte, 1);
        //i2c_write(0x32, 0x19, 1, &byte, 1);
        //i2c_write(0x33, 0x19, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x1a 0x53"); //wait 600 ms
        byte = 0x66;
        //i2c_write(0x30, 0x1a, 1, &byte, 1);
        i2c_write(0x31, 0x1a, 1, &byte, 1);
        //i2c_write(0x32, 0x1a, 1, &byte, 1);
        //i2c_write(0x33, 0x1a, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x1b 0x00");
        byte = 0x00;
        //i2c_write(0x30, 0x1b, 1, &byte, 1);
        i2c_write(0x31, 0x1b, 1, &byte, 1);
        //i2c_write(0x32, 0x1b, 1, &byte, 1);
        //i2c_write(0x33, 0x1b, 1, &byte, 1);

        //execute engine 1 program
        //system("i2cset -y 1 0x30 0x01 0x20");
        byte = 0x20;
        //i2c_write(0x30, 0x01, 1, &byte, 1);
        i2c_write(0x31, 0x01, 1, &byte, 1);
        //i2c_write(0x32, 0x01, 1, &byte, 1);
        //i2c_write(0x33, 0x01, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x00 0xe0"); //also enable LOG bit(logarithmic scale)
        byte = 0xe0;
        //i2c_write(0x30, 0x00, 1, &byte, 1);
        i2c_write(0x31, 0x00, 1, &byte, 1);
        //i2c_write(0x32, 0x00, 1, &byte, 1);
        //i2c_write(0x33, 0x00, 1, &byte, 1);
    }else if (3 == led_action) {
    
        unsigned int led_engine1_map = 0;

        //system("i2cset -y 1 0x30 0x0d 0xff"); //reset
        byte = 0xff;
        //i2c_write(0x30, 0x0d, 1, &byte, 1);
        i2c_write(0x31, 0x0d, 1, &byte, 1);
        //i2c_write(0x32, 0x0d, 1, &byte, 1);
        //i2c_write(0x33, 0x0d, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x00 0xc0"); //enable chip & LOG bit(logarithmic scale)
        byte = 0xc0;
        //i2c_write(0x30, 0x00, 1, &byte, 1);
        i2c_write(0x31, 0x00, 1, &byte, 1);
        //i2c_write(0x32, 0x00, 1, &byte, 1);
        //i2c_write(0x33, 0x00, 1, &byte, 1);
        //usleep(600); // should be > 500us per LP5562 data sheet
        udelay(600);
        //system("i2cset -y 1 0x30 0x01 0x10"); //use engine 1
        byte = 0x10;
        //i2c_write(0x30, 0x01, 1, &byte, 1);
        i2c_write(0x31, 0x01, 1, &byte, 1);
        //i2c_write(0x32, 0x01, 1, &byte, 1);
        //i2c_write(0x33, 0x01, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x08 0x01"); //use internal clock
        byte = 0x01;
        //i2c_write(0x30, 0x08, 1, &byte, 1);
        i2c_write(0x31, 0x08, 1, &byte, 1);
        //i2c_write(0x32, 0x08, 1, &byte, 1);
        //i2c_write(0x33, 0x08, 1, &byte, 1);

        //set current 0-255
        if (led_colors[led_color].r_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x07 0x%02x", led_colors[led_color].r_cur);
            //system(cmd);
            byte = led_colors[led_color].r_cur;
            //i2c_write(0x30, 0x07, 1, &byte, 1);
            i2c_write(0x31, 0x07, 1, &byte, 1);
            //i2c_write(0x32, 0x07, 1, &byte, 1);
            //i2c_write(0x33, 0x07, 1, &byte, 1);
            led_engine1_map |= 0x10;
        }
        if (led_colors[led_color].g_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x06 0x%02x", led_colors[led_color].g_cur);
            //system(cmd);
            byte = led_colors[led_color].g_cur;
            //i2c_write(0x30, 0x06, 1, &byte, 1);
            i2c_write(0x31, 0x06, 1, &byte, 1);
            //i2c_write(0x32, 0x06, 1, &byte, 1);
            //i2c_write(0x33, 0x06, 1, &byte, 1);
            led_engine1_map |= 0x04;
        }
        if (led_colors[led_color].b_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x05 0x%02x", led_colors[led_color].b_cur);
            //system(cmd);
            byte = led_colors[led_color].b_cur;
            //i2c_write(0x30, 0x05, 1, &byte, 1);
            i2c_write(0x31, 0x05, 1, &byte, 1);
            //i2c_write(0x32, 0x05, 1, &byte, 1);
            //i2c_write(0x33, 0x05, 1, &byte, 1);
            led_engine1_map |= 0x01;
        }
        if (led_colors[led_color].w_cur) {
            //sprintf(cmd, "i2cset -y 1 0x30 0x0f 0x%02x", led_colors[led_color].w_cur);
            //system(cmd);
            byte = led_colors[led_color].w_cur;
            //i2c_write(0x30, 0x0f, 1, &byte, 1);
            i2c_write(0x31, 0x0f, 1, &byte, 1);
            //i2c_write(0x32, 0x0f, 1, &byte, 1);
            //i2c_write(0x33, 0x0f, 1, &byte, 1);
            led_engine1_map |= 0x40;
        }

        //sprintf(cmd, "i2cset -y 1 0x30 0x70 0x%02x", led_engine1_map); //Configure related LED outputs to be controlled by engine1
        //system(cmd);
        byte = led_engine1_map;
        //i2c_write(0x30, 0x70, 1, &byte, 1);
        i2c_write(0x31, 0x70, 1, &byte, 1);
        //i2c_write(0x32, 0x70, 1, &byte, 1);
        //i2c_write(0x33, 0x70, 1, &byte, 1);

        //add program for engine 1
        //make brightness(pwm) up to 255, blink every 3 second
        //system("i2cset -y 1 0x30 0x10 0x02"); //ramp up to PWM value 128 in 125*5 ms
        byte = 0x40;
        //i2c_write(0x30, 0x10, 1, &byte, 1);
        i2c_write(0x31, 0x10, 1, &byte, 1);
        //i2c_write(0x32, 0x10, 1, &byte, 1);
        //i2c_write(0x33, 0x10, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x11 0x7f");
        byte = 0xff;
        //i2c_write(0x30, 0x11, 1, &byte, 1);
        i2c_write(0x31, 0x11, 1, &byte, 1);
        //i2c_write(0x32, 0x11, 1, &byte, 1);
        //i2c_write(0x33, 0x11, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x12 0x02"); //ramp up to PWM value 255 in 125*5 ms
        byte = 0x7f;
        //i2c_write(0x30, 0x12, 1, &byte, 1);
        i2c_write(0x31, 0x12, 1, &byte, 1);
        //i2c_write(0x32, 0x12, 1, &byte, 1);
        //i2c_write(0x33, 0x12, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x13 0x7e");
        byte = 0x00;
        //i2c_write(0x30, 0x13, 1, &byte, 1);
        i2c_write(0x31, 0x13, 1, &byte, 1);
        //i2c_write(0x32, 0x13, 1, &byte, 1);
        //i2c_write(0x33, 0x13, 1, &byte, 1);
		
		byte = 0x60;
        //i2c_write(0x30, 0x14, 1, &byte, 1);
        i2c_write(0x31, 0x14, 1, &byte, 1);
        //i2c_write(0x32, 0x14, 1, &byte, 1);
        //i2c_write(0x33, 0x14, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x13 0x7e");
        byte = 0x00;
        //i2c_write(0x30, 0x15, 1, &byte, 1);
        i2c_write(0x31, 0x15, 1, &byte, 1);
        //i2c_write(0x32, 0x15, 1, &byte, 1);
        //i2c_write(0x33, 0x15, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x14 0x4d"); //wait 200 ms
        byte = 0x40;
        //i2c_write(0x30, 0x16, 1, &byte, 1);
        i2c_write(0x31, 0x16, 1, &byte, 1);
        //i2c_write(0x32, 0x16, 1, &byte, 1);
        //i2c_write(0x33, 0x16, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x15 0x00");
        byte = 0x00;
        //i2c_write(0x30, 0x17, 1, &byte, 1);
        i2c_write(0x31, 0x17, 1, &byte, 1);
        //i2c_write(0x32, 0x17, 1, &byte, 1);
        //i2c_write(0x33, 0x17, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x16 0x02"); //ramp down to PWM value 128 in 125*5 ms
        byte = 0x7f;
        //i2c_write(0x30, 0x18, 1, &byte, 1);
        i2c_write(0x31, 0x18, 1, &byte, 1);
        //i2c_write(0x32, 0x18, 1, &byte, 1);
        //i2c_write(0x33, 0x18, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x17 0xfe");
        byte = 0x00;
        //i2c_write(0x30, 0x19, 1, &byte, 1);
        i2c_write(0x31, 0x19, 1, &byte, 1);
        //i2c_write(0x32, 0x19, 1, &byte, 1);
        //i2c_write(0x33, 0x19, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x18 0x02"); //ramp down to PWM value 0 in 125*5 ms


        //execute engine 1 program
        //system("i2cset -y 1 0x30 0x01 0x20");
        byte = 0x20;
        //i2c_write(0x30, 0x01, 1, &byte, 1);
        i2c_write(0x31, 0x01, 1, &byte, 1);
        //i2c_write(0x32, 0x01, 1, &byte, 1);
        //i2c_write(0x33, 0x01, 1, &byte, 1);
        //system("i2cset -y 1 0x30 0x00 0xe0"); //also enable LOG bit(logarithmic scale)
        byte = 0xe0;
        //i2c_write(0x30, 0x00, 1, &byte, 1);
        i2c_write(0x31, 0x00, 1, &byte, 1);
        //i2c_write(0x32, 0x00, 1, &byte, 1);
        //i2c_write(0x33, 0x00, 1, &byte, 1);
    }

    return 0;
}
/* Foxconn add end, Max Ding, 08/08/2016 */
