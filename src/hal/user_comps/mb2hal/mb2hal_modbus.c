#include <sys/time.h>
#include "mb2hal.h"

static retCode map_read(mb_tx_t *this_mb_tx, uint16_t *data)
{
	for (int counter = 0; counter < this_mb_tx->nb_hal_map_pin; counter++) {
		hal_map_pin_t *m = this_mb_tx->hal_map_pin + counter;
		int addr = m->addr - this_mb_tx->mb_tx_1st_addr;
		uint32_t val;

		if (this_mb_tx->hal_map_pin[counter].width == 2) {
			val = ((uint32_t)data[addr] << 16) | data[addr+1];
		}
		else {
			if (m->type == HAL_U32) val = data[addr];
			else val = (int16_t)data[addr];
		}

		switch (m->type) {
		case HAL_S32:
			this_mb_tx->pin_value[counter]->_s = (hal_s32_t)val * m->scale + m->offset;
			break;
		case HAL_U32:
			this_mb_tx->pin_value[counter]->_u = (hal_u32_t)val * m->scale + m->offset;
			break;
		case HAL_FLOAT:
			this_mb_tx->pin_value[counter]->_f = (hal_s32_t)val * m->scale + m->offset;
			break;
		default:
			return retERR;
		}
	}

	return retOK;
}

static retCode map_write(mb_tx_t *this_mb_tx, uint16_t *data)
{
	for (int counter = 0; counter < this_mb_tx->nb_hal_map_pin; counter++) {
		hal_map_pin_t *m = this_mb_tx->hal_map_pin + counter;
		int addr = m->addr - this_mb_tx->mb_tx_1st_addr;
		uint32_t val;

		switch (m->type) {
		case HAL_S32:
			val = this_mb_tx->pin_value[counter]->_s * m->scale + m->offset;
			break;
		case HAL_U32:
			val = this_mb_tx->pin_value[counter]->_u * m->scale + m->offset;
			break;
		case HAL_FLOAT:
			val = this_mb_tx->pin_value[counter]->_f * m->scale + m->offset;
			break;
		default:
			return retERR;
		}

		if (m->width == 2) {
			data[addr] = val >> 16;
			data[addr+1] = val & 0xFFFF;
		}
		else {
			data[addr] = val;
		}
	}

	return retOK;
}

retCode fnct_02_read_discrete_inputs(mb_tx_t *this_mb_tx, mb_link_t *this_mb_link)
{
    char *fnct_name = "fnct_02_read_discrete_inputs";
    int counter, ret;
    uint8_t bits[MB2HAL_MAX_FNCT02_ELEMENTS];

    if (this_mb_tx == NULL || this_mb_link == NULL) {
        return retERR;
    }
    if (this_mb_tx->mb_tx_nelem > MB2HAL_MAX_FNCT02_ELEMENTS) {
        return retERR;
    }

    DBG(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] fd[%d] 1st_addr[%d] nelem[%d]",
        this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id, modbus_get_socket(this_mb_link->modbus),
        this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem);

    ret = modbus_read_input_bits(this_mb_link->modbus, this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem, bits);
    if (ret < 0) {
        if (modbus_get_socket(this_mb_link->modbus) < 0) {
            modbus_close(this_mb_link->modbus);
        }
        ERR(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] = ret[%d] fd[%d]",
            this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id, ret,
            modbus_get_socket(this_mb_link->modbus));
        return retERR;
    }

    if (!this_mb_tx->nb_hal_map_pin) {
		for (counter = 0; counter < this_mb_tx->mb_tx_nelem; counter++) {
			*(this_mb_tx->bit[counter]) = bits[counter];
		}
    }
    else {
    	for (int counter = 0; counter < this_mb_tx->nb_hal_map_pin; counter++) {
    		hal_map_pin_t *m = this_mb_tx->hal_map_pin + counter;
    		int addr = m->addr - this_mb_tx->mb_tx_1st_addr;
			this_mb_tx->pin_value[counter]->_b = bits[addr];
		}
    }

    return retOK;
}

retCode fnct_03_read_holding_registers(mb_tx_t *this_mb_tx, mb_link_t *this_mb_link)
{
    char *fnct_name = "fnct_03_read_holding_registers";
    int counter, ret;
    uint16_t data[MB2HAL_MAX_FNCT03_ELEMENTS];

    if (this_mb_tx == NULL || this_mb_link == NULL) {
        return retERR;
    }
    if (this_mb_tx->mb_tx_nelem > MB2HAL_MAX_FNCT03_ELEMENTS) {
        return retERR;
    }

    DBG(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] fd[%d] 1st_addr[%d] nelem[%d]",
        this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id,
        modbus_get_socket(this_mb_link->modbus), this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem);

    ret = modbus_read_registers(this_mb_link->modbus, this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem, data);
    if (ret < 0) {
        if (modbus_get_socket(this_mb_link->modbus) < 0) {
            modbus_close(this_mb_link->modbus);
        }
        ERR(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] = ret[%d] fd[%d]",
            this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id, ret,
            modbus_get_socket(this_mb_link->modbus));
        return retERR;
    }

    if (!this_mb_tx->nb_hal_map_pin) {
        for (counter = 0; counter < this_mb_tx->mb_tx_nelem; counter++) {
            float val = data[counter];
            //val *= this_mb_tx->scale[counter];
            //val += this_mb_tx->offset[counter];
            *(this_mb_tx->float_value[counter]) = val;
            *(this_mb_tx->int_value[counter]) = (hal_s32_t) val;
        }
    }
    else {
    	map_read(this_mb_tx, data);
    }

    return retOK;
}

retCode fnct_04_read_input_registers(mb_tx_t *this_mb_tx, mb_link_t *this_mb_link)
{
    char *fnct_name = "fnct_04_read_input_registers";
    int counter, ret;
    uint16_t data[MB2HAL_MAX_FNCT04_ELEMENTS];

    if (this_mb_tx == NULL || this_mb_link == NULL) {
        return retERR;
    }
    if (this_mb_tx->mb_tx_nelem > MB2HAL_MAX_FNCT04_ELEMENTS) {
        return retERR;
    }

    DBG(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] fd[%d] 1st_addr[%d] nelem[%d]",
        this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id,
        modbus_get_socket(this_mb_link->modbus), this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem);

    ret = modbus_read_input_registers(this_mb_link->modbus, this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem, data);
    if (ret < 0) {
        if (modbus_get_socket(this_mb_link->modbus) < 0) {
            modbus_close(this_mb_link->modbus);
        }
        ERR(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] = ret[%d] fd[%d]",
            this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id, ret,
            modbus_get_socket(this_mb_link->modbus));
        return retERR;
    }

    if (!this_mb_tx->nb_hal_map_pin) {
        for (counter = 0; counter < this_mb_tx->mb_tx_nelem; counter++) {
            float val = data[counter];
            //val *= this_mb_tx->scale[counter];
            //val += this_mb_tx->offset[counter];
            *(this_mb_tx->float_value[counter]) = val;
            *(this_mb_tx->int_value[counter]) = (hal_s32_t) val;
        }
    }
    else {
    	map_read(this_mb_tx, data);
    }

    return retOK;
}

retCode fnct_05_write_single_coil(mb_tx_t *this_mb_tx, mb_link_t *this_mb_link)
{
    char *fnct_name = "fnct_05_write_single_coil";
    int ret;
    uint8_t bit;

    if (this_mb_tx == NULL || this_mb_link == NULL) {
        return retERR;
    }
    if (this_mb_tx->mb_tx_nelem != 1) {
        return retERR;
    }

    if (!this_mb_tx->nb_hal_map_pin) {
    	bit = *(this_mb_tx->bit[0]);
    }
    else {
    	bit = this_mb_tx->pin_value[0]->_b;
    }

    DBG(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] fd[%d] 1st_addr[%d] nelem[%d]",
        this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id,
        modbus_get_socket(this_mb_link->modbus), this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem);

    ret = modbus_write_bit(this_mb_link->modbus, this_mb_tx->mb_tx_1st_addr, bit);
    if (ret < 0) {
        if (modbus_get_socket(this_mb_link->modbus) < 0) {
            modbus_close(this_mb_link->modbus);
        }
        ERR(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] = ret[%d] fd[%d]",
            this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id, ret,
            modbus_get_socket(this_mb_link->modbus));
        return retERR;
    }

    return retOK;
}

retCode fnct_15_write_multiple_coils(mb_tx_t *this_mb_tx, mb_link_t *this_mb_link)
{
    char *fnct_name = "fnct_15_write_multiple_coils";
    int counter, ret;
    uint8_t bits[MB2HAL_MAX_FNCT15_ELEMENTS];

    if (this_mb_tx == NULL || this_mb_link == NULL) {
        return retERR;
    }
    if (this_mb_tx->mb_tx_nelem > MB2HAL_MAX_FNCT15_ELEMENTS) {
        return retERR;
    }

    if (!this_mb_tx->nb_hal_map_pin) {
        for (counter = 0; counter < this_mb_tx->mb_tx_nelem; counter++) {
            bits[counter] = *(this_mb_tx->bit[counter]);
        }
    }
    else {
    	for (int counter = 0; counter < this_mb_tx->nb_hal_map_pin; counter++) {
    		hal_map_pin_t *m = this_mb_tx->hal_map_pin + counter;
    		int addr = m->addr - this_mb_tx->mb_tx_1st_addr;
    		bits[addr] = this_mb_tx->pin_value[counter]->_b;
		}
    }

    DBG(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] fd[%d] 1st_addr[%d] nelem[%d]",
        this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id,
        modbus_get_socket(this_mb_link->modbus), this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem);

    ret = modbus_write_bits(this_mb_link->modbus, this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem, bits);
    if (ret < 0) {
        if (modbus_get_socket(this_mb_link->modbus) < 0) {
            modbus_close(this_mb_link->modbus);
        }
        ERR(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] = ret[%d] fd[%d]",
            this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id, ret,
            modbus_get_socket(this_mb_link->modbus));
        return retERR;
    }

    return retOK;
}

retCode fnct_16_write_multiple_registers(mb_tx_t *this_mb_tx, mb_link_t *this_mb_link)
{
    char *fnct_name = "fnct_16_write_multiple_registers";
    int counter, ret;
    uint16_t data[MB2HAL_MAX_FNCT16_ELEMENTS];

    if (this_mb_tx == NULL || this_mb_link == NULL) {
        return retERR;
    }
    if (this_mb_tx->mb_tx_nelem > MB2HAL_MAX_FNCT16_ELEMENTS) {
        return retERR;
    }

    if (!this_mb_tx->nb_hal_map_pin) {
		for (counter = 0; counter < this_mb_tx->mb_tx_nelem; counter++) {
			//float val = *(this_mb_tx->float_value[counter]) / this_mb_tx->scale[counter];
			//val -= this_mb_tx->offset[counter];
			float val = *(this_mb_tx->float_value[counter]);
			data[counter] = (int) val;
		}
    }
    else {
    	map_write(this_mb_tx, data);
    }

    DBG(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] fd[%d] 1st_addr[%d] nelem[%d]",
        this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id,
        modbus_get_socket(this_mb_link->modbus), this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem);

    ret = modbus_write_registers(this_mb_link->modbus, this_mb_tx->mb_tx_1st_addr, this_mb_tx->mb_tx_nelem, data);
    if (ret < 0) {
        if (modbus_get_socket(this_mb_link->modbus) < 0) {
            modbus_close(this_mb_link->modbus);
        }
        ERR(this_mb_tx->cfg_debug, "mb_tx[%d] mb_links[%d] slave[%d] = ret[%d] fd[%d]",
            this_mb_tx->mb_tx_num, this_mb_tx->mb_link_num, this_mb_tx->mb_tx_slave_id, ret,
            modbus_get_socket(this_mb_link->modbus));
        return retERR;
    }

    return retOK;
}
