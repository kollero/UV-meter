

#include "VEML6075.h"

VEML6075::VEML6075() {
	// Despite the datasheet saying this isn't the default on startup, it appears
	// like it is. So tell the thing to actually start gathering data.
	config = 0;
	config |= VEML6075_CONF_SD_OFF;
	// App note only provided math for this one...
	config |= VEML6075_CONF_IT_100MS;
}

bool VEML6075::begin() {
	// Write config to make sure device is enabled
	write16(VEML6075_REG_CONF, 0b0000000000010000);//config 
	if (getDevID() != VEML6075_DEVID) {
		return false;
	}
	
return true;
}
// Poll sensor for latest values and cache them
void VEML6075::poll() {
	raw_uva = read16(VEML6075_REG_UVA);
	raw_uvb = read16(VEML6075_REG_UVB);
	raw_dark = read16(VEML6075_REG_DUMMY);
	raw_vis = read16(VEML6075_REG_UVCOMP1);
	raw_ir = read16(VEML6075_REG_UVCOMP2);
}
uint16_t VEML6075::getDevID() {
	return read16(VEML6075_REG_DEVID);
}

double VEML6075::getUVA() {
	double comp_vis = abs(raw_vis - raw_dark);
	double comp_ir = abs(raw_ir - raw_dark);
	double comp_uva = abs(raw_uva - raw_dark);
	comp_uva -= VEML6075_UVI_UVA_VIS_COEFF * comp_vis;
	comp_uva -= VEML6075_UVI_UVA_IR_COEFF * comp_ir;
return abs(comp_uva);
}
double VEML6075::getUVB() {
	double comp_vis = abs(raw_vis - raw_dark);
	double comp_ir = abs(raw_ir - raw_dark);
	double comp_uvb = abs(raw_uvb - raw_dark);
	comp_uvb -= VEML6075_UVI_UVB_VIS_COEFF * comp_vis;
	comp_uvb -= VEML6075_UVI_UVB_IR_COEFF * comp_ir;
return abs(comp_uvb);
}
double VEML6075::getUVIndex() {
	double uva_weighted = getUVA() * 0.001100110011;
	double uvb_weighted = getUVB() * 0.00125;
return abs((uva_weighted + uvb_weighted) / 2.0);
}
uint16_t VEML6075::read16(uint8_t reg) {
	//slave,command,slave,datalow with ack, datahigh without ack
	uint8_t msb = 0;
	uint8_t lsb = 0;
	
TWCR = 0;
TWCR =(1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //start
while(!(TWCR &(1<<TWINT))); //wait till start condition has been send
if((TWSR & 0xF8) != TW_START){error=0xF8;} //248 error
	
//transmit address
TWDR = VEML6075_ADDR;//write
TWCR = (1<<TWINT)|(1<<TWEN);
while(!(TWCR &(1<<TWINT)));
if(error==0){
	if( (TWSR & 0xF8) != TW_MT_SLA_ACK)  {error=0x18;} //24
}

//command transmit
TWDR = reg; //check ID
TWCR = (1<<TWINT)|(1<<TWEN);
while(!(TWCR &(1<<TWINT)));
if(error==0){
	if ( (TWSR & 0xF8) != TW_MT_DATA_ACK )  {error=0x28;}//40
}

TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN); //repeated start
while(!(TWCR &(1<<TWINT)));
if((TWSR & 0xF8) != TW_REP_START){error=0x10;} //16 error

TWDR=VEML6075_ADDR|0x01; //read
TWCR=(1<<TWINT)|(1<<TWEN); //resend address with read bit
while(!(TWCR &(1<<TWINT)));
if(error==0){
	if ( (TWSR & 0xF8) != TW_MR_SLA_ACK )  {error=0x40;}//64
}

//reading data LSB
TWCR=(1<<TWINT)|(1<<TWEA)|(1<<TWEN); // ack
while(!(TWCR &(1<<TWINT)));
if(error==0){
	if ( (TWSR & 0xF8) != TW_MR_DATA_ACK )  {error=0x50;}//80
}
lsb=TWDR; //38 devID
//reading data MSB
TWCR = (1<<TWINT)|(1<<TWEN); //no ack
while(!(TWCR &(1<<TWINT)));
msb=TWDR;
if(error==0){
	if ( (TWSR & 0xF8) != TW_MR_DATA_NACK )  {error=0x58;}//88
}

TWCR =(1<<TWINT)|(1<<TWEN)|(1<<TWSTO); //stop
		
return (msb << 8) | lsb;
}
void VEML6075::write16(uint8_t reg, uint16_t data) {
//slave address, command, datalow, datahigh
TWCR = 0;
TWCR =(1<<TWINT)|(1<<TWEN)|(1<<TWSTA); //start
while(!(TWCR &(1<<TWINT))); //wait till start condition has been send
if((TWSR & 0xF8) != TW_START){error=0x08;} //8 error

//transmit address
TWDR=VEML6075_ADDR; //write
TWCR=(1<<TWINT)|(1<<TWEN);
while(!(TWCR &(1<<TWINT)));
if(error==0){
	if( (TWSR & 0xF8) != TW_MT_SLA_ACK)  {error=0x18;} //24
}
//command transmit
TWDR = reg;
TWCR = (1<<TWINT)|(1<<TWEN);
while(!(TWCR &(1<<TWINT)));
if(error==0){
	if ( (TWSR & 0xF8) != TW_MT_DATA_ACK )  {error=0x28;}//40	
} 
TWDR = 0xFF & data ;
TWCR = (1<<TWINT)|(1<<TWEN);//begin transmission
while(!(TWCR &(1<<TWINT))); //till has been sent
if(error==0){
	if ( (TWSR & 0xF8) != TW_MT_DATA_ACK )  {error=0x28;}//40
}
TWDR =(0xFF & (data >> 8));
TWCR = (1<<TWINT)|(1<<TWEN);//begin transmission
while(!(TWCR &(1<<TWINT))); //till has been sent
if(error==0){
	if ( (TWSR & 0xF8) != TW_MT_DATA_ACK )  {error=0x28;}//40
}
TWCR =(1<<TWINT)|(1<<TWSTO)|(1<<TWEN); //stop	

}