#include <SpiPort.h>
SpiPort::SpiPort() {
	this->port = GPIOA;
	this->pin = 0;
}

SpiPort::SpiPort(GPIO_TypeDef *port, uint8_t pin) {
	this->port = port;
	this->pin = pin;
}

void SpiPort::setCS(GPIO_TypeDef *port, uint8_t pin) {
	this->port = port;
	this->pin = pin;
}

void SpiPort::setTu(uint8_t *dat0, uint8_t *dat1, uint8_t *dat2, uint8_t *dat3) {
	this->tu[0] = *dat0;
	this->tu[1] = *dat1;
	this->tu[2] = *dat2;
	this->tu[3] = *dat3;
	this->tu[5] = this->tu[0] + this->tu[1] + this->tu[2] + this->tu[3] + this->tu[4];
}

void SpiPort::select() {
	this->port->BRR = (1 << this->pin);
}

void SpiPort::unSelect() {
	this->port->BSRR = (1 << this->pin);
}

uint8_t SpiPort::isRxSummOk() {
	if (this->ts[0] + this->ts[1] + this->ts[2] + this->ts[3] + this->ts[4] == this->ts[5]) {
		return 1;
	}
	return 0;
}

uint8_t SpiPort::isChanged() {
	if (this->rxPrev[0] != this->ts[0] || this->rxPrev[1] != this->ts[1] ||
		this->rxPrev[2] != this->ts[2] || this->rxPrev[3] != this->ts[3] || this->rxPrev[4] != this->ts[4]) {

		this->rxPrev[0] = this->ts[0];
		this->rxPrev[1] = this->ts[1];
		this->rxPrev[2] = this->ts[2];
		this->rxPrev[3] = this->ts[3];
		this->rxPrev[4] = this->ts[4];
		return 1;
	}
	return 0;
}

uint8_t* SpiPort::getTs() {
	return &this->ts[0];
}
uint8_t* SpiPort::getTu() {
	return &this->tu[0];
}
