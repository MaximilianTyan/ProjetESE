/*
 * nfc_sensor.c
 *
 *  Created on: Jan 13, 2025
 *      Author: xana
 */

#include "nfc_sensor.h"
#include "MFRC522.h"
#include "stdio.h"

MFRC522 *RfChip;

void init_nfc(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint32_t cs_pin, GPIO_TypeDef* reset_port, uint32_t reset_pin) {
	RfChip = new MFRC522(hspi, cs_port, cs_pin, reset_port, reset_pin);
	RfChip->PCD_Init();
}

bool get_nfc_id(uint32_t *id) {
	for (uint8_t i = 0; i < 4; i++) {
		if (!RfChip->PICC_IsNewCardPresent()) {
			continue;
		}
		if (!RfChip->PICC_ReadCardSerial()) {
			continue;
		}
		if (RfChip->uid.size >= 4) {
			*id = RfChip->uid.uidByte[0] << 0
					| RfChip->uid.uidByte[1] << 1 * 8
					| RfChip->uid.uidByte[2] << 2 * 8
					| RfChip->uid.uidByte[3] << 3 * 8;
			return true;
		}
	}
	*id = 0x00;
	return false;
}

