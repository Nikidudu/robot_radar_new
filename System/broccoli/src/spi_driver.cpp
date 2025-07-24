/*
 *  NUS Calibur Robotics
 *  SPI_driver.cpp
 *
 *  Created on: 9 Dec 2024
 *      Author: JL
 *      Edited by : ______
 */

#ifdef BUILD_WITH_SPI
//#include "Debug/Debug.h"

#include <cstring>
#include <inttypes.h>
#include "stdio.h"
#include <algorithm>


static SPIDriver* instance;

/**
 * @brief Construct a new SPIDriver::SPIDriver object
 *
 * @param hspi the SPI port to initialize
 */

SPIDriver::SPIDriver(SPI_HandleTypeDef* hspi): Thread("SPIDriver", osPriorityNormal), _hspi(hspi), last_dma_index(0) {
	instance = this;
	spi_driver_list.push_back(this);
	this->buffer = (uint8_t*) pvPortMalloc(SPI_BUFFER_SIZE);

    if(buffer == nullptr){
        printf("[Broccoli] [SPIDriverInit] Unable to allocate DMA buffer for MCU#%" PRIu32 "\r\n", getSenderID(hspi));
    }

    this->semaphore = xSemaphoreCreateCounting(16, 0);

    if(semaphore == nullptr) {
        printf("[Broccoli] [SPIDriverInit] Unable to allocate semaphore for MCU#%" PRIu32 "\r\n", getSenderID(hspi));
    }

    setTickDelay(0);
}

SPIDriver::~SPIDriver() {
    vPortFree(buffer);
    spi_driver_list.erase(std::remove(spi_driver_list.begin(), spi_driver_list.end(), this), spi_driver_list.end());
}

void SPIDriver::init() {

	this->last_dma_index = 0;

//	__HAL_SPI_SEND_REQ(hspi, SPI_RXDATA_FLUSH_REQUEST);
#ifdef BUILD_WITH_CACHES
	SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)buffer) & ~(uint32_t)0x1F), SPI_BUFFER_SIZE+32);
#endif
}

void SPIDriver::loop() {
	// int haha = 100 ;
	if(xSemaphoreTake(semaphore, portMAX_DELAY)) {
		uint32_t end_dma_index = SPI_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(_hspi->hdmarx);

		uint8_t sender = getSenderID(_hspi);

		#ifdef BUILD_WITH_CACHES
			SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)buffer) & ~(uint32_t)0x1F), SPI_BUFFER_SIZE+32);
		#endif

		if(end_dma_index < last_dma_index) { // Finish buffer
			receiveSPI(sender, buffer + last_dma_index, SPI_BUFFER_SIZE - last_dma_index);
			this->last_dma_index = 0;
		}

		if(end_dma_index > last_dma_index) {
			receiveSPI(sender, buffer + last_dma_index, end_dma_index - last_dma_index);
			this->last_dma_index = end_dma_index;
		}
	}
}

void SPIDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) {
    this->receiver_func = receiver; // will be bound to IOBus::receive
}

void SPIDriver::transmit(uint8_t* buffer, uint32_t length) {
    if(HAL_SPI_Transmit(_hspi, buffer, length, portMAX_DELAY) != HAL_OK){
//        scanf("[Broccoli] [SPIDriverTransmit] Transmission failed for MCU#%" PRIu32 "\r\n", getSenderID(hspi));
        printf("[Error] SPI transmission failed\r\n");
    }
	#ifdef BUILD_WITH_CACHES
		SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)buffer) & ~(uint32_t)0x1F), SPI_BUFFER_SIZE+32);
	#endif
}


/**
 * @brief Getter to the reference of the buffer
 *
 * @return uint8_t* the reference to the buffer
 */
uint8_t* SPIDriver::getBuffer() {
	return this->buffer;
}


xSemaphoreHandle SPIDriver::getSemaphore() {
	return this->semaphore;
}

/**
 * @brief Function handling the call to the user-defined callback routine
 *
 * @param sender_id the ID of the MCU
 * @param buffer the buffer to provide to the user-defined callback function
 * @param length the size of the data in the buffer to provide
 */
void SPIDriver::receiveSPI(uint8_t sender_id, uint8_t* buffer, uint32_t length) {
	this->receiver_func(sender_id, buffer, length);
}

SPI_HandleTypeDef* SPIDriver::getHspi() {
	return this->_hspi;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi, uint16_t Size) {
	SPIDriver* driver =  (instance)->getInstance(hspi);
	if (driver != nullptr){
		xSemaphoreGiveFromISR(driver->getSemaphore(), nullptr);
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
	SPIDriver* driver = (instance)->getInstance(hspi);
	if (driver != nullptr){
		while(xSemaphoreTakeFromISR(driver->getSemaphore(), nullptr)); // Clear semaphore
		driver->init(); // Reinitialize the driver
	}
}

SPIDriver* SPIDriver::getInstance(SPI_HandleTypeDef* hspi) {
	for (auto & driver : spi_driver_list) {
		if (driver->getHspi() == hspi)
			return driver;
	}
}

/**
 * @brief Get the sender id from the SPI port ID
 *
 * @param hspi the SPI port to get
 * @return uint8_t the sender_id
 */
uint8_t SPIDriver::getSenderID(SPI_HandleTypeDef* hspi) {
    for(int i = 0; i < NB_SPI_PORTS; ++i){
        if(this->mapper[i] == hspi->Instance){
            return i+1;
        }
    }
    return 0;
}

std::vector<SPIDriver*> SPIDriver::spi_driver_list;

#endif /* BUILD_WITH_SPI */
